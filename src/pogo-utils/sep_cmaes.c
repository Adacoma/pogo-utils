/**
 * @file sep_cmaes.c
 * @brief Implementation of lightweight separable CMA-ES (diag covariance) for float arrays.
 */
#include "sep_cmaes.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------- Small helpers -------------------------------- */
static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline int better(sep_mode_t mode, float f_try, float f_best) {
    return (mode == SEP_MINIMIZE) ? (f_try < f_best) : (f_try > f_best);
}

/* Box–Muller with spare */
static float randn01(sep_cmaes_t *es) {
    if (es->have_spare) {
        es->have_spare = 0;
        return es->spare;
    }
    float u1 = 0.0f, u2 = 0.0f;
    do { u1 = (float)rand() / (float)RAND_MAX; } while (u1 <= 1e-7f);
    u2 = (float)rand() / (float)RAND_MAX;
    float r = sqrtf(-2.0f * logf(u1));
    float th = 2.0f * (float)M_PI * u2;
    es->spare = r * sinf(th);
    es->have_spare = 1;
    return r * cosf(th);
}

/* Expected ||N(0,I)|| for CSA (approx) */
static float approx_chi_n(int n) {
    float nf = (float)n;
    return sqrtf(nf) * (1.0f - 1.0f/(4.0f*nf) + 1.0f/(21.0f*nf*nf));
}

/* Build normalized log-weights into local_w if w==NULL, else normalize user weights into local_w. */
static void build_norm_weights(int mu, const float *w, float *local_w) {
    float sumw = 0.0f;
    if (!w) {
        for (int i = 0; i < mu && i < 32; ++i) {
            local_w[i] = logf((float)(mu + 0.5f)) - logf((float)(i + 1));
            if (local_w[i] < 0.0f) local_w[i] = 0.0f;
            sumw += local_w[i];
        }
        if (sumw <= 0.0f) {
            for (int i = 0; i < mu && i < 32; ++i) local_w[i] = 1.0f / (float)mu;
        } else {
            for (int i = 0; i < mu && i < 32; ++i) local_w[i] /= sumw;
        }
    } else {
        for (int i = 0; i < mu && i < 32; ++i) { local_w[i] = w[i]; sumw += local_w[i]; }
        if (sumw <= 0.0f) sumw = 1.0f;
        for (int i = 0; i < mu && i < 32; ++i) local_w[i] /= sumw;
    }
}

/* Derive mu_w and defaults given n, lambda, mu and weights. */
static void derive_defaults(int n, int lambda, int mu, const float *weights,
                            float *w_sum, float *mu_w,
                            float *cc, float *cs, float *c1, float *cmu, float *damps) {
    (void)lambda;
    float local_w[32];
    build_norm_weights(mu, weights, local_w);

    *w_sum = 0.0f;
    float w2_sum = 0.0f;
    for (int i = 0; i < mu && i < 32; ++i) {
        float wi = local_w[i];
        *w_sum += wi;
        w2_sum += wi * wi;
    }
    if (*w_sum <= 0.0f) *w_sum = 1.0f;
    *mu_w = (*w_sum) * (*w_sum) / w2_sum;

    float nf = (float)n;
    if (*cc <= 0.0f) *cc  = 2.0f / (nf + 1.4142f);
    if (*cs <= 0.0f) *cs  = ((*mu_w) + 2.0f) / (nf + (*mu_w) + 5.0f);
    if (*c1 <= 0.0f) *c1  = 2.0f / (((nf + 1.3f)*(nf + 1.3f)) + (*mu_w));
    if (*cmu<= 0.0f) *cmu = fminf(1.0f - (*c1), 2.0f * ((*mu_w) - 1.0f) / (((nf + 2.0f)*(nf + 2.0f)) + (*mu_w)));
    if (*damps <= 0.0f) *damps = 1.0f + 2.0f * fmaxf(0.0f, sqrtf(((*mu_w) - 1.0f) / (nf + 1.0f)) - 1.0f) + *cs;
}

/* Tiny insertion sort of indices by fitness (ascending for minimize, descending for maximize). */
static void sort_indices_by_fitness(const float *fit, int *idx, int nidx, sep_mode_t mode) {
    for (int i = 0; i < nidx; ++i) idx[i] = i;
    for (int i = 1; i < nidx; ++i) {
        int j = i;
        int key = idx[i];
        float fkey = fit[key];
        if (mode == SEP_MINIMIZE) {
            while (j > 0 && fit[idx[j-1]] > fkey) { idx[j] = idx[j-1]; --j; }
        } else {
            while (j > 0 && fit[idx[j-1]] < fkey) { idx[j] = idx[j-1]; --j; }
        }
        idx[j] = key;
    }
}

/* ------------------------------- API ------------------------------------ */
void sep_cmaes_init(sep_cmaes_t *es, int n,
                    float *restrict x, float *restrict x_try,
                    float *restrict c_diag,
                    float *restrict p_sigma, float *restrict p_c,
                    float *restrict z_try, float *restrict y_try,
                    float *restrict store_y,
                    float *restrict fit_buf, int *restrict idx,
                    const sep_params_t *params) {
    if (!es || !x || !x_try || !p_sigma || !p_c || !z_try || !y_try || !store_y || !fit_buf || !idx || n <= 0) return;

    memset(es, 0, sizeof(*es));
    es->n = n;
    es->x = x;
    es->x_try = x_try;
    es->c_diag = c_diag;
    es->p_sigma = p_sigma;
    es->p_c = p_c;
    es->z_try = z_try;
    es->y_try = y_try;
    es->store_y = store_y;
    es->fit_buf = fit_buf;
    es->idx = idx;

    /* Defaults */
    es->p.mode = SEP_MINIMIZE;
    es->p.lambda = 8;
    es->p.mu = 4;
    es->p.sigma0 = 0.3f;
    es->p.sigma_min = 1e-8f;
    es->p.sigma_max = 5.0f;
    es->p.weights = NULL;
    es->p.cc = 0.0f;
    es->p.cs = 0.0f;
    es->p.c1 = 0.0f;
    es->p.cmu = 0.0f;
    es->p.damps = 0.0f;
    es->p.lo = NULL;
    es->p.hi = NULL;

    if (params) {
        es->p = *params;
    }
    if (es->p.mu < 1) es->p.mu = (es->p.lambda > 1) ? (es->p.lambda/2) : 1;
    if (es->p.mu > es->p.lambda) es->p.mu = es->p.lambda;

    /* Derive effective masses and defaults */
    float cc = es->p.cc, cs = es->p.cs, c1 = es->p.c1, cmu = es->p.cmu, damps = es->p.damps;
    derive_defaults(n, es->p.lambda, es->p.mu, es->p.weights, &es->w_sum, &es->mu_w, &cc, &cs, &c1, &cmu, &damps);
    es->p.cc = cc; es->p.cs = cs; es->p.c1 = c1; es->p.cmu = cmu; es->p.damps = damps;

    es->sigma = (es->p.sigma0 > 0.0f) ? es->p.sigma0 : 0.3f;
    es->chi_n = approx_chi_n(n);

    /* Initialize arrays */
    for (int i = 0; i < n; ++i) {
        if (es->c_diag) es->c_diag[i] = 1.0f;
        es->p_sigma[i] = 0.0f;
        es->p_c[i] = 0.0f;
    }
    /* Zero μ-store to avoid using uninitialized values. */
    memset(es->store_y, 0, (size_t)es->p.mu * (size_t)n * sizeof(float));

    /* μ-best cache state */
    es->mu_filled = 0;

    es->have_spare = 0;
    es->spare = 0.0f;
    es->iter_total = 0;
    es->gen = 0;
    es->k_in_gen = 0;
    es->have_initial = 0;
}

void sep_cmaes_tell_initial(sep_cmaes_t *es, float f0) {
    if (!es) return;
    es->best_f = f0;
    es->have_initial = 1;
}

/* ask: sample z ~ N(0,I), y = sqrt(c_diag) ⊙ z, x' = x + sigma * y */
const float *sep_cmaes_ask(sep_cmaes_t *es) {
    if (!es || !es->have_initial) return NULL;

    for (int i = 0; i < es->n; ++i) {
        float z = randn01(es);
        es->z_try[i] = z;
        float sd = (es->c_diag ? sqrtf(fmaxf(es->c_diag[i], 0.0f)) : 1.0f);
        float yi = sd * z;
        es->y_try[i] = yi;
        float xi = es->x[i] + es->sigma * yi;
        if (es->p.lo && es->p.hi) {
            xi = clampf(xi, es->p.lo[i], es->p.hi[i]);
        } else if (es->p.lo) {
            if (xi < es->p.lo[i]) xi = es->p.lo[i];
        } else if (es->p.hi) {
            if (xi > es->p.hi[i]) xi = es->p.hi[i];
        }
        es->x_try[i] = xi;
    }
    return es->x_try;
}

/* Insert current (f_try, y_try) into μ-best leaderboard (stable, small μ<=32). */
static void mu_best_insert(sep_cmaes_t *es, float f_try) {
    const int mu = es->p.mu < 32 ? es->p.mu : 32;

    /* On first offspring of a generation, reset cache and clear store_y. */
    if (es->k_in_gen == 0) {
        es->mu_filled = 0;
        for (int j = 0; j < mu; ++j) {
            es->best_fit_mu[j] = (es->p.mode == SEP_MINIMIZE) ? INFINITY : -INFINITY;
            /* zero rows to be safe */
            float *row = es->store_y + (size_t)j * (size_t)es->n;
            for (int i = 0; i < es->n; ++i) row[i] = 0.0f;
        }
    }

    int insert_pos = -1;
    for (int j = 0; j < mu; ++j) {
        if ((es->p.mode == SEP_MINIMIZE && f_try < es->best_fit_mu[j]) ||
            (es->p.mode == SEP_MAXIMIZE && f_try > es->best_fit_mu[j])) {
            insert_pos = j; break;
        }
    }
    if (insert_pos < 0) {
        if (es->mu_filled < mu) {
            /* place at the end */
            int j = es->mu_filled;
            es->best_fit_mu[j] = f_try;
            float *dst = es->store_y + (size_t)j * (size_t)es->n;
            for (int i = 0; i < es->n; ++i) dst[i] = es->y_try[i];
            es->mu_filled++;
        }
        return;
    }

    /* shift down */
    int limit = (es->mu_filled < mu) ? es->mu_filled : (mu - 1);
    for (int j = limit; j > insert_pos; --j) {
        es->best_fit_mu[j] = es->best_fit_mu[j-1];
        float *dst = es->store_y + (size_t)j * (size_t)es->n;
        float *src = es->store_y + (size_t)(j-1) * (size_t)es->n;
        for (int i = 0; i < es->n; ++i) dst[i] = src[i];
    }
    es->best_fit_mu[insert_pos] = f_try;
    float *dst = es->store_y + (size_t)insert_pos * (size_t)es->n;
    for (int i = 0; i < es->n; ++i) dst[i] = es->y_try[i];
    if (es->mu_filled < mu) es->mu_filled++;
}

float sep_cmaes_tell(sep_cmaes_t *es, float f_try) {
    if (!es || !es->have_initial) return es ? es->best_f : 0.0f;

    /* Incremental μ-best cache (uses current y_try). */
    mu_best_insert(es, f_try);

    int k = es->k_in_gen;
    es->fit_buf[k] = f_try;
    es->k_in_gen++;

    if (better(es->p.mode, f_try, es->best_f)) {
        es->best_f = f_try;
    }

    es->iter_total++;

    /* If generation is complete, do the updates. */
    if (es->k_in_gen >= es->p.lambda) {
        /* Sort indices only for logging/diagnostics; not needed for μ-best because we cached top-μ already. */
        sort_indices_by_fitness(es->fit_buf, es->idx, es->p.lambda, es->p.mode);

        /* Recombine mean shift y_w = sum_i w_i * y_i over μ_filled slots */
        float y_w_stack[256];
        float *y_w = (es->n <= 256) ? y_w_stack : (float*)alloca((size_t)es->n * sizeof(float));
        for (int i = 0; i < es->n; ++i) y_w[i] = 0.0f;

        float wloc[32];
        build_norm_weights(es->p.mu, es->p.weights, wloc);

        const int m = es->mu_filled;
        for (int j = 0; j < m; ++j) {
            const float *yj = es->store_y + (size_t)j * (size_t)es->n;
            float wj = (j < 32) ? wloc[j] : 0.0f;
            for (int i = 0; i < es->n; ++i) y_w[i] += wj * yj[i];
        }

        /* Update mean */
        for (int i = 0; i < es->n; ++i) {
            es->x[i] += es->sigma * y_w[i];
            if (es->p.lo && es->x[i] < es->p.lo[i]) es->x[i] = es->p.lo[i];
            if (es->p.hi && es->x[i] > es->p.hi[i]) es->x[i] = es->p.hi[i];
        }

        /* Update p_sigma (CSA) using z_w = y_w / sqrt(c_diag) */
        float norm_ps = 0.0f;
        for (int i = 0; i < es->n; ++i) {
            float inv_sd = (es->c_diag ? 1.0f / sqrtf(fmaxf(es->c_diag[i], 1e-20f)) : 1.0f);
            float z_w = y_w[i] * inv_sd;
            es->p_sigma[i] = (1.0f - es->p.cs) * es->p_sigma[i] + sqrtf(es->p.cs * (2.0f - es->p.cs) * es->mu_w) * z_w;
            norm_ps += es->p_sigma[i] * es->p_sigma[i];
        }
        float norm_z_w = sqrtf(fmaxf(norm_ps, 1e-30f));

        /* Sigma update */
        float exponent = (es->p.cs / es->p.damps) * (norm_z_w / es->chi_n - 1.0f);
        if (isfinite(exponent)) {
            es->sigma *= expf(exponent);
        }
        if (!(es->sigma == es->sigma) || es->sigma <= 0.0f) { /* NaN or non-positive -> reset softly */
            es->sigma = fmaxf(es->p.sigma_min, 0.1f * ( (es->p.sigma0>0)?es->p.sigma0:0.3f ));
        }
        if (es->sigma < es->p.sigma_min) es->sigma = es->p.sigma_min;
        if (es->sigma > es->p.sigma_max) es->sigma = es->p.sigma_max;

        /* Update p_c and diagonal covariance */
        float one_minus_cs_pow = powf(1.0f - es->p.cs, 2.0f * (es->gen + 1));
        float denom = 1.0f - one_minus_cs_pow;
        if (denom < 1e-9f) denom = 1e-9f;
        float thresh = (1.4f + 2.0f / (float)(es->n + 1)) * es->chi_n;
        float lhs = norm_z_w / sqrtf(denom);
        float h_sigma = (lhs < thresh) ? 1.0f : 0.0f;

        for (int i = 0; i < es->n; ++i) {
            es->p_c[i] = (1.0f - es->p.cc) * es->p_c[i] + h_sigma * sqrtf(es->p.cc * (2.0f - es->p.cc) * es->mu_w) * y_w[i];
            if (es->c_diag) {
                float rank_mu_term = 0.0f;
                for (int j = 0; j < m; ++j) {
                    const float *yj = es->store_y + (size_t)j * (size_t)es->n;
                    float wj = (j < 32) ? wloc[j] : 0.0f;
                    float yji = yj[i];
                    rank_mu_term += wj * yji * yji;
                }
                float Cii = es->c_diag[i];
                Cii = (1.0f - es->p.c1 - es->p.cmu) * Cii + es->p.c1 * es->p_c[i] * es->p_c[i] + es->p.cmu * rank_mu_term;
                if (!(Cii == Cii) || Cii < 1e-12f) Cii = 1e-12f;
                if (Cii > 1e6f) Cii = 1e6f;
                es->c_diag[i] = Cii;
            }
        }

        /* Reset generation */
        es->k_in_gen = 0;
        es->gen++;
    }

    return es->best_f;
}

float sep_cmaes_step(sep_cmaes_t *es, sep_objective_fn fn, void *userdata) {
    if (!es || !fn || !es->have_initial) return es ? es->best_f : 0.0f;
    const float *x_try = sep_cmaes_ask(es);
    if (!x_try) return es->best_f;
    float f_try = fn(x_try, es->n, userdata);
    return sep_cmaes_tell(es, f_try);
}

