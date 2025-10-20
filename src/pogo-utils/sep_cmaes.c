/**
 * @file sep_cmaes.c
 * @brief Implementation of a malloc-free, online-capable Separable CMA-ES (diagonal covariance).
 */
#include "sep_cmaes.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ===== Helpers =========================================================== */

static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline int better(sep_cmaes_mode_t mode, float a, float b) {
    return (mode == SEP_CMAES_MINIMIZE) ? (a < b) : (a > b);
}

/* Box–Muller with spare, using rand() — seed with srand(). */
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

static inline float expected_norm(int n) {
    /* E||N(0,I)|| ≈ sqrt(n) * (1 - 1/(4n) + 1/(21 n^2)) */
    float nf = (float)n;
    return sqrtf(nf) * (1.0f - 1.0f/(4.0f*nf) + 1.0f/(21.0f*nf*nf));
}

static int default_lambda(int n) {
    float ln_n = logf((float)n);
    int L = 4 + (int)floorf(3.0f * ln_n);
    if (L < 4) L = 4;
    return L;
}

static void compute_default_params(sep_cmaes_t *es) {
    const int n = es->n;
    if (es->p.lambda <= 0) es->lambda = default_lambda(n);
    else es->lambda = es->p.lambda;

    if (es->p.mu <= 0) es->mu = es->lambda / 2;
    else {
        es->mu = es->p.mu;
        if (es->mu > es->lambda) es->mu = es->lambda;
    }

    if (es->p.sigma0 <= 0.0f) es->p.sigma0 = 0.3f;
    if (es->p.sigma_min <= 0.0f) es->p.sigma_min = 1e-8f;
    if (es->p.sigma_max <= 0.0f) es->p.sigma_max = 10.0f;

    /* CMA-ish defaults */
    if (es->p.cs <= 0.0f) es->p.cs = (n + 4.0f) / (n + 10.0f);
    if (es->p.ds <= 0.0f) es->p.ds = 1.0f + 2.0f * fmaxf(0.0f, sqrtf((float)es->mu) - 1.0f) + es->p.cs;
    if (es->p.cc <= 0.0f) es->p.cc = 2.0f / ((float)n + 2.0f);

    /* sep-CMA learning rates (conservative, robust) */
    if (es->p.c1_sep <= 0.0f)  es->p.c1_sep  = 2.0f / ((float)(n*n) + 6.0f);
    if (es->p.cmu_sep <= 0.0f) es->p.cmu_sep = fminf(1.0f - es->p.c1_sep,  0.3f);

    if (es->p.evals_per_tick < 1) es->p.evals_per_tick = 1;
}

static void compute_log_weights(sep_cmaes_t *es) {
    /* Hansen log-weights: w_i ~ log(mu+0.5) - log(i+1), i=0..mu-1 */
    float sum_w = 0.0f;
    float sum_w_sq = 0.0f;
    const float log_mu = logf((float)es->mu + 0.5f);
    for (int i = 0; i < es->mu; ++i) {
        float wi = log_mu - logf((float)(i + 1));
        es->weights[i] = wi;
        sum_w += wi;
    }
    /* Normalize to sum(w)=1 */
    if (sum_w <= 0.0f) sum_w = 1.0f;
    for (int i = 0; i < es->mu; ++i) {
        es->weights[i] /= sum_w;
        sum_w_sq += es->weights[i] * es->weights[i];
    }
    es->mu_eff = 1.0f / sum_w_sq;
}

/* Portable O(lambda^2) insertion sort of indices by fitness (mode-aware). */
static void sort_indices_by_fitness(sep_cmaes_t *es) {
    const int lambda = es->lambda;
    for (int i = 0; i < lambda; ++i) es->idx[i] = i;

    if (es->mode == SEP_CMAES_MINIMIZE) {
        for (int i = 1; i < lambda; ++i) {
            int key = es->idx[i];
            float fkey = es->fitness[key];
            int j = i - 1;
            while (j >= 0 && es->fitness[es->idx[j]] > fkey) {
                es->idx[j + 1] = es->idx[j];
                --j;
            }
            es->idx[j + 1] = key;
        }
    } else { /* maximize */
        for (int i = 1; i < lambda; ++i) {
            int key = es->idx[i];
            float fkey = es->fitness[key];
            int j = i - 1;
            while (j >= 0 && es->fitness[es->idx[j]] < fkey) {
                es->idx[j + 1] = es->idx[j];
                --j;
            }
            es->idx[j + 1] = key;
        }
    }
}

/* ===== API =============================================================== */

void sep_cmaes_init(sep_cmaes_t *es, int n,
                    float *restrict m,
                    float *restrict diagC, float *restrict invsqrtC,
                    float *restrict ps, float *restrict pc,
                    float *restrict z_work, float *restrict y_work,
                    float *restrict cand_X, float *restrict fitness, int *restrict idx,
                    float *restrict weights,
                    const float *restrict lo, const float *restrict hi,
                    sep_cmaes_objective_fn fn, void *user,
                    const sep_cmaes_params_t *params) {
    if (!es || !m || !diagC || !invsqrtC || !ps || !pc ||
        !z_work || !y_work || !cand_X || !fitness || !idx || !weights || !fn || n <= 0) {
        return;
    }

    memset(es, 0, sizeof(*es));
    es->n = n;
    es->m = m;
    es->diagC = diagC;
    es->invsqrtC = invsqrtC;
    es->ps = ps;
    es->pc = pc;
    es->z_work = z_work;
    es->y_work = y_work;
    es->cand_X = cand_X;
    es->fitness = fitness;
    es->idx = idx;
    es->weights = weights;
    es->lo = lo;
    es->hi = hi;
    es->fn = fn;
    es->user = user;

    es->mode = params ? params->mode : SEP_CMAES_MINIMIZE;
    if (params) es->p = *params; else memset(&es->p, 0, sizeof(es->p));
    compute_default_params(es);

    /* Initialize paths, covariance, σ */
    for (int i = 0; i < n; ++i) {
        es->ps[i] = 0.0f;
        es->pc[i] = 0.0f;
        if (es->diagC) {
            if (es->diagC[i] <= 0.0f) es->diagC[i] = 1.0f;
        }
        es->invsqrtC[i] = 1.0f / sqrtf(es->diagC[i]);
    }
    es->sigma = es->p.sigma0;
    es->chi_n = expected_norm(n);
    es->k = 0;
    es->gen = 0;
    es->evals = 0;
    es->have_spare = 0;

    /* Precompute default log-weights if caller didn't fill them. */
    compute_log_weights(es);

    /* Evaluate m to set best_f and ensure bounds respected if provided */
    if (es->lo && es->hi) {
        for (int i = 0; i < n; ++i) {
            es->m[i] = clampf(es->m[i], es->lo[i], es->hi[i]);
        }
    } else if (es->lo) {
        for (int i = 0; i < n; ++i) if (es->m[i] < es->lo[i]) es->m[i] = es->lo[i];
    } else if (es->hi) {
        for (int i = 0; i < n; ++i) if (es->m[i] > es->hi[i]) es->m[i] = es->hi[i];
    }
    es->best_f = es->fn(es->m, es->n, es->user);
    es->evals++;
}

/* Generate and evaluate one offspring (index k). Returns its fitness. */
static float sample_and_eval_k(sep_cmaes_t *es, int k) {
    const int n = es->n;
    float *xk = &es->cand_X[(size_t)k * (size_t)n];

    for (int i = 0; i < n; ++i) {
        float z = randn01(es);
        es->z_work[i] = z;
        es->y_work[i] = (1.0f / es->invsqrtC[i]) * z; /* sqrt(diagC) = 1 / invsqrtC */
        xk[i] = es->m[i] + es->sigma * es->y_work[i];
    }

    if (es->lo && es->hi) {
        for (int i = 0; i < n; ++i) xk[i] = clampf(xk[i], es->lo[i], es->hi[i]);
    } else if (es->lo) {
        for (int i = 0; i < n; ++i) if (xk[i] < es->lo[i]) xk[i] = es->lo[i];
    } else if (es->hi) {
        for (int i = 0; i < n; ++i) if (xk[i] > es->hi[i]) xk[i] = es->hi[i];
    }

    float f = es->fn(xk, n, es->user);
    es->fitness[k] = f;
    if (better(es->mode, f, es->best_f)) {
        es->best_f = f;
    }
    es->evals++;
    return f;
}

/* One generation update (after λ offspring are evaluated). */
static void cma_update(sep_cmaes_t *es) {
    const int n = es->n, lambda = es->lambda, mu = es->mu;
    (void)lambda;

    /* Sort indices by fitness (portable) */
    sort_indices_by_fitness(es);

    /* Save m_old into y_work temporarily, then we'll reuse buffers */
    for (int i = 0; i < n; ++i) es->y_work[i] = es->m[i];

    /* Compute weighted mean shift in standardized coordinates:
       y_i = (x_i - m_old) / sigma, and z_i = invsqrtC ⊙ y_i  (since sep-CMA uses diag)
       For the mean: m_new = m_old + sigma * sum_i w_i * y_i
     */
    for (int i = 0; i < n; ++i) es->z_work[i] = 0.0f; /* will accumulate sum w_i * z_i */
    for (int i = 0; i < n; ++i) es->m[i] = es->y_work[i]; /* start from m_old again for clarity */

    for (int r = 0; r < mu; ++r) {
        const int k = es->idx[r];
        const float w = es->weights[r];
        const float *xk = &es->cand_X[(size_t)k * (size_t)n];
        for (int i = 0; i < n; ++i) {
            float y_i = (xk[i] - es->y_work[i]) / es->sigma;
            float z_i = y_i * es->invsqrtC[i];
            es->m[i] += es->sigma * w * y_i;
            es->z_work[i] += w * z_i; /* accumulate for ps update */
        }
    }

    /* Update ps (σ-path) */
    const float cs = es->p.cs;
    const float one_minus_cs = 1.0f - cs;
    const float alpha_ps = sqrtf(cs * (2.0f - cs) * es->mu_eff);
    float ps_norm = 0.0f;
    for (int i = 0; i < n; ++i) {
        es->ps[i] = one_minus_cs * es->ps[i] + alpha_ps * es->z_work[i];
        ps_norm += es->ps[i] * es->ps[i];
    }
    ps_norm = sqrtf(ps_norm);

    /* Heaviside for rank-one correction */
    const float hsig_thr = (1.4f + 2.0f / ((float)n + 1.0f)) * es->chi_n;
    const int hsig = (ps_norm / sqrtf(1.0f - powf(one_minus_cs, 2.0f * (float)(es->gen + 1))) < hsig_thr) ? 1 : 0;

    /* Update pc (covariance path) */
    const float cc = es->p.cc;
    const float one_minus_cc = 1.0f - cc;
    for (int i = 0; i < n; ++i) {
        float y_c = 0.0f; /* sum w_i * y_i (recompute cheaply) */
        for (int r = 0; r < mu; ++r) {
            int k = es->idx[r];
            float w = es->weights[r];
            const float *xk = &es->cand_X[(size_t)k * (size_t)n];
            y_c += w * (xk[i] - es->y_work[i]) / es->sigma;
        }
        es->pc[i] = one_minus_cc * es->pc[i] + (hsig ? sqrtf(cc * (2.0f - cc) * es->mu_eff) * y_c : 0.0f);
    }

    /* Update diagonal covariance: C <- (1-c1-cmu)C + c1*(pc◦pc + (1-hsig)*cc*(2-cc)C) + cmu*Σ w_i * (y_i◦y_i) */
    const float c1  = es->p.c1_sep;
    const float cmu = es->p.cmu_sep;
    const float one_minus_c = fmaxf(0.0f, 1.0f - c1 - cmu);
    for (int i = 0; i < n; ++i) {
        float rank_mu = 0.0f;
        for (int r = 0; r < mu; ++r) {
            int k = es->idx[r];
            float w = es->weights[r];
            const float *xk = &es->cand_X[(size_t)k * (size_t)n];
            float yi = (xk[i] - es->y_work[i]) / es->sigma;
            rank_mu += w * yi * yi;
        }
        float rank_one = es->pc[i] * es->pc[i];
        float oldC = es->diagC[i];
        float add_hsig = (hsig ? 0.0f : cc * (2.0f - cc)) * oldC;
        float newC = one_minus_c * oldC + c1 * (rank_one + add_hsig) + cmu * rank_mu;
        /* Ensure positivity and numerical stability */
        if (newC < 1e-16f) newC = 1e-16f;
        es->diagC[i] = newC;
        es->invsqrtC[i] = 1.0f / sqrtf(newC);
    }

    /* Update sigma */
    const float ds = es->p.ds;
    es->sigma *= expf((cs / ds) * (ps_norm / es->chi_n - 1.0f));
    if (es->sigma < es->p.sigma_min) es->sigma = es->p.sigma_min;
    if (es->sigma > es->p.sigma_max) es->sigma = es->p.sigma_max;

    es->gen++;
}

float sep_cmaes_step(sep_cmaes_t *es) {
    if (!es || !es->fn) return 0.0f;

    /* Generate/evaluate up to evals_per_tick offspring */
    int todo = es->p.evals_per_tick;
    while (todo-- > 0) {
        /* If generation complete, perform update and reset k */
        if (es->k >= es->lambda) {
            cma_update(es);
            es->k = 0;
        }
        /* Generate/evaluate offspring k */
        sample_and_eval_k(es, es->k);
        es->k++;
    }

    /* If we just finished the generation exactly at the end of the batch, apply update now */
    if (es->k >= es->lambda) {
        cma_update(es);
        es->k = 0;
    }
    return es->best_f;
}

