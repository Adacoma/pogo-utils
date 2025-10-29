/**
 * @file pgpe.c
 * @brief Implementation of PGPE (ask–tell, antithetic) over float arrays.
 */
#include "pgpe.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ---------- Helpers ----------------------------------------------------- */
static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline float to_reward(pgpe_mode_t mode, float fitness) {
    return (mode == PGPE_MAXIMIZE) ? fitness : -fitness;
}

/* Box–Muller with spare */
static float randn01(pgpe_t *pg) {
    if (pg->have_spare) {
        pg->have_spare = 0;
        return pg->spare;
    }
    float u1 = 0.0f, u2 = 0.0f;
    do { u1 = (float)rand() / (float)RAND_MAX; } while (u1 <= 1e-7f);
    u2 = (float)rand() / (float)RAND_MAX;
    float r = sqrtf(-2.0f * logf(u1));
    float th = 2.0f * (float)M_PI * u2;
    pg->spare = r * sinf(th);
    pg->have_spare = 1;
    return r * cosf(th);
}

static void clamp_sigma_vec(pgpe_t *pg) {
    for (int i = 0; i < pg->n; ++i) {
        if (pg->sigma[i] < pg->p.sigma_min) pg->sigma[i] = pg->p.sigma_min;
        if (pg->sigma[i] > pg->p.sigma_max) pg->sigma[i] = pg->p.sigma_max;
    }
}

static void clamp_mu_to_bounds(pgpe_t *pg) {
    if (!pg->lo && !pg->hi) return;
    for (int i = 0; i < pg->n; ++i) {
        float mi = pg->mu[i];
        if (pg->lo) { if (mi < pg->lo[i]) mi = pg->lo[i]; }
        if (pg->hi) { if (mi > pg->hi[i]) mi = pg->hi[i]; }
        pg->mu[i] = mi;
    }
}

static void build_candidate(pgpe_t *pg, int sign /*+1 for plus, -1 for minus*/) {
    for (int i = 0; i < pg->n; ++i) {
        float xi = pg->mu[i] + (float)sign * pg->sigma[i] * pg->eps[i];
        if (pg->lo && pg->hi) {
            xi = clampf(xi, pg->lo[i], pg->hi[i]);
        } else if (pg->lo) {
            if (xi < pg->lo[i]) xi = pg->lo[i];
        } else if (pg->hi) {
            if (xi > pg->hi[i]) xi = pg->hi[i];
        }
        pg->x_try[i] = xi;
    }
}

/* ---------- API --------------------------------------------------------- */
void pgpe_init(pgpe_t *pg, int n,
               float *restrict mu, float *restrict sigma,
               float *restrict x_try, float *restrict eps,
               const float *restrict lo, const float *restrict hi,
               const pgpe_params_t *params) {
    if (!pg || !mu || !sigma || !x_try || !eps || n <= 0) return;

    memset(pg, 0, sizeof(*pg));
    pg->n     = n;
    pg->mu    = mu;
    pg->sigma = sigma;
    pg->x_try = x_try;
    pg->eps   = eps;
    pg->lo    = lo;
    pg->hi    = hi;

    /* Defaults */
    pg->p.mode           = PGPE_MINIMIZE;
    pg->p.eta_mu         = 0.05f;
    pg->p.eta_sigma      = 0.10f;
    pg->p.sigma_min      = 1e-6f;
    pg->p.sigma_max      = 1.0f;
    pg->p.baseline_alpha = 0.10f;
    pg->p.normalize_pair = true;

    if (params) pg->p = *params;

    clamp_sigma_vec(pg);
    clamp_mu_to_bounds(pg);

    pg->phase      = 0;
    pg->r_plus     = 0.0f;
    pg->r_minus    = 0.0f;
    pg->baseline   = 0.0f;  /* start neutral */
    pg->have_spare = 0;
    pg->iters      = 0;
}

const float *pgpe_ask(pgpe_t *pg) {
    if (!pg) return NULL;

    if (pg->phase == 0) {
        /* Start a new antithetic pair: sample ε and output θ+ */
        for (int i = 0; i < pg->n; ++i) {
            pg->eps[i] = randn01(pg);
        }
        build_candidate(pg, +1);
        pg->phase = 1; /* next we want θ− */
        return pg->x_try;
    } else {
        /* Second half: θ− with the same ε */
        build_candidate(pg, -1);
        pg->phase = 2; /* next we will update after tell() */
        return pg->x_try;
    }
}

void pgpe_tell(pgpe_t *pg, float fitness) {
    if (!pg) return;

    float r = to_reward(pg->p.mode, fitness);

    if (pg->phase == 1) {
        /* We just evaluated θ+ */
        pg->r_plus = r;
        return;
    }

    if (pg->phase == 2) {
        /* We just evaluated θ− — perform update */
        pg->r_minus = r;

        float r_plus = pg->r_plus;
        float r_minus = pg->r_minus;

        if (pg->p.normalize_pair) {
            /* Center the pair to reduce variance (pair baseline) */
            float m = 0.5f * (r_plus + r_minus);
            r_plus  -= m;
            r_minus -= m;
        }

        /* Update moving baseline (EWMA) using original rewards */
        float raw_pair_mean = 0.5f * (pg->r_plus + pg->r_minus);
        pg->baseline = (1.0f - pg->p.baseline_alpha) * pg->baseline
                       + pg->p.baseline_alpha * raw_pair_mean;

        /* Gradients and updates */
        const float tiny = 1e-12f;
        float diff = 0.5f * (r_plus - r_minus);  /* scalar factor for μ gradient */
        float sumc = 0.5f * ((pg->r_plus + pg->r_minus) - 2.0f * pg->baseline); /* for log σ */

        for (int i = 0; i < pg->n; ++i) {
            float sig = (pg->sigma[i] > pg->p.sigma_min) ? pg->sigma[i] : pg->p.sigma_min;
            float eps = pg->eps[i];

            /* ∇_μ J ≈ diff * ε / σ */
            float g_mu = diff * (eps / (sig + tiny));
            pg->mu[i] += pg->p.eta_mu * g_mu;

            /* ∇_{log σ} J ≈ sumc * (ε^2 - 1) */
            float g_logsig = sumc * (eps * eps - 1.0f);
            float delta_logsig = pg->p.eta_sigma * g_logsig;
            /* Stable multiplicative update for positivity */
            pg->sigma[i] *= expf(delta_logsig);
        }

        clamp_sigma_vec(pg);
        clamp_mu_to_bounds(pg);

        pg->iters++;
        pg->phase = 0; /* ready for a new pair */
        return;
    }

    /* If phase==0 and tell() is called, it's a user error; ignore gracefully. */
}

float pgpe_step(pgpe_t *pg, pgpe_objective_fn fn, void *userdata) {
    if (!pg || !fn) return 0.0f;

    const float *x = pgpe_ask(pg);
    float f = fn(x, pg->n, userdata);
    pgpe_tell(pg, f);

    x = pgpe_ask(pg);
    f = fn(x, pg->n, userdata);
    pgpe_tell(pg, f);

    return to_reward(pg->p.mode, f);
}

