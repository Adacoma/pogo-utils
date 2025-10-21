/**
 * @file pgpe.c
 * @brief PGPE implementation (malloc‑free, C11) for online optimization on float arrays.
 */
#include "pgpe.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>

/* ===== Helpers =========================================================== */
static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline float reward_from_f(pgpe_mode_t mode, float f) {
    /* For minimize: reward = -f. For maximize: reward = +f. */
    return (mode == PGPE_MINIMIZE) ? (-f) : (f);
}

static inline void project_into_bounds(float *restrict x, int n,
                                       const float *restrict lo,
                                       const float *restrict hi) {
    if (!lo && !hi) return;
    for (int i = 0; i < n; ++i) {
        if (lo) x[i] = (x[i] < lo[i]) ? lo[i] : x[i];
        if (hi) x[i] = (x[i] > hi[i]) ? hi[i] : x[i];
    }
}

static void pgpe_default_params(pgpe_params_t *p) {
    p->mode = PGPE_MINIMIZE;
    p->lr_mu = 0.1f;         /* tune to problem scale */
    p->lr_sigma = 0.05f;     /* multiplicative log-\sigma step */
    p->sigma_min = 1e-4f;
    p->sigma_max = 1.0f;
    p->samples_per_tick = 8; /* cost: this many f-evals per tick (x2 if antithetic) */
    p->antithetic = true;
    p->baseline_beta = 0.9f; /* EWMA; set 0 to disable */
    p->project_after_update = true;
}

/* Standard Normal via Box–Muller using rand(); adequate for embedded/demo. */
static inline float randu01(void) { return (float)rand() / (float)RAND_MAX; }
static float randn(void) {
    float u1 = randu01(); if (u1 < 1e-7f) u1 = 1e-7f;
    const float u2 = randu01();
    const float r = sqrtf(-2.0f * logf(u1));
    const float z = r * cosf(2.0f * (float)M_PI * u2);
    return z;
}

/* ===== API =============================================================== */
void pgpe_init(pgpe_t *pgpe, int n,
               float *restrict mu,
               float *restrict sigma,
               float *restrict theta,
               float *restrict eps,
               const float *restrict lo, const float *restrict hi,
               pgpe_objective_fn fn, void *user,
               const pgpe_params_t *params) {
    if (!pgpe || !mu || !sigma || !theta || !eps || !fn || n <= 0) return;

    memset(pgpe, 0, sizeof(*pgpe));
    pgpe->n   = n;
    pgpe->mu  = mu;
    pgpe->sigma = sigma;
    pgpe->theta = theta;
    pgpe->eps   = eps;
    pgpe->lo  = lo;
    pgpe->hi  = hi;
    pgpe->fn  = fn;
    pgpe->user= user;

    if (params) {
        pgpe->p = *params;
        if (pgpe->p.samples_per_tick < 1) pgpe->p.samples_per_tick = 1;
        if (pgpe->p.sigma_min <= 0.0f) pgpe->p.sigma_min = 1e-6f;
    } else {
        pgpe_default_params(&pgpe->p);
    }

    /* Clamp initial mu within bounds; clamp sigma to [min,max]. */
    project_into_bounds(pgpe->mu, pgpe->n, pgpe->lo, pgpe->hi);
    for (int i = 0; i < pgpe->n; ++i)
        pgpe->sigma[i] = clampf(pgpe->sigma[i], pgpe->p.sigma_min, pgpe->p.sigma_max);

    /* Monitor f(mu) */
    pgpe->f_curr = pgpe->fn(pgpe->mu, pgpe->n, pgpe->user);
    pgpe->f_best = pgpe->f_curr;
    pgpe->baseline = 0.0f;
    pgpe->k = 0;
}

float pgpe_step(pgpe_t *pgpe) {
    if (!pgpe || !pgpe->fn) return 0.0f;

    /* Accumulate gradients over the mini-batch, then update once. */
    float *mu = pgpe->mu;
    float *sigma = pgpe->sigma;
    const int n = pgpe->n;

    const float inv_batch = 1.0f / (float)(pgpe->p.samples_per_tick * (pgpe->p.antithetic ? 2 : 1));

    /* Zero accumulators */
    float g_mu_acc[n];
    float g_logs_acc[n];
    for (int i = 0; i < n; ++i) { g_mu_acc[i] = 0.0f; g_logs_acc[i] = 0.0f; }

    for (int sample = 0; sample < pgpe->p.samples_per_tick; ++sample) {
        /* Draw eps ~ N(0,I) */
        for (int i = 0; i < pgpe->n; ++i) pgpe->eps[i] = randn();

        /* Evaluate candidate theta = mu + sigma * eps */
        for (int i = 0; i < pgpe->n; ++i)
            pgpe->theta[i] = pgpe->mu[i] + pgpe->sigma[i] * pgpe->eps[i];
        project_into_bounds(pgpe->theta, pgpe->n, pgpe->lo, pgpe->hi);
        float f_pos = pgpe->fn(pgpe->theta, pgpe->n, pgpe->user);
        float R_pos = reward_from_f(pgpe->p.mode, f_pos);

        float f_neg = 0.0f, R_neg = 0.0f;
        if (pgpe->p.antithetic) {
            for (int i = 0; i < n; ++i)
                pgpe->theta[i] = mu[i] - sigma[i] * pgpe->eps[i];
            project_into_bounds(pgpe->theta, n, pgpe->lo, pgpe->hi);
            f_neg = pgpe->fn(pgpe->theta, n, pgpe->user);
            R_neg = reward_from_f(pgpe->p.mode, f_neg);
        }

        /* Update EWMA baseline with the average reward of this pair (or single). */
        const float R_avg = pgpe->p.antithetic ? 0.5f*(R_pos + R_neg) : R_pos;
        if (pgpe->p.baseline_beta > 0.0f && pgpe->p.baseline_beta < 1.0f) {
            pgpe->baseline = pgpe->p.baseline_beta * pgpe->baseline + (1.0f - pgpe->p.baseline_beta) * R_avg;
        }

        /* Compute advantages and apply updates. */
        const float A_pos = (pgpe->p.baseline_beta > 0.0f) ? (R_pos - pgpe->baseline) : R_pos;
        const float A_neg = (pgpe->p.antithetic) ? ((pgpe->p.baseline_beta > 0.0f) ? (R_neg - pgpe->baseline) : R_neg) : 0.0f;

        for (int i = 0; i < n; ++i) {
            const float inv_sigma = 1.0f / sigma[i];
            float g_mu   = A_pos * pgpe->eps[i] * inv_sigma;
            float g_logs = A_pos * (pgpe->eps[i]*pgpe->eps[i] - 1.0f);
            if (pgpe->p.antithetic) {
                g_mu   += A_neg * (-pgpe->eps[i]) * inv_sigma;
                g_logs += A_neg * (pgpe->eps[i]*pgpe->eps[i] - 1.0f);
                g_mu   *= 0.5f; g_logs *= 0.5f;
            }
            g_mu_acc[i]   += g_mu;
            g_logs_acc[i] += g_logs;
        }

        if (pgpe->p.project_after_update) {
            project_into_bounds(pgpe->mu, pgpe->n, pgpe->lo, pgpe->hi);
        }

        pgpe->k++;
    }

    /* Apply averaged update once per tick (stability). */
    for (int i = 0; i < n; ++i) {
        const float gmu   = g_mu_acc[i]   * inv_batch;
        const float glogs = g_logs_acc[i] * inv_batch;
        mu[i]    += pgpe->p.lr_mu * gmu;
        /* multiplicative sigma update with small cap to avoid bursts */
        float m = pgpe->p.lr_sigma * glogs;
        if (m > 0.25f) m = 0.25f; else if (m < -0.25f) m = -0.25f;
        const float new_sigma = sigma[i] * expf(m);
        sigma[i] = clampf(new_sigma, pgpe->p.sigma_min, pgpe->p.sigma_max);
    }

    if (pgpe->p.project_after_update) {
        project_into_bounds(mu, n, pgpe->lo, pgpe->hi);
    }

    /* Monitor f(mu) once per tick */
    pgpe->f_curr = pgpe->fn(pgpe->mu, pgpe->n, pgpe->user);
    if ((pgpe->p.mode == PGPE_MINIMIZE && pgpe->f_curr < pgpe->f_best) ||
        (pgpe->p.mode == PGPE_MAXIMIZE && pgpe->f_curr > pgpe->f_best)) {
        pgpe->f_best = pgpe->f_curr;
    }

    return pgpe->f_curr;
}

