/**
 * @file spsa.c
 * @brief SPSA implementation (malloc‑free, C11) for online optimization on float arrays.
 */
#include "spsa.h"
#include <stdlib.h>
#include <string.h>

/* ===== Helpers =========================================================== */

static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline float sign_for_mode(spsa_mode_t mode) {
    /* For minimization we subtract (+1), for maximization we add (−1). */
    return (mode == SPSA_MINIMIZE) ? 1.0f : -1.0f;
}

static inline float pow_fast(float base, float exp) {
    /* Tiny wrapper for clarity; defer to libm powf if needed in the future. */
    extern float powf(float, float);
    return powf(base, exp);
}

static inline int rademacher(void) {
    /* Return +1 or −1 with equal probability using rand(). */
    return (rand() & 1) ? +1 : -1;
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

static void spsa_default_params(spsa_params_t *p) {
    p->mode = SPSA_MINIMIZE;
    p->a = 0.1f;           /* Step-size scale; tune to problem scale. */
    p->c = 0.1f;           /* Perturbation scale. */
    p->alpha = 0.602f;
    p->gamma = 0.101f;
    p->A = 0.0f;           /* Often ≈0.1 * expected iters; 0 is fine online. */
    p->evals_per_tick = 1;
    p->project_after_update = true;
}

/* ===== API =============================================================== */

void spsa_init(spsa_t *spsa, int n,
               float *restrict x,
               float *restrict delta,
               float *restrict x_plus,
               float *restrict x_minus,
               const float *restrict lo, const float *restrict hi,
               spsa_objective_fn fn, void *user,
               const spsa_params_t *params) {
    if (!spsa || !x || !delta || !x_plus || !x_minus || !fn || n <= 0) return;

    memset(spsa, 0, sizeof(*spsa));
    spsa->n = n;
    spsa->x = x;
    spsa->delta = delta;
    spsa->x_plus = x_plus;
    spsa->x_minus = x_minus;
    spsa->lo = lo;
    spsa->hi = hi;
    spsa->fn = fn;
    spsa->user = user;

    if (params) {
        spsa->p = *params;
        if (spsa->p.evals_per_tick < 1) spsa->p.evals_per_tick = 1;
    } else {
        spsa_default_params(&spsa->p);
    }

    /* Initial projection & evaluation */
    project_into_bounds(spsa->x, spsa->n, spsa->lo, spsa->hi);
    spsa->f_curr = spsa->fn(spsa->x, spsa->n, spsa->user);
    spsa->f_best = spsa->f_curr;
    spsa->k = 0;
}

float spsa_step(spsa_t *spsa) {
    if (!spsa || !spsa->fn) return 0.0f;

    const float s = sign_for_mode(spsa->p.mode);

    for (int it = 0; it < spsa->p.evals_per_tick; ++it) {
        /* Gains a_k and c_k */
        const float k1 = (float)spsa->k + 1.0f;
        const float ak = spsa->p.a / pow_fast(k1 + spsa->p.A, spsa->p.alpha);
        const float ck = spsa->p.c / pow_fast(k1,           spsa->p.gamma);

        /* Draw Δ ∈ {−1,+1}^n and build x± */
        for (int i = 0; i < spsa->n; ++i) {
            const int di = rademacher();
            spsa->delta[i] = (float)di;
            spsa->x_plus[i]  = spsa->x[i] + ck * (float)di;
            spsa->x_minus[i] = spsa->x[i] - ck * (float)di;
        }
        project_into_bounds(spsa->x_plus,  spsa->n, spsa->lo, spsa->hi);
        project_into_bounds(spsa->x_minus, spsa->n, spsa->lo, spsa->hi);

        /* Two measurements of the objective */
        const float f_plus  = spsa->fn(spsa->x_plus,  spsa->n, spsa->user);
        const float f_minus = spsa->fn(spsa->x_minus, spsa->n, spsa->user);

        /* Gradient estimate and update */
        const float scale = (f_plus - f_minus) / (2.0f * ck);
        for (int i = 0; i < spsa->n; ++i) {
            const float gi = scale / spsa->delta[i]; /* since Δ_i ∈ {±1}, division is just ±scale */
            spsa->x[i] -= s * ak * gi;
        }
        if (spsa->p.project_after_update) {
            project_into_bounds(spsa->x, spsa->n, spsa->lo, spsa->hi);
        }

        /* Evaluate at new x_k for monitoring (optional but handy online) */
        spsa->f_curr = spsa->fn(spsa->x, spsa->n, spsa->user);
        if ((spsa->p.mode == SPSA_MINIMIZE && spsa->f_curr < spsa->f_best) ||
            (spsa->p.mode == SPSA_MAXIMIZE && spsa->f_curr > spsa->f_best)) {
            spsa->f_best = spsa->f_curr;
        }

        spsa->k++;
    }

    return spsa->f_curr;
}

