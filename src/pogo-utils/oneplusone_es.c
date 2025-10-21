/**
 * @file oneplusone_es.c
 * @brief Implementation of ask–tell (1+1)-ES over float arrays.
 */
#include "oneplusone_es.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

/* ===== Helpers =========================================================== */
static inline float clampf(float x, float lo, float hi) {
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

static inline int better(es1p1_mode_t mode, float f_try, float f_best) {
    return (mode == ES1P1_MINIMIZE) ? (f_try < f_best) : (f_try > f_best);
}

/* Box–Muller with spare */
static float randn01(es1p1_t *es) {
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

static float default_c_sigma(int n) {
    if (n < 1) n = 1;
    return 0.6f / sqrtf((float)n);
}

/* ===== API =============================================================== */
void es1p1_init(es1p1_t *es, int n,
                float *restrict x, float *restrict x_try,
                const float *restrict lo, const float *restrict hi,
                const es1p1_params_t *params) {
    if (!es || !x || !x_try || n <= 0) return;

    memset(es, 0, sizeof(*es));
    es->n   = n;
    es->x   = x;
    es->x_try = x_try;
    es->lo  = lo;
    es->hi  = hi;

    /* Defaults */
    es->p.mode       = ES1P1_MINIMIZE;
    es->p.sigma0     = 0.1f;
    es->p.sigma_min  = 1e-6f;
    es->p.sigma_max  = 1.0f;
    es->p.s_target   = 0.2f;
    es->p.s_alpha    = 0.2f;
    es->p.c_sigma    = default_c_sigma(n);

    if (params) {
        es->p = *params;
        if (es->p.c_sigma <= 0.0f) es->p.c_sigma = default_c_sigma(n);
    }

    es->sigma = es->p.sigma0;
    es->s_ewma = es->p.s_target; /* start unbiased */
    es->have_spare = 0;
    es->spare = 0.0f;
    es->iter = 0;
    es->have_initial = 0;
    es->have_candidate = 0;
}

void es1p1_tell_initial(es1p1_t *es, float f0) {
    if (!es) return;
    es->best_f = f0;
    es->have_initial = 1;
}

const float *es1p1_ask(es1p1_t *es) {
    if (!es || !es->have_initial) return NULL;

    for (int i = 0; i < es->n; ++i) {
        float z = randn01(es);
        float xi = es->x[i] + es->sigma * z;
        if (es->lo && es->hi) {
            xi = clampf(xi, es->lo[i], es->hi[i]);
        } else if (es->lo) {
            if (xi < es->lo[i]) xi = es->lo[i];
        } else if (es->hi) {
            if (xi > es->hi[i]) xi = es->hi[i];
        }
        es->x_try[i] = xi;
    }
    es->have_candidate = 1;
    return es->x_try;
}

float es1p1_tell(es1p1_t *es, float f_try) {
    if (!es || !es->have_initial || !es->have_candidate) return es ? es->best_f : 0.0f;

    int success = 0;
    if (better(es->p.mode, f_try, es->best_f)) {
        memcpy(es->x, es->x_try, (size_t)es->n * sizeof(float));
        es->best_f = f_try;
        success = 1;
    }

    /* Update smoothed success and step-size */
    es->s_ewma = (1.0f - es->p.s_alpha) * es->s_ewma + es->p.s_alpha * (float)success;
    float factor = expf(es->p.c_sigma * (es->s_ewma - es->p.s_target));
    es->sigma *= factor;
    if (es->sigma < es->p.sigma_min) es->sigma = es->p.sigma_min;
    if (es->sigma > es->p.sigma_max) es->sigma = es->p.sigma_max;

    es->iter++;
    es->have_candidate = 0;
    return es->best_f;
}

float es1p1_step(es1p1_t *es, es1p1_objective_fn fn, void *userdata) {
    if (!es || !fn || !es->have_initial) return es ? es->best_f : 0.0f;
    const float *x_try = es1p1_ask(es);
    if (!x_try) return es->best_f;
    float f_try = fn(x_try, es->n, userdata);
    return es1p1_tell(es, f_try);
}


