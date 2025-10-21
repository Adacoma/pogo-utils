/**
 * @file spsa.c
 * @brief Ask–tell SPSA for float arrays (no malloc). Uses separate Δ buffer.
 */
#include "spsa.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>

static inline float powfk(float x, float p) { return powf(x, p); }

static inline void compute_gains(spsa_t *s) {
    float t = (float)(s->k + 1);
    s->ck = s->p.c / powfk(t, s->p.gamma);
    s->ak = s->p.a / powfk(s->p.A + t, s->p.alpha);
}

static inline int rademacher(void) {
    return (rand() & 1) ? +1 : -1;
}

static void draw_delta(int n, int *restrict d) {
    for (int i = 0; i < n; ++i) d[i] = rademacher();
}

void spsa_init(spsa_t *s, int n,
               float *restrict x, float *restrict x_work, int *restrict delta,
               const float *restrict lo, const float *restrict hi,
               const spsa_params_t *P, spsa_mode_t mode) {
    if (!s || !x || !x_work || !delta || n <= 0) return;
    memset(s, 0, sizeof(*s));
    s->n = n;
    s->x = x;
    s->x_work = x_work;
    s->delta = delta;
    s->lo = lo;
    s->hi = hi;
    s->mode = mode;
    s->p = P ? *P : spsa_default_params();
    s->k = 0;
    s->phase = 0;
    s->have_f_plus = 0;
    s->have_probe = 0;
    compute_gains(s);
}

const float *spsa_ask(spsa_t *s) {
    if (!s) return NULL;
    if (s->have_probe) return NULL;

    if (s->phase == 0) {
        compute_gains(s);
        draw_delta(s->n, s->delta);
        for (int i = 0; i < s->n; ++i) {
            s->x_work[i] = s->x[i] + s->ck * (float)s->delta[i];
        }
    } else {
        for (int i = 0; i < s->n; ++i) {
            s->x_work[i] = s->x[i] - s->ck * (float)s->delta[i];
        }
    }
    s->have_probe = 1;
    return s->x_work;
}

uint32_t spsa_tell(spsa_t *s, float f_probe) {
    if (!s || !s->have_probe) return s ? s->k : 0u;

    if (s->phase == 0) {
        s->f_plus = f_probe;
        s->have_f_plus = 1;
        s->phase = 1;
        s->have_probe = 0;
        return s->k;
    }

    /* phase==1: have f_plus and f_minus, do update */
    float f_minus = f_probe;

    for (int i = 0; i < s->n; ++i) {
        float denom = 2.0f * s->ck * (float)s->delta[i];
        float g_i = (s->f_plus - f_minus) / denom;
        if (s->p.g_clip > 0.0f) {
            if (g_i >  s->p.g_clip) g_i =  s->p.g_clip;
            if (g_i < -s->p.g_clip) g_i = -s->p.g_clip;
        }
        float step = s->ak * g_i;
        if (s->mode == SPSA_MINIMIZE) s->x[i] -= step;
        else                          s->x[i] += step;
    }

    if (s->lo || s->hi) {
        for (int i = 0; i < s->n; ++i) {
            float xi = s->x[i];
            if (s->lo && xi < s->lo[i]) xi = s->lo[i];
            if (s->hi && xi > s->hi[i]) xi = s->hi[i];
            s->x[i] = xi;
        }
    }

    s->k++;
    s->phase = 0;
    s->have_f_plus = 0;
    s->have_probe = 0;
    return s->k;
}

uint32_t spsa_step(spsa_t *s, spsa_objective_fn fn, void *userdata) {
    if (!s || !fn) return s ? s->k : 0u;

    const float *xp = spsa_ask(s);
    if (!xp) return s->k;
    float fp = fn(xp, s->n, userdata);
    (void)spsa_tell(s, fp);

    const float *xm = spsa_ask(s);
    if (!xm) return s->k;
    float fm = fn(xm, s->n, userdata);
    return spsa_tell(s, fm);
}

