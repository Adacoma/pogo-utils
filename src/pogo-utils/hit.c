/**
 * @file hit.c
 * @brief Strict Horizontal Information Transfer (HIT) implementation.
 *
 * This follows the core of the CEC 2020 HIT algorithm:
 *   - Deterministic acceptance: neighbour is used iff it is strictly
 *     better than the current genome (according to mode).
 *   - Horizontal transfer: copy exactly α·n coordinates from neighbour.
 *   - Mutation: additive Gaussian mutation (stddev σ) on all coordinates.
 *
 * The ask/tell interface is deliberately simple: ask() just returns the
 * current genome, and tell() just updates its fitness. All “evolution”
 * happens inside hit_observe_remote().
 */
#include "hit.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ===== Helpers =========================================================== */

static inline float clampf(float x, float lo, float hi){
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline int better(hit_mode_t mode, float f_candidate, float f_ref){
    return (mode == HIT_MINIMIZE)
           ? (f_candidate < f_ref)
           : (f_candidate > f_ref);
}

/* Box–Muller with spare (state stored in hit_t) */
static float randn01(hit_t *h){
    if (h->have_spare){
        h->have_spare = 0;
        return h->spare;
    }

    float u1 = 0.0f, u2 = 0.0f;
    do {
        u1 = (float)rand() / (float)RAND_MAX;
    } while (u1 <= 1e-7f);
    u2 = (float)rand() / (float)RAND_MAX;

    float r  = sqrtf(-2.0f * logf(u1));
    float th = 2.0f * (float)M_PI * u2;

    h->spare      = r * sinf(th);
    h->have_spare = 1;
    return r * cosf(th);
}

/* ===== Core adoption operator =========================================== */

/**
 * @brief Apply strict HIT transfer + mutation from remote to local genome.
 *
 * Assumes that f_remote is strictly better than current f (according to mode).
 */
static void hit_adopt_from_remote(hit_t *h, const float *x_remote){
    if (!h || !x_remote || h->n <= 0) return;

    int n = h->n;

    /* Determine number of transferred coordinates: k = round(alpha * n). */
    float a = h->alpha;
    if (!isfinite(a) || a < 0.0f) a = 0.0f;
    if (a > 1.0f) a = 1.0f;

    int k = (int)floorf(a * (float)n + 0.5f);
    if (k < 1 && n > 0 && a > 0.0f) k = 1;   /* at least 1 if alpha>0 */
    if (k > n) k = n;

    /* If alpha == 0 or k == 0, we will only mutate our own genome. */
    int do_transfer = (k > 0);

    /* Work buffer: either x_buf, or x in place. */
    float *dst = h->x_buf ? h->x_buf : h->x;

    /* Start from current genome. */
    memcpy(dst, h->x, (size_t)n * sizeof(float));

    if (do_transfer){
        /* Choose k indices without replacement using partial Fisher–Yates. */
        int *indices = (int *)malloc((size_t)n * sizeof(int));
        if (!indices){
            /* Fallback: if malloc fails, just copy all coordinates. */
            for (int d = 0; d < n; ++d){
                dst[d] = x_remote[d];
            }
        } else {
            for (int d = 0; d < n; ++d){
                indices[d] = d;
            }
            for (int i = 0; i < k; ++i){
                int range = n - i;
                int j = i + (rand() % range);
                int tmp = indices[i];
                indices[i] = indices[j];
                indices[j] = tmp;
            }
            for (int i = 0; i < k; ++i){
                int d = indices[i];
                dst[d] = x_remote[d];
            }
            free(indices);
        }
    }

    /* Mutation: additive Gaussian with stddev sigma on ALL coordinates. */
    float sigma = h->sigma;
    if (!isfinite(sigma) || sigma < 0.0f) sigma = 0.0f;

    if (sigma > 0.0f){
        for (int d = 0; d < n; ++d){
            float v = dst[d] + sigma * randn01(h);

            if (h->lo && h->hi){
                float lo = h->lo[d];
                float hi = h->hi[d];
                if (isfinite(lo) && isfinite(hi) && hi > lo){
                    v = clampf(v, lo, hi);
                }
            }
            dst[d] = v;
        }
    }

    /* Commit dst to x, and keep x_buf in sync if it exists and is distinct. */
    if (dst != h->x){
        memcpy(h->x, dst, (size_t)n * sizeof(float));
    }
    if (h->x_buf && h->x_buf != h->x){
        memcpy(h->x_buf, h->x, (size_t)n * sizeof(float));
    }

    /* Mark that this genome must be re-evaluated. */
    h->have_f = 0;
    h->epoch += 1;
}

/* ===== Public API ======================================================== */

void hit_init(hit_t *h, int n,
              float *x, float *x_buf,
              const float *lo, const float *hi,
              const hit_params_t *params)
{
    if (!h || !x || n <= 0){
        return;
    }
    memset(h, 0, sizeof(*h));

    h->n   = n;
    h->x   = x;
    h->x_buf = x_buf;
    h->lo  = lo;
    h->hi  = hi;

    /* Defaults consistent with HIT behaviour. */
    h->mode  = HIT_MINIMIZE;
    h->alpha = 0.3f;
    h->sigma = 0.1f;

    if (params){
        h->mode  = params->mode;
        h->alpha = params->alpha;
        h->sigma = params->sigma;
    }

    h->f        = 0.0f;
    h->have_f   = 0;
    h->epoch    = 0;
    h->have_spare = 0;
    h->spare      = 0.0f;
}

void hit_tell_initial(hit_t *h, float f0){
    if (!h) return;
    h->f      = f0;
    h->have_f = 1;
}

const float *hit_ask(hit_t *h){
    if (!h) return NULL;
    /* In strict HIT, the genome to evaluate is always x. */
    return h->x;
}

void hit_tell(hit_t *h, float f){
    if (!h) return;
    h->f      = f;
    h->have_f = 1;
}

void hit_observe_remote(hit_t *h, uint16_t from_id, uint32_t epoch,
                        const float *x_remote, float f_remote)
{
    (void)from_id; /* currently unused, kept for possible logging/debug. */
    (void)epoch;   /* not used in strict HIT core.                      */

    if (!h || !x_remote || h->n <= 0){
        return;
    }

    /* If we have no fitness yet, we treat the neighbour as better by default. */
    if (!h->have_f){
        hit_adopt_from_remote(h, x_remote);
        return;
    }

    /* Deterministic acceptance: adopt iff neighbour is strictly better. */
    if (better(h->mode, f_remote, h->f)){
        hit_adopt_from_remote(h, x_remote);
    }
}

int hit_ready(const hit_t *h){
    return h && h->have_f;   /* or use a separate have_initial flag if you prefer */
}

uint32_t hit_iterations(const hit_t *h){
    return h ? h->epoch : 0u;
}

