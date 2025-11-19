/**
 * @file hit.c
 * @brief Horizontal Information Transfer (HIT) implementation (CEC 2020).
 *
 * Main differences compared to the previous "strict HIT" implementation:
 *   - Sliding-window score: hit_tell() now takes an instantaneous reward
 *     (or cost), and HIT maintains a window of length T to compute the
 *     score f = G_t = sum_{k=0}^{T-1} r_{t-k}.
 *   - Maturation: communication is disabled until the window is full after
 *     initialisation or after each successful transfer. During maturation,
 *     hit_observe_remote() ignores all messages, which matches Algorithm 1.
 *   - Optional evolving α: when evolve_alpha==true, the transfer rate α is
 *     itself an evolvable parameter copied from better neighbours and
 *     mutated with Gaussian noise alpha_sigma.
 *
 * We keep a simple ask/tell/observe_remote structure so that the optimiser
 * can be used as a drop-in component on Pogobot/Pogosim.
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
    float th = 2.0f * 3.14159265358979323846f * u2;

    h->spare      = r * sinf(th);
    h->have_spare = 1;
    return r * cosf(th);
}

/* Reset maturation and sliding-window state */
static void hit_reset_maturation(hit_t *h){
    if (!h) return;
    h->step   = 0;
    h->f      = 0.0f;
    h->have_f = 0;
    if (h->reward_buf && h->T > 1){
        memset(h->reward_buf, 0, (size_t)h->T * sizeof(float));
    }
}

/* ===== Core adoption operator =========================================== */

/**
 * @brief Apply HIT transfer + mutation from remote to local genome.
 *
 * Assumes that f_remote is strictly better than current f (according to mode),
 * and that the agent is mature (window full).
 */
static void hit_adopt_from_remote(hit_t *h,
                                  const float *x_remote,
                                  float alpha_remote)
{
    if (!h || !x_remote || h->n <= 0) return;

    int n = h->n;

    /* Optionally copy and mutate α from the neighbour */
    if (h->evolve_alpha){
        float a = alpha_remote;
        if (!isfinite(a)) a = h->alpha;
        if (a < h->alpha_min) a = h->alpha_min;
        if (a > h->alpha_max) a = h->alpha_max;

        float as = h->alpha_sigma;
        if (isfinite(as) && as > 0.0f){
            a += as * randn01(h);
            if (a < h->alpha_min) a = h->alpha_min;
            if (a > h->alpha_max) a = h->alpha_max;
        }
        h->alpha = a;
    }

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
        /* Choose k indices without replacement using partial Fisher–Yates.
         */
        int indices[n];

        for (int d = 0; d < n; ++d){
            indices[d] = d;
        }
        for (int i = 0; i < k; ++i){
            int range = n - i;
            int j = i + (rand() % range);
            int tmp      = indices[i];
            indices[i]   = indices[j];
            indices[j]   = tmp;
        }
        for (int i = 0; i < k; ++i){
            int d = indices[i];
            dst[d] = x_remote[d];
        }
    }

    /* Mutation: additive Gaussian with stddev sigma on ALL coordinates. */
    float sigma = h->sigma;
    if (!isfinite(sigma) || sigma < 0.0f) sigma = 0.0f;

    /* Always loop once over all coordinates;
     * apply mutation only if sigma>0, but always clamp.
     */
    for (int d = 0; d < n; ++d){
        float v = dst[d];

        if (sigma > 0.0f){
            v += sigma * randn01(h);
        }

        if (h->lo && h->hi){
            float lo = h->lo[d];
            float hi = h->hi[d];
            if (isfinite(lo) && isfinite(hi) && hi > lo){
                v = clampf(v, lo, hi);
            }
        }
        dst[d] = v;
    }

    /* Commit dst to x, and keep x_buf in sync if it exists and is distinct. */
    if (dst != h->x){
        memcpy(h->x, dst, (size_t)n * sizeof(float));
    }
    if (h->x_buf && h->x_buf != h->x){
        memcpy(h->x_buf, h->x, (size_t)n * sizeof(float));
    }

    /* Successful transfer: increase epoch and reset maturation window. */
    h->epoch += 1;
    hit_reset_maturation(h);
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

    h->n    = n;
    h->x    = x;
    h->x_buf= x_buf;
    h->lo   = lo;
    h->hi   = hi;

    /* Reasonable HIT-like defaults. */
    h->mode        = HIT_MINIMIZE;
    h->alpha       = 0.3f;
    h->sigma       = 0.001f;
    h->T           = 5;
    h->evolve_alpha= false;
    h->alpha_sigma = 0.001f;
    h->alpha_min   = 0.0f;
    h->alpha_max   = 0.9f;

    if (params){
        h->mode        = params->mode;
        h->alpha       = params->alpha;
        h->sigma       = params->sigma;
        if (params->eval_T > 0){
            h->T = params->eval_T;
        }
        h->evolve_alpha= params->evolve_alpha;
        h->alpha_sigma = params->alpha_sigma;
        h->alpha_min   = params->alpha_min;
        h->alpha_max   = params->alpha_max;
    }

    if (h->T < 1) h->T = 1;

    h->reward_buf = NULL;
    if (h->T > 1){
        if (h->T > HIT_MAX_T){
            /* Clamp T to the maximum supported window length */
            h->T = HIT_MAX_T;
        }
        h->reward_buf = h->reward_buf_static;
        memset(h->reward_buf, 0, (size_t)h->T * sizeof(float));
    }

    h->f          = 0.0f;
    h->have_f     = 0;
    h->step       = 0;
    h->epoch      = 0;
    h->have_spare = 0;
    h->spare      = 0.0f;
}


void hit_tell_initial(hit_t *h, float f0){
    if (!h) return;
    /* Explicitly set the score and mark as mature. */
    h->f      = f0;
    h->have_f = 1;
    h->step   = (uint32_t)h->T;
}

const float *hit_ask(hit_t *h){
    if (!h) return NULL;
    /* In HIT, the genome to evaluate is always x. */
    return h->x;
}

void hit_tell(hit_t *h, float r){
    if (!h) return;

    h->step += 1;

    if (h->T <= 1 || !h->reward_buf){
        /* Degenerate case: no window, f is simply the latest reward. */
        h->f      = r;
        h->have_f = 1;
        return;
    }

    uint32_t idx = (h->step - 1u) % (uint32_t)h->T;
    if (h->step <= (uint32_t)h->T){
        /* Filling the window. */
        h->reward_buf[idx] = r;
        h->f += r;
        if (h->step == (uint32_t)h->T){
            h->have_f = 1; /* window is now full => mature */
        }
    } else {
        /* Sliding window: remove oldest, add newest. */
        float old = h->reward_buf[idx];
        h->reward_buf[idx] = r;
        h->f += r - old;
        h->have_f = 1;
    }
}

void hit_observe_remote(hit_t *h, uint16_t from_id, uint32_t epoch,
                        const float *x_remote, float f_remote,
                        float alpha_remote)
{
    (void)from_id; /* currently unused, kept for logging/debug. */
    (void)epoch;   /* not used in HIT core (could be used for staleness). */

    if (!h || !x_remote || h->n <= 0){
        return;
    }

    /* Maturation: ignore all messages until we have a full window. */
    if (!h->have_f){
        return;
    }

    /* Deterministic acceptance: adopt iff neighbour is strictly better. */
    float f_local = hit_get_f(h);  // local average
    if (better(h->mode, f_remote, f_local)){
        hit_adopt_from_remote(h, x_remote, alpha_remote);
    }
}

int hit_ready(const hit_t *h){
    return h && h->have_f;
}

uint32_t hit_iterations(const hit_t *h){
    return h ? h->epoch : 0u;
}

