/**
 * @file social_learning.c
 * @brief Implementation of an ask–tell mEDEA-style optimizer + bounded repo.
 */
#include "social_learning.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ===== Small helpers ===================================================== */
static inline float clampf(float x, float lo, float hi){ return (x<lo)?lo:((x>hi)?hi:x); }

static inline float norm01(float x, float lo, float hi){
    if (!isfinite(x) || !isfinite(lo) || !isfinite(hi) || hi<=lo) return 0.5f;
    float t = (x - lo) / (hi - lo);
    return (t<0.f)?0.f:((t>1.f)?1.f:t);
}
static inline float denorm01(float t, float lo, float hi){
    if (!isfinite(t) || !isfinite(lo) || !isfinite(hi) || hi<=lo) return lo;
    return lo + (hi - lo) * clampf(t, 0.f, 1.f);
}

/* Better comparator: for MIN we want f_try < f_best; for MAX f_try > f_best */
static inline int better(sl_mode_t mode, float f_try, float f_best){
    return (mode == SL_MINIMIZE) ? (f_try < f_best) : (f_try > f_best);
}

/* Fitness used by roulette:
 *  - For MIN: fitness = 1 / max(loss, eps)
 *  - For MAX: fitness = max(f, eps)   (assumes non-negative or rescaled)
 */
static inline float roulette_weight(sl_mode_t mode, float f){
    const float eps = 1e-9f;
    if (mode == SL_MINIMIZE){
        float L = f; if (L < eps) L = eps;
        return 1.0f / L;
    } else {
        float val = f; if (val < eps) val = eps;
        return val;
    }
}

/* Box–Muller with spare (state inside sl) */
static float randn01(sl_t *sl){
    if (sl->have_spare){ sl->have_spare=0; return sl->spare; }
    float u1=0.f, u2=0.f;
    do { u1 = (float)rand() / (float)RAND_MAX; } while (u1 <= 1e-7f);
    u2 = (float)rand() / (float)RAND_MAX;
    float r = sqrtf(-2.0f * logf(u1));
    float th = 2.0f * (float)M_PI * u2;
    sl->spare = r * sinf(th); sl->have_spare = 1;
    return r * cosf(th);
}

/* Row pointer for repo_X */
static inline float *repo_row(sl_t *sl, uint16_t i){ return sl->repo_X + (size_t)i * (size_t)sl->n; }
static inline const float *repo_row_ro(const sl_t *sl, uint16_t i){ return sl->repo_X + (size_t)i * (size_t)sl->n; }

/* Duplicate if all dims within dup_eps in normalized space. */
static int is_duplicate(const sl_t *sl, const float *a, const float *b){
    float eps = sl->dup_eps;
    for (int d=0; d<sl->n; ++d){
        float lo = sl->lo ? sl->lo[d] : 0.f;
        float hi = sl->hi ? sl->hi[d] : 1.f;
        float an = norm01(a[d], lo, hi);
        float bn = norm01(b[d], lo, hi);
        if (fabsf(an - bn) > eps) return 0;
    }
    return 1;
}

/* Insert-or-improve (duplicate-aware). If at capacity, evict worst. */
static void repo_upsert(sl_t *sl, const float *x, float f, uint32_t epoch, uint16_t from_id){
    /* Try duplicate first */
    for (uint16_t i=0; i<sl->rsize; ++i){
        const float *xi = repo_row_ro(sl, i);
        if (is_duplicate(sl, xi, x)){
            /* Keep the better of the two (i.e., smaller loss for MIN, larger for MAX) */
            int keep_new = better(sl->mode, f, sl->repo_F[i]);
            if (keep_new){
                memcpy(repo_row(sl,i), x, (size_t)sl->n * sizeof(float));
                sl->repo_F[i] = f; sl->repo_epoch[i] = epoch; sl->repo_from[i] = from_id;
            }
            return;
        }
    }

    if (sl->rsize < sl->capacity){
        uint16_t i = sl->rsize++;
        memcpy(repo_row(sl,i), x, (size_t)sl->n * sizeof(float));
        sl->repo_F[i] = f; sl->repo_epoch[i] = epoch; sl->repo_from[i] = from_id;
        return;
    }

    /* Evict the worst according to direction */
    uint16_t wi = 0;
    for (uint16_t i=1; i<sl->rsize; ++i){
        int new_is_worse = (sl->mode == SL_MINIMIZE) ? (sl->repo_F[i] > sl->repo_F[wi])
                                                     : (sl->repo_F[i] < sl->repo_F[wi]);
        if (new_is_worse) wi = i;
    }

    /* Replace worst only if newcomer is better */
    int newcomer_better_than_worst = better(sl->mode, f, sl->repo_F[wi]);
    if (newcomer_better_than_worst){
        memcpy(repo_row(sl,wi), x, (size_t)sl->n * sizeof(float));
        sl->repo_F[wi] = f; sl->repo_epoch[wi] = epoch; sl->repo_from[wi] = from_id;
    }
}

/* Roulette selection over repository entries.
 * Falls back to current parent or fresh random if empty or degenerate. */
static void roulette_pick(const sl_t *sl, float *out){
    if (sl->rsize == 0){
        memcpy(out, sl->x, (size_t)sl->n * sizeof(float));
        return;
    }
    float total = 0.0;
    for (uint16_t i=0; i<sl->rsize; ++i) total += (float)roulette_weight(sl->mode, sl->repo_F[i]);
    if (!(total > 0.0)){
        memcpy(out, sl->x, (size_t)sl->n * sizeof(float));
        return;
    }
    float rpick = ((float)rand() / (float)RAND_MAX) * total;
    float acc = 0.0;
    for (uint16_t i=0; i<sl->rsize; ++i){
        acc += (float)roulette_weight(sl->mode, sl->repo_F[i]);
        if (rpick <= acc){
            memcpy(out, repo_row_ro(sl,i), (size_t)sl->n * sizeof(float));
            return;
        }
    }
    memcpy(out, repo_row_ro(sl, sl->rsize-1), (size_t)sl->n * sizeof(float));
}

/* Fresh random genome in bounds (uniform in [lo,hi]) */
static void random_genome(const sl_t *sl, float *out){
    for (int d=0; d<sl->n; ++d){
        float lo = sl->lo ? sl->lo[d] : 0.f;
        float hi = sl->hi ? sl->hi[d] : 1.f;
        float u = (float)rand() / (float)RAND_MAX;
        out[d] = lo + (hi - lo) * u;
    }
}

/* ===== API ================================================================= */
void sl_init(sl_t *sl, int n,
             float *restrict x, float *restrict x_try,
             const float *restrict lo, const float *restrict hi,
             float *restrict repo_X, float *restrict repo_F,
             uint32_t *restrict repo_epoch, uint16_t *restrict repo_from,
             uint16_t capacity, const sl_params_t *params){
    if (!sl || !x || !x_try || !repo_X || !repo_F || !repo_epoch || !repo_from || n<=0 || capacity==0) return;

    memset(sl, 0, sizeof(*sl));
    sl->n = n;
    sl->x = x; sl->x_try = x_try; sl->lo = lo; sl->hi = hi;
    sl->repo_X = repo_X; sl->repo_F = repo_F; sl->repo_epoch = repo_epoch; sl->repo_from = repo_from;
    sl->capacity = capacity; sl->rsize = 0;

    /* Defaults */
    sl->mode = SL_MINIMIZE;
    sl->roulette_random_prob = 0.10f;
    sl->loss_mut_gain = 0.5f;
    sl->loss_mut_clip = 1.0f;
    sl->dup_eps = 1e-3f;

    if (params){
        sl->mode = params->mode;
        sl->roulette_random_prob = params->roulette_random_prob;
        sl->loss_mut_gain = params->loss_mut_gain;
        sl->loss_mut_clip = params->loss_mut_clip;
        sl->dup_eps = params->dup_eps;
        sl->capacity = (params->repo_capacity > 0 && params->repo_capacity <= capacity)
                       ? params->repo_capacity : capacity;
    }

    sl->epoch_local = 0;
    sl->last_adv_f = 0.f;
    sl->have_spare = 0; sl->spare = 0.f;
    sl->have_initial = 0;
}

void sl_tell_initial(sl_t *sl, float f0){
    if (!sl) return;
    sl->last_adv_f = f0;
    sl->have_initial = 1;

    /* Seed repository with our initial parent */
    repo_upsert(sl, sl->x, f0, sl->epoch_local, /*from_id*/0);
}

/* Ask: select donor (roulette vs random), mutate in normalized space. */
const float *sl_ask(sl_t *sl, float loss_for_scale){
    if (!sl || !sl->have_initial) return NULL;

    /* Maybe pick a fresh random donor */
    float u = (float)rand() / (float)RAND_MAX;
    if (u < sl->roulette_random_prob){
        random_genome(sl, sl->x_try);
    } else {
        float donor[64]; /* Local small buffer for source; use x_try as workspace after copy */
        if (sl->n <= (int)(sizeof(donor)/sizeof(donor[0]))){
            roulette_pick(sl, donor);
            memcpy(sl->x_try, donor, (size_t)sl->n * sizeof(float));
        } else {
            /* If n is large, pick directly into x_try then mutate in-place */
            roulette_pick(sl, sl->x_try);
        }
    }

    /* Mutation amplitude from loss scale */
    float L = loss_for_scale;
    if (!isfinite(L) || L < 0.f) L = 0.f;
    if (L > sl->loss_mut_clip) L = sl->loss_mut_clip;
    float amp = sl->loss_mut_gain * (L + 1e-12f);
    if (amp < 1e-4f) amp = 1e-4f;

    /* Mutate in normalized space then denormalize */
    for (int d=0; d<sl->n; ++d){
        float lo = sl->lo ? sl->lo[d] : 0.f;
        float hi = sl->hi ? sl->hi[d] : 1.f;
        float xn = norm01(sl->x_try[d], lo, hi);
        xn = clampf(xn + 0.25f * amp * randn01(sl), 0.f, 1.f);
        sl->x_try[d] = denorm01(xn, lo, hi);
    }

    return sl->x_try;
}

/* Tell: evaluate replacement, update repo, bump epoch if improved parent */
float sl_tell(sl_t *sl, float f_try){
    if (!sl || !sl->have_initial) return sl ? sl->last_adv_f : 0.f;

    int success = better(sl->mode, f_try, sl->last_adv_f);
    if (success){
        /* Replace parent by x_try */
        memcpy(sl->x, sl->x_try, (size_t)sl->n * sizeof(float));
        sl->last_adv_f = f_try;
        sl->epoch_local += 1;
        /* Store the improved parent into repo as our own advert */
        repo_upsert(sl, sl->x, sl->last_adv_f, sl->epoch_local, /*from*/0);
    } else {
        /* Still keep the offspring observation in the repository */
        repo_upsert(sl, sl->x_try, f_try, sl->epoch_local, /*from*/0);
    }
    return sl->last_adv_f;
}

void sl_observe_remote(sl_t *sl, uint16_t from_id, uint32_t epoch,
                       const float *x_remote, float f_adv){
    if (!sl || !x_remote) return;
    repo_upsert(sl, x_remote, f_adv, epoch, from_id);
}

