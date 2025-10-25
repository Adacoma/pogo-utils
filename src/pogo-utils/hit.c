/**
 * @file hit.c
 * @brief Implementation of Horizontal Information Transfer (HIT) optimizer.
 */
#include "hit.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

/* ===== Helpers =========================================================== */
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
static inline int better(hit_mode_t mode, float f_try, float f_best){
    return (mode == HIT_MINIMIZE) ? (f_try < f_best) : (f_try > f_best);
}
/* Box–Muller with spare (state in h) */
static float randn01(hit_t *h){
    if (h->have_spare){ h->have_spare=0; return h->spare; }
    float u1=0.f, u2=0.f;
    do { u1 = (float)rand() / (float)RAND_MAX; } while (u1 <= 1e-7f);
    u2 = (float)rand() / (float)RAND_MAX;
    float r = sqrtf(-2.0f * logf(u1));
    float th = 2.0f * (float)M_PI * u2;
    h->spare = r * sinf(th); h->have_spare = 1;
    return r * cosf(th);
}
static inline float sigmoid(float z){ return 1.f / (1.f + expf(-z)); }

/* Row pointer for repo_X */
static inline float *repo_row(hit_t *h, uint16_t i){ return h->repo_X + (size_t)i * (size_t)h->n; }
static inline const float *repo_row_ro(const hit_t *h, uint16_t i){ return h->repo_X + (size_t)i * (size_t)h->n; }

/* Duplicate if all dims within dup_eps in normalized space. */
static int is_duplicate(const hit_t *h, const float *a, const float *b){
    float eps = h->dup_eps;
    for (int d=0; d<h->n; ++d){
        float lo = h->lo ? h->lo[d] : 0.f;
        float hi = h->hi ? h->hi[d] : 1.f;
        float an = norm01(a[d], lo, hi);
        float bn = norm01(b[d], lo, hi);
        if (fabsf(an - bn) > eps) return 0;
    }
    return 1;
}

/* Insert-or-improve (duplicate-aware). If at capacity, evict the *oldest* (FIFO). */
static void repo_upsert_fifo(hit_t *h, const float *x, float f, uint32_t epoch, uint16_t from_id){
    /* Duplicate: keep best according to direction */
    for (uint16_t i=0; i<h->rsize; ++i){
        const float *xi = repo_row_ro(h, i);
        if (is_duplicate(h, xi, x)){
            int keep_new = better(h->mode, f, h->repo_F[i]);
            if (keep_new){
                memcpy(repo_row(h,i), x, (size_t)h->n * sizeof(float));
                h->repo_F[i] = f; h->repo_epoch[i] = epoch; h->repo_from[i] = from_id;
            }
            return;
        }
    }
    if (h->rsize < h->capacity){
        uint16_t i = h->rsize++;
        memcpy(repo_row(h,i), x, (size_t)h->n * sizeof(float));
        h->repo_F[i] = f; h->repo_epoch[i] = epoch; h->repo_from[i] = from_id;
        return;
    }
    /* FIFO eviction: shift left and insert at end */
    memmove(h->repo_X, h->repo_X + (size_t)h->n, (size_t)(h->capacity - 1) * (size_t)h->n * sizeof(float));
    memmove(h->repo_F, h->repo_F + 1, (size_t)(h->capacity - 1) * sizeof(float));
    memmove(h->repo_epoch, h->repo_epoch + 1, (size_t)(h->capacity - 1) * sizeof(uint32_t));
    memmove(h->repo_from, h->repo_from + 1, (size_t)(h->capacity - 1) * sizeof(uint16_t));

    uint16_t i = h->capacity - 1;
    memcpy(repo_row(h,i), x, (size_t)h->n * sizeof(float));
    h->repo_F[i] = f; h->repo_epoch[i] = epoch; h->repo_from[i] = from_id;
}

/* Pick a donor index using acceptance-weighted sampling relative to our parent. */
static int pick_donor_idx(const hit_t *h){
    if (h->rsize == 0) return -1;
    /* Acceptance probability ~ sigmoid(beta * Δ), with a floor pmin.
       For MIN: Δ = (f_self - f_donor). For MAX: Δ = (f_donor - f_self). */
    float f_self = h->last_adv_f;
    float total = 0.0;
    static float w[256]; /* small stack buffer for typical capacities */
    const uint16_t cap = h->rsize;
    if (cap > 256) {
        /* Fallback: unweighted uniform if very large (unlikely on tiny robots) */
        return (int)((rand() % cap));
    }
    for (uint16_t i=0; i<cap; ++i){
        float fd = h->repo_F[i];
        float delta = (h->mode == HIT_MINIMIZE) ? (f_self - fd) : (fd - f_self);
        float paccept = h->accept_pmin + (1.f - h->accept_pmin) * sigmoid(h->accept_beta * delta);
        if (paccept < 1e-6f) paccept = 1e-6f;
        w[i] = paccept;
        total += paccept;
    }
    if (!(total > 0.0)) return (int)((rand() % cap));
    float r = ((float)rand() / (float)RAND_MAX) * total, acc = 0.0;
    for (uint16_t i=0; i<cap; ++i){ acc += w[i]; if (r <= acc) return (int)i; }
    return (int)(cap - 1);
}

/* Fresh random genome (uniform in bounds) */
static void random_genome(const hit_t *h, float *out){
    for (int d=0; d<h->n; ++d){
        float lo = h->lo ? h->lo[d] : 0.f;
        float hi = h->hi ? h->hi[d] : 1.f;
        float u = (float)rand() / (float)RAND_MAX;
        out[d] = lo + (hi - lo) * u;
    }
}

/* ===== API =============================================================== */
void hit_init(hit_t *h, int n,
              float *restrict x, float *restrict x_try,
              const float *restrict lo, const float *restrict hi,
              float *restrict repo_X, float *restrict repo_F,
              uint32_t *restrict repo_epoch, uint16_t *restrict repo_from,
              uint16_t capacity, const hit_params_t *params){
    if (!h || !x || !x_try || !repo_X || !repo_F || !repo_epoch || !repo_from || n<=0 || capacity==0) return;

    memset(h, 0, sizeof(*h));
    h->n = n;
    h->x = x; h->x_try = x_try; h->lo = lo; h->hi = hi;
    h->repo_X = repo_X; h->repo_F = repo_F; h->repo_epoch = repo_epoch; h->repo_from = repo_from;
    h->capacity = capacity; h->rsize = 0;

    /* Defaults aimed at HIT-like behaviour */
    h->mode = HIT_MINIMIZE;
    h->transfer_prob = 0.35f;
    h->random_donor_prob = 0.05f;
    h->accept_beta = 5.0f;
    h->accept_pmin = 0.02f;
    h->mut_gain = 0.4f;
    h->mut_clip = 1.0f;
    h->dup_eps = 1e-3f;

    if (params){
        h->mode = params->mode;
        h->transfer_prob = params->transfer_prob;
        h->random_donor_prob = params->random_donor_prob;
        h->accept_beta = params->accept_beta;
        h->accept_pmin = params->accept_pmin;
        h->mut_gain = params->mut_gain;
        h->mut_clip = params->mut_clip;
        h->dup_eps = params->dup_eps;
        h->capacity = (params->repo_capacity > 0 && params->repo_capacity <= capacity)
                      ? params->repo_capacity : capacity;
    }

    h->epoch_local = 0;
    h->last_adv_f = 0.f;
    h->have_spare = 0; h->spare = 0.f;
    h->have_initial = 0;
}

void hit_tell_initial(hit_t *h, float f0){
    if (!h) return;
    h->last_adv_f = f0;
    h->have_initial = 1;
    /* Seed repo with our own parent (so other robots may accept it too) */
    repo_upsert_fifo(h, h->x, f0, h->epoch_local, /*from*/0);
}

const float *hit_ask(hit_t *h, float loss_for_scale){
    if (!h || !h->have_initial) return NULL;

    /* Choose donor: sometimes fresh random, else acceptance-weighted from repo. */
    float u = (float)rand() / (float)RAND_MAX;
    int didx = -1;
    if (u >= h->random_donor_prob) didx = pick_donor_idx(h);

    float donor_local[64];
    const float *donor = NULL;
    if (didx >= 0){
        donor = repo_row_ro(h, (uint16_t)didx);
    } else {
        /* No donor yet: use a random vector as “cultural spark” */
        if (h->n <= (int)(sizeof(donor_local)/sizeof(donor_local[0]))){
            random_genome(h, donor_local);
            donor = donor_local;
        } else {
            random_genome(h, h->x_try);
            donor = h->x_try;
        }
    }

    /* Offspring = subset copy from donor + (1-transfer) keep-parent, then mutation */
    for (int d=0; d<h->n; ++d){
        float lo = h->lo ? h->lo[d] : 0.f;
        float hi = h->hi ? h->hi[d] : 1.f;

        float take = ((float)rand() / (float)RAND_MAX) < h->transfer_prob ? 1.f : 0.f;
        float base = take ? donor[d] : h->x[d];

        /* Mutate in normalized space */
        float xn = norm01(base, lo, hi);

        float L = loss_for_scale;
        if (!isfinite(L) || L < 0.f) L = 0.f;
        if (L > h->mut_clip) L = h->mut_clip;
        float amp = h->mut_gain * (L + 1e-12f);
        if (amp < 1e-4f) amp = 1e-4f;

        xn = clampf(xn + 0.25f * amp * randn01(h), 0.f, 1.f);
        h->x_try[d] = denorm01(xn, lo, hi);
    }

    return h->x_try;
}

float hit_tell(hit_t *h, float f_try){
    if (!h || !h->have_initial) return h ? h->last_adv_f : 0.f;

    int success = better(h->mode, f_try, h->last_adv_f);
    if (success){
        memcpy(h->x, h->x_try, (size_t)h->n * sizeof(float));
        h->last_adv_f = f_try;
        h->epoch_local += 1;
        /* Self-advert (so others may adopt) */
        repo_upsert_fifo(h, h->x, h->last_adv_f, h->epoch_local, /*from*/0);
    } else {
        /* Keep the offspring as a candidate observation */
        repo_upsert_fifo(h, h->x_try, f_try, h->epoch_local, /*from*/0);
    }
    return h->last_adv_f;
}

void hit_observe_remote(hit_t *h, uint16_t from_id, uint32_t epoch,
                        const float *x_remote, float f_adv){
    if (!h || !x_remote) return;

    /* Compute acceptance prob as in HIT; sample whether to *keep* this advert. */
    float f_self = h->last_adv_f;
    float delta = (h->mode == HIT_MINIMIZE) ? (f_self - f_adv) : (f_adv - f_self);
    float paccept = h->accept_pmin + (1.f - h->accept_pmin) * sigmoid(h->accept_beta * delta);

    float u = (float)rand() / (float)RAND_MAX;
    if (u <= paccept){
        /* Upsert into local repo; FIFO eviction preserves "recent cultural flow". */
        repo_upsert_fifo(h, x_remote, f_adv, epoch, from_id);
    }
}

