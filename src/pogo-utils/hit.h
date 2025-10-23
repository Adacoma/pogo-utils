/**
 * @file hit.h
 * @brief Horizontal Information Transfer (HIT) optimizer in strict ask–tell form.
 *
 * @details
 * HIT performs social learning by *copying a subset of parameters* from
 * neighbors with a probability that increases with the neighbor’s advertised
 * performance; copied parameters are then perturbed (mutation). This follows
 * the Horizontal Information Transfer principle described for social learning
 * in swarm robotics. See: Bredeche & Fontbonne, *Social learning in swarm
 * robotics*, Phil. Trans. B, 2022. DOI: 10.1098/rstb.2020.0309.
 *
 * Design mirrors social_learning.{h,c}:
 *   - Strict ask–tell (no objective callback inside).
 *   - Caller-provided parent x and offspring x_try buffers.
 *   - Optional bounds (lo/hi) with normalized-space mutation.
 *   - Small bounded repository of recent neighbor adverts (for donor picks).
 *   - Duplicate-aware upsert; no malloc when buffers are provided.
 *
 * Typical usage (MINIMIZE):
 *   hit_init(...); hit_tell_initial(&hit, f0);
 *   loop {
 *     const float *x_try = hit_ask(&hit, loss_for_scale);
 *     float f_try = objective(x_try);
 *     (void)hit_tell(&hit, f_try);
 *   }
 *   // On each received advert:
 *   hit_observe_remote(&hit, sender_id, epoch, x_remote, f_adv_remote);
 */
#ifndef POGO_UTILS_HIT_H
#define POGO_UTILS_HIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/** Optimization direction. */
typedef enum { HIT_MINIMIZE = 0, HIT_MAXIMIZE = 1 } hit_mode_t;

/** Tunables for HIT. */
typedef struct {
    hit_mode_t mode;           /**< MIN or MAX. */
    float transfer_prob;       /**< Prob. to copy each dim from donor (typ. 0.2..0.5). */
    float random_donor_prob;   /**< Chance to ignore repo and use fresh random donor. */
    float accept_beta;         /**< Sigmoid slope for neighbor acceptance (2..10). */
    float accept_pmin;         /**< Floor acceptance prob. for worse neighbors (0..0.2). */
    float mut_gain;            /**< Mutation amplitude gain (normalized space). */
    float mut_clip;            /**< Clip for loss_for_scale driver. */
    float dup_eps;             /**< Duplicate tolerance in normalized space. */
    uint16_t repo_capacity;    /**< Bounded repository capacity (<= provided buffers). */
} hit_params_t;

/** HIT handle. */
typedef struct {
    /* Problem */
    int n;
    hit_mode_t mode;

    /* Parent & offspring (owned by caller) */
    float *restrict x;
    float *restrict x_try;
    const float *restrict lo;
    const float *restrict hi;

    /* Repository of neighbor adverts (owned by caller) */
    float    *restrict repo_X;     /**< capacity * n floats (row-major). */
    float    *restrict repo_F;     /**< capacity floats (advertised perf). */
    uint32_t *restrict repo_epoch; /**< capacity uint32. */
    uint16_t *restrict repo_from;  /**< capacity uint16. */
    uint16_t capacity;
    uint16_t rsize;

    /* Parameters */
    float transfer_prob;
    float random_donor_prob;
    float accept_beta;
    float accept_pmin;
    float mut_gain;
    float mut_clip;
    float dup_eps;

    /* Book-keeping */
    uint32_t epoch_local;      /**< Increments on accepted parent updates. */
    float    last_adv_f;       /**< Last advertised fitness (parent). */
    int      have_initial;

    /* RNG cache */
    int   have_spare;
    float spare;
} hit_t;

/* ===== API ===== */

void hit_init(hit_t *h, int n,
              float *restrict x, float *restrict x_try,
              const float *restrict lo, const float *restrict hi,
              float *restrict repo_X, float *restrict repo_F,
              uint32_t *restrict repo_epoch, uint16_t *restrict repo_from,
              uint16_t capacity, const hit_params_t *params);

void hit_tell_initial(hit_t *h, float f0);

/**
 * @brief Generate an offspring by copying a subset from a donor and mutating.
 *
 * @param h               Handle.
 * @param loss_for_scale  Driver for mutation amplitude (see mut_clip, mut_gain).
 * @return Pointer to x_try (length n), or NULL if called before tell_initial().
 */
const float *hit_ask(hit_t *h, float loss_for_scale);

/**
 * @brief Tell the candidate’s fitness and perform parent replacement if better.
 *
 * @return Current parent’s fitness after potential replacement.
 */
float hit_tell(hit_t *h, float f_try);

/**
 * @brief Ingest a neighbor advert (duplicate-aware upsert). Acceptance
 *        probability follows a sigmoid of performance difference as in HIT.
 */
void hit_observe_remote(hit_t *h, uint16_t from_id, uint32_t epoch,
                        const float *x_remote, float f_adv);

/* Getters */
static inline const float *hit_get_x(const hit_t *h){ return h->x; }
static inline uint32_t     hit_get_epoch(const hit_t *h){ return h->epoch_local; }
static inline float        hit_get_last_advert(const hit_t *h){ return h->last_adv_f; }

#ifdef __cplusplus
}
#endif
#endif /* POGO_UTILS_HIT_H */

