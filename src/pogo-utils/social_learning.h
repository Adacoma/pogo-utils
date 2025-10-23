/**
 * @file social_learning.h
 * @brief mEDEA-style Social Learning (SL) + bounded repository in strict ask–tell form.
 *
 * @details
 * This library implements a decentralized social-learning algorithm as
 * a reusable, robot-agnostic optimizer:
 *
 *   - Strict ask–tell usage (no objective callback inside the library).
 *   - A bounded repository of encountered genomes (vectors) with their
 *     advertised fitness (loss) and metadata (epoch, sender id).
 *   - Roulette selection over the repository ∪ {current parent} using
 *     fitness=1/L for minimization (or raw for maximization).
 *   - Gaussian mutation in *normalized* [0,1] space, then denormalization
 *     using optional per-dimension bounds (lo/hi).
 *   - Mutation amplitude scales with the (clipped) current window loss via
 *     `loss_mut_gain` and `loss_mut_clip`.
 *   - Decentralization hook: ingest remote adverts via sl_observe_remote().
 *
 * Design goals:
 *   - No malloc if the caller provides all buffers (parent x, offspring x_try,
 *     repo arrays). A tiny amount of metadata lives in the sl_t struct.
 *   - Works for MINIMIZE or MAXIMIZE.
 *
 * Typical usage:
 *   1) sl_init(...), set the initial parent vector in `x`.
 *   2) Evaluate f0 = objective(x) and sl_tell_initial(sl, f0).
 *   3) In each optimization tick:
 *        - const float *x_try = sl_ask(sl, loss_for_scale);
 *          (loss_for_scale is often the last/ongoing window loss; for
 *           MAXIMIZE, pass a *positive* difficulty scale you consider
 *           proportional to “error”; 0 is allowed.)
 *        - float f_try = objective(x_try);
 *        - float f_best = sl_tell(sl, f_try);  // also updates advert state
 *   4) Whenever you receive a neighbor’s advert, call:
 *        sl_observe_remote(sl, sender_id, sender_epoch, x_remote, f_adv_remote);
 *
 * Repository memory layout expected from the caller:
 *   - repo_X:  capacity * n floats (row-major), genome i at &repo_X[i*n].
 *   - repo_F:  capacity floats (fitness/loss as advertised).
 *   - repo_epoch: capacity uint32_t.
 *   - repo_from:  capacity uint16_t (or any 16-bit id; 0 ok).
 *
 */
#ifndef POGO_UTILS_SOCIAL_LEARNING_H
#define POGO_UTILS_SOCIAL_LEARNING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/** Optimization direction. */
typedef enum {
    SL_MINIMIZE = 0,
    SL_MAXIMIZE = 1
} sl_mode_t;

/** Tunables for SL. */
typedef struct {
    sl_mode_t mode;            /**< Minimization or maximization. */
    float roulette_random_prob;/**< Chance to pick a fresh random genome (0..1). */
    float loss_mut_gain;       /**< Mutation amplitude gain (typ. 0.5). */
    float loss_mut_clip;       /**< Clip for loss scale (typ. 1.0). */
    float dup_eps;             /**< Duplicate tolerance in normalized space (~1e-3). */
    uint16_t repo_capacity;    /**< Bounded repository size (<= buffers’ capacity). */
} sl_params_t;

/** SL handle (state). */
typedef struct {
    /* Problem */
    int n;
    sl_mode_t mode;

    /* Parent & offspring (owned by caller) */
    float *restrict x;         /**< Current parent — in/out. */
    float *restrict x_try;     /**< Offspring workspace. */
    const float *restrict lo;  /**< Optional bounds (nullable). */
    const float *restrict hi;  /**< Optional bounds (nullable). */

    /* Repo (owned by caller, no malloc here) */
    float    *restrict repo_X;     /**< capacity * n, row-major. */
    float    *restrict repo_F;     /**< capacity.  (Loss if MIN; fitness if MAX) */
    uint32_t *restrict repo_epoch; /**< capacity. */
    uint16_t *restrict repo_from;  /**< capacity. */
    uint16_t capacity;             /**< capacity actually usable. */
    uint16_t rsize;                /**< current number of entries [0..capacity]. */

    /* Parameters */
    float roulette_random_prob;
    float loss_mut_gain;
    float loss_mut_clip;
    float dup_eps;

    /* Book-keeping */
    uint32_t epoch_local;      /**< Bumps after each accepted parent “window”. */
    float    last_adv_f;       /**< Last advertised fitness (window average). */
    int      have_initial;     /**< 0 until sl_tell_initial(). */

    /* RNG cache (Box–Muller spare) */
    int   have_spare;
    float spare;
} sl_t;

/* ===== API ===== */

/**
 * @brief Initialize SL on caller-provided buffers.
 *
 * @param sl         Handle (zero-/stack-allocated).
 * @param n          Dimension (>=1).
 * @param x          Parent vector (length n). Will be modified in place.
 * @param x_try      Offspring workspace (length n).
 * @param lo         Optional per-dim lower bounds (nullable).
 * @param hi         Optional per-dim upper bounds (nullable).
 * @param repo_X     Repository genomes  (capacity * n floats).
 * @param repo_F     Repository fitness  (capacity floats).
 * @param repo_epoch Repository epochs   (capacity uint32).
 * @param repo_from  Repository senders  (capacity uint16).
 * @param capacity   Repository capacity (#rows in repo_X/repo_F/...).
 * @param params     Tunables (sensible defaults if NULL).
 *
 * @post You MUST call sl_tell_initial(sl, f0) before sl_ask().
 */
void sl_init(sl_t *sl, int n,
             float *restrict x, float *restrict x_try,
             const float *restrict lo, const float *restrict hi,
             float *restrict repo_X, float *restrict repo_F,
             uint32_t *restrict repo_epoch, uint16_t *restrict repo_from,
             uint16_t capacity, const sl_params_t *params);

/** Provide the initial fitness of the parent after sl_init(). */
void sl_tell_initial(sl_t *sl, float f0);

/**
 * @brief Sample and return a new offspring x' using roulette+mutation.
 *
 * @param sl              Handle.
 * @param loss_for_scale  Mutation scale driver:
 *                        - SL_MINIMIZE: pass the (clipped) current window loss.
 *                        - SL_MAXIMIZE: pass a positive “difficulty” scale you deem
 *                          proportional to error; pass 0 for minimal mutation.
 * @return Pointer to x' (length n), or NULL if called before sl_tell_initial().
 */
const float *sl_ask(sl_t *sl, float loss_for_scale);

/**
 * @brief Tell the SL the candidate’s fitness and perform replacement + repo update.
 *
 * @param sl     Handle.
 * @param f_try  Fitness of the last candidate returned by sl_ask().
 * @return       Current parent’s fitness after potential replacement.
 */
float sl_tell(sl_t *sl, float f_try);

/**
 * @brief Ingest a remote advert into the repository (duplicate-aware upsert).
 *
 * @param sl        Handle.
 * @param from_id   Sender id (free-form, 0..65535).
 * @param epoch     Sender’s epoch for that advert.
 * @param x_remote  Remote parent vector (length n).
 * @param f_adv     Remote advertised fitness (window avg).
 */
void sl_observe_remote(sl_t *sl, uint16_t from_id, uint32_t epoch,
                       const float *x_remote, float f_adv);

/* === Convenience getters ================================================== */
static inline const float *sl_get_x(const sl_t *sl)       { return sl->x; }
static inline uint32_t     sl_get_epoch(const sl_t *sl)   { return sl->epoch_local; }
static inline float        sl_get_last_advert(const sl_t *sl) { return sl->last_adv_f; }

#ifdef __cplusplus
}
#endif
#endif /* POGO_UTILS_SOCIAL_LEARNING_H */

