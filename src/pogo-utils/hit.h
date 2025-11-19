#ifndef POGO_UTILS_HIT_H
#define POGO_UTILS_HIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifndef HIT_MAX_T
#define HIT_MAX_T 32
#endif


/**
 * @brief Optimization direction.
 *
 * HIT-2020 maximizes a reward. Here we support both by flipping the
 * comparison in a single place.
 */
typedef enum {
    HIT_MINIMIZE = 0,
    HIT_MAXIMIZE = 1
} hit_mode_t;

/**
 * @brief Parameters for HIT (CEC 2020 flavour).
 *
 * - mode        : minimize or maximize the scalar objective.
 * - alpha       : initial transfer rate α (0..1), i.e. fraction of parameters
 *                 to copy from a better neighbour.
 * - sigma       : std-dev of Gaussian mutation applied to all coordinates of x.
 * - eval_T      : evaluation / maturation time T (sliding-window length).
 *                 Communication is disabled until at least T rewards have
 *                 been observed since the last parameter change.
 * - evolve_alpha: if non-zero, α itself is treated as an evolvable parameter:
 *                 it is copied from better neighbours and mutated with a small
 *                 Gaussian noise alpha_sigma, then clamped into
 *                 [alpha_min, alpha_max].
 *
 * Typical CEC2020-like values are:
 *   eval_T      = 200
 *   alpha       in [0.3, 0.8]
 *   sigma       ≈ 1e-3 (task-dependent)
 *   evolve_alpha= true with alpha_min=0.0, alpha_max<1.0 and alpha_sigma≈1e-3.
 */
typedef struct {
    hit_mode_t mode;
    float      alpha;        /**< Initial transfer rate α (0..1).          */
    float      sigma;        /**< Mutation stddev σ on genome coordinates. */

    int        eval_T;       /**< Sliding-window length T (>=1).           */

    bool       evolve_alpha; /**< false: fixed α, true: evolvable α.              */
    float      alpha_sigma;  /**< Mutation stddev on α when evolvable.     */
    float      alpha_min;    /**< Lower bound for α when evolvable.        */
    float      alpha_max;    /**< Upper bound for α when evolvable.        */
} hit_params_t;

/**
 * @brief HIT optimizer state.
 *
 * Design follows HIT-2020:
 *   - Each agent holds a single genome x and a scalar transfer rate α.
 *   - Performance is assessed using a sliding window of length T over
 *     instantaneous rewards r_t supplied through hit_tell().
 *   - Robots are in a "maturation" phase until the window is full; during
 *     maturation, communication is disabled (no adoption of neighbours and
 *     no broadcast should occur).
 *   - When hit_observe_remote() sees a better neighbour (w.r.t. the current
 *     sliding-window score), it:
 *       1. Copies k = round(α·n) randomly chosen coordinates from the
 *          neighbour genome.
 *       2. Mutates ALL coordinates of x with Gaussian noise σ.
 *       3. If evolve_alpha=true, copies and mutates the neighbour's α.
 *       4. Resets the maturation timer and the sliding window.
 */
typedef struct {
    /* Problem size / mode */
    int        n;
    hit_mode_t mode;

    /* HIT hyperparameters (α may evolve) */
    float alpha;        /**< Current transfer rate α.         */
    float sigma;        /**< Mutation stddev σ.               */
    int   T;            /**< Evaluation time / window size T. */

    bool  evolve_alpha; /**< false: fixed α, true: evolvable α.      */
    float alpha_sigma;  /**< Mutation stddev for α.           */
    float alpha_min;    /**< Clamp range for α.               */
    float alpha_max;

    /* Genomes (owned by caller) */
    float       *x;      /**< Current genome to evaluate.         */
    float       *x_buf;  /**< Optional work buffer (may be NULL). */
    const float *lo;     /**< Optional lower bounds (size n).     */
    const float *hi;     /**< Optional upper bounds (size n).     */

    /* Sliding-window score G_t (accumulated over last T rewards) */
    float    f;          /**< Current score over the window.      */
    int      have_f;     /**< 0 = not yet mature, 1 = mature.     */

    /* Maturation / sliding-window bookkeeping */
    uint32_t step;       /**< Steps since last parameter change.  */
    float    reward_buf_static[HIT_MAX_T];
    float   *reward_buf; /**< Circular buffer of size T (or NULL
                              if T<=1 and no buffering is needed). */

    /* Book-keeping */
    uint32_t epoch;      /**< Increments at each successful transfer. */

    /* RNG cache for Box–Muller */
    int   have_spare;
    float spare;
} hit_t;

/* ===== API ===== */

/**
 * @brief Initialize a HIT optimizer.
 *
 * @param h      Handle to initialize.
 * @param n      Dimension of the genome.
 * @param x      Pointer to current genome (size n).
 * @param x_buf  Optional work buffer (size n). If NULL, x is used in-place.
 * @param lo     Optional lower bounds (size n), or NULL.
 * @param hi     Optional upper bounds (size n), or NULL.
 * @param params HIT parameters (mode, alpha, sigma, ...). If NULL, defaults
 *               are roughly HIT-like:
 *                 mode        = HIT_MINIMIZE
 *                 alpha       = 0.3f
 *                 sigma       = 0.001f
 *                 eval_T      = 200
 *                 evolve_alpha= false
 *                 alpha_sigma = 0.001f
 *                 alpha_min   = 0.0f
 *                 alpha_max   = 0.9f
 */
void hit_init(hit_t *h, int n,
              float *x, float *x_buf,
              const float *lo, const float *hi,
              const hit_params_t *params);

/**
 * @brief Set the initial sliding-window score explicitly.
 *
 * This is mostly for backwards compatibility. In the CEC2020 flavour,
 * you typically DO NOT call this function, and instead feed instantaneous
 * rewards r_t to hit_tell(), which fills the window and sets the score.
 */
void hit_tell_initial(hit_t *h, float f0);

/**
 * @brief Ask for the genome to evaluate / execute.
 *
 * In HIT, the current genome is always h->x. This function exists to keep
 * the ask/tell structure consistent with other optimizers.
 *
 * @return Pointer to genome of length n, or NULL if h is NULL.
 */
const float *hit_ask(hit_t *h);

/**
 * @brief Tell HIT the instantaneous reward (or cost) at this step.
 *
 * This must be called at every control step with the scalar reward r_t
 * (or cost, if mode == HIT_MINIMIZE). Internally, HIT maintains a sliding
 * window of the last T values and updates the score f = G_t.
 *
 * Communication (adoption of neighbours) is disabled until at least T
 * values have been observed since the last parameter change.
 *
 * @param h Handle.
 * @param r Instantaneous reward (or cost).
 */
void hit_tell(hit_t *h, float r);

/**
 * @brief Notify HIT of a neighbour's advertised genome and score.
 *
 * This is the core of HIT:
 *   - If the local agent is still in maturation (i.e. not enough samples
 *     to fill the sliding window), the message is ignored.
 *   - Otherwise, if the neighbour is better (according to mode), apply
 *     horizontal transfer + mutation immediately:
 *       1. Choose k = round(alpha * n) distinct indices without
 *          replacement.
 *       2. Copy those coordinates from x_remote into the local genome.
 *       3. Mutate ALL coordinates with additive Gaussian noise of
 *          stddev sigma (and optional clipping to [lo, hi]).
 *       4. If evolve_alpha=true, copy neighbour's alpha_remote, mutate it
 *          with alpha_sigma, and clamp it into [alpha_min, alpha_max].
 *       5. Reset maturation: clear the sliding-window state and mark the
 *          current score as unknown until the new window is filled again.
 *
 * @param h            Handle.
 * @param from_id      Sender id (unused, but kept for API symmetry).
 * @param epoch        Sender epoch (unused here).
 * @param x_remote     Neighbour genome (size n).
 * @param f_remote     Neighbour sliding-window score.
 * @param alpha_remote Neighbour transfer rate α (ignored if
 *                     evolve_alpha==false).
 */
void hit_observe_remote(hit_t *h, uint16_t from_id, uint32_t epoch,
                        const float *x_remote, float f_remote,
                        float alpha_remote);

/* ===== Convenience getters ===== */

static inline const float *hit_get_x(const hit_t *h){ return h ? h->x : NULL; }
static inline uint32_t     hit_get_epoch(const hit_t *h){ return h ? h->epoch : 0u; }
static inline float        hit_get_alpha(const hit_t *h){ return h ? h->alpha : 0.0f; }

/**
 * @brief Get the current score as a sliding-window *average*.
 *
 * - Before maturation (window not full), returns the average over the
 *   samples seen so far, if any; 0 otherwise.
 * - After maturation, returns the average over the last T rewards:
 *       f_avg = (1/T) * sum_{k=0}^{T-1} r_{t-k}
 * - For T <= 1, this is just the latest reward.
 */
static inline float hit_get_f(const hit_t *h){
    if (!h){
        return 0.0f;
    }

    /* Degenerate case: T <= 1 → no window, f is just the last reward. */
    if (h->T <= 1){
        return h->f;
    }

    /* Before maturation (window not full yet): average over step samples. */
    if (!h->have_f){
        if (h->step > 0){
            return h->f / (float)h->step;
        } else {
            return 0.0f;
        }
    }

    /* Mature: full window → average over T samples. */
    return h->f / (float)h->T;
}

/**
 * @brief (Optional) Get the raw sliding-window *sum* (for debugging/advanced use).
 */
static inline float hit_get_f_sum(const hit_t *h){
    return h ? h->f : 0.0f;
}


/**
 * @brief Return 1 if the agent is "mature", i.e. the sliding window is full
 *        and a valid score is available.
 */
int      hit_ready(const hit_t *h);

/**
 * @brief Number of successful transfers so far (for logging).
 */
uint32_t hit_iterations(const hit_t *h);

#ifdef __cplusplus
}
#endif
#endif /* POGO_UTILS_HIT_H */

