#ifndef POGO_UTILS_HIT_H
#define POGO_UTILS_HIT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

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
 * @brief Parameters for strict HIT (CEC 2020).
 *
 * - mode  : minimize or maximize the scalar objective.
 * - alpha : fraction of parameters (0..1) to copy from a better neighbour.
 *           In the paper this is the transfer rate α; exactly floor(alpha*n)
 *           dimensions are copied at each accepted transfer.
 * - sigma : standard deviation of Gaussian mutation applied AFTER transfer.
 *           Mutation is additive in parameter space: x_d ← x_d + σ·N(0,1),
 *           with optional clamping to [lo_d, hi_d] when bounds are provided.
 */
typedef struct {
    hit_mode_t mode;
    float      alpha;   /**< Transfer rate α (0..1). */
    float      sigma;   /**< Mutation stddev σ.     */
} hit_params_t;

/**
 * @brief HIT optimizer state.
 *
 * Design is “strict HIT”:
 *   - Single parent genome x (no repository, no probabilistic acceptance).
 *   - When hit_observe_remote() sees a better neighbour:
 *       - Choose exactly α·n indices without replacement.
 *       - Copy those coordinates from remote genome.
 *       - Apply additive Gaussian mutation to ALL coordinates.
 *       - Mark the genome as “needing re-evaluation” (have_f = 0).
 *   - ask/tell are just a clean interface around “current genome + fitness”.
 */
typedef struct {
    /* Problem size / mode */
    int        n;
    hit_mode_t mode;

    /* HIT hyperparameters */
    float alpha;   /**< Transfer rate α.      */
    float sigma;   /**< Mutation stddev σ.    */

    /* Genomes (owned by caller) */
    float       *x;      /**< Current genome to evaluate.     */
    float       *x_buf;  /**< Optional work buffer (may be NULL). */
    const float *lo;     /**< Optional lower bounds (size n). */
    const float *hi;     /**< Optional upper bounds (size n). */

    /* Current fitness */
    float    f;          /**< Fitness of x.            */
    int      have_f;     /**< 0 = unknown, 1 = valid.  */

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
 * @param params HIT parameters (mode, alpha, sigma). If NULL, defaults are:
 *               mode = HIT_MINIMIZE, alpha = 0.3f, sigma = 0.1f.
 */
void hit_init(hit_t *h, int n,
              float *x, float *x_buf,
              const float *lo, const float *hi,
              const hit_params_t *params);

/**
 * @brief Set the initial fitness of the current genome.
 *
 * Must be called once after you evaluate the initial x, before the
 * main ask/tell loop. (If you never call this, neighbours will be
 * considered better by default and the first incoming advert will
 * be accepted.)
 */
void hit_tell_initial(hit_t *h, float f0);

/**
 * @brief Ask for the genome to evaluate.
 *
 * In strict HIT, the current genome is always h->x. This function
 * mainly exists for symmetry with other optimizers and to keep the
 * ask/tell structure consistent. It does not create a new offspring:
 * new genomes are produced only when a better neighbour is observed.
 *
 * @return Pointer to genome of length n, or NULL if h is NULL.
 */
const float *hit_ask(hit_t *h);

/**
 * @brief Tell HIT the fitness of the last evaluated genome.
 *
 * @param h Handle.
 * @param f Fitness of the genome returned by hit_ask().
 */
void hit_tell(hit_t *h, float f);

/**
 * @brief Notify HIT of a neighbour's advertised genome and fitness.
 *
 * This is the core of strict HIT:
 *   - If neighbour is better (according to mode), apply horizontal
 *     transfer + mutation immediately to the local genome:
 *       1. Choose k = round(alpha * n) distinct indices without
 *          replacement.
 *       2. Copy those coordinates from x_remote into the local genome.
 *       3. Mutate ALL coordinates with additive Gaussian noise of
 *          stddev sigma (and optional clipping to [lo, hi]).
 *       4. Set have_f = 0 and increment epoch.
 *
 * @param h         Handle.
 * @param from_id   Sender id (unused, but kept for API symmetry).
 * @param epoch     Sender epoch (unused).
 * @param x_remote  Neighbour genome (size n).
 * @param f_remote  Neighbour fitness.
 */
void hit_observe_remote(hit_t *h, uint16_t from_id, uint32_t epoch,
                        const float *x_remote, float f_remote);

/* ===== Convenience getters ===== */

static inline const float *hit_get_x(const hit_t *h){ return h ? h->x : NULL; }
static inline uint32_t     hit_get_epoch(const hit_t *h){ return h ? h->epoch : 0u; }
static inline float        hit_get_f(const hit_t *h){ return h ? h->f : 0.0f; }

int      hit_ready(const hit_t *h);      /* 0/1: has initial fitness been set? */
uint32_t hit_iterations(const hit_t *h); /* e.g. just return hit_get_epoch(h) */

#ifdef __cplusplus
}
#endif
#endif /* POGO_UTILS_HIT_H */

