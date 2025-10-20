/**
 * @file oneplusone_es.h
 * @brief Lightweight (μ+λ) = (1+1) Evolution Strategy for float parameter arrays.
 *
 * @details
 * The 1+1-ES maintains a single parent vector x (dimension n) and a single
 * offspring x' = x + sigma * N(0, I). If the offspring is better, it replaces
 * the parent. Step-size sigma is adapted online via a smoothed 1/5th success
 * rule:
 *
 *     s_ewma <- (1 - alpha) * s_ewma + alpha * success
 *     sigma  <- clamp( sigma * exp(c_sigma * (s_ewma - s_target)), [sigma_min, sigma_max] )
 *
 * - No malloc: user binds all buffers at init (parent x, candidate x_try, optional bounds).
 * - Supports minimization or maximization.
 * - Optional variable bounds per coordinate (clamped after mutation).
 * - Uses a Box–Muller normal RNG built on top of `rand()`; seed with `srand()`.
 *
 * Typical usage:
 *  1) Fill params; call es1p1_init().
 *  2) At each control tick, call es1p1_step().
 *  3) Read back current best with es1p1_get_x() and fitness with es1p1_best_f().
 */

#ifndef POGO_UTILS_ONEPLUSONE_ES_H
#define POGO_UTILS_ONEPLUSONE_ES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/** Objective callback: returns fitness f(x). Lower is better if mode=MIN. */
typedef float (*es1p1_objective_fn)(const float *restrict x, int n, void *user);

/** Optimization direction. */
typedef enum {
    ES1P1_MINIMIZE = 0,
    ES1P1_MAXIMIZE = 1
} es1p1_mode_t;

/** Tunable parameters (choose small c_sigma, e.g., 0.6 / sqrt(n)). */
typedef struct {
    es1p1_mode_t mode;     /**< Minimize or maximize. */
    float sigma0;          /**< Initial step-size (e.g., 0.1f). */
    float sigma_min;       /**< Lower bound on sigma (e.g., 1e-6f). */
    float sigma_max;       /**< Upper bound on sigma (e.g., 1.0f). */
    float s_target;        /**< Target success rate (default 0.2f). */
    float s_alpha;         /**< EWMA smoothing for success (e.g., 0.2f). */
    float c_sigma;         /**< Step-size learning rate (e.g., 0.6f / sqrt(n)). */
    int   evals_per_tick;  /**< How many ES iterations per control step (>=1). */
} es1p1_params_t;

/**
 * Internal state (POD). Bind user-provided buffers to avoid allocations.
 * Buffers:
 *  - x      : parent vector (length n), owned by caller (will be modified in-place).
 *  - x_try  : offspring workspace (length n).
 *  - lo, hi : optional bounds (length n each) or NULL (no bounds).
 */
typedef struct {
    // Problem definition
    int n;
    es1p1_objective_fn fn;
    void *user;

    // Parameters
    es1p1_params_t p;

    // Buffers (provided by user)
    float *restrict x;       /**< Current solution (parent) — in/out. */
    float *restrict x_try;   /**< Candidate (offspring) — workspace. */
    const float *restrict lo;/**< Optional per-dim lower bounds (nullable). */
    const float *restrict hi;/**< Optional per-dim upper bounds (nullable). */

    // State
    float sigma;
    float best_f;
    float s_ewma;            /**< Smoothed success rate. */

    // RNG (Box–Muller cache)
    int   have_spare;
    float spare;

    // Book-keeping
    uint32_t iter;           /**< Number of ES iterations performed. */
} es1p1_t;

/**
 * @brief Initialize the 1+1-ES on user-provided buffers.
 *
 * @param es      ES handle (zero- or stack-allocated).
 * @param n       Dimension (>=1).
 * @param x       Parent vector (length n). Will be modified in place.
 * @param x_try   Offspring workspace (length n).
 * @param lo      Optional per-coordinate lower bounds (nullable).
 * @param hi      Optional per-coordinate upper bounds (nullable).
 * @param fn      Objective function pointer (must be non-NULL).
 * @param user    User pointer passed to fn (nullable).
 * @param params  ES parameters; sensible defaults if NULL.
 *
 * @post  es->best_f is set by one evaluation of x; es->sigma = sigma0.
 */
void es1p1_init(es1p1_t *es, int n,
                float *restrict x, float *restrict x_try,
                const float *restrict lo, const float *restrict hi,
                es1p1_objective_fn fn, void *user,
                const es1p1_params_t *params);

/**
 * @brief Perform one control-tick worth of optimization.
 *
 * Runs `evals_per_tick` independent (1+1) iterations:
 *  - Sample x' ~ N(x, sigma^2 I)
 *  - Clamp to [lo, hi] if bounds are provided
 *  - Accept if better according to @ref es1p1_params_t::mode
 *  - Update s_ewma and adapt sigma
 *
 * @return The current best fitness after the batch.
 */
float es1p1_step(es1p1_t *es);

/** @brief Get current best vector (parent). */
static inline const float *es1p1_get_x(const es1p1_t *es) { return es->x; }

/** @brief Get current best fitness. */
static inline float es1p1_best_f(const es1p1_t *es) { return es->best_f; }

/** @brief Get current step-size sigma. */
static inline float es1p1_sigma(const es1p1_t *es) { return es->sigma; }

/** @brief Total number of ES iterations performed so far. */
static inline uint32_t es1p1_iterations(const es1p1_t *es) { return es->iter; }

#ifdef __cplusplus
}
#endif

#endif /* POGO_UTILS_ONEPLUSONE_ES_H */

