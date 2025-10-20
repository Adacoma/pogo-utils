/**
 * @file spsa.h
 * @brief Simultaneous Perturbation Stochastic Approximation (SPSA) for float arrays (malloc‑free).
 *
 * @details
 * SPSA is a gradient-free stochastic optimizer well-suited for online and
 * embedded settings where the objective is noisy or expensive. At iteration k,
 * it perturbs all coordinates simultaneously with a random Rademacher vector
 * Δ_k ∈ {−1, +1}^n and forms two evaluations:
 *
 *   x_+ = x_k + c_k Δ_k,   x_- = x_k − c_k Δ_k,
 *   ĝ_k(i) = [f(x_+) − f(x_-)] / [2 c_k Δ_k(i)]  for i = 1..n,
 *   x_{k+1} = x_k − s * a_k * ĝ_k   (s = +1 for minimize, s = −1 maximize),
 *
 * with gain sequences a_k = a / (k + 1 + A)^α and c_k = c / (k + 1)^γ.
 * Common choices: α≈0.602, γ≈0.101, A≈10% of expected iterations.
 *
 * Features:
 *  - No dynamic allocation: caller provides all buffers (x, delta, work).
 *  - Works for minimization and maximization.
 *  - Optional per-dimension bounds (lo/hi) with projection (clamping).
 *  - Online-friendly: run a fixed number of SPSA iterations per control tick.
 *  - Reentrant POD state; seed RNG with srand().
 *
 * Typical usage:
 *  1) Fill x with an initial guess; call spsa_init().
 *  2) Each control step, call spsa_step().
 *  3) Read back spsa_get_x() and spsa_best_f().
 */

#ifndef POGO_UTILS_SPSA_H
#define POGO_UTILS_SPSA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/** Objective callback: returns fitness f(x). Lower is better if mode=MIN. */
typedef float (*spsa_objective_fn)(const float *restrict x, int n, void *user);

/** Optimization direction. */
typedef enum {
    SPSA_MINIMIZE = 0,  /**< Minimize f(x). */
    SPSA_MAXIMIZE = 1   /**< Maximize f(x). */
} spsa_mode_t;

/** Tunable parameters for SPSA gain sequences and runtime. */
typedef struct {
    spsa_mode_t mode;       /**< Minimize or maximize. */
    float a;                /**< Gain for a_k (step size). */
    float c;                /**< Gain for c_k (perturbation magnitude). */
    float alpha;            /**< Exponent for a_k denominator (≈0.602). */
    float gamma;            /**< Exponent for c_k denominator (≈0.101). */
    float A;                /**< Stability offset in a_k denominator. */
    int   evals_per_tick;   /**< Iterations per control step (>=1); cost 2 evals/iter. */
    bool  project_after_update; /**< If true, clamp x after the update as well. */
} spsa_params_t;

/**
 * SPSA optimizer state (POD). User must provide all buffers:
 *  - x         : current parameter vector (length n); modified in place.
 *  - delta     : workspace for the ±1 perturbation vector (length n).
 *  - x_plus    : workspace (length n) storing x + c_k Δ.
 *  - x_minus   : workspace (length n) storing x − c_k Δ.
 *  - lo, hi    : optional per-dimension bounds (nullable).
 */
typedef struct {
    /* Problem */
    int n;                         /**< Dimension. */
    spsa_objective_fn fn;          /**< Objective callback. */
    void *user;                    /**< User data pointer (nullable). */

    /* Parameters */
    spsa_params_t p;

    /* Buffers (provided by user) */
    float *restrict x;             /**< Current solution — in/out. */
    float *restrict delta;         /**< Rademacher vector workspace. */
    float *restrict x_plus;        /**< Perturbed + candidate. */
    float *restrict x_minus;       /**< Perturbed − candidate. */
    const float *restrict lo;      /**< Optional lower bounds (nullable). */
    const float *restrict hi;      /**< Optional upper bounds (nullable). */

    /* State */
    uint32_t k;                    /**< Iteration counter (0-based). */
    float f_curr;                  /**< f(x_k). */
    float f_best;                  /**< Best seen f. */

    /* RNG */
    uint32_t rng_state;            /**< Optional internal RNG state if desired (unused for rand()). */
} spsa_t;

/** Initialize SPSA on pre-allocated buffers. */
void spsa_init(spsa_t *spsa, int n,
               float *restrict x,
               float *restrict delta,
               float *restrict x_plus,
               float *restrict x_minus,
               const float *restrict lo, const float *restrict hi,
               spsa_objective_fn fn, void *user,
               const spsa_params_t *params);

/** Perform one control-tick worth of SPSA (evals_per_tick iterations). */
float spsa_step(spsa_t *spsa);

/** Accessors */
static inline const float *spsa_get_x(const spsa_t *spsa) { return spsa->x; }
static inline float spsa_f(const spsa_t *spsa) { return spsa->f_curr; }
static inline float spsa_best_f(const spsa_t *spsa) { return spsa->f_best; }
static inline uint32_t spsa_iterations(const spsa_t *spsa) { return spsa->k; }

#ifdef __cplusplus
}
#endif

#endif /* POGO_UTILS_SPSA_H */

