/**
 * @file pgpe.h
 * @brief Parameter-based Policy Gradient (PGPE / NES-style) for float arrays (malloc‑free).
 *
 * @details
 * This implements a lightweight, online-friendly PGPE optimizer that maintains
 * a diagonal Gaussian over parameters: \theta ~ N(\mu, diag(\sigma^2)). At each step,
 * it samples a small batch of perturbations \varepsilon ~ N(0, I), forms candidates
 * \theta = \mu + \sigma \odot \varepsilon (optionally with antithetic pairs),
 * evaluates the objective, and updates (\mu, \sigma) using score-function gradients
 * with an EWMA baseline for variance reduction. Works for minimization and maximization.
 *
 * Features:
 *  - No dynamic allocation: caller provides all buffers (mu, sigma, theta, eps).
 *  - Antithetic sampling (variance reduction) with a single eps buffer reused.
 *  - Per-dimension bounds for samples and mean; diagonal exploration scales.
 *  - Online control: choose samples_per_tick to bound per-tick cost.
 */
#ifndef POGO_UTILS_PGPE_H
#define POGO_UTILS_PGPE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/** Objective callback: returns fitness f(x). Lower is better if mode=MIN. */
typedef float (*pgpe_objective_fn)(const float *restrict x, int n, void *user);

/** Optimization direction. */
typedef enum {
    PGPE_MINIMIZE = 0,
    PGPE_MAXIMIZE = 1
} pgpe_mode_t;

/** Tunable PGPE parameters. */
typedef struct {
    pgpe_mode_t mode;          /**< Minimize or maximize. */
    float lr_mu;               /**< Learning rate for mean \mu. */
    float lr_sigma;            /**< Learning rate for log-\sigma updates (multiplicative). */
    float sigma_min;           /**< Lower clamp for \sigma (strictly > 0). */
    float sigma_max;           /**< Upper clamp for \sigma. */
    int   samples_per_tick;    /**< Number of candidate samples evaluated per tick (>=1). */
    bool  antithetic;          /**< If true, use antithetic pairs (doubles effective samples). */
    float baseline_beta;       /**< EWMA baseline coefficient in [0,1); 0 disables baseline. */
    bool  project_after_update;/**< If true, clamp \mu after parameter update. */
} pgpe_params_t;

/**
 * PGPE optimizer state (POD). User must provide all buffers:
 *  - mu, sigma : current distribution parameters (length n); mu is the solution.
 *  - theta     : workspace for a candidate sample (length n).
 *  - eps       : workspace for standard normal vector (length n).
 *  - lo, hi    : optional per-dimension bounds for \theta and \mu (nullable).
 */
typedef struct {
    /* Problem */
    int n;
    pgpe_objective_fn fn;
    void *user;

    /* Parameters */
    pgpe_params_t p;

    /* Buffers (provided by user) */
    float *restrict mu;        /**< Mean parameters (solution) — in/out. */
    float *restrict sigma;     /**< Per-dimension exploration scales (>0) — in/out. */
    float *restrict theta;     /**< Workspace: current sampled candidate. */
    float *restrict eps;       /**< Workspace: standard normal noise. */
    const float *restrict lo;  /**< Optional lower bounds (nullable). */
    const float *restrict hi;  /**< Optional upper bounds (nullable). */

    /* State */
    uint32_t k;                /**< Step counter. */
    float f_curr;              /**< f(\mu) at last monitoring evaluation. */
    float f_best;              /**< Best seen f(\mu) according to mode. */
    float baseline;            /**< EWMA baseline on reward (R). */

    /* RNG */
    uint32_t rng_state;        /**< Optional RNG state if you switch from rand(). */
} pgpe_t;

/** Initialize PGPE on pre-allocated buffers. */
void pgpe_init(pgpe_t *pgpe, int n,
               float *restrict mu,
               float *restrict sigma,
               float *restrict theta,
               float *restrict eps,
               const float *restrict lo, const float *restrict hi,
               pgpe_objective_fn fn, void *user,
               const pgpe_params_t *params);

/** Perform one control-tick worth of PGPE (samples_per_tick evaluations, or x2 with antithetic). */
float pgpe_step(pgpe_t *pgpe);

/** Accessors */
static inline const float *pgpe_get_mu(const pgpe_t *pgpe) { return pgpe->mu; }
static inline const float *pgpe_get_sigma(const pgpe_t *pgpe) { return pgpe->sigma; }
static inline float pgpe_f(const pgpe_t *pgpe) { return pgpe->f_curr; }
static inline float pgpe_best_f(const pgpe_t *pgpe) { return pgpe->f_best; }
static inline uint32_t pgpe_iterations(const pgpe_t *pgpe) { return pgpe->k; }

#ifdef __cplusplus
}
#endif

#endif /* POGO_UTILS_PGPE_H */

