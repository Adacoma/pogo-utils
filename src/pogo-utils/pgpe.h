/**
 * @file pgpe.h
 * @brief PGPE (Policy Gradient with Parameter-based Exploration), ask–tell API.
 *
 * @details
 * PGPE optimizes a parameter vector μ with multiplicative exploration σ using
 * antithetic sampling θ± = μ ± σ ⊙ ε, ε ~ N(0, I). After receiving the two
 * fitness values f(θ+), f(θ−), it performs a policy-gradient style update:
 *
 *  - Convert fitness to reward r depending on mode (maximize/minimize).
 *  - Baseline b is an EWMA of rewards to reduce variance.
 *  - Gradient estimates (per dimension i):
 *        g_μ[i]   ≈ ((r+ - r−) / 2) * ε[i] / σ[i]
 *        g_logσ[i]≈ ((r+ + r− - 2 b) / 2) * (ε[i]^2 - 1)
 *    We update μ additively and log σ additively (i.e., σ *= exp(η_σ g_logσ)).
 *
 * Usage (ask–tell, no dynamic allocation):
 *   1) Fill μ and σ arrays (length n); set optional lo/hi bounds.
 *   2) pgpe_init(&pg, n, mu, sigma, x_try, eps, lo, hi, &params);
 *   3) Repeatedly:
 *        const float *x = pgpe_ask(&pg);         // θ+
 *        float f = objective(x); pgpe_tell(&pg, f);
 *        x = pgpe_ask(&pg);                      // θ− (antithetic)
 *        f = objective(x); pgpe_tell(&pg, f);    // triggers update of μ, σ
 *
 * Notes:
 *  - No malloc: caller supplies μ, σ, x_try, ε (all length n).
 *  - Bounds (nullable) clamp candidates θ±; μ is also kept within bounds.
 *  - Designed for online control loops (one ask–tell pair spans two evaluations).
 */

#ifndef POGO_UTILS_PGPE_H
#define POGO_UTILS_PGPE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/** Optimization direction. */
typedef enum {
    PGPE_MINIMIZE = 0,
    PGPE_MAXIMIZE = 1
} pgpe_mode_t;

/** Tunable parameters. */
typedef struct {
    pgpe_mode_t mode;     /**< Minimize or maximize (reward sign). */
    float eta_mu;         /**< Learning-rate for μ (e.g., 0.05f). */
    float eta_sigma;      /**< Learning-rate for log σ (e.g., 0.1f). */
    float sigma_min;      /**< Lower bound on σ (e.g., 1e-6f). */
    float sigma_max;      /**< Upper bound on σ (e.g., 1.0f). */
    float baseline_alpha; /**< EWMA smoothing for baseline b (0..1, e.g., 0.1f). */
    bool   normalize_pair;/**< If true center the pair: r←r - mean({r+,r−}). */
} pgpe_params_t;

/** Internal state (POD; no heap). */
typedef struct {
    int n;                           /**< Dimension. */
    pgpe_params_t p;

    /* Caller-provided buffers (length n) */
    float *restrict mu;              /**< Mean parameters μ (updated in place). */
    float *restrict sigma;           /**< Exploration scales σ (updated in place). */
    float *restrict x_try;           /**< Workspace for candidates θ± (ask output). */
    float *restrict eps;             /**< Current noise vector ε for the pair. */
    const float *restrict lo;        /**< Optional per-dim lower bounds (nullable). */
    const float *restrict hi;        /**< Optional per-dim upper bounds (nullable). */

    /* Pair bookkeeping */
    int   phase;                     /**< 0: need θ+, 1: need θ−, 2: ready to update */
    float r_plus;                    /**< Reward for θ+. */
    float r_minus;                   /**< Reward for θ−. */
    float baseline;                  /**< Reward baseline b (EWMA). */

    /* RNG (Box–Muller spare) */
    int   have_spare;
    float spare;

    uint32_t iters;                  /**< Number of completed PGPE updates. */
} pgpe_t;

/**
 * @brief Initialize PGPE with caller-provided buffers (no malloc).
 *
 * @param pg     Handle (stack- or zero-allocated).
 * @param n      Dimension (>=1).
 * @param mu     Mean vector μ (in/out, length n).
 * @param sigma  Exploration σ (in/out, length n). Values will be clamped to [sigma_min, sigma_max].
 * @param x_try  Candidate workspace θ (ask writes here).
 * @param eps    Noise workspace ε (ask writes here).
 * @param lo     Optional lower bounds per dim (nullable).
 * @param hi     Optional upper bounds per dim (nullable).
 * @param params Optional PGPE params; sensible defaults if NULL.
 */
void pgpe_init(pgpe_t *pg, int n,
               float *restrict mu, float *restrict sigma,
               float *restrict x_try, float *restrict eps,
               const float *restrict lo, const float *restrict hi,
               const pgpe_params_t *params);

/**
 * @brief Produce the next candidate θ to evaluate (antithetic pair).
 *
 * On the first call of a pair (phase=0), samples ε~N(0,I), returns θ+=μ+σ⊙ε.
 * On the second call (phase=1), returns θ−=μ−σ⊙ε. After both are told, an
 * update is performed and phase resets to 0.
 *
 * @return Pointer to candidate θ (length n). Never NULL after init.
 */
const float *pgpe_ask(pgpe_t *pg);

/**
 * @brief Provide fitness for the last ask() and possibly update μ and σ.
 *
 * @param pg      PGPE handle.
 * @param fitness Fitness value (the library converts to reward according to mode).
 */
void pgpe_tell(pgpe_t *pg, float fitness);

/*** Inline getters *********************************************************/
static inline const float *pgpe_get_mu(const pgpe_t *pg) { return pg->mu; }
static inline const float *pgpe_get_sigma(const pgpe_t *pg) { return pg->sigma; }
static inline uint32_t     pgpe_iterations(const pgpe_t *pg) { return pg->iters; }

/**
 * @brief Convenience helper: perform one full PGPE update (two evaluations).
 *
 * @param pg       PGPE handle (must be initialized).
 * @param fn       Objective callback: returns fitness given θ (length n).
 * @param userdata User context passed to callback.
 * @return         Reward of θ− (second evaluation) after the internal update.
 */
typedef float (*pgpe_objective_fn)(const float *x, int n, void *userdata);
float pgpe_step(pgpe_t *pg, pgpe_objective_fn fn, void *userdata);

#ifdef __cplusplus
}
#endif
#endif /* POGO_UTILS_PGPE_H */

