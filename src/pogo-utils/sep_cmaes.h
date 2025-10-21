/**
 * @file sep_cmaes.h
 * @brief Lightweight Separable CMA‑ES (diagonal covariance) in strict ask–tell form for float arrays.
 *
 * @details
 * **Purpose.** This module implements a minimal-yet-solid *separable* CMA‑ES (diag‑C) that is suitable
 * for embedded/robotic online optimization. It mirrors the ask–tell workflow used in the provided (1+1)-ES:
 *
 *   1) Call ::sep_cmaes_init() with caller-provided buffers (no malloc).
 *   2) Evaluate the initial fitness f(x) of the provided parent vector `x` and call ::sep_cmaes_tell_initial().
 *   3) Repeatedly for each control step (one or more evaluations per step):
 *        - const float* x_try = sep_cmaes_ask(&es);
 *        - float f_try = objective(x_try);
 *        - sep_cmaes_tell(&es, f_try);
 *
 * **Algorithm.** This is a (μ/λ) CMA‑ES with diagonal covariance only (SEP‑CMA‑ES).
 * It uses:
 *   - Weighted recombination of the best μ out of λ offspring.
 *   - Cumulative step-size adaptation (CSA) with path `p_sigma`.
 *   - Diagonal covariance update (rank‑one via `p_c` + rank‑μ on diagonal only).
 *
 * **Design goals.**
 *   - C11, deterministic, no dynamic allocation.
 *   - Same ask–tell style as the 1+1‑ES already used in the codebase.
 *   - Works for *online* optimization: evaluate 1 candidate per tick; a “generation” is λ tells.
 *   - Bounds are optional per-dimension (lo/hi). Candidates are clamped if provided.
 *
 * **Coordinate conventions.**
 *   - We store the *mean* vector `x` in caller memory (in/out).
 *   - We maintain diag(C) (variances) in `c_diag`. Sampling uses:  x' = x + sigma * sqrt(c_diag) ⊙ z, z~N(0,I).
 *
 * **Complexity & buffers.**
 *   - Per ask: O(n).
 *   - Per generation update: O(n * μ) where μ ≤ λ is small (typically 2..8 on embedded).
 *   - Callers must provide buffers for: x, x_try, lo/hi (optional), c_diag, p_sigma, p_c, z_try, y_try,
 *     store_y (size μ×n), fit_buf (size λ), and idx (size λ). See ::sep_cmaes_init().
 *
 * **Min/Max.** Set `mode` to ::SEP_MINIMIZE or ::SEP_MAXIMIZE.
 *
 * @note This is a simplified separable CMA‑ES; for large dimensions or ill‑conditioned problems,
 *       full CMA‑ES may perform better but is more expensive.
 */

#ifndef POGO_UTILS_SEP_CMAES_H
#define POGO_UTILS_SEP_CMAES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** Optimization direction. */
typedef enum {
    SEP_MINIMIZE = 0,
    SEP_MAXIMIZE = 1
} sep_mode_t;

/**
 * @brief Tunable parameters for SEP‑CMA‑ES.
 *
 * Values here follow standard CMA‑ES heuristics. For embedded use, keep λ small (e.g., 4..12)
 * and choose μ=⌊λ/2⌋ with log‑weights.
 */
typedef struct {
    sep_mode_t mode;     /**< Minimize/Maximize. */
    int lambda;          /**< Population size (offspring per “generation”). */
    int mu;              /**< Parents for recombination (μ ≤ λ). */
    float sigma0;        /**< Initial global step‑size (e.g., 0.3f). */
    float sigma_min;     /**< Lower clamp for sigma (e.g., 1e-8f). */
    float sigma_max;     /**< Upper clamp for sigma (e.g., 5.0f). */

    /* Recombination weights: length μ, positive and sum to 1.0. */
    const float *weights; /**< If NULL, defaults to log‑weights normalized. */

    /* Learning rates (defaults if <=0): */
    float cc;     /**< Cumulation for rank‑one path p_c.      (default ~ 2/(n+√2)) */
    float cs;     /**< Cumulation for sigma path p_sigma.     (default ~ (μ_w+2)/(n+μ_w+5)) */
    float c1;     /**< Rank‑one update for covariance.        (default ~ 2/((n+1.3)^2+μ_w)) */
    float cmu;    /**< Rank‑μ update for diagonal covariance. (default ~ min(1−c1, 2*(μ_w−1)/((n+2)^2+μ_w))) */
    float damps;  /**< Damping for sigma (default ~ 1 + 2*max(0, √((μ_w−1)/(n+1)) − 1) + cs). */

    /* Bound handling (clamp if provided): */
    const float *lo; /**< Optional per‑dimension lower bounds (nullable). */
    const float *hi; /**< Optional per‑dimension upper bounds (nullable). */
} sep_params_t;

/**
 * @brief Internal state (POD). Caller owns all buffers.
 *
 * Memory layout & sizes you must provide to ::sep_cmaes_init():
 *   - x        : length n  (mean vector, in/out).
 *   - x_try    : length n  (offspring workspace).
 *   - c_diag   : length n  (diagonal variances; init to 1.0).
 *   - p_sigma  : length n  (path for sigma; init to 0).
 *   - p_c      : length n  (path for covariance; init to 0).
 *   - z_try    : length n  (standard normal sample storage).
 *   - y_try    : length n  (scaled step y = sqrt(c_diag) ⊙ z_try).
 *   - store_y  : length mu*n (buffer to store μ best y vectors of current generation).
 *   - fit_buf  : length lambda (fitnesses of the λ offspring in current generation).
 *   - idx      : length lambda (indices 0..lambda-1 used for sorting by fitness).
 */
typedef struct {
    int n;
    sep_params_t p;

    /* Buffers (caller‑owned) */
    float *restrict x;
    float *restrict x_try;
    float *restrict c_diag;
    float *restrict p_sigma;
    float *restrict p_c;
    float *restrict z_try;
    float *restrict y_try;
    float *restrict store_y; /* μ × n, row‑major: y_i stored at &store_y[i*n] */
    float *restrict fit_buf;
    int   *restrict idx;

    /* RNG cache (Box‑Muller) */
    int have_spare;
    float spare;

    /* State */
    float sigma;
    float best_f;
    uint32_t iter_total;      /**< # of ask–tell calls so far. */
    uint32_t gen;             /**< Generation counter. */
    int k_in_gen;             /**< How many offspring evaluated in current generation [0..λ]. */
    int have_initial;         /**< 0 until sep_cmaes_tell_initial() is called. */

    /* Derived */
    float mu_w;               /**< Effective selection mass: 1/sum w_i^2. */
    float w_sum;              /**< Sum of weights (should be 1). */
    float chi_n;              /**< E||N(0,I)|| for CSA. */

    /* μ-best cache (no malloc, μ<=32 assumed) */
    float best_fit_mu[32];
    int mu_filled;
} sep_cmaes_t;

/* ============================= API ===================================== */

/**
 * @brief Initialize SEP‑CMA‑ES with caller‑provided buffers (no malloc).
 *
 * @param es        Handle (zero-/stack‑allocated). Will be fully written.
 * @param n         Dimension (≥1).
 * @param x         Mean vector (length n). Will be modified in place.
 * @param x_try     Offspring workspace (length n).
 * @param c_diag    Diagonal variances (length n). If NULL, treated as 1.0 and not updated.
 * @param p_sigma   Path for sigma (length n).
 * @param p_c       Path for covariance (length n).
 * @param z_try     Std‑normal sample buffer (length n).
 * @param y_try     Step buffer y = sqrt(c_diag) ⊙ z (length n).
 * @param store_y   Buffer for μ best steps of current generation (length μ*n).
 * @param fit_buf   Fitness buffer for λ offspring (length λ).
 * @param idx       Index buffer for sorting (length λ).
 * @param params    Parameters; sensible defaults if NULL or fields ≤0 where relevant.
 */
void sep_cmaes_init(sep_cmaes_t *es, int n,
                    float *restrict x, float *restrict x_try,
                    float *restrict c_diag,
                    float *restrict p_sigma, float *restrict p_c,
                    float *restrict z_try, float *restrict y_try,
                    float *restrict store_y,
                    float *restrict fit_buf, int *restrict idx,
                    const sep_params_t *params);

/**
 * @brief Provide initial fitness f(x) of the mean vector after init.
 */
void sep_cmaes_tell_initial(sep_cmaes_t *es, float f0);

/**
 * @brief Sample and return the next candidate x' (offspring) into `x_try` (clamped to bounds if provided).
 *
 * One candidate per call. After λ calls to ::sep_cmaes_tell() (per generation), internal parameters are updated.
 *
 * @return Pointer to candidate of length n, or NULL if called before ::sep_cmaes_tell_initial().
 */
const float *sep_cmaes_ask(sep_cmaes_t *es);

/**
 * @brief Tell SEP‑CMA‑ES the candidate's fitness and perform selection bookkeeping.
 *
 * When λ candidates have been told within a generation, this performs the mean, covariance, and sigma updates.
 *
 * @param es     Handle.
 * @param f_try  Fitness of the most recent candidate returned by ask().
 * @return       Current best fitness after potential improvement.
 */
float sep_cmaes_tell(sep_cmaes_t *es, float f_try);

/* Convenience one‑shot step (ask→objective→tell) */
typedef float (*sep_objective_fn)(const float *x, int n, void *userdata);
/**
 * @brief One‑shot evaluation helper around ask–tell (the objective remains outside the library).
 */
float sep_cmaes_step(sep_cmaes_t *es, sep_objective_fn fn, void *userdata);

/* Inline getters */
static inline const float *sep_cmaes_get_x(const sep_cmaes_t *es) { return es->x; }
static inline float sep_cmaes_sigma(const sep_cmaes_t *es) { return es->sigma; }
static inline uint32_t sep_cmaes_iterations(const sep_cmaes_t *es) { return es->iter_total; }
static inline uint32_t sep_cmaes_generation(const sep_cmaes_t *es) { return es->gen; }
static inline int sep_cmaes_ready(const sep_cmaes_t *es) { return es && es->have_initial; }

#ifdef __cplusplus
}
#endif

#endif /* POGO_UTILS_SEP_CMAES_H */

