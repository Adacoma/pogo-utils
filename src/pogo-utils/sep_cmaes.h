/**
 * @file sep_cmaes.h
 * @brief Separable CMA-ES (diagonal covariance) over float arrays — malloc-free and online-capable.
 *
 * @details
 * This is a compact, embedded-friendly implementation of **sep-CMA-ES** (Ros & Hansen, 2008),
 * where the covariance matrix is constrained to be diagonal. It runs as a finite-state
 * optimizer suitable for control loops:
 *
 *   - Each call to @ref sep_cmaes_step can generate/evaluate K offspring (K = evals_per_tick).
 *   - When a full generation of λ offspring has been evaluated, the algorithm performs the
 *     usual CMA updates of m, σ, and the diagonal C, then starts the next generation.
 *
 * Design goals:
 *  - **No malloc**: all memory (mean, offspring, fitness, diagC, paths, scratch, etc.)
 *    is provided by the user at init.
 *  - **Bounds** (optional) are per-coordinate; candidates are clamped.
 *  - **Minimize or maximize** controlled by a mode flag.
 *  - **Online optimization**: partial progress per control tick (like your 1+1-ES flow).
 *
 * References (for parameter choices and update equations):
 *   N. Hansen & A. Ostermeier (2001), CMA-ES
 *   R. Ros & N. Hansen (2008), "A Simple Modification in CMA-ES..." (Separable CMA-ES)
 *
 * Typical usage:
 *  1) Fill @ref sep_cmaes_params_t (or pass NULL for sensible defaults).
 *  2) Bind buffers and call @ref sep_cmaes_init.
 *  3) In each control step, call @ref sep_cmaes_step.
 *  4) Read current mean with @ref sep_cmaes_mean() and best fitness with @ref sep_cmaes_best_f().
 */

#ifndef POGO_UTILS_SEP_CMAES_H
#define POGO_UTILS_SEP_CMAES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/** Objective callback: returns fitness f(x). Lower is better if mode=MIN. */
typedef float (*sep_cmaes_objective_fn)(const float *restrict x, int n, void *user);

/** Optimization direction. */
typedef enum {
    SEP_CMAES_MINIMIZE = 0,
    SEP_CMAES_MAXIMIZE = 1
} sep_cmaes_mode_t;

/**
 * @brief Tunable high-level parameters. If any field is zero, a robust default is used.
 *
 * Notes:
 *  - If lambda==0, it defaults to 4 + floor(3 * ln(n)); mu=floor(lambda/2).
 *  - Weights are log-based (Hansen default) and normalized to sum(w)=1.
 *  - Learning rates (cs, ds, cc, c1_sep, cmu_sep) follow CMA defaults adapted to sep-CMA-ES.
 */
typedef struct {
    sep_cmaes_mode_t mode;      /**< Minimize or maximize. */
    int   lambda;               /**< Offspring per generation (0 => default). */
    int   mu;                   /**< Parents for recombination (0 => default=floor(lambda/2)). */
    float sigma0;               /**< Initial global step-size σ0 (e.g., 0.3f). */
    float sigma_min;            /**< σ lower clamp (e.g., 1e-8f). */
    float sigma_max;            /**< σ upper clamp (e.g., 10.0f). */
    float cs;                   /**< Cumulation for σ-path (e.g., ~0.3). */
    float ds;                   /**< Damping for σ (e.g., ~1 + 2*max(0, sqrt(mu_w)-1)+cs). */
    float cc;                   /**< Cumulation for covariance path (e.g., ~2/(n+2)). */
    float c1_sep;               /**< Rank-one learning rate for diagC. */
    float cmu_sep;              /**< Rank-μ learning rate for diagC. */
    int   evals_per_tick;       /**< How many offspring to evaluate per control step (>=1). */
} sep_cmaes_params_t;

/**
 * @brief Optimizer state — POD with user-bound buffers. No dynamic allocation.
 *
 * User must provide all buffers listed below. Sizes:
 *  - mean m:                  n
 *  - diagC, invsqrtC, ps, pc: n
 *  - z_work, y_work:          n (scratch)
 *  - cand_X:                  lambda * n  (flattened: candidate k at &cand_X[k*n])
 *  - fitness:                 lambda
 *  - idx:                     lambda      (indices for partial sort)
 *  - weights:                 mu          (precomputed at init)
 *
 * Bounds are optional (per-dimension). If non-NULL, candidates are clamped into [lo, hi].
 */
typedef struct {
    /* Problem definition */
    int n;
    sep_cmaes_objective_fn fn;
    void *user;

    /* Parameters */
    sep_cmaes_params_t p;
    sep_cmaes_mode_t mode;
    int lambda;
    int mu;

    /* Recombination weights (sum to 1), effective mu = 1/sum(w^2) */
    float mu_eff;

    /* Global strategy parameters */
    float sigma;
    float chi_n;        /* E||N(0,I)|| in n dims, used in σ update */

    /* Generation bookkeeping */
    int k;              /* next offspring index to generate/evaluate in current generation */
    uint32_t gen;       /* completed generations */
    uint64_t evals;     /* total function evaluations */

    /* RNG (Box–Muller spare) */
    int   have_spare;
    float spare;

    /* ==== User-provided buffers (no malloc) ==== */
    float *restrict m;          /**< Mean vector (dim n). Updated in-place. */
    float *restrict diagC;      /**< Diagonal covariance (dim n), initialized to 1. */
    float *restrict invsqrtC;   /**< 1/sqrt(diagC) (dim n). */
    float *restrict ps;         /**< σ-path (dim n). */
    float *restrict pc;         /**< Covariance path (dim n). */
    float *restrict z_work;     /**< Scratch z ~ N(0,I) (dim n). */
    float *restrict y_work;     /**< Scratch y = sqrt(diagC) ⊙ z (dim n). */

    /* Offspring storage for current generation */
    float *restrict cand_X;     /**< Flattened λ×n array (row-major). */
    float *restrict fitness;    /**< λ fitness values. */
    int   *restrict idx;        /**< λ indices for sorting/selecting μ-best. */

    /* Recombination weights (length μ). Will be filled at init if non-NULL. */
    float *restrict weights;

    /* Optional bounds */
    const float *restrict lo;
    const float *restrict hi;

    /* Best-so-far (for easy monitoring) */
    float best_f;
} sep_cmaes_t;

/**
 * @brief Initialize sep-CMA-ES with user-provided buffers and parameters.
 *
 * @param es       Optimizer handle (zeroed by caller or stack-allocated).
 * @param n        Dimension (>=1).
 * @param m        Mean vector (length n), will be modified in-place.
 * @param diagC    Diagonal covariance (length n), set to 1 if you pass NULL (but pointer is required).
 * @param invsqrtC 1/sqrt(diagC) (length n), will be filled by init.
 * @param ps, pc   Evolution paths (length n), will be zeroed.
 * @param z_work, y_work Scratch vectors (length n) for sampling and updates.
 * @param cand_X   Offspring matrix (length lambda*n).
 * @param fitness  Fitness array (length lambda).
 * @param idx      Index array (length lambda).
 * @param weights  Recombination weights (length mu). If NULL, defaults will be computed into it (must not be NULL).
 * @param lo, hi   Optional per-dimension bounds (nullable).
 * @param fn, user Objective function and user pointer.
 * @param params   Strategy parameters (nullable -> sensible defaults).
 */
void sep_cmaes_init(sep_cmaes_t *es, int n,
                    float *restrict m,
                    float *restrict diagC, float *restrict invsqrtC,
                    float *restrict ps, float *restrict pc,
                    float *restrict z_work, float *restrict y_work,
                    float *restrict cand_X, float *restrict fitness, int *restrict idx,
                    float *restrict weights,
                    const float *restrict lo, const float *restrict hi,
                    sep_cmaes_objective_fn fn, void *user,
                    const sep_cmaes_params_t *params);

/**
 * @brief Perform one control-tick worth of work: generate/evaluate up to `evals_per_tick` offspring.
 *
 * When a full generation (λ offspring) has been evaluated, this function performs the CMA updates
 * (m, σ, diagC) and starts the next generation. It returns the current best fitness (best-so-far).
 *
 * @return Current best fitness after the batch.
 */
float sep_cmaes_step(sep_cmaes_t *es);

/** @brief Current mean vector (the incumbent solution m). */
static inline const float *sep_cmaes_mean(const sep_cmaes_t *es) { return es->m; }

/** @brief Current step-size σ. */
static inline float sep_cmaes_sigma(const sep_cmaes_t *es) { return es->sigma; }

/** @brief Best-so-far fitness. */
static inline float sep_cmaes_best_f(const sep_cmaes_t *es) { return es->best_f; }

/** @brief Current generation number (completed). */
static inline uint32_t sep_cmaes_generation(const sep_cmaes_t *es) { return es->gen; }

/** @brief Total number of function evaluations. */
static inline uint64_t sep_cmaes_evals(const sep_cmaes_t *es) { return es->evals; }

#ifdef __cplusplus
}
#endif

#endif /* POGO_UTILS_SEP_CMAES_H */

