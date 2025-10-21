/**
 * @file oneplusone_es.h
 * @brief (1+1)-ES in strict ask–tell form for float parameter arrays.
 *
 * @details
 * This variant removes any objective callback from the library and enforces
 * an ask–tell usage:
 *   1) Call es1p1_init(...), set your initial parent vector in `x`.
 *   2) Evaluate the initial fitness f(x) in your code and call
 *      es1p1_tell_initial(es, f0).
 *   3) Repeatedly:
 *        - const float* x_try = es1p1_ask(es);
 *        - float f_try = objective(x_try);
 *        - es1p1_tell(es, f_try);
 *
 * Step-size `sigma` is adapted online via an EWMA 1/5th-success rule.
 * - No malloc: caller provides all buffers (x, x_try, optional lo/hi bounds).
 * - Supports per-dimension bounds.
 * - Minimization or maximization via `mode`.
 */
#ifndef POGO_UTILS_ONEPLUSONE_ES_H
#define POGO_UTILS_ONEPLUSONE_ES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/** Optimization direction. */
typedef enum {
    ES1P1_MINIMIZE = 0,
    ES1P1_MAXIMIZE = 1
} es1p1_mode_t;

/** Tunable parameters. */
typedef struct {
    es1p1_mode_t mode;     /**< Minimize or maximize. */
    float sigma0;          /**< Initial step-size (e.g., 0.1f). */
    float sigma_min;       /**< Lower bound on sigma (e.g., 1e-6f). */
    float sigma_max;       /**< Upper bound on sigma (e.g., 1.0f). */
    float s_target;        /**< Target success rate (default 0.2f). */
    float s_alpha;         /**< EWMA smoothing (e.g., 0.2f). */
    float c_sigma;         /**< Step-size learning rate (0 => auto=0.6/sqrt(n)). */
} es1p1_params_t;

/** Internal state (POD). */
typedef struct {
    // Problem definition
    int n;

    // Parameters
    es1p1_params_t p;

    // Buffers (owned by caller)
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
    uint32_t iter;           /**< Number of ask–tell pairs performed. */
    int have_initial;        /**< 0 until es1p1_tell_initial() is called. */
    int have_candidate;      /**< 1 after ask(), consumed by tell(). */
} es1p1_t;

/**
 * @brief Initialize the (1+1)-ES on caller-provided buffers.
 *
 * @param es    ES handle (zero-/stack-allocated).
 * @param n     Dimension (>=1).
 * @param x     Parent vector (length n). Will be modified in place.
 * @param x_try Offspring workspace (length n).
 * @param lo    Optional per-coordinate lower bounds (nullable).
 * @param hi    Optional per-coordinate upper bounds (nullable).
 * @param params ES parameters; sensible defaults if NULL.
 *
 * @post  You MUST call es1p1_tell_initial(es, f0) once before the first ask().
 */
void es1p1_init(es1p1_t *es, int n,
                float *restrict x, float *restrict x_try,
                const float *restrict lo, const float *restrict hi,
                const es1p1_params_t *params);

/**
 * @brief Provide the initial fitness f(x) of the parent vector after init.
 */
void es1p1_tell_initial(es1p1_t *es, float f0);

/**
 * @brief Sample and return a new offspring candidate x' into the bound workspace.
 *
 * Generates x_try ~ N(x, sigma^2 I), clamps to bounds if provided, stores it
 * internally, and returns a pointer to `x_try` for evaluation by the caller.
 *
 * @return Pointer to the candidate vector to evaluate (length n), or NULL if
 *         called before es1p1_tell_initial().
 */
const float *es1p1_ask(es1p1_t *es);

/**
 * @brief Tell the ES the candidate's fitness and perform selection + sigma update.
 *
 * @param es     ES handle.
 * @param f_try  Fitness of the last candidate returned by ask().
 * @return       Current best fitness after potential replacement.
 */
float es1p1_tell(es1p1_t *es, float f_try);

/*** Inline getters *********************************************************/
static inline const float *es1p1_get_x(const es1p1_t *es) { return es->x; }
static inline float        es1p1_best_f(const es1p1_t *es) { return es->best_f; }
static inline float        es1p1_sigma(const es1p1_t *es)  { return es->sigma; }
static inline uint32_t     es1p1_iterations(const es1p1_t *es) { return es->iter; }
static inline int          es1p1_ready(const es1p1_t *es) { return es && es->have_initial; }

#ifdef __cplusplus
}
#endif

/**
 * @brief Convenience one-shot step that performs ask→objective→tell.
 *
 * Thin wrapper around ask–tell. The objective remains outside the library.
 */
typedef float (*es1p1_objective_fn)(const float *x, int n, void *userdata);
float es1p1_step(es1p1_t *es, es1p1_objective_fn fn, void *userdata);

#endif /* POGO_UTILS_ONEPLUSONE_ES_H */

