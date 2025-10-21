/**
 * @file spsa.h
 * @brief Simultaneous Perturbation Stochastic Approximation (SPSA), strict ask–tell API (no malloc).
 *
 * Key change vs previous draft:
 *   - Rademacher vector Δ is now stored in a separate caller-provided buffer (no aliasing with x_work).
 */
#ifndef POGO_UTILS_SPSA_H
#define POGO_UTILS_SPSA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    SPSA_MINIMIZE = 0,
    SPSA_MAXIMIZE = 1
} spsa_mode_t;

/** Tunable parameters (see Spall 1998/2003). */
typedef struct {
    float a;       /**< Step-size numerator (typ. ~ scale of x). */
    float c;       /**< Perturbation numerator (≈ 1–10% of param range). */
    float A;       /**< Stability offset (~10% of total iters). */
    float alpha;   /**< Step-size exponent (≈0.602). */
    float gamma;   /**< Perturb exponent (≈0.101). */
    float g_clip;  /**< L∞ gradient clip (0 => disabled). */
} spsa_params_t;

/** Optimizer state (no dynamic allocation). */
typedef struct {
    int n;
    spsa_mode_t mode;
    spsa_params_t p;

    /* Caller-owned buffers (must be length n) */
    float *restrict x;        /**< Parameter vector (updated in-place). */
    float *restrict x_work;   /**< Probe workspace (x_plus/x_minus). */
    int   *restrict delta;    /**< Rademacher ±1 vector, separate buffer. */
    const float *restrict lo; /**< Optional lower bounds (nullable). */
    const float *restrict hi; /**< Optional upper bounds (nullable). */

    /* Internal */
    uint32_t k;         /**< Completed iterations. */
    float ck, ak;       /**< Current gains. */
    int phase;          /**< 0=>next ask is x_plus, 1=>x_minus. */
    float f_plus;       /**< Stores f(x_plus) between tells. */
    int have_f_plus;
    int have_probe;
} spsa_t;

/** Safe baseline parameters. */
static inline spsa_params_t spsa_default_params(void) {
    spsa_params_t P = {
        .a = 0.1f,
        .c = 0.05f,   /* better default for [-2,2]-scaled problems */
        .A = 10.0f,
        .alpha = 0.602f,
        .gamma = 0.101f,
        .g_clip = 0.0f
    };
    return P;
}

/**
 * @brief Initialize SPSA (no malloc).
 *
 * @param s      Optimizer handle.
 * @param n      Dimension (>=1).
 * @param x      Parameters (len n), updated in-place after each full iteration.
 * @param x_work Workspace for probes (len n).
 * @param delta  Rademacher ±1 vector (len n), kept across the +/− phases.
 * @param lo     Optional lower bounds (nullable).
 * @param hi     Optional upper bounds (nullable).
 * @param P      SPSA parameters (use spsa_default_params() as a start).
 * @param mode   MINIMIZE or MAXIMIZE.
 */
void spsa_init(spsa_t *s, int n,
               float *restrict x, float *restrict x_work, int *restrict delta,
               const float *restrict lo, const float *restrict hi,
               const spsa_params_t *P, spsa_mode_t mode);

/**
 * @brief Return the next probe vector to evaluate (x_plus or x_minus), or NULL if you need to call tell() first.
 */
const float *spsa_ask(spsa_t *s);

/**
 * @brief Provide f(x_probe) of the last ask(). On the second tell (− phase), applies the update and returns new k.
 */
uint32_t spsa_tell(spsa_t *s, float f_probe);

/** Convenience one-shot (does both probes internally). */
typedef float (*spsa_objective_fn)(const float *x, int n, void *userdata);
uint32_t spsa_step(spsa_t *s, spsa_objective_fn fn, void *userdata);

/* Small getters */
static inline uint32_t spsa_iterations(const spsa_t *s) { return s ? s->k : 0u; }
static inline const float *spsa_get_x(const spsa_t *s) { return s ? s->x : (const float*)0; }
static inline float spsa_ak(const spsa_t *s) { return s ? s->ak : 0.0f; }
static inline float spsa_ck(const spsa_t *s) { return s ? s->ck : 0.0f; }

#ifdef __cplusplus
}
#endif

#endif /* POGO_UTILS_SPSA_H */

