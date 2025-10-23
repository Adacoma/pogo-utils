/**
 * @file optimizer.h
 * @brief Unified ask–tell optimization interface + factory for swarm robotics.
 *
 * @details
 * This header defines a small abstraction layer that wraps several optimizers
 * already used in the codebase under a single, tight C API:
 *   - Centralized/on‑robot (local): (1+1)-ES, SPSA, PGPE, SEP‑CMA‑ES
 *   - Decentralized/across robots: Social Learning (mEDEA‑style)
 *
 * All wrapped optimizers use a strict ask–tell paradigm. This layer lets you
 * switch algorithms by changing a single enum value at init, without changing
 * the rest of your control loop.
 *
 * Design rules:
 *   - C11, no dynamic allocation here (callers provide all buffers).
 *   - Zero‑overhead wrappers around the underlying libraries.
 *   - Bounds and MIN/MAX direction are forwarded to the selected backend.
 *   - A single \p aux_scale value can be passed to ::opt_ask(): it is used by
 *     Social Learning as the mutation scale driver (e.g., current window loss)
 *     and ignored by other algorithms.
 */
#ifndef POGO_UTILS_OPTIMIZER_H
#define POGO_UTILS_OPTIMIZER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/* ==== Backends (forward declarations from existing headers) ============= */
#include "oneplusone_es.h"   /* ES1P1 */
#include "spsa.h"            /* SPSA  */
#include "pgpe.h"            /* PGPE  */
#include "sep_cmaes.h"       /* SEP‑CMA‑ES */
#include "social_learning.h" /* Social Learning / mEDEA */
#include "hit.h"             /* Social Learning HIT */

/* ==== Public types ====================================================== */

/** Optimization algorithm choice. Change this single enum to switch. */
typedef enum {
    OPT_ES1P1 = 0,       /**< (1+1)-ES (local, single‑parent evolution). */
    OPT_SPSA,            /**< SPSA (gradient‑free stochastic approx). */
    OPT_PGPE,            /**< PGPE (policy gradient with parameter noise). */
    OPT_SEP_CMAES,       /**< Separable CMA‑ES (diag‑covariance). */
    OPT_SOCIAL_LEARNING, /**< mEDEA‑style decentralized social learning. */
    OPT_HIT,             /**< HIT‑style decentralized social learning. */
} opt_algo_t;

/** Shared minimization/maximization direction. */
typedef enum { OPT_MINIMIZE = 0, OPT_MAXIMIZE = 1 } opt_mode_t;

typedef struct {
    /* Discriminant */
    opt_algo_t algo;
    opt_mode_t mode;
    int n;

    /* Backends (no heap) */
    union {
        es1p1_t     es1p1;
        spsa_t      spsa;
        pgpe_t      pgpe;
        sep_cmaes_t sep;
        sl_t        sl;
        hit_t       hit;
    } o;

    /* Cached pointer to the current parent/mean vector for opt_get_x() */
    const float *x_ptr;
} opt_t;

/** Caller‑provided buffers, specific to each backend. Supply only the one you use. */
typedef struct {
    int n; /**< Dimension (>=1), repeated for convenience. */

    union {
        struct { /* (1+1)-ES */
            float *x; float *x_try; const float *lo; const float *hi;
            es1p1_params_t params; /* Optional: set .c_sigma<=0 for auto */
        } es1p1;

        struct { /* SPSA */
            float *x; float *x_work; int *delta; const float *lo; const float *hi;
            spsa_params_t params; /* use spsa_default_params() as a start */
        } spsa;

        struct { /* PGPE */
            float *mu; float *sigma; float *x_try; float *eps; const float *lo; const float *hi;
            pgpe_params_t params; /* provide sensible η and bounds for σ */
        } pgpe;

        struct { /* SEP‑CMA‑ES */
            float *x; float *x_try; float *c_diag; float *p_sigma; float *p_c;
            float *z_try; float *y_try; float *store_y; float *fit_buf; int *idx;
            sep_params_t params; /* set lambda, mu, sigma0, (lo/hi via params) */
        } sep;

        struct { /* Social Learning (mEDEA) */
            float *x; float *x_try; const float *lo; const float *hi;
            float *repo_X; float *repo_F; uint32_t *repo_epoch; uint16_t *repo_from; uint16_t capacity;
            sl_params_t params; /* includes MIN/MAX + repo size, etc. */
        } sl;

        struct { /* Social Learning (HIT) */
            float *x; float *x_try; const float *lo; const float *hi;
            float *repo_X; float *repo_F; uint32_t *repo_epoch; uint16_t *repo_from; uint16_t capacity;
            hit_params_t params;
        } hit;
    } u;
} opt_buffers_t;



#define gen_opt_internals_t(O,D) \
    union { \
        /* (1+1)-ES */ \
        struct { \
            float x[D], x_try[D]; \
            es1p1_params_t P; \
        } es1p1; \
 \
        /* SPSA */ \
        struct { \
            float x[D], x_work[D]; \
            int   delta[D]; \
            spsa_params_t P; \
        } spsa; \
 \
        /* PGPE */ \
        struct { \
            float mu[D], sigma[D], x_try[D], eps[D]; \
            pgpe_params_t P; \
        } pgpe; \
 \
        /* SEP-CMA-ES */ \
        struct { \
            float x[D], x_try[D]; \
            float c_diag[D], p_sigma[D], p_c[D]; \
            float z_try[D], y_try[D], store_y[D]; \
            float fit_buf[LAMBDA_MAX]; int idx[LAMBDA_MAX]; \
            sep_params_t P; \
        } sep; \
 \
        /* Social Learning (single-robot demo repo) */ \
        struct { \
            float x[D], x_try[D]; \
            float repo_X[REPO_CAP * D]; \
            float repo_F[REPO_CAP]; \
            uint32_t repo_epoch[REPO_CAP]; \
            uint16_t repo_from[REPO_CAP]; \
            sl_params_t P; \
        } sl; \
 \
        /* Social Learning (HIT) */ \
        struct { \
            float x[D], x_try[D]; \
            float repo_X[REPO_CAP * D]; \
            float repo_F[REPO_CAP]; \
            uint32_t repo_epoch[REPO_CAP]; \
            uint16_t repo_from[REPO_CAP]; \
            hit_params_t P; \
        } hit; \
    } O; 


/* ==== Factory & unified API ============================================= */

/** Create/initialize an optimizer instance. No allocations are performed here. */
void opt_init(opt_t *opt, opt_algo_t algo, opt_mode_t mode, const opt_buffers_t *buf);

/** Provide initial fitness where required ((1+1)-ES, SEP‑CMA‑ES, Social Learning). Others ignore. */
void opt_tell_initial(opt_t *opt, float f0);

/**
 * @brief Ask for the next candidate parameters to evaluate.
 *
 * @param opt       Optimizer handle.
 * @param aux_scale Optional scale used by Social Learning as mutation driver
 *                  (e.g., current loss in MIN mode). Ignored for other algos.
 * @return Pointer to candidate vector of length n, or NULL if called before
 *         ::opt_tell_initial() on algorithms that require it.
 */
const float *opt_ask(opt_t *opt, float aux_scale);

/** Tell the fitness of the last ask(); returns the current best/parent fitness if relevant. */
float opt_tell(opt_t *opt, float f);

/** Lightweight helpers. */
int          opt_ready(const opt_t *opt);      /**< 1 if ready to ask. */
uint32_t     opt_iterations(const opt_t *opt); /**< Backend‑specific iteration counter. */
const float *opt_get_x(const opt_t *opt);      /**< Current parent/mean vector address. */
opt_algo_t   opt_algo(const opt_t *opt);       /**< Which algorithm this instance wraps. */
int          opt_dim(const opt_t *opt);        /**< Problem dimension n. */

/**
 * @brief Decentralization hook used only by Social Learning. No‑op for others.
 */
void opt_observe_remote(opt_t *opt, uint16_t from_id, uint32_t epoch,
                           const float *x_remote, float f_adv);

float opt_get_last_advert(const opt_t *opt);

#ifdef __cplusplus
}
#endif

#endif /* POGO_UTILS_OPTIMIZER_H */

