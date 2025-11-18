// optim.h
#pragma once
#ifdef __cplusplus
extern "C" { 
#endif

#include <stdint.h>
#include <stddef.h>
#include "tiny_alloc.h"

#include "oneplusone_es.h"
#include "spsa.h"
#include "pgpe.h"
#include "sep_cmaes.h"
#include "social_learning.h"
#include "hit.h"

/* ================= Algorithms & modes ================= */

typedef enum {
    OPT_ES1P1 = 0,
    OPT_SPSA,
    OPT_PGPE,
    OPT_SEP_CMAES,
    OPT_SOCIAL_LEARNING,
    OPT_HIT,
} opt_algo_t;

typedef enum { OPT_MINIMIZE = 0, OPT_MAXIMIZE = 1 } opt_mode_t;

/* ================== User config with sensible defaults ===================

   Call opt_default_cfg(algo, n) to get a ready-to-use baseline, then override
   what you need before opt_create(). Anything left as default stays consistent.
*/

typedef struct {
    /* common toggles */
    int use_defaults;   /* if non-zero, per-algo defaults are applied first */

    union {
        es1p1_params_t es1p1;
        spsa_params_t  spsa;
        pgpe_params_t  pgpe;
        sep_params_t   sep;
        sl_params_t    sl;
        hit_params_t   hit;
    } P;

    /* Extra knobs that affect buffer sizing (optional; 0 => default) */
    struct {
        /* SEP-CMA-ES */
        uint16_t lambda;   /* population size; if 0 use default */
        uint16_t mu;       /* parents; if 0 use default */

        /* Social learning (SL/HIT) */
        uint16_t repo_capacity;  /* if 0 use default */
    } sz;
} opt_cfg_t;

/* Compute a default configuration for (algo, n). You can then edit fields. */
opt_cfg_t opt_default_cfg(opt_algo_t algo, int n);

/* ========================= Opaque optimizer ============================== */

typedef struct opt_t opt_t;

/* Create optimizer, allocating its internal work buffers from tiny_alloc.
   - lo/hi may be NULL => default to [-1, +1] for each dim.
   - cfg may be NULL => use defaults from opt_default_cfg(algo, n).
   Returns true on success, false if allocation or params are invalid. */
int  opt_create(opt_t **self_out, tiny_alloc_t *ta, int n,
                opt_algo_t algo, opt_mode_t mode,
                const float *lo, const float *hi,
                const opt_cfg_t *cfg);

/* Destroy optimizer: frees the tiny_alloc blocks it owns (no-op if NULL). */
void opt_destroy(opt_t *self);

/* =============== Unified ask–tell interface (unchanged semantics) ======= */

void          opt_tell_initial(opt_t *self, float f0);
const float  *opt_ask(opt_t *self, float aux_scale);
float         opt_tell(opt_t *self, float f);

int           opt_ready(const opt_t *self);
uint32_t      opt_iterations(const opt_t *self);
const float  *opt_get_x(const opt_t *self);
opt_algo_t    opt_algo(const opt_t *self);
int           opt_dim(const opt_t *self);

/* Social learning hooks (no-op for non-SL/HIT). */
void  opt_observe_remote(opt_t *self, uint16_t from_id, uint32_t epoch,
                         const float *x_remote, float f_adv, float alpha_remote);

/* Current transfer rate α (only meaningful for OPT_HIT). */
float opt_get_alpha(const opt_t *self);

/* Last advertised fitness (SL/HIT), 0 otherwise. */
float opt_get_last_advert(const opt_t *self);

/* Set the current design vector before priming (writes backend's mean/parent). */
void opt_set_x(opt_t *self, const float *x);

/* Randomize current design vector uniformly within bounds. */
void opt_randomize_x(opt_t *self, uint32_t seed);

#ifdef __cplusplus
}
#endif

