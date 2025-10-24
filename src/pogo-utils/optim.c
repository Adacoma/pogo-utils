/**
 * @file optimizer.c
 * @brief Implementation of a unified ask–tell optimization interface + factory.
 */
#include "optim.h"
#include <string.h>


/* =========================== API ====================================== */

void opt_init(opt_t *opt, opt_algo_t algo, opt_mode_t mode, const opt_buffers_t *buf){
    if (!opt || !buf) return;
    memset(opt, 0, sizeof(*opt));
    opt->algo = algo;
    opt->mode = mode;
    opt->n    = buf->n;

    switch (algo){
    case OPT_ES1P1: {
        es1p1_params_t P = buf->u.es1p1.params; /* struct copy */
        /* Map direction */
        P.mode = (mode == OPT_MINIMIZE) ? ES1P1_MINIMIZE : ES1P1_MAXIMIZE;
        es1p1_init(&opt->o.es1p1, buf->n,
                   buf->u.es1p1.x, buf->u.es1p1.x_try,
                   buf->u.es1p1.lo, buf->u.es1p1.hi,
                   &P);
        opt->x_ptr = buf->u.es1p1.x;
    } break;

    case OPT_SPSA: {
        spsa_params_t P = buf->u.spsa.params; /* struct copy */
        spsa_mode_t M = (mode == OPT_MINIMIZE) ? SPSA_MINIMIZE : SPSA_MAXIMIZE;
        spsa_init(&opt->o.spsa, buf->n,
                  buf->u.spsa.x, buf->u.spsa.x_work, buf->u.spsa.delta,
                  buf->u.spsa.lo, buf->u.spsa.hi,
                  &P, M);
        opt->x_ptr = buf->u.spsa.x;
    } break;

    case OPT_PGPE: {
        pgpe_params_t P = buf->u.pgpe.params; /* struct copy */
        P.mode = (mode == OPT_MINIMIZE) ? PGPE_MINIMIZE : PGPE_MAXIMIZE;
        pgpe_init(&opt->o.pgpe, buf->n,
                  buf->u.pgpe.mu, buf->u.pgpe.sigma,
                  buf->u.pgpe.x_try, buf->u.pgpe.eps,
                  buf->u.pgpe.lo, buf->u.pgpe.hi,
                  &P);
        opt->x_ptr = buf->u.pgpe.mu; /* mean vector is the parent */
    } break;

    case OPT_SEP_CMAES: {
        sep_params_t P = buf->u.sep.params; /* struct copy */
        P.mode = (mode == OPT_MINIMIZE) ? SEP_MINIMIZE : SEP_MAXIMIZE;
        /* Bounds are supplied in P.lo/P.hi as per sep_cmaes.h */
        sep_cmaes_init(&opt->o.sep, buf->n,
                       buf->u.sep.x, buf->u.sep.x_try,
                       buf->u.sep.c_diag,
                       buf->u.sep.p_sigma, buf->u.sep.p_c,
                       buf->u.sep.z_try, buf->u.sep.y_try,
                       buf->u.sep.store_y,
                       buf->u.sep.fit_buf, buf->u.sep.idx,
                       &P);
        opt->x_ptr = buf->u.sep.x;
    } break;

    case OPT_SOCIAL_LEARNING: {
        sl_params_t P = buf->u.sl.params; /* struct copy */
        P.mode = (mode == OPT_MINIMIZE) ? SL_MINIMIZE : SL_MAXIMIZE;
        sl_init(&opt->o.sl, buf->n,
                buf->u.sl.x, buf->u.sl.x_try,
                buf->u.sl.lo, buf->u.sl.hi,
                buf->u.sl.repo_X, buf->u.sl.repo_F,
                buf->u.sl.repo_epoch, buf->u.sl.repo_from,
                buf->u.sl.capacity, &P);
        opt->x_ptr = buf->u.sl.x;
    } break;

    case OPT_HIT: {
        hit_params_t P = buf->u.hit.params; /* struct copy */
        P.mode = (mode == OPT_MINIMIZE) ? HIT_MINIMIZE : HIT_MAXIMIZE;
        hit_init(&opt->o.hit, buf->n,
                buf->u.hit.x, buf->u.hit.x_try,
                buf->u.hit.lo, buf->u.hit.hi,
                buf->u.hit.repo_X, buf->u.hit.repo_F,
                buf->u.hit.repo_epoch, buf->u.hit.repo_from,
                buf->u.hit.capacity, &P);
        opt->x_ptr = buf->u.hit.x;
    } break;
    }
}

void opt_tell_initial(opt_t *opt, float f0){
    if (!opt) return;
    switch (opt->algo){
    case OPT_ES1P1:        es1p1_tell_initial(&opt->o.es1p1, f0); break;
    case OPT_SEP_CMAES:    sep_cmaes_tell_initial(&opt->o.sep, f0); break;
    case OPT_SOCIAL_LEARNING: sl_tell_initial(&opt->o.sl, f0); break;
    case OPT_HIT: hit_tell_initial(&opt->o.hit, f0); break;
    case OPT_SPSA: /* no‑op */ break;
    case OPT_PGPE: /* no‑op */ break;
    }
}

const float *opt_ask(opt_t *opt, float aux_scale){
    if (!opt) return (const float*)0;
    switch (opt->algo){
    case OPT_ES1P1:       return es1p1_ask(&opt->o.es1p1);
    case OPT_SPSA:        return spsa_ask(&opt->o.spsa);
    case OPT_PGPE:        return pgpe_ask(&opt->o.pgpe);
    case OPT_SEP_CMAES:   return sep_cmaes_ask(&opt->o.sep);
    case OPT_SOCIAL_LEARNING: return sl_ask(&opt->o.sl, aux_scale);
    case OPT_HIT:         return hit_ask(&opt->o.hit, aux_scale);
    }
    return (const float*)0;
}

float opt_tell(opt_t *opt, float f){
    if (!opt) return 0.0f;
    switch (opt->algo){
    case OPT_ES1P1:       return es1p1_tell(&opt->o.es1p1, f);
    case OPT_SPSA:        (void)spsa_tell(&opt->o.spsa, f); return f; /* x updated every 2 tells */
    case OPT_PGPE:        pgpe_tell(&opt->o.pgpe, f); return f;       /* μ/σ updated after pair */
    case OPT_SEP_CMAES:   return sep_cmaes_tell(&opt->o.sep, f);
    case OPT_SOCIAL_LEARNING: return sl_tell(&opt->o.sl, f);
    case OPT_HIT:         return hit_tell(&opt->o.hit, f);
    }
    return 0.0f;
}

int opt_ready(const opt_t *opt){
    if (!opt) return 0;
    switch (opt->algo){
    case OPT_ES1P1:       return es1p1_ready(&opt->o.es1p1);
    case OPT_SPSA:        return 1; /* SPSA can ask immediately after init */
    case OPT_PGPE:        return 1; /* PGPE can ask immediately after init */
    case OPT_SEP_CMAES:   return sep_cmaes_ready(&opt->o.sep);
    case OPT_SOCIAL_LEARNING: return opt->o.sl.have_initial; /* tiny leak of internals */
    case OPT_HIT:         return opt->o.hit.have_initial; /* tiny leak of internals */
    }
    return 0;
}

uint32_t opt_iterations(const opt_t *opt){
    if (!opt) return 0u;
    switch (opt->algo){
    case OPT_ES1P1:       return es1p1_iterations(&opt->o.es1p1);
    case OPT_SPSA:        return spsa_iterations(&opt->o.spsa);
    case OPT_PGPE:        return pgpe_iterations(&opt->o.pgpe);
    case OPT_SEP_CMAES:   return sep_cmaes_iterations(&opt->o.sep);
    case OPT_SOCIAL_LEARNING: return opt->o.sl.epoch_local; /* per accepted parent */
    case OPT_HIT:         return opt->o.hit.epoch_local; /* per accepted parent */
    }
    return 0u;
}

const float *opt_get_x(const opt_t *opt){ return opt ? opt->x_ptr : (const float*)0; }
opt_algo_t   opt_algo(const opt_t *opt){ return opt ? opt->algo : OPT_ES1P1; }
int          opt_dim(const opt_t *opt){ return opt ? opt->n : 0; }

void opt_observe_remote(opt_t *opt, uint16_t from_id, uint32_t epoch,
                           const float *x_remote, float f_adv){
    if (!opt) return;
    if (opt->algo == OPT_SOCIAL_LEARNING){
        sl_observe_remote(&opt->o.sl, from_id, epoch, x_remote, f_adv);
    } else if (opt->algo == OPT_HIT){
        hit_observe_remote(&opt->o.hit, from_id, epoch, x_remote, f_adv);
    }
}

float opt_get_last_advert(const opt_t *opt) {
    if (!opt) return 0.0f;
    if (opt->algo == OPT_SOCIAL_LEARNING) {
        return sl_get_last_advert(&opt->o.sl);
    } else if (opt->algo == OPT_HIT) {
        return hit_get_last_advert(&opt->o.hit);
    } else {
        return 0.0f;
    }
}

