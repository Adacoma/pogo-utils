/**
 * @file example_optim.c
 * @brief Sphere minimization using the unified optimizer factory (ask–tell).
 *
 * Choose backend at compile time:
 *   -DOPT_EXAMPLE_ALGO=0  -> OPT_ES1P1
 *   -DOPT_EXAMPLE_ALGO=1  -> OPT_SPSA
 *   -DOPT_EXAMPLE_ALGO=2  -> OPT_PGPE
 *   -DOPT_EXAMPLE_ALGO=3  -> OPT_SEP_CMAES
 *   -DOPT_EXAMPLE_ALGO=4  -> OPT_SOCIAL_LEARNING
 *
 * Example build (adjust sources as needed):
 *   gcc -std=c11 -O3 -DOPT_EXAMPLE_ALGO=0 example_optim.c \
 *       optim.c oneplusone_es.c spsa.c pgpe.c sep_cmaes.c social_learning.c \
 *       -o example_optim
 */

#include "pogobase.h"
#include "pogo-utils/optim.h"   /* unified factory/API */
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <float.h>

#ifndef D
#define D 8
#endif

#ifndef OPT_EXAMPLE_ALGO
#define OPT_EXAMPLE_ALGO 5 /* default: OPT_ES1P1 */
#endif

#define LAMBDA_MAX 4
#define REPO_CAP   8

/* Map compile-time integer -> enum */
static inline opt_algo_t chosen_algo(void){
    switch (OPT_EXAMPLE_ALGO){
    default:
    case 0: return OPT_ES1P1;
    case 1: return OPT_SPSA;
    case 2: return OPT_PGPE;
    case 3: return OPT_SEP_CMAES;
    case 4: return OPT_SOCIAL_LEARNING;
    case 5: return OPT_HIT;
    }
}

/* ====================== Objective: Sphere (minimize) ==================== */
static float sphere_fn(const float *restrict x, int n){
    float s = 0.0f;
    for (int i = 0; i < n; ++i) s += x[i] * x[i];
    return s;
}

/* ============================ USERDATA ================================== */
typedef struct {
    opt_t         opt;        /* now complete thanks to header patch */
    opt_buffers_t buf;

    float f0, f_best;
    float lo[D], hi[D];

    gen_opt_internals_t(internals, D);

    /* Timers */
    uint32_t last_print_ms;
    uint32_t last_tx_ms;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA)

/* ==== Messaging ========================================================== */
/* Minimal advert payload: epoch + D floats + adv fitness. */
typedef struct __attribute__((__packed__)) {
    uint16_t sender_id;
    uint32_t epoch;
    float    x[D];
    float    f_adv;
} sl_msg_t;
#define SL_MSG_BYTES ((uint16_t)sizeof(sl_msg_t))


/* ==== Messaging RX ======================================================= */
static void on_rx(message_t *mr){
    if (mr->header.payload_length < SL_MSG_BYTES) return;

    /* Copy out of the packed payload into aligned locals */
    sl_msg_t msg;
    memcpy(&msg, mr->payload, sizeof(sl_msg_t));

    if (msg.sender_id == pogobot_helper_getid()) return; /* ignore own */

    float x_buf[D];
    memcpy(x_buf, msg.x, sizeof(x_buf));

    // Decentralization hook used only by Social Learning. No‑op for others.
    opt_observe_remote(&mydata->opt, msg.sender_id, msg.epoch, x_buf, msg.f_adv);
}


/* ==== Messaging TX ======================================================= */
static bool on_tx(void){
    uint32_t now = current_time_milliseconds();
    const uint32_t period_ms = 200; /* 5 Hz beacons */
    if (now - mydata->last_tx_ms < period_ms) return false;

    sl_msg_t m;
    m.sender_id = pogobot_helper_getid();
    m.epoch     = opt_iterations(&mydata->opt);
    memcpy(m.x, opt_get_x(&mydata->opt), sizeof(float)*D);
    m.f_adv = opt_get_last_advert(&mydata->opt);

    mydata->last_tx_ms = now;
    return pogobot_infrared_sendShortMessage_omni((uint8_t*)&m, SL_MSG_BYTES);
}


/* ============================== INIT ==================================== */
void user_init(void){
    srand(pogobot_helper_getRandSeed());

    main_loop_hz = 60;
    max_nb_processed_msg_per_tick = 100;
    msg_rx_fn = on_rx;
    msg_tx_fn = on_tx;
    error_codes_led_idx = 3;

    for (int i = 0; i < D; ++i){
        mydata->lo[i] = -2.0f;
        mydata->hi[i] =  2.0f;
    }

    memset(&mydata->buf, 0, sizeof(mydata->buf));
    mydata->buf.n = D;

    const opt_algo_t algo = chosen_algo();
    const opt_mode_t mode = OPT_MINIMIZE;

    switch (algo){
    default:
    case OPT_ES1P1: {
        for (int i = 0; i < D; ++i){ mydata->internals.es1p1.x[i] = 1.5f; }
        mydata->internals.es1p1.P.mode       = ES1P1_MINIMIZE; /* will be overwritten by factory */
        mydata->internals.es1p1.P.sigma0     = 0.2f;
        mydata->internals.es1p1.P.sigma_min  = 1e-5f;
        mydata->internals.es1p1.P.sigma_max  = 0.8f;
        mydata->internals.es1p1.P.s_target   = 0.2f;
        mydata->internals.es1p1.P.s_alpha    = 0.2f;
        mydata->internals.es1p1.P.c_sigma    = 0.0f; /* 0 => auto */

        mydata->buf.u.es1p1.x      = mydata->internals.es1p1.x;
        mydata->buf.u.es1p1.x_try  = mydata->internals.es1p1.x_try;
        mydata->buf.u.es1p1.lo     = mydata->lo;
        mydata->buf.u.es1p1.hi     = mydata->hi;
        mydata->buf.u.es1p1.params = mydata->internals.es1p1.P;
    } break;

    case OPT_SPSA: {
        for (int i = 0; i < D; ++i){ mydata->internals.spsa.x[i] = 1.5f; }
        /* Use only fields that exist in your spsa_params_t */
        mydata->internals.spsa.P.a      = 0.3f;
        mydata->internals.spsa.P.c      = 0.15f;
        mydata->internals.spsa.P.A      = 20.0f;
        mydata->internals.spsa.P.alpha  = 0.602f;
        mydata->internals.spsa.P.gamma  = 0.101f;
        mydata->internals.spsa.P.g_clip = 1.0f;

        mydata->buf.u.spsa.x       = mydata->internals.spsa.x;
        mydata->buf.u.spsa.x_work  = mydata->internals.spsa.x_work;
        mydata->buf.u.spsa.delta   = mydata->internals.spsa.delta;
        mydata->buf.u.spsa.lo      = mydata->lo;
        mydata->buf.u.spsa.hi      = mydata->hi;
        mydata->buf.u.spsa.params  = mydata->internals.spsa.P;
    } break;

    case OPT_PGPE: {
        for (int i = 0; i < D; ++i){ mydata->internals.pgpe.mu[i] = 1.5f; mydata->internals.pgpe.sigma[i] = 0.2f; }
        mydata->internals.pgpe.P.mode       = PGPE_MINIMIZE;
        mydata->internals.pgpe.P.eta_mu     = 0.05f;
        mydata->internals.pgpe.P.eta_sigma  = 0.10f;
        mydata->internals.pgpe.P.sigma_min  = 1e-5f;
        mydata->internals.pgpe.P.sigma_max  = 0.8f;
        mydata->internals.pgpe.P.baseline_alpha = 0.10f;
        mydata->internals.pgpe.P.normalize_pair = 1;

        mydata->buf.u.pgpe.mu      = mydata->internals.pgpe.mu;
        mydata->buf.u.pgpe.sigma   = mydata->internals.pgpe.sigma;
        mydata->buf.u.pgpe.x_try   = mydata->internals.pgpe.x_try;
        mydata->buf.u.pgpe.eps     = mydata->internals.pgpe.eps;
        mydata->buf.u.pgpe.lo      = mydata->lo;
        mydata->buf.u.pgpe.hi      = mydata->hi;
        mydata->buf.u.pgpe.params  = mydata->internals.pgpe.P;
    } break;

    case OPT_SEP_CMAES: {
        for (int i = 0; i < D; ++i){ mydata->internals.sep.x[i] = 1.5f; mydata->internals.sep.c_diag[i] = 1.0f; }
        /* Small lambda to fit static buffers above */
        mydata->internals.sep.P.mode   = SEP_MINIMIZE;
        mydata->internals.sep.P.lambda = 8;
        mydata->internals.sep.P.mu     = 4;
        mydata->internals.sep.P.sigma0 = 0.3f;
        mydata->internals.sep.P.sigma_min = 1e-6f;
        mydata->internals.sep.P.sigma_max = 2.0f;
        mydata->internals.sep.P.weights = NULL;
        mydata->internals.sep.P.cc = 0.0f;
        mydata->internals.sep.P.cs = 0.0f;
        mydata->internals.sep.P.c1 = 0.0f;
        mydata->internals.sep.P.cmu = 0.0f;
        mydata->internals.sep.P.damps = 0.0f;
        mydata->internals.sep.P.lo     = mydata->lo;
        mydata->internals.sep.P.hi     = mydata->hi;

        mydata->buf.u.sep.x        = mydata->internals.sep.x;
        mydata->buf.u.sep.x_try    = mydata->internals.sep.x_try;
        mydata->buf.u.sep.c_diag   = mydata->internals.sep.c_diag;
        mydata->buf.u.sep.p_sigma  = mydata->internals.sep.p_sigma;
        mydata->buf.u.sep.p_c      = mydata->internals.sep.p_c;
        mydata->buf.u.sep.z_try    = mydata->internals.sep.z_try;
        mydata->buf.u.sep.y_try    = mydata->internals.sep.y_try;
        mydata->buf.u.sep.store_y  = mydata->internals.sep.store_y;
        mydata->buf.u.sep.fit_buf  = mydata->internals.sep.fit_buf;
        mydata->buf.u.sep.idx      = mydata->internals.sep.idx;
        mydata->buf.u.sep.params   = mydata->internals.sep.P;
    } break;

    case OPT_SOCIAL_LEARNING: {
        for (int i = 0; i < D; ++i){ mydata->internals.sl.x[i] = 1.5f; }
        memset(&mydata->internals.sl.P, 0, sizeof(mydata->internals.sl.P));
        mydata->internals.sl.P.mode = SL_MINIMIZE;
        mydata->internals.sl.P.roulette_random_prob = 0.10f;
        mydata->internals.sl.P.loss_mut_gain = 0.01f;
        mydata->internals.sl.P.loss_mut_clip = 1.0f;
        mydata->internals.sl.P.dup_eps = 1e-3f;
        mydata->internals.sl.P.repo_capacity = REPO_CAP;

        mydata->buf.u.sl.x          = mydata->internals.sl.x;
        mydata->buf.u.sl.x_try      = mydata->internals.sl.x_try;
        mydata->buf.u.sl.lo         = mydata->lo;
        mydata->buf.u.sl.hi         = mydata->hi;
        mydata->buf.u.sl.repo_X     = mydata->internals.sl.repo_X;
        mydata->buf.u.sl.repo_F     = mydata->internals.sl.repo_F;
        mydata->buf.u.sl.repo_epoch = mydata->internals.sl.repo_epoch;
        mydata->buf.u.sl.repo_from  = mydata->internals.sl.repo_from;
        mydata->buf.u.sl.capacity   = REPO_CAP;
        mydata->buf.u.sl.params     = mydata->internals.sl.P;
    } break;

    case OPT_HIT: {
        for (int i = 0; i < D; ++i){ mydata->internals.hit.x[i] = 1.5f; }
        memset(&mydata->internals.hit.P, 0, sizeof(mydata->internals.hit.P));
        mydata->internals.hit.P.mode = HIT_MINIMIZE;
        mydata->internals.hit.P.transfer_prob = 0.35f;
        mydata->internals.hit.P.random_donor_prob = 0.05f;
        mydata->internals.hit.P.accept_beta = 5.0f;
        mydata->internals.hit.P.accept_pmin = 0.02f;
        mydata->internals.hit.P.mut_gain = 0.4f;
        mydata->internals.hit.P.mut_clip = 1.0f;
        mydata->internals.hit.P.dup_eps = 1e-3f;
        mydata->internals.hit.P.repo_capacity = REPO_CAP;

        mydata->buf.u.hit.x          = mydata->internals.hit.x;
        mydata->buf.u.hit.x_try      = mydata->internals.hit.x_try;
        mydata->buf.u.hit.lo         = mydata->lo;
        mydata->buf.u.hit.hi         = mydata->hi;
        mydata->buf.u.hit.repo_X     = mydata->internals.hit.repo_X;
        mydata->buf.u.hit.repo_F     = mydata->internals.hit.repo_F;
        mydata->buf.u.hit.repo_epoch = mydata->internals.hit.repo_epoch;
        mydata->buf.u.hit.repo_from  = mydata->internals.hit.repo_from;
        mydata->buf.u.hit.capacity   = REPO_CAP;
        mydata->buf.u.hit.params     = mydata->internals.hit.P;
    } break;
    }

    opt_init(&mydata->opt, algo, mode, &mydata->buf);

    /* Prime algorithms that need an initial fitness (ES, SEP-CMA-ES, SL, HIT). */
    mydata->f0    = sphere_fn(opt_get_x(&mydata->opt), D);
    opt_tell_initial(&mydata->opt, mydata->f0);
    mydata->f_best = mydata->f0;

    printf("[OPT] algo=%d  n=%d  f0=%.6f\n", (int)algo, D, mydata->f0);
    mydata->last_print_ms = current_time_milliseconds();
}

/* =============================== STEP =================================== */
void user_step(void){
    const int evals_per_tick = 1;

    for (int k = 0; k < evals_per_tick; ++k){
        if (!opt_ready(&mydata->opt)) break;

        const float *parent_x = opt_get_x(&mydata->opt);
        float parent_f  = parent_x ? sphere_fn(parent_x, D) : 0.0f;
        if (chosen_algo() == OPT_SOCIAL_LEARNING) parent_f = 1.0f;

        /* aux_scale is meaningful only for Social Learning; ignored otherwise. */
        const float *x_try = opt_ask(&mydata->opt, parent_f);
        if (!x_try) break;

        const float f_try = sphere_fn(x_try, D);
        (void)opt_tell(&mydata->opt, f_try);
    }

    const float f_now = sphere_fn(opt_get_x(&mydata->opt), D);
    float g = f_now; if (g < 0.0f) g = 0.0f; if (g > 1.0f) g = 1.0f;
    uint8_t val = (uint8_t)(25.0f * (1.0f - g));
    pogobot_led_setColors(val, 0, val, 0);

    uint32_t now = current_time_milliseconds();
    if (now - mydata->last_print_ms > 2000){
        const float *x = opt_get_x(&mydata->opt);
        printf("[OPT] it=%u  f=%.6f  x[0]=%.4f ... x[%d]=%.4f\n",
               opt_iterations(&mydata->opt), f_now, x[0], D-1, x[D-1]);
        mydata->last_print_ms = now;
    }
}

/* ============================== MAIN ==================================== */
int main(void){
    pogobot_init();
#ifndef SIMULATOR
    printf("init ok\n");
#endif
    pogobot_start(user_init, user_step);
    return 0;
}

