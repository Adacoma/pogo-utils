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
#include "pogo-utils/tiny_alloc.h"
#include "pogo-utils/optim.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <float.h>

#ifndef D
#define D 8
#endif

#ifndef OPT_EXAMPLE_ALGO
#define OPT_EXAMPLE_ALGO 3 /* default: OPT_ES1P1 */
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
    opt_t *opt;
    float lo[D], hi[D];
    float f0;

    tiny_alloc_t ta;
    uint8_t heap[4096];

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
    opt_observe_remote(mydata->opt, msg.sender_id, msg.epoch, x_buf, msg.f_adv);
}


/* ==== Messaging TX ======================================================= */
static bool on_tx(void){
    uint32_t now = current_time_milliseconds();
    const uint32_t period_ms = 200; /* 5 Hz beacons */
    if (now - mydata->last_tx_ms < period_ms) return false;

    sl_msg_t m;
    m.sender_id = pogobot_helper_getid();
    m.epoch     = opt_iterations(mydata->opt);
    memcpy(m.x, opt_get_x(mydata->opt), sizeof(float)*D);
    m.f_adv = opt_get_last_advert(mydata->opt);

    mydata->last_tx_ms = now;
    return pogobot_infrared_sendShortMessage_omni((uint8_t*)&m, SL_MSG_BYTES);
}


/* ============================== INIT ==================================== */
void user_init(void){
    srand(pogobot_helper_getRandSeed());

    main_loop_hz = 30;
    max_nb_processed_msg_per_tick = 100;
    msg_rx_fn = on_rx;
    msg_tx_fn = on_tx;
    error_codes_led_idx = 3;

    // Initialize the heap, with large chunks (512) allowed
     const uint16_t classes[] = { 32, 48, 64, 128, 512 };
     tiny_alloc_init(&mydata->ta, mydata->heap, sizeof(mydata->heap), classes, 5);

    for (int i = 0; i < D; ++i){
        mydata->lo[i] = -2.0f;
        mydata->hi[i] =  2.0f;
    }

    opt_cfg_t cfg = opt_default_cfg(chosen_algo(), D);
    //cfg.sz.lambda = 8; cfg.sz.mu = 4;           // optional tweak
    int ok = opt_create(&mydata->opt, &mydata->ta, D,
            chosen_algo(), OPT_MINIMIZE,
            mydata->lo, mydata->hi, &cfg);
    if (!ok || !mydata->opt) {
        printf("[OPT] create failed (heap=%uB). Try bigger heap or smaller lambda/repo.\n",
               (unsigned)sizeof(mydata->heap));
    }

    opt_randomize_x(mydata->opt, /*seed*/ 12345);

    /* Prime algorithms that need an initial fitness (ES, SEP-CMA-ES, SL, HIT). */
    mydata->f0    = sphere_fn(opt_get_x(mydata->opt), D);
    opt_tell_initial(mydata->opt, mydata->f0);

    printf("[OPT] algo=%d  ok=%d  n=%d  f0=%.6f\n", ok, (int)OPT_EXAMPLE_ALGO, D, mydata->f0);
    mydata->last_print_ms = current_time_milliseconds();
}

/* =============================== STEP =================================== */
void user_step(void){
    const int evals_per_tick = 1;

    for (int k = 0; k < evals_per_tick; ++k){
        if (!opt_ready(mydata->opt)) break;

        const float *parent_x = opt_get_x(mydata->opt);
        float parent_f  = parent_x ? sphere_fn(parent_x, D) : 0.0f;
        if (chosen_algo() == OPT_SOCIAL_LEARNING) parent_f = 1.0f;

        /* aux_scale is meaningful only for Social Learning; ignored otherwise. */
        const float *x_try = opt_ask(mydata->opt, parent_f);
        if (!x_try) break;

        const float f_try = sphere_fn(x_try, D);
        (void)opt_tell(mydata->opt, f_try);
    }

    const float f_now = sphere_fn(opt_get_x(mydata->opt), D);
    float g = f_now; if (g < 0.0f) g = 0.0f; if (g > 1.0f) g = 1.0f;
    uint8_t val = (uint8_t)(25.0f * (1.0f - g));
    pogobot_led_setColors(val, 0, val, 0);

    uint32_t now = current_time_milliseconds();
    if (now - mydata->last_print_ms > 1000){
        const float *x = opt_get_x(mydata->opt);
        printf("[OPT] it=%u  f=%.6f  x[0]=%.4f ... x[%d]=%.4f\n",
               opt_iterations(mydata->opt), f_now, x[0], D-1, x[D-1]);
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

