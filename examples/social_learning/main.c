/**
 * @file example_social_learning.c
 * @brief Decentralized SL (mEDEA-style) example on Pogobot/Pogosim (no motility).
 *
 * Each robot minimizes the D-dimensional Sphere function f(x)=sum x_i^2 using the
 * Social Learning library in strict ask–tell form. Robots periodically broadcast
 * their *current parent* vector and advertised (window-averaged) fitness, and
 * ingest neighbors’ adverts into their local repository (roulette selection).
 *
 * Notes:
 *  - To keep the message small, use a modest D (e.g., D<=4..6) or compress.
 *  - Robots do not move; LEDs can visualize progress.
 *  - “loss_for_scale” for sl_ask() is the current (smoothed) window loss.
 */
#include "pogobase.h"
#include "pogo-utils/social_learning.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

#ifndef D
#define D 4   /* keep small to fit into the IR message comfortably */
#endif
#ifndef REPO_CAP
#define REPO_CAP 32
#endif

/* ==== Messaging ========================================================== */
/* Minimal advert payload: epoch + D floats + adv fitness. */
typedef struct __attribute__((__packed__)) {
    uint16_t sender_id;
    uint32_t epoch;
    float    x[D];
    float    f_adv;
} sl_msg_t;
#define SL_MSG_BYTES ((uint16_t)sizeof(sl_msg_t))

/* ==== Objective ========================================================== */
static float sphere_fn(const float *x, int n){
    float s = 0.f;
    for (int i=0;i<n;++i) s += x[i]*x[i];
    return s;
}

/* ==== USERDATA =========================================================== */
typedef struct {
    sl_t sl;

    /* Buffers (no malloc) */
    float x[D];
    float x_try[D];
    float lo[D];
    float hi[D];

    /* Repo buffers (row-major genomes) */
    float    repo_X[REPO_CAP * D];
    float    repo_F[REPO_CAP];
    uint32_t repo_epoch[REPO_CAP];
    uint16_t repo_from[REPO_CAP];

    /* Timers */
    uint32_t last_tx_ms;
    uint32_t last_print_ms;

    /* Window averaging for advertised fitness */
    float    win_acc_f;
    float    win_acc_w;
    uint32_t win_t0_ms;
    uint32_t win_ms;
    uint32_t quiet_ms;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA)

/* ==== Messaging RX ======================================================= */
static void on_rx(message_t *mr){
    if (mr->header.payload_length < SL_MSG_BYTES) return;

    /* Copy out of the packed payload into aligned locals */
    sl_msg_t msg;
    memcpy(&msg, mr->payload, sizeof(sl_msg_t));

    if (msg.sender_id == pogobot_helper_getid()) return; /* ignore own */

    float x_buf[D];
    memcpy(x_buf, msg.x, sizeof(x_buf));

    /* Ingest remote advert into repository */
    sl_observe_remote(&mydata->sl, msg.sender_id, msg.epoch, x_buf, msg.f_adv);
}


/* ==== Messaging TX ======================================================= */
static bool on_tx(void){
    uint32_t now = current_time_milliseconds();
    const uint32_t period_ms = 200; /* 5 Hz beacons */
    if (now - mydata->last_tx_ms < period_ms) return false;

    sl_msg_t m;
    m.sender_id = pogobot_helper_getid();
    m.epoch     = sl_get_epoch(&mydata->sl);
    memcpy(m.x, sl_get_x(&mydata->sl), sizeof(float)*D);
    m.f_adv = sl_get_last_advert(&mydata->sl);

    mydata->last_tx_ms = now;
    return pogobot_infrared_sendShortMessage_omni((uint8_t*)&m, SL_MSG_BYTES);
}

/* ==== Init =============================================================== */
void user_init(void){
    srand(pogobot_helper_getRandSeed());

    main_loop_hz = 60;
    max_nb_processed_msg_per_tick = 200;
    msg_rx_fn = on_rx;
    msg_tx_fn = on_tx;
    error_codes_led_idx = 3;

    /* Problem bounds and initial parent */
    for (int i=0;i<D;++i){
        mydata->lo[i] = -2.0f;
        mydata->hi[i] =  2.0f;
        mydata->x[i]  =  1.5f;  /* start away from optimum */
    }

    /* SL params (minimize loss) */
    sl_params_t p = {
        .mode = SL_MINIMIZE,
        .roulette_random_prob = 0.10f,
        .loss_mut_gain = 0.01f,
        .loss_mut_clip = 1.0f,
        .dup_eps = 1e-3f,
        .repo_capacity = REPO_CAP
    };

    sl_init(&mydata->sl, D,
            mydata->x, mydata->x_try,
            mydata->lo, mydata->hi,
            mydata->repo_X, mydata->repo_F,
            mydata->repo_epoch, mydata->repo_from,
            REPO_CAP, &p);

    /* Initial fitness and advert */
    float f0 = sphere_fn(mydata->x, D);
    sl_tell_initial(&mydata->sl, f0);

    /* Window config (advertised fitness smoothing) */
    mydata->win_ms = 3000;   /* average over 3 s windows */
    mydata->quiet_ms = 100;  /* initial guard before sampling */
    mydata->win_t0_ms = current_time_milliseconds();
    mydata->win_acc_f = 0.f; mydata->win_acc_w = 0.f;

    mydata->last_tx_ms = 0;
    mydata->last_print_ms = current_time_milliseconds();

#ifndef SIMULATOR
    printf("[SL] init: f0=%.6f\n", sl_get_last_advert(&mydata->sl));
#endif
}

/* ==== Step =============================================================== */
void user_step(void){
    uint32_t now = current_time_milliseconds();
    static uint32_t last_ms = 0;
    float dt = (now > last_ms) ? (now - last_ms) * 1e-3f : 0.f;
    last_ms = now;

    /* --- Optimization: one or more ask–tell per tick --- */
    const int evals_per_tick = 1;
    for (int k=0; k<evals_per_tick; ++k){
        /* Use current advertised loss as mutation scale (MINIMIZE) */
        float loss_for_scale = sl_get_last_advert(&mydata->sl);

        const float *x_try = sl_ask(&mydata->sl, loss_for_scale);
        if (!x_try) break;
        float f_try = sphere_fn(x_try, D);
        float f_best = sl_tell(&mydata->sl, f_try);
        (void)f_best;
    }

    /* --- Maintain windowed advertised fitness --- */
    if (now - mydata->win_t0_ms > mydata->quiet_ms){
        float f_cur = sl_get_last_advert(&mydata->sl); /* parent’s best so far */
        mydata->win_acc_f += f_cur * dt;
        mydata->win_acc_w += dt;
    }
    if (now - mydata->win_t0_ms >= mydata->win_ms){
        float f_win = (mydata->win_acc_w > 1e-6f) ? (mydata->win_acc_f / mydata->win_acc_w)
                                                  : sl_get_last_advert(&mydata->sl);
        /* Overwrite library’s last_adv with our window-avg for broadcast. */
        /* (Not strictly necessary; here we simply set our own advert scale) */
        /* A light trick: call observe_remote() on self to update repo + advert */
        sl_observe_remote(&mydata->sl, pogobot_helper_getid(),
                          sl_get_epoch(&mydata->sl), sl_get_x(&mydata->sl), f_win);

        mydata->win_t0_ms = now;
        mydata->win_acc_f = 0.f; mydata->win_acc_w = 0.f;
    }

    /* --- Visualize progress on LEDs (brighter as loss ↓) --- */
    float f = sl_get_last_advert(&mydata->sl);  /* ~ current parent loss */
    float g = f; if (g < 0.f) g = 0.f; if (g > 1.f) g = 1.f;
    uint8_t val = (uint8_t)(25.0f * (1.0f - g));
    pogobot_led_setColors(val, 0, val, 0);

    /* --- Status print --- */
    if (now - mydata->last_print_ms > 2000){
        const float *x = sl_get_x(&mydata->sl);
        printf("[SL] id=%u ep=%u f_adv=%.6f  x[0]=%.4f ... x[%d]=%.4f  repo=%u\n",
               pogobot_helper_getid(),
               (unsigned)sl_get_epoch(&mydata->sl),
               sl_get_last_advert(&mydata->sl),
               x[0], D-1, x[D-1],
               (unsigned)mydata->sl.rsize);
        mydata->last_print_ms = now;
    }
}

/* ==== Main =============================================================== */
int main(void){
    pogobot_init();
    pogobot_start(user_init, user_step);
    return 0;
}

