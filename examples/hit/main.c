/**
 * @file example_hit.c
 * @brief Decentralized HIT example on Pogobot/Pogosim (no motility).
 *
 * Robots minimize the D-dim Sphere f(x)=sum x_i^2 using HIT (subset copy
 * from neighbors + mutation). Messaging identical to the SL example, so you
 * can A/B swap the optimizer by changing includes and init.
 */
#include "pogobase.h"
#include "pogo-utils/hit.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

#ifndef D
#define D 4
#endif
#ifndef REPO_CAP
#define REPO_CAP 32
#endif

/* ==== Messaging (same payload as SL) ===================================== */
typedef struct __attribute__((__packed__)) {
    uint16_t sender_id;
    uint32_t epoch;
    float    x[D];
    float    f_adv;
} sl_msg_t;
#define SL_MSG_BYTES ((uint16_t)sizeof(sl_msg_t))

/* ==== Objective ========================================================== */
static float sphere_fn(const float *x, int n){
    float s = 0.f; for (int i=0;i<n;++i) s += x[i]*x[i]; return s;
}

/* ==== USERDATA =========================================================== */
typedef struct {
    hit_t hit;

    /* Buffers */
    float x[D];
    float x_try[D];
    float lo[D];
    float hi[D];

    /* Repo buffers */
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

/* ==== RX: ingest neighbor adverts ======================================= */
static void on_rx(message_t *mr){
    if (mr->header.payload_length < SL_MSG_BYTES) return;
    sl_msg_t msg; memcpy(&msg, mr->payload, sizeof(sl_msg_t));
    if (msg.sender_id == pogobot_helper_getid()) return;
    float x_buf[D]; memcpy(x_buf, msg.x, sizeof(x_buf));
    hit_observe_remote(&mydata->hit, msg.sender_id, msg.epoch, x_buf, msg.f_adv);
}

/* ==== TX: broadcast current parent + advertised perf ===================== */
static bool on_tx(void){
    uint32_t now = current_time_milliseconds();
    const uint32_t period_ms = 200;
    if (now - mydata->last_tx_ms < period_ms) return false;

    sl_msg_t m;
    m.sender_id = pogobot_helper_getid();
    m.epoch     = hit_get_epoch(&mydata->hit);
    memcpy(m.x, hit_get_x(&mydata->hit), sizeof(float)*D);
    m.f_adv = hit_get_last_advert(&mydata->hit);

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

    for (int i=0;i<D;++i){
        mydata->lo[i] = -2.0f;
        mydata->hi[i] =  2.0f;
        mydata->x[i]  =  1.5f;
    }

    hit_params_t p = {
        .mode = HIT_MINIMIZE,
        .transfer_prob = 0.35f,
        .random_donor_prob = 0.05f,
        .accept_beta = 5.0f,
        .accept_pmin = 0.02f,
        .mut_gain = 0.4f,
        .mut_clip = 1.0f,
        .dup_eps = 1e-3f,
        .repo_capacity = REPO_CAP
    };

    hit_init(&mydata->hit, D,
             mydata->x, mydata->x_try,
             mydata->lo, mydata->hi,
             mydata->repo_X, mydata->repo_F,
             mydata->repo_epoch, mydata->repo_from,
             REPO_CAP, &p);

    float f0 = sphere_fn(mydata->x, D);
    hit_tell_initial(&mydata->hit, f0);

    mydata->win_ms = 3000;
    mydata->quiet_ms = 100;
    mydata->win_t0_ms = current_time_milliseconds();
    mydata->win_acc_f = 0.f; mydata->win_acc_w = 0.f;

    mydata->last_tx_ms = 0;
    mydata->last_print_ms = current_time_milliseconds();

#ifndef SIMULATOR
    printf("[HIT] init: f0=%.6f\n", hit_get_last_advert(&mydata->hit));
#endif
}

/* ==== Step =============================================================== */
void user_step(void){
    uint32_t now = current_time_milliseconds();
    static uint32_t last_ms = 0;
    float dt = (now > last_ms) ? (now - last_ms) * 1e-3f : 0.f;
    last_ms = now;

    const int evals_per_tick = 1;
    for (int k=0; k<evals_per_tick; ++k){
        float loss_for_scale = hit_get_last_advert(&mydata->hit);
        const float *x_try = hit_ask(&mydata->hit, loss_for_scale);
        if (!x_try) break;
        float f_try = sphere_fn(x_try, D);
        (void)hit_tell(&mydata->hit, f_try);
    }

    /* Maintain window-avg advertised fitness (like SL example) */
    if (now - mydata->win_t0_ms > mydata->quiet_ms){
        float f_cur = hit_get_last_advert(&mydata->hit);
        mydata->win_acc_f += f_cur * dt;
        mydata->win_acc_w += dt;
    }
    if (now - mydata->win_t0_ms >= mydata->win_ms){
        float f_win = (mydata->win_acc_w > 1e-6f) ? (mydata->win_acc_f / mydata->win_acc_w)
                                                  : hit_get_last_advert(&mydata->hit);
        /* “Advertise” windowed value by upserting self (cheap trick) */
        hit_observe_remote(&mydata->hit, pogobot_helper_getid(),
                           hit_get_epoch(&mydata->hit), hit_get_x(&mydata->hit), f_win);
        mydata->win_t0_ms = now;
        mydata->win_acc_f = 0.f; mydata->win_acc_w = 0.f;
    }

    /* LEDs: greener as loss decreases */
    float f = hit_get_last_advert(&mydata->hit);
    float g = f; if (g < 0.f) g = 0.f; if (g > 1.f) g = 1.f;
    uint8_t val = (uint8_t)(25.0f * (1.0f - g));
    pogobot_led_setColors(val, 0, val, 0);

    if (now - mydata->last_print_ms > 2000){
        const float *x = hit_get_x(&mydata->hit);
        printf("[HIT] id=%u ep=%u f_adv=%.6f  x[0]=%.4f ... x[%d]=%.4f  repo=%u\n",
               pogobot_helper_getid(),
               (unsigned)hit_get_epoch(&mydata->hit),
               hit_get_last_advert(&mydata->hit),
               x[0], D-1, x[D-1],
               (unsigned)mydata->hit.rsize);
        mydata->last_print_ms = now;
    }
}

/* ==== Main =============================================================== */
int main(void){
    pogobot_init();
    pogobot_start(user_init, user_step);
    return 0;
}

