/**
 * @file example_hit_strict.c
 * @brief Decentralized strict-HIT example on Pogobot/Pogosim (no motility).
 *
 * Robots minimize the D-dim Sphere f(x)=sum x_i^2 using strict HIT (subset copy
 * from neighbors + mutation), with the new ask/tell/observe_remote API.
 */
#include "pogobase.h"
#include "pogo-utils/hit.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

#ifndef D
#define D 4
#endif

/* ==== Messaging (same payload as SL) ===================================== */
typedef struct __attribute__((__packed__)) {
    uint16_t sender_id;
    uint32_t epoch;
    float    x[D];
    float    f_adv;   /* here: advertised fitness = current f */
} sl_msg_t;
#define SL_MSG_BYTES ((uint16_t)sizeof(sl_msg_t))

/* ==== Objective ========================================================== */
static float sphere_fn(const float *x, int n){
    float s = 0.f;
    for (int i = 0; i < n; ++i) s += x[i] * x[i];
    return s;
}

/* ==== USERDATA =========================================================== */
typedef struct {
    hit_t hit;

    /* Parameter vector + bounds */
    float x[D];
    float lo[D];
    float hi[D];

    /* Timers */
    uint32_t last_tx_ms;
    uint32_t last_print_ms;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA)

/* ==== RX: ingest neighbor adverts ======================================= */
static void on_rx(message_t *mr){
    if (mr->header.payload_length < SL_MSG_BYTES) return;

    sl_msg_t msg;
    memcpy(&msg, mr->payload, sizeof(sl_msg_t));

    if (msg.sender_id == pogobot_helper_getid()) return;

    /* Strict HIT: directly pass neighbour genome + fitness */
    hit_observe_remote(&mydata->hit,
                       msg.sender_id,
                       msg.epoch,
                       msg.x,
                       msg.f_adv);
}

/* ==== TX: broadcast current genome + fitness ============================ */
static bool on_tx(void){
    uint32_t now = current_time_milliseconds();
    const uint32_t period_ms = 200;

    if (now - mydata->last_tx_ms < period_ms) return false;

    sl_msg_t m;
    m.sender_id = pogobot_helper_getid();
    m.epoch     = hit_get_epoch(&mydata->hit);
    memcpy(m.x, hit_get_x(&mydata->hit), sizeof(float)*D);
    m.f_adv     = hit_get_f(&mydata->hit);  /* advertise current fitness */

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

    /* Initial genome and bounds */
    for (int i = 0; i < D; ++i){
        mydata->lo[i] = -2.0f;
        mydata->hi[i] =  2.0f;

        // Random initial genome in [-2, 2]
        float u = (float)rand() / (float)RAND_MAX;
        mydata->x[i] = mydata->lo[i] + u * (mydata->hi[i] - mydata->lo[i]);
    }

    /* Strict HIT hyperparameters: alpha ~ old transfer_prob, sigma ~ mutation */
    hit_params_t p = {
        .mode  = HIT_MINIMIZE,
        .alpha = 0.35f,   /* fraction of coordinates copied from neighbour */
        .sigma = 0.15f    /* mutation stddev */
    };

    /* No repo, no extra buffers needed; x_buf = NULL */
    hit_init(&mydata->hit, D,
             mydata->x,       /* x */
             NULL,            /* x_buf (unused) */
             mydata->lo, mydata->hi,
             &p);

    /* Evaluate initial genome once and set initial fitness */
    float f0 = sphere_fn(mydata->x, D);
    hit_tell_initial(&mydata->hit, f0);

    mydata->last_tx_ms    = 0;
    mydata->last_print_ms = current_time_milliseconds();

#ifndef SIMULATOR
    printf("[HIT] init: f0=%.6f\n", hit_get_f(&mydata->hit));
#endif
}

/* ==== Step =============================================================== */
void user_step(void){
    uint32_t now = current_time_milliseconds();

    /* Strict HIT: current genome is always hit_get_x(&hit).
       We just (re)evaluate it and update fitness. */
    const float *x = hit_ask(&mydata->hit);
    if (x){
        float f = sphere_fn(x, D);
        hit_tell(&mydata->hit, f);
    }

    /* LEDs: greener as loss decreases (very rough normalization) */
    float f = hit_get_f(&mydata->hit);
    float g = f;
    if (g < 0.f) g = 0.f;
    if (g > 1.f) g = 1.f;
    uint8_t val = (uint8_t)(25.0f * (1.0f - g));
    pogobot_led_setColors(val, 0, val, 0);

    /* Periodic debug print */
    if (now - mydata->last_print_ms > 2000){
        const float *xcur = hit_get_x(&mydata->hit);
        printf("[HIT] id=%u ep=%u f=%.6f  x[0]=%.4f ... x[%d]=%.4f\n",
               pogobot_helper_getid(),
               (unsigned)hit_get_epoch(&mydata->hit),
               hit_get_f(&mydata->hit),
               xcur[0], D-1, xcur[D-1]);
        mydata->last_print_ms = now;
    }
}

/* ==== Main =============================================================== */
int main(void){
    pogobot_init();
    pogobot_start(user_init, user_step);
    return 0;
}

