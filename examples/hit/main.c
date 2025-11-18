/**
 * @file example_hit_cec2020.c
 * @brief Decentralized HIT (CEC 2020 flavour) example on Pogobot/Pogosim.
 *
 * Robots minimise the D-dim Sphere f(x)=sum x_i^2 using HIT with:
 *   - sliding-window score over eval_T control steps,
 *   - maturation delay (no communication until the window is full, and
 *     again after each parameter change),
 *   - optional evolution of the transfer rate α.
 *
 * This is a purely numerical example (no motility / sensors): each control
 * step simply evaluates the current genome and feeds the instantaneous
 * value as a "cost" to HIT.
 */
#include "pogobase.h"
#include "pogo-utils/hit.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef D
#define D 4
#endif

/* ==== Messaging (payload follows CEC2020 logic) ========================= */
typedef struct __attribute__((__packed__)) {
    uint16_t sender_id;
    uint32_t epoch;
    float    alpha;       /* sender's current transfer rate α */
    float    x[D];        /* genome */
    float    f_adv;       /* advertised sliding-window score */
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

/* ==== RX: ingest neighbour adverts ====================================== */
static void on_rx(message_t *mr){
    if (mr->header.payload_length < SL_MSG_BYTES) return;

    sl_msg_t msg;
    memcpy(&msg, mr->payload, sizeof(sl_msg_t));

    if (msg.sender_id == pogobot_helper_getid()) return;

    /* HIT (CEC2020): only mature agents may adopt neighbours.
       Maturation gating is handled inside hit_observe_remote(). */
    hit_observe_remote(&mydata->hit,
                       msg.sender_id,
                       msg.epoch,
                       msg.x,
                       msg.f_adv,
                       msg.alpha);
}

/* ==== TX: broadcast current genome + score ============================== */
static bool on_tx(void){
    uint32_t now = current_time_milliseconds();
    const uint32_t period_ms = 200;

    /* Do not broadcast during maturation: only once we have a valid score. */
    if (!hit_ready(&mydata->hit)) return false;
    if (now - mydata->last_tx_ms < period_ms) return false;

    sl_msg_t m;
    m.sender_id = pogobot_helper_getid();
    m.epoch     = hit_get_epoch(&mydata->hit);
    m.alpha     = hit_get_alpha(&mydata->hit);
    memcpy(m.x, hit_get_x(&mydata->hit), sizeof(float)*D);
    m.f_adv     = hit_get_f(&mydata->hit);  /* advertise sliding-window score */

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

        /* Random initial genome in [-2, 2] */
        float u = (float)rand() / (float)RAND_MAX;
        mydata->x[i] = mydata->lo[i] + u * (mydata->hi[i] - mydata->lo[i]);
    }

    /* HIT hyperparameters close to CEC2020:
     *   - minimise accumulated cost (Sphere),
     *   - eval_T = 400 (maturation delay),
     *   - α initially uniform in [0, 0.9], then evolved by selection,
     *   - σ chosen fairly large here for a simple demo.
     */
    hit_params_t p;
    p.mode         = HIT_MINIMIZE;
    p.sigma        = 0.15f;
    p.eval_T       = 200;
    p.evolve_alpha = true;
    p.alpha_min    = 0.0f;
    p.alpha_max    = 0.9f;
    p.alpha_sigma  = 1e-3f;

    float u_alpha  = (float)rand() / (float)RAND_MAX; /* in [0,1] */
    p.alpha        = p.alpha_min + u_alpha * (p.alpha_max - p.alpha_min);

    hit_init(&mydata->hit, D,
             mydata->x,       /* x */
             NULL,            /* x_buf (unused) */
             mydata->lo, mydata->hi,
             &p);

    mydata->last_tx_ms    = 0;
    mydata->last_print_ms = current_time_milliseconds();

#ifndef SIMULATOR
    printf("[HIT] init: alpha0=%.3f\n", hit_get_alpha(&mydata->hit));
#endif
}

/* ==== Step =============================================================== */
void user_step(void){
    uint32_t now = current_time_milliseconds();

    /* HIT: current genome is hit_get_x(&hit). We (re-)evaluate it at each
       control step and feed the instantaneous cost as a reward/cost to HIT.
       With mode=HIT_MINIMIZE, lower f means better. */
    const float *x = hit_ask(&mydata->hit);
    if (x){
        float f_inst = sphere_fn(x, D);
        hit_tell(&mydata->hit, f_inst);
    }

    /* LEDs: greener as accumulated cost decreases (very rough normalisation).
       We simply map the current sliding-window score to a [0,1] range. */
    float f = hit_get_f(&mydata->hit);
    if (f < 0.f) f = 0.f;
    /* crude rescaling assuming interesting values roughly in [0,1] */
    if (f > 1.f) f = 1.f;
    uint8_t val = (uint8_t)(25.0f * (1.0f - f));
    pogobot_led_setColors(val, 0, val, 0);

    /* Periodic debug print */
    if (now - mydata->last_print_ms > 2000){
        const float *xcur = hit_get_x(&mydata->hit);
        printf("[HIT] id=%u ep=%u ready=%d alpha=%.3f f=%.6f  x[0]=%.4f ... x[%d]=%.4f\n",
               pogobot_helper_getid(),
               (unsigned)hit_get_epoch(&mydata->hit),
               hit_ready(&mydata->hit),
               hit_get_alpha(&mydata->hit),
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

