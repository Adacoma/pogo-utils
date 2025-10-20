/**
 * @file example_oneplusone_es_main.c
 * @brief Example: 1+1-ES inside the Pogobot/Pogosim control loop.
 *
 * Minimizes the Sphere function f(x)=sum_i x_i^2 in dimension D.
 * Prints the current best every ~2 s and blinks LEDs (optional).
 */
#include "pogobase.h"
#include "pogo-utils/oneplusone_es.h"
#include <stdio.h>
#include <math.h>

#ifndef D
#define D 8
#endif

typedef struct {
    es1p1_t es;

    /* Buffers (no malloc) */
    float x[D];
    float x_try[D];
    float lo[D];
    float hi[D];

    uint32_t last_print_ms;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA)

/* Objective: Sphere (minimize) */
static float sphere_fn(const float *restrict x, int n, void *user) {
    (void)user;
    float s = 0.0f;
    for (int i = 0; i < n; ++i) {
        s += x[i] * x[i];
    }
    return s;
}

void user_init(void) {
    srand(pogobot_helper_getRandSeed());

    main_loop_hz = 60;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;

    /* Init parent vector and bounds */
    for (int i = 0; i < D; ++i) {
        mydata->x[i] = 1.5f;    /* start away from optimum */
        mydata->lo[i] = -2.0f;  /* optional */
        mydata->hi[i] =  2.0f;
    }

    es1p1_params_t p = {
        .mode = ES1P1_MINIMIZE,
        .sigma0 = 0.2f,
        .sigma_min = 1e-5f,
        .sigma_max = 0.8f,
        .s_target = 0.2f,
        .s_alpha = 0.2f,
        .c_sigma = 0.0f,         /* 0 => auto = 0.6/sqrt(n) */
        .evals_per_tick = 1
    };

    es1p1_init(&mydata->es, D,
               mydata->x, mydata->x_try,
               mydata->lo, mydata->hi,
               sphere_fn, NULL, &p);

    printf("[1+1-ES] init: f0=%.6f, sigma=%.4f\n",
           es1p1_best_f(&mydata->es), es1p1_sigma(&mydata->es));

    mydata->last_print_ms = current_time_milliseconds();
}

void user_step(void) {
    /* Run one batch of ES iterations */
    float f = es1p1_step(&mydata->es);

    /* Optional: LED intensity proportional to progress (just for feedback) */
    float g = f; if (g < 0.0f) g = 0.0f;
    if (g > 1.0f) g = 1.0f;
    uint8_t val = (uint8_t)(25.0f * (1.0f - g));
    pogobot_led_setColors(val, 0, val, 0);

    uint32_t now = current_time_milliseconds();
    if (now - mydata->last_print_ms > 2000) {
        const float *x = es1p1_get_x(&mydata->es);
        printf("[1+1-ES] it=%u  f=%.6f  sigma=%.5f  x[0]=%.4f ... x[%d]=%.4f\n",
               es1p1_iterations(&mydata->es), f, es1p1_sigma(&mydata->es),
               x[0], D-1, x[D-1]);
        mydata->last_print_ms = now;
    }
}

/* Program entry point */
int main(void) {
    pogobot_init();
#ifndef SIMULATOR
    printf("init ok\n");
#endif
    pogobot_start(user_init, user_step);
    return 0;
}

