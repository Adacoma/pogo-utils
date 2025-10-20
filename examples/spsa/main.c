/**
 * @file example_spsa_main.c
 * @brief Example: SPSA inside the Pogobot/Pogosim control loop.
 *
 * Minimizes the Sphere function f(x)=∑ x_i^2 in dimension D.
 * Prints progress every ~2 s. Uses bounds and runs one SPSA iteration per tick.
 */
#include "pogobase.h"
#include "pogo-utils/spsa.h"
#include <stdio.h>
#include <math.h>

#ifndef D
#define D 8
#endif

typedef struct {
    spsa_t opt;

    /* Buffers (no malloc) */
    float x[D];
    float delta[D];
    float x_plus[D];
    float x_minus[D];
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
    for (int i = 0; i < n; ++i) s += x[i] * x[i];
    return s;
}

void user_init(void) {
    srand(pogobot_helper_getRandSeed());

    main_loop_hz = 60;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;

    /* Init vector and bounds */
    for (int i = 0; i < D; ++i) {
        mydata->x[i] = 1.5f;   /* start away from optimum */
        mydata->lo[i] = -2.0f;
        mydata->hi[i] =  2.0f;
    }

    spsa_params_t p = {
        .mode = SPSA_MINIMIZE,
        .a = 0.2f,           /* step gain (tune to problem scale) */
        .c = 0.1f,           /* perturbation gain */
        .alpha = 0.602f,
        .gamma = 0.101f,
        .A = 0.0f,           /* can try 0.1 * expected iters */
        .evals_per_tick = 1, /* 1 iteration => 2 f-evals per tick */
        .project_after_update = true
    };

    spsa_init(&mydata->opt, D,
              mydata->x,
              mydata->delta,
              mydata->x_plus,
              mydata->x_minus,
              mydata->lo, mydata->hi,
              sphere_fn, NULL, &p);

    printf("[SPSA] init: f0=%.6f\n", spsa_f(&mydata->opt));
    mydata->last_print_ms = current_time_milliseconds();
}

void user_step(void) {
    float f = spsa_step(&mydata->opt);

    /* LED feedback (optional) */
    float g = f; if (g < 0.0f) g = 0.0f; if (g > 1.0f) g = 1.0f;
    uint8_t val = (uint8_t)(25.0f * (1.0f - g));
    pogobot_led_setColors(val, 0, val, 0);

    uint32_t now = current_time_milliseconds();
    if (now - mydata->last_print_ms > 2000) {
        const float *x = spsa_get_x(&mydata->opt);
        printf("[SPSA] it=%u  f=%.6f  x[0]=%.4f ... x[%d]=%.4f\n",
               spsa_iterations(&mydata->opt), f, x[0], D-1, x[D-1]);
        mydata->last_print_ms = now;
    }
}

int main(void) {
    pogobot_init();
#ifndef SIMULATOR
    printf("init ok\n");
#endif
    pogobot_start(user_init, user_step);
    return 0;
}


