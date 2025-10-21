/**
 * @file example_pgpe_main.c
 * @brief Example: PGPE inside the Pogobot/Pogosim control loop.
 *
 * Minimizes the Sphere function f(x)=\sum x_i^2 in dimension D.
 * Uses diagonal PGPE with antithetic sampling and an EWMA baseline.
 */
#include "pogobase.h"
#include "pogo-utils/pgpe.h"
#include <stdio.h>
#include <math.h>

#ifndef D
#define D 8
#endif

typedef struct {
    pgpe_t opt;

    /* Buffers (no malloc) */
    float mu[D];
    float sigma[D];
    float theta[D];
    float eps[D];
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

    /* Init mean, scales, and bounds */
    for (int i = 0; i < D; ++i) {
        mydata->mu[i] = 1.5f;     /* start away from optimum */
        mydata->sigma[i] = 0.2f;  /* ~10% of range */
        mydata->lo[i] = -2.0f;
        mydata->hi[i] =  2.0f;
    }

    pgpe_params_t p = {
        .mode = PGPE_MINIMIZE,
        .lr_mu = 0.2f,
        .lr_sigma = 0.05f,
        .sigma_min = 1e-4f,
        .sigma_max = 1.0f,
        .samples_per_tick = 8,
        .antithetic = true,
        .baseline_beta = 0.9f,
        .project_after_update = true
    };

    pgpe_init(&mydata->opt, D,
              mydata->mu,
              mydata->sigma,
              mydata->theta,
              mydata->eps,
              mydata->lo, mydata->hi,
              sphere_fn, NULL, &p);

    printf("[PGPE] init: f(mu)=%.6f\n", pgpe_f(&mydata->opt));
    mydata->last_print_ms = current_time_milliseconds();
}

void user_step(void) {
    float f = pgpe_step(&mydata->opt);

    /* LED feedback (optional) */
    float g = f; if (g < 0.0f) g = 0.0f; if (g > 1.0f) g = 1.0f;
    uint8_t val = (uint8_t)(25.0f * (1.0f - g));
    pogobot_led_setColors(val, 0, val, 0);

    uint32_t now = current_time_milliseconds();
    if (now - mydata->last_print_ms > 2000) {
        const float *mu = pgpe_get_mu(&mydata->opt);
        const float *sigma = pgpe_get_sigma(&mydata->opt);
        printf("[PGPE] it=%u  f(mu)=%.6f  mu[0]=%.4f  sigma[0]=%.4f ... mu[%d]=%.4f  sigma[%d]=%.4f\n",
               pgpe_iterations(&mydata->opt), f, mu[0], sigma[0], D-1, mu[D-1], D-1, sigma[D-1]);
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

