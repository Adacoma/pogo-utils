/**
 * @file example_sep_cmaes_main.c
 * @brief Example: Separable CMA‑ES (diag‑C) inside Pogobot/Pogosim control loop.
 *
 * Minimizes Sphere f(x)=Σ x_i^2 in D dimensions. Uses λ=8, μ=4 by default.
 */
#include "pogobase.h"
#include "pogo-utils/sep_cmaes.h"
#include <stdio.h>
#include <math.h>

#ifndef D
#define D 8
#endif

#ifndef LAMBDA
#define LAMBDA 8
#endif
#ifndef MU
#define MU (LAMBDA/2)
#endif

typedef struct {
    sep_cmaes_t es;

    /* Buffers (no malloc) */
    float x[D];
    float x_try[D];
    float c_diag[D];
    float p_sigma[D];
    float p_c[D];
    float z_try[D];
    float y_try[D];
    float store_y[MU * D];
    float fit_buf[LAMBDA];
    int   idx[LAMBDA];

    /* Optional bounds */
    float lo[D];
    float hi[D];

    uint32_t last_print_ms;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA)

/* Objective: Sphere (minimize) */
static float sphere_fn(const float *restrict x, int n) {
    float s = 0.0f;
    for (int i = 0; i < n; ++i) s += x[i] * x[i];
    return s;
}

/* Adapter for sep_cmaes_step() */
static float sphere_adapter(const float *x, int n, void *ud) {
    (void)ud;
    return sphere_fn(x, n);
}

void user_init(void) {
    srand(pogobot_helper_getRandSeed());

    main_loop_hz = 60;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;

    for (int i = 0; i < D; ++i) {
        mydata->x[i] = 1.5f;
        mydata->lo[i] = -2.0f;
        mydata->hi[i] =  2.0f;
    }

    sep_params_t p = {
        .mode = SEP_MINIMIZE,
        .lambda = LAMBDA,
        .mu = MU,
        .sigma0 = 0.3f,
        .sigma_min = 1e-6f,
        .sigma_max = 2.0f,
        .weights = NULL, /* default log-weights */
        .cc = 0.0f, .cs = 0.0f, .c1 = 0.0f, .cmu = 0.0f, .damps = 0.0f, /* defaults */
        .lo = mydata->lo,
        .hi = mydata->hi
    };

    sep_cmaes_init(&mydata->es, D,
                   mydata->x, mydata->x_try,
                   mydata->c_diag,
                   mydata->p_sigma, mydata->p_c,
                   mydata->z_try, mydata->y_try,
                   mydata->store_y,
                   mydata->fit_buf, mydata->idx,
                   &p);

    float f0 = sphere_fn(mydata->x, D);
    sep_cmaes_tell_initial(&mydata->es, f0);

    printf("[SEP-CMAES] init: f0=%.6f, sigma=%.4f\n", f0, sep_cmaes_sigma(&mydata->es));
    mydata->last_print_ms = current_time_milliseconds();
}

void user_step(void) {
    const int evals_per_tick = 1;
    for (int k = 0; k < evals_per_tick; ++k) {
        (void)sep_cmaes_step(&mydata->es, sphere_adapter, NULL);
    }

    /* Optional LED feedback */
    float f_now = sep_cmaes_step(&mydata->es, sphere_adapter, NULL);
    float g = f_now; if (g < 0.0f) g = 0.0f; if (g > 1.0f) g = 1.0f;
    uint8_t val = (uint8_t)(25.0f * (1.0f - g));
    pogobot_led_setColors(val, 0, val, 0);

    uint32_t now = current_time_milliseconds();
    if (now - mydata->last_print_ms > 2000) {
        const float *x = sep_cmaes_get_x(&mydata->es);
        printf("[SEP-CMAES] gen=%u it=%u sigma=%.5f x[0]=%.4f ... x[%d]=%.4f\n",
               sep_cmaes_generation(&mydata->es), sep_cmaes_iterations(&mydata->es),
               sep_cmaes_sigma(&mydata->es), x[0], D-1, x[D-1]);
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

