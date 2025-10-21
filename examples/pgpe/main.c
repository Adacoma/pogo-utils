/**
 * @brief Example: PGPE ask–tell inside a Pogobot/Pogosim control loop.
 *
 * Minimizes Sphere f(x)=sum_i x_i^2 in dimension D (mode=MINIMIZE).
 * Prints progress every ~2s. Uses antithetic pairs (two evals per update).
 */
#include "pogobase.h"
#include "pogo-utils/pgpe.h"   /* place pgpe.{h,c} under pogo-utils/ */
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#ifndef D
#define D 8
#endif

typedef struct {
    pgpe_t pg;

    /* Buffers (no malloc) */
    float mu[D];     /* will converge toward 0 */
    float sigma[D];  /* exploration scales */
    float x_try[D];  /* candidate */
    float eps[D];    /* noise */

    float lo[D];
    float hi[D];

    uint32_t last_print_ms;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA)

/* Objective: Sphere (minimize) */
static float sphere_fn(const float *restrict x, int n, void *userdata) {
    (void)userdata;
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

    /* Init μ, σ and bounds */
    for (int i = 0; i < D; ++i) {
        mydata->mu[i]    = 1.5f;   /* start away from optimum */
        mydata->sigma[i] = 0.20f;  /* initial exploration */
        mydata->lo[i]    = -2.0f;
        mydata->hi[i]    =  2.0f;
    }

    pgpe_params_t p = {
        .mode = PGPE_MINIMIZE,
        .eta_mu = 0.05f,
        .eta_sigma = 0.10f,
        .sigma_min = 1e-5f,
        .sigma_max = 0.8f,
        .baseline_alpha = 0.10f,
        .normalize_pair = 1
    };

    pgpe_init(&mydata->pg, D,
              mydata->mu, mydata->sigma,
              mydata->x_try, mydata->eps,
              mydata->lo, mydata->hi,
              &p);

    mydata->last_print_ms = current_time_milliseconds();

    printf("[PGPE] init: |mu|^2=%.3f  sigma[0]=%.4f\n",
           sphere_fn(mydata->mu, D, NULL), mydata->sigma[0]);
}

void user_step(void) {
    /* Two evaluations complete one PGPE update (antithetic pair). */
    const int evals_per_tick = 2; /* could be 1 if your loop needs it tighter */
    for (int k = 0; k < evals_per_tick; ++k) {
        const float *theta = pgpe_ask(&mydata->pg);
        float f = sphere_fn(theta, D, NULL);
        pgpe_tell(&mydata->pg, f);
    }

    /* Optional LED feedback proportional to progress (lower f => brighter) */
    float f_now = sphere_fn(pgpe_get_mu(&mydata->pg), D, NULL);
    float g = f_now; if (g < 0.0f) g = 0.0f; if (g > 1.0f) g = 1.0f;
    uint8_t val = (uint8_t)(25.0f * (1.0f - g));
    pogobot_led_setColors(val, 0, val, 0);

    uint32_t now = current_time_milliseconds();
    if (now - mydata->last_print_ms > 2000) {
        const float *mu = pgpe_get_mu(&mydata->pg);
        const float *sg = pgpe_get_sigma(&mydata->pg);
        printf("[PGPE] it=%u  f=%.6f  mu[0]=%.4f ... mu[%d]=%.4f  sigma[0]=%.5f\n",
               pgpe_iterations(&mydata->pg), f_now, mu[0], D-1, mu[D-1], sg[0]);
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

