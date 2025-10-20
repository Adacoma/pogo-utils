/**
 * @file example_sep_cmaes_main.c
 * @brief Example: sep-CMA-ES inside the Pogobot/Pogosim control loop (online).
 *
 * Minimizes the Sphere function f(x)=sum_i x_i^2 in dimension D, with λ offspring per
 * generation but evaluating only a small number per control tick (evals_per_tick).
 * Prints progress every ~2 s.
 */
#include "pogobase.h"
#include "pogo-utils/sep_cmaes.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

#ifndef D
#define D 10
#endif

/* If 0, defaults to 4 + floor(3 ln n) */
#ifndef LAMBDA
#define LAMBDA 0
#endif

typedef struct {
    sep_cmaes_t es;

    /* ---- Buffers (no malloc) ---- */
    float m[D];               /* mean */
    float diagC[D];
    float invsqrtC[D];
    float ps[D], pc[D];
    float z[D], y[D];

    /* Offspring generation storage */
    /* Note: Max lambda if LAMBDA==0 is small; here we allocate for worst-case L up to, say, 32. */
#ifndef MAX_LAMBDA
#define MAX_LAMBDA 32
#endif
    float cand_X[MAX_LAMBDA * D];
    float fitness[MAX_LAMBDA];
    int   idx[MAX_LAMBDA];
    float weights[MAX_LAMBDA]; /* only first μ entries are used */

    float lo[D], hi[D];

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

    /* Initial mean and bounds */
    for (int i = 0; i < D; ++i) {
        mydata->m[i] = 1.5f;
        mydata->diagC[i] = 1.0f;     /* start with identity diag */
        mydata->lo[i] = -2.0f;
        mydata->hi[i] =  2.0f;
    }

    sep_cmaes_params_t p = {
        .mode = SEP_CMAES_MINIMIZE,
        .lambda = LAMBDA,        /* 0 => default based on D */
        .mu = 0,                 /* 0 => lambda/2 */
        .sigma0 = 0.3f,
        .sigma_min = 1e-8f,
        .sigma_max = 2.0f,
        .cs = 0.0f,              /* 0 => default */
        .ds = 0.0f,
        .cc = 0.0f,
        .c1_sep = 0.0f,
        .cmu_sep = 0.0f,
        .evals_per_tick = 2      /* evaluate 2 offspring per tick => online */
    };

    /* We don't know lambda before init (it may default). We just ensure buffers are big enough (MAX_LAMBDA). */
    sep_cmaes_init(&mydata->es, D,
                   mydata->m,
                   mydata->diagC, mydata->invsqrtC,
                   mydata->ps, mydata->pc,
                   mydata->z, mydata->y,
                   mydata->cand_X, mydata->fitness, mydata->idx,
                   mydata->weights,
                   mydata->lo, mydata->hi,
                   sphere_fn, NULL, &p);

    /* Sanity check: ensure lambda didn't exceed our static MAX_LAMBDA (compile-time guard) */
    if (mydata->es.lambda > MAX_LAMBDA) {
        printf("[sep-CMA-ES] ERROR: lambda=%d exceeds MAX_LAMBDA=%d. Recompile with higher MAX_LAMBDA.\n",
               mydata->es.lambda, MAX_LAMBDA);
        /* Optional: lower evals_per_tick to 0 to stop */
        while (1) { pogobot_led_setColors(50, 0, 0, 0); } /* hang visibly */
    }

    printf("[sep-CMA-ES] init: f0=%.6f, sigma=%.4f, lambda=%d, mu=%d\n",
           sep_cmaes_best_f(&mydata->es), sep_cmaes_sigma(&mydata->es),
           mydata->es.lambda, mydata->es.mu);

    mydata->last_print_ms = current_time_milliseconds();
}

void user_step(void) {
    /* Run a small batch of offspring evaluations (online) */
    float f = sep_cmaes_step(&mydata->es);

    /* LED feedback (lower f => brighter green) */
    float g = f; if (g < 0.0f) g = 0.0f; if (g > 1.0f) g = 1.0f;
    uint8_t val = (uint8_t)(25.0f * (1.0f - g));
    pogobot_led_setColors(val, 0, val, 0);

    uint32_t now = current_time_milliseconds();
    if (now - mydata->last_print_ms > 2000) {
        const float *m = sep_cmaes_mean(&mydata->es);
        printf("[sep-CMA-ES] gen=%u  evals=%llu  f=%.6f  sigma=%.5f  m[0]=%.4f ... m[%d]=%.4f\n",
               (unsigned)sep_cmaes_generation(&mydata->es),
               (unsigned long long)sep_cmaes_evals(&mydata->es),
               f, sep_cmaes_sigma(&mydata->es),
               m[0], D-1, m[D-1]);
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

