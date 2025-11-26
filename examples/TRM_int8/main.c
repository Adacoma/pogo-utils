#include "pogobase.h"
#include "pogo-utils/version.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

/* ---- TRM config: ~9.4k parameters ----
 * Trunk:
 *   W1: H x (X+Y+Z)
 *   b1: H
 *   W2: H x H
 *   b2: H
 * Heads:
 *   Wz: Z x H,  bz: Z
 *   Wy: Y x H,  by: Y
 *
 * Params = H*(X+Y+Z) + H + H*H + H + Z*H + Z + Y*H + Y
 * For X=16, Y=16, Z=16, H=64:
 * Params = 9376
 */

#define TRM_INT8_X_DIM        16
#define TRM_INT8_Y_DIM        16
#define TRM_INT8_Z_DIM        16
#define TRM_INT8_HIDDEN_DIM   64

/* Recursive schedule (can be tuned at compile time) */
#define TRM_INT8_N_LATENT     4
#define TRM_INT8_T_STEPS      3

#define BENCH_RUNS 100

/* Include TRM implementation */
#include "pogo-utils/TRM_int8.h"

typedef struct {
    time_reference_t timer_it;
} USERDATA;
DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

#ifdef SIMULATOR
#define printf0(fmt, ...) if (pogobot_helper_getid() == 0) { printf(fmt, ##__VA_ARGS__ ); }
#else
#define printf0(fmt, ...) printf(fmt, ##__VA_ARGS__ );
#endif

/* Random int8 in full range [-128, 127] (interpreted as Q0.7). */
static int8_t rand_int8_q0_7(void) {
    return (int8_t)(rand() & 0xFF);
}

/* Benchmark a ~9.4k-parameter int8 TRM with hard tanh activations. */
void bench_trm_int8(void) {
    printf0("\n");
    printf0("==============================================\n");
    printf0("=== int8 TRM benchmark (Q0.7, hard tanh)   ===\n");
    printf0("==============================================\n");

    static TRM_INT8 net;  /* ~9.4 kB in BSS for 9376 int8 params */

    int8_t x[TRM_INT8_X_DIM];
    int8_t y[TRM_INT8_Y_DIM];
    int8_t z[TRM_INT8_Z_DIM];
    volatile int8_t sink = 0;  /* prevents optimization-away */

    /* Parameter count (must match comment above) */
    uint32_t n_params =
        (uint32_t)TRM_INT8_HIDDEN_DIM * (uint32_t)(TRM_INT8_X_DIM + TRM_INT8_Y_DIM + TRM_INT8_Z_DIM) +
        (uint32_t)TRM_INT8_HIDDEN_DIM +
        (uint32_t)TRM_INT8_HIDDEN_DIM * (uint32_t)TRM_INT8_HIDDEN_DIM +
        (uint32_t)TRM_INT8_HIDDEN_DIM +
        (uint32_t)TRM_INT8_Z_DIM * (uint32_t)TRM_INT8_HIDDEN_DIM +
        (uint32_t)TRM_INT8_Z_DIM +
        (uint32_t)TRM_INT8_Y_DIM * (uint32_t)TRM_INT8_HIDDEN_DIM +
        (uint32_t)TRM_INT8_Y_DIM;

    printf0("TRM_INT8 dims: X=%d, Y=%d, Z=%d, H=%d, params=%lu\n",
            (int)TRM_INT8_X_DIM,
            (int)TRM_INT8_Y_DIM,
            (int)TRM_INT8_Z_DIM,
            (int)TRM_INT8_HIDDEN_DIM,
            (unsigned long)n_params);

    printf0("TRM recursion: T=%d outer steps, n=%d latent steps\n",
            (int)TRM_INT8_T_STEPS,
            (int)TRM_INT8_N_LATENT);

    /* Init RNG (deterministic for reproducibility) */
    srand(1);

    /* Fill shared trunk with random Q0.7 values */
    for (int i = 0; i < TRM_INT8_HIDDEN_DIM; ++i) {
        for (int j = 0; j < TRM_INT8_CORE_INPUT_DIM; ++j) {
            net.W1[i][j] = rand_int8_q0_7();
        }
        net.b1[i] = rand_int8_q0_7();
    }

    for (int i = 0; i < TRM_INT8_HIDDEN_DIM; ++i) {
        for (int j = 0; j < TRM_INT8_HIDDEN_DIM; ++j) {
            net.W2[i][j] = rand_int8_q0_7();
        }
        net.b2[i] = rand_int8_q0_7();
    }

    /* Heads */
    for (int i = 0; i < TRM_INT8_Z_DIM; ++i) {
        for (int j = 0; j < TRM_INT8_HIDDEN_DIM; ++j) {
            net.Wz[i][j] = rand_int8_q0_7();
        }
        net.bz[i] = rand_int8_q0_7();
    }

    for (int i = 0; i < TRM_INT8_Y_DIM; ++i) {
        for (int j = 0; j < TRM_INT8_HIDDEN_DIM; ++j) {
            net.Wy[i][j] = rand_int8_q0_7();
        }
        net.by[i] = rand_int8_q0_7();
    }

    /* Random question x, and initial y,z (here random, but you could use zeros). */
    for (int i = 0; i < TRM_INT8_X_DIM; ++i) x[i] = rand_int8_q0_7();
    for (int i = 0; i < TRM_INT8_Y_DIM; ++i) y[i] = rand_int8_q0_7();
    for (int i = 0; i < TRM_INT8_Z_DIM; ++i) z[i] = rand_int8_q0_7();

    /* Benchmark several forward passes */
    uint32_t elapsed_trm = 0;

    pogobot_stopwatch_reset(&mydata->timer_it);
    for (uint16_t it = 0; it < BENCH_RUNS; ++it) {
        /* Copy initial y,z so each iteration has same starting point */
        int8_t y_run[TRM_INT8_Y_DIM];
        int8_t z_run[TRM_INT8_Z_DIM];

        for (int i = 0; i < TRM_INT8_Y_DIM; ++i) y_run[i] = y[i];
        for (int i = 0; i < TRM_INT8_Z_DIM; ++i) z_run[i] = z[i];

        trm_int8_forward(&net, x, y_run, z_run);

        /* Accumulate outputs so the compiler cannot remove the loop */
        for (int k = 0; k < TRM_INT8_Y_DIM; ++k) {
            sink = (int8_t)(sink + y_run[k]);  /* wrap is fine */
        }
    }
    elapsed_trm = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);

    printf0("TRM_INT8: %u forward recursions took %lu us (sink=%d)\n",
            (unsigned)BENCH_RUNS,
            (unsigned long)elapsed_trm,
            (int)sink);
}

void user_init(void) {
#ifndef SIMULATOR
    printf0("setup ok\n");
#endif
    // Set main loop frequency, message sending frequency, message processing frequency
    main_loop_hz = 20;
    max_nb_processed_msg_per_tick = 0;
    // Specify functions to send/transmit messages
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;

    // Set led index to show error codes
    error_codes_led_idx = 3; // Default value, negative values to disable

    // Benchmark int8 TRM
    bench_trm_int8();
}

void user_step(void) {
    // ...
}

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    return 0;
}

