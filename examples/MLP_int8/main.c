#include "pogobase.h"
#include "pogo-utils/version.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

/* ---- MLP config: ~10k parameters ----
 * params = H*I + H + H*O + O
 * for I=32, H=160, O=32:
 * params = 160*32 + 160 + 160*32 + 32 = 10432
 */

#define MLP_INT8_INPUT_DIM   32
#define MLP_INT8_HIDDEN_DIM  160
#define MLP_INT8_OUTPUT_DIM  32
#define MLP_INT8_OUTPUT_HARD_TANH   /* optional: also hard-tanh on outputs */

#define BENCH_RUNS 100

#include "pogo-utils/MLP_int8.h"

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

/* Simple helper: random int8 in full range [-128, 127] (interpreted as Q0.7) */
static int8_t rand_int8_q0_7(void) {
    /* Take lower 8 bits of rand(), interpret as signed */
    return (int8_t)(rand() & 0xFF);
}

/* Benchmark a ~10k-parameter int8 MLP with hard tanh activations. */
void bench_mlp_int8(void) {
    printf0("\n");
    printf0("==============================================\n");
    printf0("=== int8 MLP benchmark (Q0.7, hard tanh)   ===\n");
    printf0("==============================================\n");

    static MLP_INT8 net;  /* ~10 kB in BSS for 10k params (int8) */

    int8_t in[MLP_INT8_INPUT_DIM];
    int8_t out[MLP_INT8_OUTPUT_DIM];
    volatile int8_t sink = 0;  /* prevents optimization-away */

    /* Parameter count */
    uint32_t n_params =
        (uint32_t)MLP_INT8_INPUT_DIM  * (uint32_t)MLP_INT8_HIDDEN_DIM +
        (uint32_t)MLP_INT8_HIDDEN_DIM +
        (uint32_t)MLP_INT8_HIDDEN_DIM * (uint32_t)MLP_INT8_OUTPUT_DIM +
        (uint32_t)MLP_INT8_OUTPUT_DIM;

    printf0("MLP_INT8 dims: in=%d, hidden=%d, out=%d, params=%lu\n",
            (int)MLP_INT8_INPUT_DIM,
            (int)MLP_INT8_HIDDEN_DIM,
            (int)MLP_INT8_OUTPUT_DIM,
            (unsigned long)n_params);

    /* Init RNG (deterministic for reproducibility) */
    srand(1);

    /* Fill weights and biases with random Q0.7 values */
    for (int i = 0; i < MLP_INT8_HIDDEN_DIM; ++i) {
        for (int j = 0; j < MLP_INT8_INPUT_DIM; ++j) {
            net.W1[i][j] = rand_int8_q0_7();
        }
        net.b1[i] = rand_int8_q0_7();
    }

    for (int i = 0; i < MLP_INT8_OUTPUT_DIM; ++i) {
        for (int j = 0; j < MLP_INT8_HIDDEN_DIM; ++j) {
            net.W2[i][j] = rand_int8_q0_7();
        }
        net.b2[i] = rand_int8_q0_7();
    }

    /* Random input vector */
    for (int i = 0; i < MLP_INT8_INPUT_DIM; ++i) {
        in[i] = rand_int8_q0_7();
    }

    /* Benchmark several forward passes */
    uint32_t elapsed_mlp = 0;

    pogobot_stopwatch_reset(&mydata->timer_it);
    for (uint16_t it = 0; it < BENCH_RUNS; ++it) {
        mlp_int8_forward(&net, in, out);

        /* Accumulate outputs so the compiler cannot remove the loop */
        for (int k = 0; k < MLP_INT8_OUTPUT_DIM; ++k) {
            sink = (int8_t)(sink + out[k]);  /* wrap is fine, we only want side-effects */
        }
    }
    elapsed_mlp = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);

    printf0("MLP_INT8: %u forward passes took %lu us (sink=%d)\n",
            (unsigned)BENCH_RUNS,
            (unsigned long)elapsed_mlp,
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

    // Benchmark int8 MLP
    bench_mlp_int8();
}

void user_step(void) {
    // ...
}

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    return 0;
}

