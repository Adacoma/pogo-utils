#include "pogobase.h"
#include "pogo-utils/fixp.h"
#include "pogo-utils/version.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdlib.h>

/* ---- MLP config: ~10k parameters ----
 * params = H*I + H + H*O + O
 * for I=32, H=160, O=32:
 * params = 160*32 + 160 + 160*32 + 32 = 10432
 */

#define MLP_Q1_15_INPUT_DIM   32
#define MLP_Q1_15_HIDDEN_DIM  160
#define MLP_Q1_15_OUTPUT_DIM  32
#define MLP_Q1_15_OUTPUT_HARD_TANH   /* optional: also hard-tanh on outputs */

#define BENCH_RUNS 100

#include "pogo-utils/MLP_Q1_15.h"

typedef struct {
    time_reference_t timer_it;
} USERDATA;
DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

#ifdef SIMULATOR
#define printf0(fmt, ...) if (pogobot_helper_getid() == 0) { printf(fmt, ##__VA_ARGS__ ); }
#define printf_fixp0(fmt, ...) if (pogobot_helper_getid() == 0) { printf_fixp(fmt, ##__VA_ARGS__ ); }
#else
#define printf0(fmt, ...) printf(fmt, ##__VA_ARGS__ );
#define printf_fixp0(fmt, ...) printf_fixp(fmt, ##__VA_ARGS__ );
#endif


/* Simple helper: random Q1.15 in full range [-1, 1) */
static q1_15_t rand_q1_15(void) {
    /* Take lower 16 bits of rand(), interpret as signed */
    int16_t r = (int16_t)rand();
    return (q1_15_t)r;
}

/* Benchmark a ~10k-parameter Q1.15 MLP with hard tanh activations. */
void bench_mlp_q1_15(void) {
    printf0("\n");
    printf0("==============================================\n");
    printf0("=== Q1.15 MLP benchmark (hard tanh)        ===\n");
    printf0("==============================================\n");

    static MLP_Q1_15 net;  /* ~20 kB in BSS for 10k params (int16) */

    q1_15_t in[MLP_Q1_15_INPUT_DIM];
    q1_15_t out[MLP_Q1_15_OUTPUT_DIM];
    volatile q1_15_t sink = 0;  /* prevents optimization-away */

    /* Parameter count */
    uint32_t n_params =
        (uint32_t)MLP_Q1_15_INPUT_DIM  * (uint32_t)MLP_Q1_15_HIDDEN_DIM +
        (uint32_t)MLP_Q1_15_HIDDEN_DIM +
        (uint32_t)MLP_Q1_15_HIDDEN_DIM * (uint32_t)MLP_Q1_15_OUTPUT_DIM +
        (uint32_t)MLP_Q1_15_OUTPUT_DIM;

    printf0("MLP_Q1_15 dims: in=%d, hidden=%d, out=%d, params=%lu\n",
            (int)MLP_Q1_15_INPUT_DIM,
            (int)MLP_Q1_15_HIDDEN_DIM,
            (int)MLP_Q1_15_OUTPUT_DIM,
            (unsigned long)n_params);

    /* Init RNG (deterministic for reproducibility) */
    srand(1);

    /* Fill weights and biases with random Q1.15 values */
    for (int i = 0; i < MLP_Q1_15_HIDDEN_DIM; ++i) {
        for (int j = 0; j < MLP_Q1_15_INPUT_DIM; ++j) {
            net.W1[i][j] = rand_q1_15();
        }
        net.b1[i] = rand_q1_15();
    }

    for (int i = 0; i < MLP_Q1_15_OUTPUT_DIM; ++i) {
        for (int j = 0; j < MLP_Q1_15_HIDDEN_DIM; ++j) {
            net.W2[i][j] = rand_q1_15();
        }
        net.b2[i] = rand_q1_15();
    }

    /* Random input vector */
    for (int i = 0; i < MLP_Q1_15_INPUT_DIM; ++i) {
        in[i] = rand_q1_15();
    }

    /* Benchmark several forward passes */
    uint32_t elapsed_mlp = 0;

    pogobot_stopwatch_reset(&mydata->timer_it);
    for (uint16_t it = 0; it < BENCH_RUNS; ++it) {
        mlp_q1_15_forward(&net, in, out);

        /* Accumulate outputs so the compiler cannot remove the loop */
        for (int k = 0; k < MLP_Q1_15_OUTPUT_DIM; ++k) {
            sink = q1_15_add(sink, out[k]);
        }
    }
    elapsed_mlp = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);

    printf0("MLP_Q1_15: %u forward passes took %lu us (sink=%d)\n",
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

    // Initialize fixp (e.g. create LUT)
    init_fixp();

    // Benchmark Q1.15 MLP
    bench_mlp_q1_15();
}

void user_step(void) {
    // ...
}

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    return 0;
}


