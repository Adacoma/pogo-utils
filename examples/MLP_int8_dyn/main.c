#include "pogobase.h"
#include "pogo-utils/version.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

/* ---- Dynamic MLP config ----
 *
 * Total params:
 *   H*I + H                      // first hidden layer
 *   + (L-1) * (H*H + H)          // remaining hidden layers
 *   + O*H + O                    // output layer
 *
 * For I=32, H=160, O=32, L=1:
 *   params = 160*32 + 160 + 160*32 + 32 = 10432  (~10k, like original example)
 */

#define MLP_INT8_DYN_INPUT_DIM        32
#define MLP_INT8_DYN_HIDDEN_DIM       160
#define MLP_INT8_DYN_OUTPUT_DIM       32
#define MLP_INT8_DYN_NUM_HIDDEN_LAYERS 1   /* set >1 to use multiple hidden layers */

#define MLP_INT8_DYN_OUTPUT_HARD_TANH     /* optional: hard-tanh on outputs */

#define BENCH_RUNS 100

/* Compile-time parameter count for the chosen architecture */
#define MLP_INT8_DYN_PARAM_COUNT (                                   \
    (uint32_t)(MLP_INT8_DYN_HIDDEN_DIM) * (uint32_t)(MLP_INT8_DYN_INPUT_DIM) + \
    (uint32_t)(MLP_INT8_DYN_HIDDEN_DIM) +                            \
    (uint32_t)((MLP_INT8_DYN_NUM_HIDDEN_LAYERS) > 1 ?                \
        ((MLP_INT8_DYN_NUM_HIDDEN_LAYERS - 1) *                      \
        ((uint32_t)MLP_INT8_DYN_HIDDEN_DIM * (uint32_t)MLP_INT8_DYN_HIDDEN_DIM + \
         (uint32_t)MLP_INT8_DYN_HIDDEN_DIM)) : 0) +                  \
    (uint32_t)(MLP_INT8_DYN_OUTPUT_DIM) * (uint32_t)(MLP_INT8_DYN_HIDDEN_DIM) + \
    (uint32_t)(MLP_INT8_DYN_OUTPUT_DIM)                              \
)

#include "pogo-utils/MLP_int8_dyn.h"

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

/* Simple helper: small-range random int8 in [-16, 16] (Q0.7).
 * This avoids too much early saturation compared to full [-128,127]. */
static int8_t rand_int8_small_q0_7(void) {
    /* Uniform in [0, 33), shifted to [-16, 16] */
    int r = rand() % 33;
    return (int8_t)(r - 16);
}

/* Fill the flat parameter array for the dynamic MLP.
 * Layout is the one used in MLP_int8_dyn.h:
 *
 *   1) First hidden layer: input -> hidden
 *        W0: H x I
 *        b0: H
 *   2) Hidden layers 1..L-1: hidden -> hidden
 *        For each l in [1, L-1]:
 *          Wl: H x H
 *          bl: H
 *   3) Output layer: hidden -> output
 *        W_out: O x H
 *        b_out: O
 */
static void mlp_int8_dyn_random_init_params(int8_t *params,
                                            uint16_t input_dim,
                                            uint16_t hidden_dim,
                                            uint16_t output_dim,
                                            uint8_t  num_hidden_layers)
{
    int8_t *p = params;

    uint16_t I = input_dim;
    uint16_t H = hidden_dim;
    uint16_t O = output_dim;
    uint8_t  L = num_hidden_layers;

    /* First hidden layer: W0 (H x I), b0 (H) */
    uint32_t count_W0 = (uint32_t)H * (uint32_t)I;
    for (uint32_t i = 0; i < count_W0; ++i) {
        p[i] = rand_int8_small_q0_7();
    }
    p += count_W0;
    for (uint16_t i = 0; i < H; ++i) {
        p[i] = rand_int8_small_q0_7();
    }
    p += H;

    /* Remaining hidden layers: for each l in [1, L-1], Wl (H x H), bl (H) */
    for (uint8_t layer = 1; layer < L; ++layer) {
        uint32_t count_W = (uint32_t)H * (uint32_t)H;
        for (uint32_t i = 0; i < count_W; ++i) {
            p[i] = rand_int8_small_q0_7();
        }
        p += count_W;
        for (uint16_t i = 0; i < H; ++i) {
            p[i] = rand_int8_small_q0_7();
        }
        p += H;
    }

    /* Output layer: W_out (O x H), b_out (O) */
    uint32_t count_Wout = (uint32_t)O * (uint32_t)H;
    for (uint32_t i = 0; i < count_Wout; ++i) {
        p[i] = rand_int8_small_q0_7();
    }
    p += count_Wout;
    for (uint16_t i = 0; i < O; ++i) {
        p[i] = rand_int8_small_q0_7();
    }

    /* Optional: you could keep biases at 0 instead, if you prefer:
     *   memset(..., 0, H) / memset(..., 0, O);
     */
}

/* Benchmark a dynamic int8 MLP with hard tanh activations. */
void bench_mlp_int8_dyn(void) {
    printf0("\n");
    printf0("=================================================\n");
    printf0("=== int8 DYN MLP benchmark (Q0.7, hard tanh)  ===\n");
    printf0("=================================================\n");

    static MLP_int8_dyn net;

    /* Flat parameter buffer (owned here, no duplication in the library) */
    static int8_t mlp_params[MLP_INT8_DYN_PARAM_COUNT];

    int8_t in[MLP_INT8_DYN_INPUT_DIM];
    int8_t out[MLP_INT8_DYN_OUTPUT_DIM];
    volatile int8_t sink = 0;  /* prevents optimization-away */

    /* Compute parameter count via macro and via helper, and assert they match */
    uint32_t n_params_macro = (uint32_t)MLP_INT8_DYN_PARAM_COUNT;
    uint32_t n_params_func  = mlp_int8_dyn_param_count(
        MLP_INT8_DYN_INPUT_DIM,
        MLP_INT8_DYN_HIDDEN_DIM,
        MLP_INT8_DYN_OUTPUT_DIM,
        MLP_INT8_DYN_NUM_HIDDEN_LAYERS
    );
    assert(n_params_macro == n_params_func);

    printf0("MLP_INT8_DYN dims: in=%d, hidden=%d, out=%d, L=%d, params=%lu\n",
            (int)MLP_INT8_DYN_INPUT_DIM,
            (int)MLP_INT8_DYN_HIDDEN_DIM,
            (int)MLP_INT8_DYN_OUTPUT_DIM,
            (int)MLP_INT8_DYN_NUM_HIDDEN_LAYERS,
            (unsigned long)n_params_macro);

    /* Init RNG (deterministic for reproducibility) */
    srand(1);

    /* Fill parameters with small-range random values */
    mlp_int8_dyn_random_init_params(
        mlp_params,
        MLP_INT8_DYN_INPUT_DIM,
        MLP_INT8_DYN_HIDDEN_DIM,
        MLP_INT8_DYN_OUTPUT_DIM,
        MLP_INT8_DYN_NUM_HIDDEN_LAYERS
    );

    /* Initialize network descriptor (no allocation/copy) */
    mlp_int8_dyn_init(
        &net,
        MLP_INT8_DYN_INPUT_DIM,
        MLP_INT8_DYN_HIDDEN_DIM,
        MLP_INT8_DYN_OUTPUT_DIM,
        MLP_INT8_DYN_NUM_HIDDEN_LAYERS,
        mlp_params
    );

    /* Random input vector */
    for (int i = 0; i < MLP_INT8_DYN_INPUT_DIM; ++i) {
        in[i] = rand_int8_small_q0_7();
    }

    /* Benchmark several forward passes */
    uint32_t elapsed_mlp = 0;

    pogobot_stopwatch_reset(&mydata->timer_it);
    for (uint16_t it = 0; it < BENCH_RUNS; ++it) {
        mlp_int8_dyn_forward(&net, in, out);

        /* Accumulate outputs so the compiler cannot remove the loop */
        for (int k = 0; k < MLP_INT8_DYN_OUTPUT_DIM; ++k) {
            sink = (int8_t)(sink + out[k]);  /* wrap is fine, we only want side-effects */
        }
    }
    elapsed_mlp = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);

    printf0("MLP_INT8_DYN: %u forward passes took %lu us (sink=%d)\n",
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

    // Benchmark dynamic int8 MLP
    bench_mlp_int8_dyn();
}

void user_step(void) {
    // ...
}

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    return 0;
}

