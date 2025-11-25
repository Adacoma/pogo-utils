#include "pogobase.h"
#include "pogo-utils/version.h"
#include <stdio.h>
#include <stdint.h>

#define MLP_INT8_INPUT_DIM   784
#define MLP_INT8_HIDDEN_DIM  32   /* must match the Python script's --hidden-dim */
#define MLP_INT8_OUTPUT_DIM  10
#define MLP_INT8_OUTPUT_HARD_TANH

#include "pogo-utils/MLP_int8.h"

/* Declarations of parameters & data generated in mnist_mlp_params.c */
extern MLP_INT8 mnist_mlp;
extern const int MNIST_NUM_TEST_IMAGES;
extern const int8_t mnist_images[][MLP_INT8_INPUT_DIM];
extern const uint8_t mnist_labels[];

/* Userdata, same style as your benchmark example */
typedef struct {
    time_reference_t timer_it;
} USERDATA;
DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

#ifdef SIMULATOR
#define printf0(fmt, ...) \
    if (pogobot_helper_getid() == 0) { printf(fmt, ##__VA_ARGS__); }
#else
#define printf0(fmt, ...) printf(fmt, ##__VA_ARGS__)
#endif

static int argmax_int32(const int32_t *v, int n) {
    int best_idx = 0;
    int32_t best_val = v[0];
    for (int i = 1; i < n; ++i) {
        if (v[i] > best_val) {
            best_val = v[i];
            best_idx = i;
        }
    }
    return best_idx;
}


/* Run the int8 MNIST MLP on a few stored images */
static void run_mnist_demo(void) {
    printf0("\n");
    printf0("==============================================\n");
    printf0("=== int8 MNIST MLP demo (Q0.7, hard tanh)  ===\n");
    printf0("==============================================\n");

    printf0("MLP_INT8 dims: in=%d, hidden=%d, out=%d\n",
            (int)MLP_INT8_INPUT_DIM,
            (int)MLP_INT8_HIDDEN_DIM,
            (int)MLP_INT8_OUTPUT_DIM);

    int correct = 0;
    int total = MNIST_NUM_TEST_IMAGES;

    int32_t out[MLP_INT8_OUTPUT_DIM];

    for (int i = 0; i < total; ++i) {
        /* Forward pass */
        mlp_int8_forward_logits32(&mnist_mlp, mnist_images[i], out);

        int pred = argmax_int32(out, MLP_INT8_OUTPUT_DIM);
        int label = mnist_labels[i];

        if (pred == label) {
            ++correct;
        }

        printf0("Image %2d: label=%d, pred=%d [", i, label, pred);
        for (int k = 0; k < MLP_INT8_OUTPUT_DIM; ++k) {
            printf0("%d%s", (int)out[k], (k + 1 < MLP_INT8_OUTPUT_DIM) ? ", " : "");
        }
        printf0("]\n");
    }

    printf0("Summary: %d/%d correct (%.1f%%)\n",
            correct, total,
            100.0f * (float)correct / (float)total);
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

    // Set LED index to show error codes
    error_codes_led_idx = 3; // Default value, negative values to disable

    run_mnist_demo();
}

void user_step(void) {
    // Nothing to do in the main loop for this demo
}

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    return 0;
}

