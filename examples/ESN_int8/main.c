#include "pogobase.h"
#include "pogo-utils/version.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <stdint.h>

/* ---- ESN config (sparse reservoir) ----
 *
 * We use a 1D Mackey–Glass system for input/output:
 *   input_dim  = 1
 *   output_dim = 1
 *
 * Reservoir dimension and fixed-K can be tuned.
 * Keep in mind the training uses O(R^2) floats in static buffers.
 */

#define ESN_INT8_INPUT_DIM      1
#define ESN_INT8_RESERVOIR_DIM  64
#define ESN_INT8_OUTPUT_DIM     1
#define ESN_INT8_RES_FIXED_K    4

#include "pogo-utils/ESN_int8.h"

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

/* ---- Mackey–Glass parameters ----
 *
 * dx/dt = beta * x(t - tau) / (1 + x(t - tau)^n) - gamma * x(t)
 *
 * We discretize with Euler scheme:
 *   x_{t+1} = x_t + dt * RHS
 */

#define MG_BETA   0.2f
#define MG_GAMMA  0.1f
#define MG_N      10.0f
#define MG_TAU    17          /* integer delay */
#define MG_DT     1.0f

/* Data lengths (can be tuned) */
#define MG_WASHOUT_STEPS  1000
#define MG_TRAIN_STEPS    2000
#define MG_TEST_STEPS     2000

/* We need at least: tau + 1 + washout + train + test */
#define MG_TOTAL_STEPS (MG_TAU + 1 + MG_WASHOUT_STEPS + MG_TRAIN_STEPS + MG_TEST_STEPS)

/* Time series buffers */
static float  mg_series[MG_TOTAL_STEPS];            /* raw continuous values */
static float  mg_mid = 0.0f;
static float  mg_half_range = 1.0f;

static int8_t mg_input_q[MG_TOTAL_STEPS - 1][ESN_INT8_INPUT_DIM];
static int8_t mg_target_q[MG_TOTAL_STEPS - 1][ESN_INT8_OUTPUT_DIM];

/* Generate Mackey–Glass time series */
static void generate_mackey_glass(void) {
    /* Initial condition: small random fluctuations around 1.2 */
    for (int t = 0; t <= MG_TAU; ++t) {
        mg_series[t] = 1.2f + 0.01f * ((float)rand() / (float)RAND_MAX - 0.5f);
    }

    for (int t = MG_TAU; t < MG_TOTAL_STEPS - 1; ++t) {
        float x_t      = mg_series[t];
        float x_tau    = mg_series[t - MG_TAU];
        float num      = MG_BETA * x_tau;
        float den      = 1.0f + powf(x_tau, MG_N);
        float rhs      = num / den - MG_GAMMA * x_t;
        float x_next   = x_t + MG_DT * rhs;
        mg_series[t + 1] = x_next;
    }

    /* Compute min/max for scaling */
    float xmin = mg_series[0];
    float xmax = mg_series[0];
    for (int t = 1; t < MG_TOTAL_STEPS; ++t) {
        if (mg_series[t] < xmin) xmin = mg_series[t];
        if (mg_series[t] > xmax) xmax = mg_series[t];
    }

    mg_mid        = 0.5f * (xmax + xmin);
    mg_half_range = 0.5f * (xmax - xmin);
    if (mg_half_range < 1e-6f) mg_half_range = 1e-6f;

    /* Build normalized and quantized inputs/targets:
     *   Input(t)  = normalized x(t)
     *   Target(t) = normalized x(t+1)
     * for t = 0 .. MG_TOTAL_STEPS-2
     */
    for (int t = 0; t < MG_TOTAL_STEPS - 1; ++t) {
        float x_t   = mg_series[t];
        float x_tp1 = mg_series[t + 1];

        float xn    = (x_t   - mg_mid) / mg_half_range;   /* ~[-1,1] */
        float yn    = (x_tp1 - mg_mid) / mg_half_range;

        if (xn > 0.999f) xn = 0.999f;
        if (xn < -0.999f) xn = -0.999f;
        if (yn > 0.999f) yn = 0.999f;
        if (yn < -0.999f) yn = -0.999f;

        mg_input_q[t][0]  = esn_int8_sparse_from_float(xn);
        mg_target_q[t][0] = esn_int8_sparse_from_float(yn);
    }
}

/* Compute MSE on a segment [seg_start, seg_start + seg_len) of the
 * input/target sequences (same index convention as mg_input_q).
 *
 * We:
 *  - reset ESN state,
 *  - run ESN from t=0 up to t=seg_start-1 to warm-up,
 *  - then accumulate errors over seg_len steps.
 */
static void compute_mse_segment(
    ESN_INT8_SPARSE *net,
    const int8_t inputs[][ESN_INT8_INPUT_DIM],
    const int8_t targets[][ESN_INT8_OUTPUT_DIM],
    int total_len,
    int seg_start,
    int seg_len,
    float *mse_norm,
    float *mse_orig)
{
    assert(seg_start >= 0);
    assert(seg_start + seg_len <= total_len);

    esn_int8_sparse_reset_state(net);

    int8_t out[ESN_INT8_OUTPUT_DIM];

    /* Warm-up from t=0..seg_start-1 */
    for (int t = 0; t < seg_start; ++t) {
        esn_int8_sparse_step(net, inputs[t], out);
    }

    /* Accumulate squared error */
    float se_norm = 0.0f;
    float se_orig = 0.0f;

    for (int n = 0; n < seg_len; ++n) {
        int t = seg_start + n;

        esn_int8_sparse_step(net, inputs[t], out);

        float y_pred_norm = esn_int8_sparse_to_float(out[0]);
        float y_true_norm = esn_int8_sparse_to_float(targets[t][0]);

        float err_n = y_pred_norm - y_true_norm;
        se_norm += err_n * err_n;

        float y_pred_orig = y_pred_norm * mg_half_range + mg_mid;
        float y_true_orig = y_true_norm * mg_half_range + mg_mid;

        float err_o = y_pred_orig - y_true_orig;
        se_orig += err_o * err_o;
    }

    *mse_norm = se_norm / (float)seg_len;
    *mse_orig = se_orig / (float)seg_len;
}

/* Full training + evaluation on Mackey–Glass */
static void run_esn_mackey_glass_demo(void) {
    printf0("\n");
    printf0("==============================================\n");
    printf0("=== int8 sparse ESN on Mackey–Glass       ===\n");
    printf0("==============================================\n");

    printf0("Config: input_dim=%d, reservoir_dim=%d, output_dim=%d, K=%d\n",
            (int)ESN_INT8_INPUT_DIM,
            (int)ESN_INT8_RESERVOIR_DIM,
            (int)ESN_INT8_OUTPUT_DIM,
            (int)ESN_INT8_RES_FIXED_K);

    /* Info: parameter count (for curiosity) */
    uint32_t n_params =
        (uint32_t)ESN_INT8_RESERVOIR_DIM * (uint32_t)ESN_INT8_INPUT_DIM +   /* Win */
        (uint32_t)ESN_INT8_RESERVOIR_DIM * (uint32_t)ESN_INT8_RES_FIXED_K + /* Wres (sparse) */
        (uint32_t)ESN_INT8_OUTPUT_DIM    * (uint32_t)ESN_INT8_RESERVOIR_DIM + /* Wout */
        (uint32_t)ESN_INT8_OUTPUT_DIM    * (uint32_t)ESN_INT8_INPUT_DIM +   /* Wout_in */
        (uint32_t)ESN_INT8_RESERVOIR_DIM +                                  /* b_res */
        (uint32_t)ESN_INT8_OUTPUT_DIM;                                      /* b_out */

    printf0("Approx. trainable+fixed parameters: %lu (int8)\n",
            (unsigned long)n_params);

    /* Init RNG */
    srand(1);

    /* Generate Mackey–Glass data */
    generate_mackey_glass();

    /* Instantiate ESN */
    static ESN_INT8_SPARSE net;
    esn_int8_sparse_init_random_reservoir(&net);

    /* Initially set readout weights to zero (they'll be overwritten by training) */
    for (int o = 0; o < ESN_INT8_OUTPUT_DIM; ++o) {
        net.b_out[o] = 0;
        for (int j = 0; j < ESN_INT8_RESERVOIR_DIM; ++j) {
            net.Wout[o][j] = 0;
        }
        for (int j = 0; j < ESN_INT8_INPUT_DIM; ++j) {
            net.Wout_in[o][j] = 0;
        }
    }

    /* Training region:
     *  - We use the first (MG_TAU+1) points to initialize delay.
     *  - Training inputs/targets start at t=MG_TAU and go up to
     *    t = MG_TAU + MG_WASHOUT_STEPS + MG_TRAIN_STEPS - 1.
     *
     * Since mg_input_q[t] is defined for t in [0, MG_TOTAL_STEPS-2],
     * we set:
     */
    const int train_start = MG_TAU;  /* index in mg_input_q */
    const int train_len   = MG_WASHOUT_STEPS + MG_TRAIN_STEPS;
    const int total_len   = MG_TOTAL_STEPS - 1; /* last index usable in mg_input_q */

    assert(train_start + train_len <= total_len);

    /* Pointers to training sequences */
    const int8_t *train_inputs  = &mg_input_q[train_start][0];
    const int8_t *train_targets = &mg_target_q[train_start][0];

    /* Train readout using ridge regression on normalized data */
    float ridge_lambda = 1e-6f;
    int   washout      = MG_WASHOUT_STEPS;
    int   use_direct_input = 1;

    printf0("Training readout (T=%d, washout=%d, lambda=%.1e)...\n",
            train_len, washout, ridge_lambda);

    esn_int8_sparse_train_readout_ridge(
        &net,
        train_inputs,
        train_targets,
        train_len,
        washout,
        ridge_lambda,
        use_direct_input);

    /* Evaluate training MSE:
     *   segment starts after washout within train region.
     *   seg_start (global index) = train_start + washout.
     */
    int train_seg_start_global = train_start + washout;
    int train_seg_len          = MG_TRAIN_STEPS;

    float mse_train_norm = 0.0f;
    float mse_train_orig = 0.0f;

    compute_mse_segment(
        &net,
        mg_input_q,
        mg_target_q,
        total_len,
        train_seg_start_global,
        train_seg_len,
        &mse_train_norm,
        &mse_train_orig);

    /* Evaluate test MSE:
     *   test starts right after the training region.
     */
    int test_seg_start_global = train_start + train_len;
    int test_seg_len          = MG_TEST_STEPS;
    assert(test_seg_start_global + test_seg_len <= total_len);

    float mse_test_norm = 0.0f;
    float mse_test_orig = 0.0f;

    compute_mse_segment(
        &net,
        mg_input_q,
        mg_target_q,
        total_len,
        test_seg_start_global,
        test_seg_len,
        &mse_test_norm,
        &mse_test_orig);

    printf0("Training MSE (normalized units): %g\n", mse_train_norm);
    printf0("Training MSE (original units):   %g\n", mse_train_orig);
    printf0("Test MSE (normalized units):     %g\n", mse_test_norm);
    printf0("Test MSE (original units):       %g\n", mse_test_orig);
}

void user_init(void) {
#ifndef SIMULATOR
    printf0("setup ok\n");
#endif
    main_loop_hz = 20;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;

    run_esn_mackey_glass_demo();
}

void user_step(void) {
    /* No control loop here; pure training/testing example. */
}

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    return 0;
}

