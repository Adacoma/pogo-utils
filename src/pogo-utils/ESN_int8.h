#ifndef ESN_INT8_SPARSE_H_
#define ESN_INT8_SPARSE_H_

#include <stdint.h>
#include <stdlib.h>
#include <math.h>

/*
 * Compile-time configuration of the ESN dimensions.
 * You MUST define these before including this header, e.g.:
 *
 *   #define ESN_INT8_INPUT_DIM      1
 *   #define ESN_INT8_RESERVOIR_DIM  64
 *   #define ESN_INT8_OUTPUT_DIM     1
 *   #include "ESN_int8_sparse.h"
 */

#ifndef ESN_INT8_INPUT_DIM
# error "ESN_INT8_INPUT_DIM must be defined before including ESN_int8_sparse.h"
#endif

#ifndef ESN_INT8_RESERVOIR_DIM
# error "ESN_INT8_RESERVOIR_DIM must be defined before including ESN_int8_sparse.h"
#endif

#ifndef ESN_INT8_OUTPUT_DIM
# error "ESN_INT8_OUTPUT_DIM must be defined before including ESN_int8_sparse.h"
#endif

/* Fixed-point format: Q0.7 */
#define ESN_INT8_FRAC_BITS 7

/*
 * Sparse reservoir configuration:
 * Each reservoir neuron has exactly K recurrent incoming connections.
 * To preserve the echo state property, we enforce:
 *    ESN_INT8_RES_FIXED_K * ESN_INT8_RES_WEIGHT_ABS <= 96 < 127
 * so that ||Wres||_1 < 1.0 in Q0.7.
 */
#ifndef ESN_INT8_RES_FIXED_K
#define ESN_INT8_RES_FIXED_K 4
#endif

#ifndef ESN_INT8_RES_WEIGHT_ABS
#define ESN_INT8_RES_WEIGHT_ABS 24   /* 4*24 = 96 ~ 0.75 in Q0.7 */
#endif

#ifndef ESN_INT8_WIN_WEIGHT_ABS
#define ESN_INT8_WIN_WEIGHT_ABS 64   /* ~0.5 in Q0.7 */
#endif

/* Maximum feature dimension for readout training:
 *  bias (1) + reservoir state + direct input (optional).
 */
#define ESN_INT8_FEATURE_MAX_DIM (1 + ESN_INT8_RESERVOIR_DIM + ESN_INT8_INPUT_DIM)

/*
 * Sparse Echo State Network in int8 Q0.7:
 *
 *   x_{t+1} = hard_tanh( Wres * x_t + Win * u_{t+1} + b_res )
 *   y_t     = Wout * x_{t+1} + Wout_in * u_{t+1} + b_out
 *
 * Reservoir is stored with fixed-K incoming connections per neuron:
 *   For each reservoir neuron i, we store:
 *     - indices of presynaptic neurons: Wres_idx[i][k], k=0..K-1
 *     - corresponding weights:          Wres_val[i][k], k=0..K-1
 */

typedef struct {
    /* Input -> reservoir (dense, small) */
    int8_t Win[ESN_INT8_RESERVOIR_DIM][ESN_INT8_INPUT_DIM];

    /* Sparse recurrent reservoir: fixed-K connections per neuron */
    int8_t  Wres_val[ESN_INT8_RESERVOIR_DIM][ESN_INT8_RES_FIXED_K];
    uint8_t Wres_idx[ESN_INT8_RESERVOIR_DIM][ESN_INT8_RES_FIXED_K];

    /* Reservoir -> output */
    int8_t Wout[ESN_INT8_OUTPUT_DIM][ESN_INT8_RESERVOIR_DIM];

    /* Direct input -> output */
    int8_t Wout_in[ESN_INT8_OUTPUT_DIM][ESN_INT8_INPUT_DIM];

    /* Biases */
    int8_t b_res[ESN_INT8_RESERVOIR_DIM];
    int8_t b_out[ESN_INT8_OUTPUT_DIM];

    /* Reservoir state x_t */
    int8_t state[ESN_INT8_RESERVOIR_DIM];
} ESN_INT8_SPARSE;

/* --- Helpers --- */

static inline int8_t esn_int8_sparse_from_acc32(int32_t acc) {
#if ESN_INT8_FRAC_BITS > 0
    if (acc >= 0)
        acc += (1 << (ESN_INT8_FRAC_BITS - 1));
    else
        acc -= (1 << (ESN_INT8_FRAC_BITS - 1));
    acc >>= ESN_INT8_FRAC_BITS;
#endif
    if (acc > 127)   acc = 127;
    if (acc < -128)  acc = -128;
    return (int8_t)acc;
}

static inline int8_t esn_int8_sparse_hard_tanh(int8_t x) {
    if (x > 127)   return 127;
    if (x < -128)  return -128;
    return x;
}

/* Quantize float in [-1,1] to Q0.7 int8. */
static inline int8_t esn_int8_sparse_from_float(float v) {
    if (v > 0.999f)  v = 0.999f;
    if (v < -1.0f)   v = -1.0f;
    float scaled = v * (float)(1 << ESN_INT8_FRAC_BITS);
    int32_t a = (int32_t)(scaled >= 0.0f ? (scaled + 0.5f) : (scaled - 0.5f));
    if (a > 127)   a = 127;
    if (a < -128)  a = -128;
    return (int8_t)a;
}

/* Convert Q0.7 int8 to float in approx [-1,1]. */
static inline float esn_int8_sparse_to_float(int8_t q) {
    return (float)q / (float)(1 << ESN_INT8_FRAC_BITS);
}

/* --- Basic initialization --- */

static void esn_int8_sparse_init_zero(ESN_INT8_SPARSE *net) {
    for (int i = 0; i < ESN_INT8_RESERVOIR_DIM; ++i) {
        net->b_res[i] = 0;
        net->state[i] = 0;

        for (int j = 0; j < ESN_INT8_INPUT_DIM; ++j) {
            net->Win[i][j] = 0;
        }
        for (int k = 0; k < ESN_INT8_RES_FIXED_K; ++k) {
            net->Wres_val[i][k] = 0;
            net->Wres_idx[i][k] = 0;
        }
    }

    for (int o = 0; o < ESN_INT8_OUTPUT_DIM; ++o) {
        net->b_out[o] = 0;
        for (int j = 0; j < ESN_INT8_RESERVOIR_DIM; ++j) {
            net->Wout[o][j] = 0;
        }
        for (int j = 0; j < ESN_INT8_INPUT_DIM; ++j) {
            net->Wout_in[o][j] = 0;
        }
    }
}

static void esn_int8_sparse_reset_state(ESN_INT8_SPARSE *net) {
    for (int i = 0; i < ESN_INT8_RESERVOIR_DIM; ++i) {
        net->state[i] = 0;
    }
}

/* Random int8 in full range [-128, 127] */
static inline int8_t esn_int8_sparse_rand_int8(void) {
    return (int8_t)(rand() & 0xFF);
}

/*
 * Initialize sparse reservoir and Win:
 *  - Win dense with magnitude ESN_INT8_WIN_WEIGHT_ABS.
 *  - Wres: for each neuron i, exactly ESN_INT8_RES_FIXED_K distinct
 *    presynaptic indices j with weights +/-ESN_INT8_RES_WEIGHT_ABS.
 */
static void esn_int8_sparse_init_random_reservoir(ESN_INT8_SPARSE *net) {
    esn_int8_sparse_init_zero(net);

    /* Randomize Win (dense, small) */
    for (int i = 0; i < ESN_INT8_RESERVOIR_DIM; ++i) {
        for (int j = 0; j < ESN_INT8_INPUT_DIM; ++j) {
            int sign = (rand() & 1) ? 1 : -1;
            net->Win[i][j] = (int8_t)(sign * ESN_INT8_WIN_WEIGHT_ABS);
        }
    }

    /* Random sparse Wres: fixed-K unique presynaptic indices per neuron */
    for (int i = 0; i < ESN_INT8_RESERVOIR_DIM; ++i) {
        for (int k = 0; k < ESN_INT8_RES_FIXED_K; ++k) {
            uint8_t idx;
            int unique = 0;
            while (!unique) {
                idx = (uint8_t)(rand() % ESN_INT8_RESERVOIR_DIM);
                unique = 1;
                for (int kk = 0; kk < k; ++kk) {
                    if (net->Wres_idx[i][kk] == idx) {
                        unique = 0;
                        break;
                    }
                }
            }
            net->Wres_idx[i][k] = idx;
            int sign = (rand() & 1) ? 1 : -1;
            net->Wres_val[i][k] = (int8_t)(sign * ESN_INT8_RES_WEIGHT_ABS);
        }
        /* b_res[i] already 0 */
    }
}

/*
 * One ESN step (sparse reservoir):
 *
 *  x_{t+1} = hard_tanh( Wres * x_t + Win * u_{t+1} + b_res )
 *  y_t     = Wout * x_{t+1} + Wout_in * u_{t+1} + b_out
 *
 * Input u and output y are Q0.7 int8 vectors.
 */
static void esn_int8_sparse_step(ESN_INT8_SPARSE *net,
                                 const int8_t in[ESN_INT8_INPUT_DIM],
                                 int8_t out[ESN_INT8_OUTPUT_DIM])
{
    int8_t new_state[ESN_INT8_RESERVOIR_DIM];

    /* --- Reservoir update --- */
    for (int i = 0; i < ESN_INT8_RESERVOIR_DIM; ++i) {
        int32_t acc = ((int32_t)net->b_res[i]) << ESN_INT8_FRAC_BITS;

        /* Input contribution: Win[i,:] * in */
        const int8_t *win_row = &net->Win[i][0];
        for (int k = 0; k < ESN_INT8_INPUT_DIM; ++k) {
            acc += (int32_t)win_row[k] * (int32_t)in[k];
        }

        /* Sparse recurrent contribution: sum over fixed-K presynaptic indices */
        const int8_t  *w_val = &net->Wres_val[i][0];
        const uint8_t *w_idx = &net->Wres_idx[i][0];
        for (int k = 0; k < ESN_INT8_RES_FIXED_K; ++k) {
            uint8_t j = w_idx[k];
            acc += (int32_t)w_val[k] * (int32_t)net->state[j];
        }

        int8_t z = esn_int8_sparse_from_acc32(acc);
        new_state[i] = esn_int8_sparse_hard_tanh(z);
    }

    /* Commit new state */
    for (int i = 0; i < ESN_INT8_RESERVOIR_DIM; ++i) {
        net->state[i] = new_state[i];
    }

    /* --- Output computation --- */
    for (int o = 0; o < ESN_INT8_OUTPUT_DIM; ++o) {
        int32_t acc = ((int32_t)net->b_out[o]) << ESN_INT8_FRAC_BITS;

        const int8_t *wout_row    = &net->Wout[o][0];
        const int8_t *wout_in_row = &net->Wout_in[o][0];

        /* Reservoir contribution */
        for (int j = 0; j < ESN_INT8_RESERVOIR_DIM; ++j) {
            acc += (int32_t)wout_row[j] * (int32_t)net->state[j];
        }

        /* Direct input contribution */
        for (int k = 0; k < ESN_INT8_INPUT_DIM; ++k) {
            acc += (int32_t)wout_in_row[k] * (int32_t)in[k];
        }

        out[o] = esn_int8_sparse_from_acc32(acc);
    }
}

/* --- Linear regression training of readout weights (ridge) ---
 *
 * We train Wout, Wout_in and b_out with one-shot linear regression
 * (normal equations, L2-regularized).
 *
 * We build a feature vector for each time step t:
 *
 *   phi(t) = [ 1,  x_1(t), ..., x_R(t),  u_1(t), ..., u_I(t) ]
 *
 * where x(t) is the reservoir state AFTER ingesting input u(t).
 * Target y(t) can be e.g. the next value in a time series.
 *
 * Given sequences of int8 Q0.7 inputs and targets:
 *   input_seq  shape: [T][ESN_INT8_INPUT_DIM]
 *   target_seq shape: [T][ESN_INT8_OUTPUT_DIM]
 *
 * and a washout length 'washout' (number of initial steps ignored in training),
 * we solve for readout weights minimizing:
 *
 *   sum_{t >= washout} || y_target(t) - W * phi(t) ||^2 + lambda * ||W||^2
 *
 * using standard normal equations:
 *
 *   G = Phi^T Phi + lambda I,  C = Y^T Phi,   W = C * G^{-1}
 *
 * All computations are in float; weights are then quantized to int8 Q0.7.
 */

static int esn_int8_sparse_gaussian_solve(
    float A[ESN_INT8_FEATURE_MAX_DIM][ESN_INT8_FEATURE_MAX_DIM],
    float b[ESN_INT8_FEATURE_MAX_DIM],
    float x[ESN_INT8_FEATURE_MAX_DIM],
    int n)
{
    /* Simple Gaussian elimination w/ partial pivoting */
    for (int i = 0; i < n; ++i) {
        /* Pivot selection */
        int pivot = i;
        float max_val = fabsf(A[i][i]);
        for (int r = i + 1; r < n; ++r) {
            float v = fabsf(A[r][i]);
            if (v > max_val) {
                max_val = v;
                pivot = r;
            }
        }
        if (max_val < 1e-12f) {
            return -1;  /* singular / ill-conditioned */
        }
        /* Swap rows if needed */
        if (pivot != i) {
            for (int c = 0; c < n; ++c) {
                float tmp = A[i][c];
                A[i][c] = A[pivot][c];
                A[pivot][c] = tmp;
            }
            float tmpb = b[i];
            b[i] = b[pivot];
            b[pivot] = tmpb;
        }

        /* Eliminate below diagonal */
        float diag = A[i][i];
        for (int r = i + 1; r < n; ++r) {
            float factor = A[r][i] / diag;
            if (factor == 0.0f) continue;
            A[r][i] = 0.0f;
            for (int c = i + 1; c < n; ++c) {
                A[r][c] -= factor * A[i][c];
            }
            b[r] -= factor * b[i];
        }
    }

    /* Back substitution */
    for (int i = n - 1; i >= 0; --i) {
        float sum = b[i];
        for (int c = i + 1; c < n; ++c) {
            sum -= A[i][c] * x[c];
        }
        float diag = A[i][i];
        x[i] = (fabsf(diag) < 1e-12f) ? 0.0f : (sum / diag);
    }
    return 0;
}

static void esn_int8_sparse_train_readout_ridge(
    ESN_INT8_SPARSE *net,
    const int8_t *input_seq,   /* length T * ESN_INT8_INPUT_DIM */
    const int8_t *target_seq,  /* length T * ESN_INT8_OUTPUT_DIM */
    int T,
    int washout,
    float ridge_lambda,
    int use_direct_input)      /* 0 = ignore direct input features, 1 = use them */
{
    if (T <= washout) {
        return; /* nothing to train */
    }

    const int feat_dim = 1 + ESN_INT8_RESERVOIR_DIM +
                         (use_direct_input ? ESN_INT8_INPUT_DIM : 0);

    /* Static buffers to avoid large stack usage. */
    static float G[ESN_INT8_FEATURE_MAX_DIM][ESN_INT8_FEATURE_MAX_DIM];
    static float C[ESN_INT8_OUTPUT_DIM][ESN_INT8_FEATURE_MAX_DIM];
    static float A[ESN_INT8_FEATURE_MAX_DIM][ESN_INT8_FEATURE_MAX_DIM];
    static float phi[ESN_INT8_FEATURE_MAX_DIM];

    /* Zero accumulators */
    for (int i = 0; i < ESN_INT8_FEATURE_MAX_DIM; ++i) {
        for (int j = 0; j < ESN_INT8_FEATURE_MAX_DIM; ++j) {
            G[i][j] = 0.0f;
        }
    }
    for (int o = 0; o < ESN_INT8_OUTPUT_DIM; ++o) {
        for (int j = 0; j < ESN_INT8_FEATURE_MAX_DIM; ++j) {
            C[o][j] = 0.0f;
        }
    }

    /* Run ESN over the sequence, collect features/targets after washout */
    esn_int8_sparse_reset_state(net);

    int8_t dummy_out[ESN_INT8_OUTPUT_DIM];

    for (int t = 0; t < T; ++t) {
        const int8_t *u = &input_seq[t * ESN_INT8_INPUT_DIM];
        const int8_t *y = &target_seq[t * ESN_INT8_OUTPUT_DIM];

        esn_int8_sparse_step(net, u, dummy_out);

        if (t < washout) {
            continue;
        }

        /* Build feature vector phi(t) */
        int idx = 0;
        phi[idx++] = 1.0f; /* bias */

        /* Reservoir part */
        for (int r = 0; r < ESN_INT8_RESERVOIR_DIM; ++r) {
            phi[idx++] = esn_int8_sparse_to_float(net->state[r]);
        }

        /* Direct input part (optional) */
        if (use_direct_input) {
            for (int k = 0; k < ESN_INT8_INPUT_DIM; ++k) {
                phi[idx++] = esn_int8_sparse_to_float(u[k]);
            }
        }

        /* Accumulate Gram matrix G and cross term C */
        for (int i = 0; i < feat_dim; ++i) {
            float pi = phi[i];
            for (int j = 0; j < feat_dim; ++j) {
                G[i][j] += pi * phi[j];
            }
        }

        for (int o = 0; o < ESN_INT8_OUTPUT_DIM; ++o) {
            float yt = esn_int8_sparse_to_float(y[o]);
            for (int j = 0; j < feat_dim; ++j) {
                C[o][j] += yt * phi[j];
            }
        }
    }

    /* Add ridge regularization on diagonal */
    for (int i = 0; i < feat_dim; ++i) {
        G[i][i] += ridge_lambda;
    }

    /* Solve for each output dimension separately:
     *   G * w_o = C[o,:]^T
     */
    float w[ESN_INT8_FEATURE_MAX_DIM];
    float b_vec[ESN_INT8_FEATURE_MAX_DIM];

    for (int o = 0; o < ESN_INT8_OUTPUT_DIM; ++o) {
        /* Copy G into A and C[o] into b_vec */
        for (int i = 0; i < feat_dim; ++i) {
            b_vec[i] = C[o][i];
            for (int j = 0; j < feat_dim; ++j) {
                A[i][j] = G[i][j];
            }
        }
        for (int i = 0; i < feat_dim; ++i) {
            w[i] = 0.0f;
        }

        int status = esn_int8_sparse_gaussian_solve(A, b_vec, w, feat_dim);
        if (status != 0) {
            /* Fallback: zero weights if solver fails */
            net->b_out[o] = 0;
            for (int j = 0; j < ESN_INT8_RESERVOIR_DIM; ++j) {
                net->Wout[o][j] = 0;
            }
            for (int j = 0; j < ESN_INT8_INPUT_DIM; ++j) {
                net->Wout_in[o][j] = 0;
            }
            continue;
        }

        /* Map w into (b_out, Wout, Wout_in), quantized to int8 Q0.7 */
        int idx2 = 0;
        net->b_out[o] = esn_int8_sparse_from_float(w[idx2++]);

        for (int r = 0; r < ESN_INT8_RESERVOIR_DIM; ++r) {
            net->Wout[o][r] = esn_int8_sparse_from_float(w[idx2++]);
        }

        if (use_direct_input) {
            for (int k = 0; k < ESN_INT8_INPUT_DIM; ++k) {
                net->Wout_in[o][k] = esn_int8_sparse_from_float(w[idx2++]);
            }
        } else {
            for (int k = 0; k < ESN_INT8_INPUT_DIM; ++k) {
                net->Wout_in[o][k] = 0;
            }
        }
    }
}

#endif /* ESN_INT8_SPARSE_H_ */

