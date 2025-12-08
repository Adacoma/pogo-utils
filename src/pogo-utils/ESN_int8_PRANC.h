#ifndef ESN_INT8_PRANC_H_
#define ESN_INT8_PRANC_H_

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
 *   #include "ESN_int8_PRANC.h"
 */

#ifndef ESN_INT8_INPUT_DIM
# error "ESN_INT8_INPUT_DIM must be defined before including ESN_int8_PRANC.h"
#endif

#ifndef ESN_INT8_RESERVOIR_DIM
# error "ESN_INT8_RESERVOIR_DIM must be defined before including ESN_int8_PRANC.h"
#endif

#ifndef ESN_INT8_OUTPUT_DIM
# error "ESN_INT8_OUTPUT_DIM must be defined before including ESN_int8_PRANC.h"
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

/* Multiplicative (gated) term Win_mult amplitude */
#ifndef ESN_INT8_WIN_MULT_WEIGHT_ABS
#define ESN_INT8_WIN_MULT_WEIGHT_ABS 16 /* smaller to keep things stable */
#endif

/* Leaky integration parameter:
 *   new_state = ((128-LEAK)*old_state + LEAK*candidate) / 128
 * LEAK in [0,128], LEAK ~= 0.75*128 by default.
 */
#ifndef ESN_INT8_LEAK
#define ESN_INT8_LEAK 96  /* ~0.75 */
#endif

/* Enable multiplicative/gated term in the reservoir update */
#ifndef ESN_INT8_ENABLE_MULTIPLICATIVE
#define ESN_INT8_ENABLE_MULTIPLICATIVE 1
#endif

/* Maximum feature dimension for readout training:
 *  bias (1) + reservoir state + direct input (optional).
 */
#define ESN_INT8_FEATURE_MAX_DIM (1 + ESN_INT8_RESERVOIR_DIM + ESN_INT8_INPUT_DIM)

/* --- Optional PRANC configuration on the readout --- */

/*
 * If ESN_INT8_USE_PRANC is defined before including this file, we enable:
 *  - Random int8 basis over the readout feature space (reservoir + input).
 *  - Compression: dense readout -> PRANC coefficients alpha.
 *  - Reconstruction: PRANC-coded readout -> new dense int8 readout.
 *
 * This is meant for:
 *  - Studying approximation error due to PRANC compression.
 *  - Potentially storing only the PRANC basis and alpha offline, then
 *    reconstructing dense readout weights at robot boot time.
 */

#ifdef ESN_INT8_USE_PRANC

#ifndef ESN_INT8_PRANC_NUM_BASIS
#define ESN_INT8_PRANC_NUM_BASIS 32   /* basis dimension (must be <= ESN_INT8_FEATURE_MAX_DIM) */
#endif

#ifndef ESN_INT8_PRANC_BASIS_ABS
#define ESN_INT8_PRANC_BASIS_ABS 64   /* ~0.5 in Q0.7 */
#endif

#ifndef ESN_INT8_PRANC_RIDGE
#define ESN_INT8_PRANC_RIDGE 1e-6f
#endif

#endif /* ESN_INT8_USE_PRANC */

/*
 * Leaky-gated sparse Echo State Network in int8 Q0.7:
 *
 *   Candidate:
 *     h_{t+1} = hard_tanh( Wres * x_t + Win * u_{t+1}
 *                          + Wmult * (x_t ⊙ u_{t+1}) + b_res )
 *
 *   Leaky update:
 *     x_{t+1} = (1 - leak) * x_t + leak * h_{t+1}
 *
 *   Readout:
 *     y_t = Wout * x_{t+1} + Wout_in * u_{t+1} + b_out
 *
 * Reservoir is stored with fixed-K incoming connections per neuron:
 *   For each reservoir neuron i, we store:
 *     - indices of presynaptic neurons: Wres_idx[i][k], k=0..K-1
 *     - corresponding weights:          Wres_val[i][k], k=0..K-1
 *
 * Multiplicative/gated term uses an additional dense Win_mult:
 *   contribution ~ Win_mult[i,:] * (x_i(t) * u(t)) elementwise.
 */

typedef struct {
    /* Input -> reservoir (dense) */
    int8_t Win[ESN_INT8_RESERVOIR_DIM][ESN_INT8_INPUT_DIM];

    /* Multiplicative / gated term: per-neuron, per-input */
    int8_t Win_mult[ESN_INT8_RESERVOIR_DIM][ESN_INT8_INPUT_DIM];

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

#ifdef ESN_INT8_USE_PRANC
    /* PRANC basis over readout features (reservoir + input, no bias) */
    int8_t pranc_basis[ESN_INT8_PRANC_NUM_BASIS]
                      [ESN_INT8_RESERVOIR_DIM + ESN_INT8_INPUT_DIM];

    /* PRANC coefficients per output dimension (in float for training) */
    float  pranc_alpha[ESN_INT8_OUTPUT_DIM][ESN_INT8_PRANC_NUM_BASIS];
#endif

} ESN_INT8_PRANC;

/* --- Helpers --- */

static inline int8_t esn_int8_lg_from_acc32(int32_t acc) {
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

static inline int8_t esn_int8_lg_hard_tanh(int8_t x) {
    if (x > 127)   return 127;
    if (x < -128)  return -128;
    return x;
}

/* Quantize float in [-1,1] to Q0.7 int8. */
static inline int8_t esn_int8_lg_from_float(float v) {
    if (v > 0.999f)  v = 0.999f;
    if (v < -1.0f)   v = -1.0f;
    float scaled = v * (float)(1 << ESN_INT8_FRAC_BITS);
    int32_t a = (int32_t)(scaled >= 0.0f ? (scaled + 0.5f) : (scaled - 0.5f));
    if (a > 127)   a = 127;
    if (a < -128)  a = -128;
    return (int8_t)a;
}

/* Convert Q0.7 int8 to float in approx [-1,1]. */
static inline float esn_int8_lg_to_float(int8_t q) {
    return (float)q / (float)(1 << ESN_INT8_FRAC_BITS);
}

/* Leaky mixing: new = ( (128-LEAK)*old + LEAK*candidate ) / 128 */
static inline int8_t esn_int8_lg_leaky_mix(int8_t old, int8_t cand) {
    const int32_t LEAK = ESN_INT8_LEAK; /* 0..128 */
    int32_t mix = (int32_t)(128 - LEAK) * (int32_t)old
                + (int32_t)LEAK * (int32_t)cand;
    if (mix >= 0)
        mix += 64;
    else
        mix -= 64;
    mix >>= 7;
    if (mix > 127)   mix = 127;
    if (mix < -128)  mix = -128;
    return (int8_t)mix;
}

/* --- Basic initialization --- */

static void esn_int8_lg_init_zero(ESN_INT8_PRANC *net) {
    for (int i = 0; i < ESN_INT8_RESERVOIR_DIM; ++i) {
        net->b_res[i] = 0;
        net->state[i] = 0;

        for (int j = 0; j < ESN_INT8_INPUT_DIM; ++j) {
            net->Win[i][j]      = 0;
            net->Win_mult[i][j] = 0;
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

#ifdef ESN_INT8_USE_PRANC
    for (int b = 0; b < ESN_INT8_PRANC_NUM_BASIS; ++b) {
        for (int f = 0; f < ESN_INT8_RESERVOIR_DIM + ESN_INT8_INPUT_DIM; ++f) {
            net->pranc_basis[b][f] = 0;
        }
    }
    for (int o = 0; o < ESN_INT8_OUTPUT_DIM; ++o) {
        for (int b = 0; b < ESN_INT8_PRANC_NUM_BASIS; ++b) {
            net->pranc_alpha[o][b] = 0.0f;
        }
    }
#endif
}

static void esn_int8_lg_reset_state(ESN_INT8_PRANC *net) {
    for (int i = 0; i < ESN_INT8_RESERVOIR_DIM; ++i) {
        net->state[i] = 0;
    }
}

/* Random int8 in full range [-128, 127] */
static inline int8_t esn_int8_lg_rand_int8(void) {
    return (int8_t)(rand() & 0xFF);
}

/*
 * Initialize sparse reservoir, Win and Win_mult:
 *  - Win dense with magnitude ESN_INT8_WIN_WEIGHT_ABS.
 *  - Win_mult dense with magnitude ESN_INT8_WIN_MULT_WEIGHT_ABS.
 *  - Wres: for each neuron i, exactly ESN_INT8_RES_FIXED_K distinct
 *    presynaptic indices j with weights +/-ESN_INT8_RES_WEIGHT_ABS.
 */
static void esn_int8_lg_init_random_reservoir(ESN_INT8_PRANC *net) {
    esn_int8_lg_init_zero(net);

    /* Randomize Win (dense, small) */
    for (int i = 0; i < ESN_INT8_RESERVOIR_DIM; ++i) {
        for (int j = 0; j < ESN_INT8_INPUT_DIM; ++j) {
            int sign = (rand() & 1) ? 1 : -1;
            net->Win[i][j] = (int8_t)(sign * ESN_INT8_WIN_WEIGHT_ABS);
        }
    }

    /* Randomize Win_mult (dense, small) */
    for (int i = 0; i < ESN_INT8_RESERVOIR_DIM; ++i) {
        for (int j = 0; j < ESN_INT8_INPUT_DIM; ++j) {
            int sign = (rand() & 1) ? 1 : -1;
            net->Win_mult[i][j] = (int8_t)(sign * ESN_INT8_WIN_MULT_WEIGHT_ABS);
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
    }
}

/*
 * One ESN step (leaky + multiplicative/gated sparse reservoir):
 *
 *  h_{t+1} = hard_tanh( Wres * x_t + Win * u_{t+1}
 *                       + Win_mult * (x_t ⊙ u_{t+1}) + b_res )
 *  x_{t+1} = (1 - leak) * x_t + leak * h_{t+1}
 *  y_t     = Wout * x_{t+1} + Wout_in * u_{t+1} + b_out
 *
 * Input u and output y are Q0.7 int8 vectors.
 */
static void esn_int8_lg_step(ESN_INT8_PRANC *net,
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

        /* Sparse recurrent contribution: Wres * state */
        const int8_t  *w_val = &net->Wres_val[i][0];
        const uint8_t *w_idx = &net->Wres_idx[i][0];
        for (int k = 0; k < ESN_INT8_RES_FIXED_K; ++k) {
            uint8_t j = w_idx[k];
            acc += (int32_t)w_val[k] * (int32_t)net->state[j];
        }

#if ESN_INT8_ENABLE_MULTIPLICATIVE
        /* Multiplicative/gated term: Win_mult * (x_i ⊙ u) */
        const int8_t *wm_row = &net->Win_mult[i][0];
        for (int k = 0; k < ESN_INT8_INPUT_DIM; ++k) {
            int32_t mult = (int32_t)net->state[i] * (int32_t)in[k]; /* Q0.7 * Q0.7 = Q0.14 */
            mult >>= ESN_INT8_FRAC_BITS;                             /* back to Q0.7 */
            acc += (int32_t)wm_row[k] * mult;                        /* Q0.7 * Q0.7 */
        }
#endif

        int8_t z = esn_int8_lg_from_acc32(acc);
        int8_t h = esn_int8_lg_hard_tanh(z);

        /* Leaky integration */
        new_state[i] = esn_int8_lg_leaky_mix(net->state[i], h);
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

        out[o] = esn_int8_lg_from_acc32(acc);
    }
}

/* --- Linear regression training of readout weights (ridge) ---
 *
 *  phi(t) = [ 1,  x_1(t), ..., x_R(t),  u_1(t), ..., u_I(t) ]
 *
 * Training and ridge regression are done in float; weights are then
 * quantized to int8 Q0.7.
 */

static int esn_int8_lg_gaussian_solve(
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

static void esn_int8_lg_train_readout_ridge(
    ESN_INT8_PRANC *net,
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
    esn_int8_lg_reset_state(net);

    int8_t dummy_out[ESN_INT8_OUTPUT_DIM];

    for (int t = 0; t < T; ++t) {
        const int8_t *u = &input_seq[t * ESN_INT8_INPUT_DIM];
        const int8_t *y = &target_seq[t * ESN_INT8_OUTPUT_DIM];

        esn_int8_lg_step(net, u, dummy_out);

        if (t < washout) {
            continue;
        }

        /* Build feature vector phi(t) */
        int idx = 0;
        phi[idx++] = 1.0f; /* bias */

        /* Reservoir part */
        for (int r = 0; r < ESN_INT8_RESERVOIR_DIM; ++r) {
            phi[idx++] = esn_int8_lg_to_float(net->state[r]);
        }

        /* Direct input part (optional) */
        if (use_direct_input) {
            for (int k = 0; k < ESN_INT8_INPUT_DIM; ++k) {
                phi[idx++] = esn_int8_lg_to_float(u[k]);
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
            float yt = esn_int8_lg_to_float(y[o]);
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

        int status = esn_int8_lg_gaussian_solve(A, b_vec, w, feat_dim);
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
        net->b_out[o] = esn_int8_lg_from_float(w[idx2++]);

        for (int r = 0; r < ESN_INT8_RESERVOIR_DIM; ++r) {
            net->Wout[o][r] = esn_int8_lg_from_float(w[idx2++]);
        }

        if (use_direct_input) {
            for (int k = 0; k < ESN_INT8_INPUT_DIM; ++k) {
                net->Wout_in[o][k] = esn_int8_lg_from_float(w[idx2++]);
            }
        } else {
            for (int k = 0; k < ESN_INT8_INPUT_DIM; ++k) {
                net->Wout_in[o][k] = 0;
            }
        }
    }
}

/* --- Optional PRANC utilities on the readout --- */

#ifdef ESN_INT8_USE_PRANC

/* Initialize PRANC basis with random +/- ESN_INT8_PRANC_BASIS_ABS entries. */
static void esn_int8_lg_pranc_init_basis(ESN_INT8_PRANC *net) {
    const int feat_no_bias = ESN_INT8_RESERVOIR_DIM + ESN_INT8_INPUT_DIM;
    for (int b = 0; b < ESN_INT8_PRANC_NUM_BASIS; ++b) {
        for (int f = 0; f < feat_no_bias; ++f) {
            int sign = (rand() & 1) ? 1 : -1;
            net->pranc_basis[b][f] = (int8_t)(sign * ESN_INT8_PRANC_BASIS_ABS);
        }
    }
}

/*
 * Compress dense readout weights into PRANC coefficients:
 *
 * For each output dimension o, we approximate the weight vector
 *   w_o (over [reservoir | input], bias excluded)
 * as:
 *   w_o ≈ B^T alpha_o
 *
 * where B is the PRANC basis matrix (basis_dim x feat_no_bias).
 *
 * We solve (B B^T + lambda I) alpha_o = B w_o in float.
 * After this, call esn_int8_lg_pranc_reconstruct_readout() to
 * obtain an int8 Q0.7 readout approximating the original one.
 */
static void esn_int8_lg_pranc_compress_readout(ESN_INT8_PRANC *net) {
    const int feat_no_bias = ESN_INT8_RESERVOIR_DIM + ESN_INT8_INPUT_DIM;
    const int Bdim         = ESN_INT8_PRANC_NUM_BASIS;

    static float M[ESN_INT8_FEATURE_MAX_DIM][ESN_INT8_FEATURE_MAX_DIM]; /* B B^T */
    static float rhs[ESN_INT8_FEATURE_MAX_DIM];
    static float alpha[ESN_INT8_FEATURE_MAX_DIM];

    /* Build M = B B^T (independent of output dimension) */
    for (int r = 0; r < Bdim; ++r) {
        for (int c = 0; c < Bdim; ++c) {
            float sum = 0.0f;
            for (int f = 0; f < feat_no_bias; ++f) {
                float br = esn_int8_lg_to_float(net->pranc_basis[r][f]);
                float bc = esn_int8_lg_to_float(net->pranc_basis[c][f]);
                sum += br * bc;
            }
            M[r][c] = sum;
        }
    }

    /* Add ridge on the diagonal */
    for (int r = 0; r < Bdim; ++r) {
        M[r][r] += ESN_INT8_PRANC_RIDGE;
    }

    /* Solve for each output dimension */
    for (int o = 0; o < ESN_INT8_OUTPUT_DIM; ++o) {
        /* Build w_full (reservoir + input, bias excluded) */
        float w_full[ESN_INT8_RESERVOIR_DIM + ESN_INT8_INPUT_DIM];

        int idx = 0;
        for (int r = 0; r < ESN_INT8_RESERVOIR_DIM; ++r) {
            w_full[idx++] = esn_int8_lg_to_float(net->Wout[o][r]);
        }
        for (int k = 0; k < ESN_INT8_INPUT_DIM; ++k) {
            w_full[idx++] = esn_int8_lg_to_float(net->Wout_in[o][k]);
        }

        /* rhs = B * w_full */
        for (int r = 0; r < Bdim; ++r) {
            float sum = 0.0f;
            for (int f = 0; f < feat_no_bias; ++f) {
                float br = esn_int8_lg_to_float(net->pranc_basis[r][f]);
                sum += br * w_full[f];
            }
            rhs[r]   = sum;
            alpha[r] = 0.0f;
        }

        /* Copy M into a local A for solving (since Gaussian elimination is in-place) */
        static float A[ESN_INT8_FEATURE_MAX_DIM][ESN_INT8_FEATURE_MAX_DIM];
        for (int r = 0; r < Bdim; ++r) {
            for (int c = 0; c < Bdim; ++c) {
                A[r][c] = M[r][c];
            }
        }

        int status = esn_int8_lg_gaussian_solve(A, rhs, alpha, Bdim);
        if (status != 0) {
            /* If solver fails, leave alpha at zero for this output. */
            for (int b = 0; b < Bdim; ++b) {
                net->pranc_alpha[o][b] = 0.0f;
            }
            continue;
        }

        /* Store alpha */
        for (int b = 0; b < Bdim; ++b) {
            net->pranc_alpha[o][b] = alpha[b];
        }
    }
}

/*
 * Reconstruct a PRANC-approximated dense readout in Q0.7:
 *   w_hat = B^T alpha
 *
 * Overwrites net->Wout and net->Wout_in with quantized int8.
 */
static void esn_int8_lg_pranc_reconstruct_readout(ESN_INT8_PRANC *net) {
    const int feat_no_bias = ESN_INT8_RESERVOIR_DIM + ESN_INT8_INPUT_DIM;
    const int Bdim         = ESN_INT8_PRANC_NUM_BASIS;

    for (int o = 0; o < ESN_INT8_OUTPUT_DIM; ++o) {
        float w_full[ESN_INT8_RESERVOIR_DIM + ESN_INT8_INPUT_DIM];

        /* w_full = B^T alpha */
        for (int f = 0; f < feat_no_bias; ++f) {
            float sum = 0.0f;
            for (int b = 0; b < Bdim; ++b) {
                float br = esn_int8_lg_to_float(net->pranc_basis[b][f]);
                sum += net->pranc_alpha[o][b] * br;
            }
            w_full[f] = sum;
        }

        int idx = 0;
        for (int r = 0; r < ESN_INT8_RESERVOIR_DIM; ++r) {
            net->Wout[o][r] = esn_int8_lg_from_float(w_full[idx++]);
        }
        for (int k = 0; k < ESN_INT8_INPUT_DIM; ++k) {
            net->Wout_in[o][k] = esn_int8_lg_from_float(w_full[idx++]);
        }
        /* We leave b_out[o] untouched (not compressed). */
    }
}

#endif /* ESN_INT8_USE_PRANC */

#endif /* ESN_INT8_PRANC_H_ */
