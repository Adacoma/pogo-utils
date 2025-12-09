#ifndef MLP_INT8_H_
#define MLP_INT8_H_

#include <stdint.h>
#include <stdlib.h>   /* for rand() */
#include <math.h>     /* for sqrtf() */

/*
 * Compile-time configuration of the MLP dimensions.
 * You MUST define these before including this header, e.g.:
 *
 *   #define MLP_INT8_INPUT_DIM   4
 *   #define MLP_INT8_HIDDEN_DIM  8
 *   #define MLP_INT8_OUTPUT_DIM  2
 *   #include "MLP_INT8.h"
 */

#ifndef MLP_INT8_INPUT_DIM
# error "MLP_INT8_INPUT_DIM must be defined before including MLP_INT8.h"
#endif

#ifndef MLP_INT8_HIDDEN_DIM
# error "MLP_INT8_HIDDEN_DIM must be defined before including MLP_INT8.h"
#endif

#ifndef MLP_INT8_OUTPUT_DIM
# error "MLP_INT8_OUTPUT_DIM must be defined before including MLP_INT8.h"
#endif

/*
 * If you want the output layer to also use hard tanh,
 * define this macro before including the header:
 *
 *   #define MLP_INT8_OUTPUT_HARD_TANH
 */

#define MLP_INT8_FRAC_BITS 7  /* Q0.7 fixed-point */

/* Simple 1-hidden-layer MLP in int8 Q0.7:
 *
 *   h = hard_tanh( W1 * x + b1 )
 *   y = [hard_tanh or linear]( W2 * h + b2 )
 *
 * All weights, biases, and activations are in Q0.7 (int8).
 */
typedef struct {
    int8_t W1[MLP_INT8_HIDDEN_DIM][MLP_INT8_INPUT_DIM];
    int8_t b1[MLP_INT8_HIDDEN_DIM];

    int8_t W2[MLP_INT8_OUTPUT_DIM][MLP_INT8_HIDDEN_DIM];
    int8_t b2[MLP_INT8_OUTPUT_DIM];
} MLP_INT8;

/* Convert from 32-bit accumulator (Q0.14) back to int8 Q0.7 with rounding + saturation. */
static inline int8_t mlp_int8_from_acc32(int32_t acc) {
    /* Currently:
     *  - inputs, weights, biases are Q0.7 (int8)
     *  - products are Q0.14
     *  - biases are promoted by << MLP_INT8_FRAC_BITS (7)
     * So acc is in Q0.14 and we shift back by 7 to get Q0.7.
     */

#if MLP_INT8_FRAC_BITS > 0
    if (acc >= 0)
        acc += (1 << (MLP_INT8_FRAC_BITS - 1));
    else
        acc -= (1 << (MLP_INT8_FRAC_BITS - 1));
    acc >>= MLP_INT8_FRAC_BITS;
#endif

    if (acc > 127)   acc = 127;
    if (acc < -128)  acc = -128;

    return (int8_t)acc;
}

/* Hard tanh on Q0.7 int8.
 * Here it’s essentially just saturation in [-128, 127],
 * so on already-saturated int8 values it is the identity.
 */
static inline int8_t mlp_int8_hard_tanh(int8_t x) {
    if (x > 127)   return 127;
    if (x < -128)  return -128;
    return x;
}

/*
 * Internal helper: dense layer with bias + hard tanh activation.
 */

static void dense_hidden_int8_hard_tanh(const int8_t *weights,  /* [rows][cols] */
                                        const int8_t *biases,   /* [rows] */
                                        int rows,
                                        int cols,
                                        const int8_t *in,       /* [cols] */
                                        int8_t *out)            /* [rows] */
{
    for (int i = 0; i < rows; ++i) {
        /* Start accumulator from bias: Q0.7 -> Q0.14 */
        int32_t acc = ((int32_t)biases[i]) << MLP_INT8_FRAC_BITS;

        const int8_t *w_row = &weights[i * cols];

        /* Dot product: acc += sum_k w[i,k] * in[k] (Q0.14) */
        for (int k = 0; k < cols; ++k) {
            acc += (int32_t)w_row[k] * (int32_t)in[k];
        }

        /* Back to Q0.7 with rounding + saturation, then hard tanh */
        int8_t z = mlp_int8_from_acc32(acc);
        out[i] = mlp_int8_hard_tanh(z);
    }
}

/*
 * Output layer: either linear or hard-tanh.
 */
static void dense_output_int8_layer(const int8_t *weights,  /* [rows][cols] */
                                    const int8_t *biases,   /* [rows] */
                                    int rows,
                                    int cols,
                                    const int8_t *in,       /* [cols] */
                                    int8_t *out)            /* [rows] */
{
    for (int i = 0; i < rows; ++i) {
        int32_t acc = ((int32_t)biases[i]) << MLP_INT8_FRAC_BITS;
        const int8_t *w_row = &weights[i * cols];

        for (int k = 0; k < cols; ++k) {
            acc += (int32_t)w_row[k] * (int32_t)in[k];
        }

        int8_t z = mlp_int8_from_acc32(acc);

#ifdef MLP_INT8_OUTPUT_HARD_TANH
        out[i] = mlp_int8_hard_tanh(z);
#else
        out[i] = z;  /* linear output (still saturated to int8 range) */
#endif
    }
}

/**
 * @brief Forward pass of the int8 Q0.7 MLP.
 *
 * @param net   Pointer to network parameters.
 * @param in    Input vector of length MLP_INT8_INPUT_DIM.
 * @param out   Output vector of length MLP_INT8_OUTPUT_DIM.
 *
 * All vectors are int8, interpreted as Q0.7. Hidden activations always use hard tanh.
 * The output layer is:
 *   - linear if MLP_INT8_OUTPUT_HARD_TANH is NOT defined
 *   - hard_tanh if MLP_INT8_OUTPUT_HARD_TANH is defined
 */
static void mlp_int8_forward(const MLP_INT8 *net,
                             const int8_t in[MLP_INT8_INPUT_DIM],
                             int8_t out[MLP_INT8_OUTPUT_DIM]) {
    int8_t hidden[MLP_INT8_HIDDEN_DIM];

    /* First layer: input -> hidden with hard tanh */
    dense_hidden_int8_hard_tanh(&net->W1[0][0],
                                net->b1,
                                MLP_INT8_HIDDEN_DIM,
                                MLP_INT8_INPUT_DIM,
                                in,
                                hidden);

    /* Second layer: hidden -> output (linear or hard tanh) */
    dense_output_int8_layer(&net->W2[0][0],
                            net->b2,
                            MLP_INT8_OUTPUT_DIM,
                            MLP_INT8_HIDDEN_DIM,
                            hidden,
                            out);
}


static void mlp_int8_forward_logits32(const MLP_INT8 *net,
                                      const int8_t in[MLP_INT8_INPUT_DIM],
                                      int32_t out[MLP_INT8_OUTPUT_DIM]) {
    int8_t hidden[MLP_INT8_HIDDEN_DIM];

    /* First layer: input -> hidden with hard tanh */
    dense_hidden_int8_hard_tanh(&net->W1[0][0],
                                net->b1,
                                MLP_INT8_HIDDEN_DIM,
                                MLP_INT8_INPUT_DIM,
                                in,
                                hidden);

    // Second layer: hidden -> output. Keep accumulators as Q0.14 int32
    for (int i = 0; i < MLP_INT8_OUTPUT_DIM; ++i) {
        int32_t acc = ((int32_t)net->b2[i]) << MLP_INT8_FRAC_BITS;
        const int8_t *w_row = &net->W2[0][0] + i * MLP_INT8_HIDDEN_DIM;
        for (int k = 0; k < MLP_INT8_HIDDEN_DIM; ++k) {
            acc += (int32_t)w_row[k] * (int32_t)hidden[k];
        }
        out[i] = acc;  // no mlp_int8_from_acc32, no hard_tanh
    }
}


/* Total number of trainable parameters. */
static inline uint32_t mlp_int8_param_count(void) {
    return (uint32_t)MLP_INT8_INPUT_DIM  * (uint32_t)MLP_INT8_HIDDEN_DIM +
           (uint32_t)MLP_INT8_HIDDEN_DIM +
           (uint32_t)MLP_INT8_HIDDEN_DIM * (uint32_t)MLP_INT8_OUTPUT_DIM +
           (uint32_t)MLP_INT8_OUTPUT_DIM;
}

/* Helper: convert float in [-1, 1] to Q0.7 int8 with rounding + saturation. */
static inline int8_t mlp_int8_float_to_q0_7(float x) {
    /* Clamp to representable range (slightly less than 1.0 in Q0.7). */
    if (x > 0.999f)  x = 0.999f;
    if (x < -1.0f)   x = -1.0f;

    float scaled = x * (float)(1 << MLP_INT8_FRAC_BITS);  // e.g., *128
    if (scaled >= 0.0f)
        scaled += 0.5f;
    else
        scaled -= 0.5f;
    int32_t v = (int32_t)scaled;

    if (v > 127)   v = 127;
    if (v < -128)  v = -128;

    return (int8_t)v;
}

/**
 * @brief Xavier-style initialization of an int8 MLP to reduce early saturation.
 *
 * We use a uniform Xavier scheme:
 *   W ~ U(-a, a), with a = sqrt(6 / (fan_in + fan_out))
 * then scaled and quantized to Q0.7.
 *
 * Biases are initialized to 0.
 *
 * IMPORTANT: call srand(...) in user code before using this, if you want
 * deterministic / controlled randomness.
 */
static inline void mlp_int8_init_xavier(MLP_INT8 *net) {
    /* ---- First layer: W1 (hidden x input) ---- */
    const float fan_in1  = (float)MLP_INT8_INPUT_DIM;
    const float fan_out1 = (float)MLP_INT8_HIDDEN_DIM;
    float limit1 = sqrtf(6.0f / (fan_in1 + fan_out1));

    /* Optional extra shrink to be conservative wrt hard-tanh saturation. */
    limit1 *= 0.5f;

    for (int i = 0; i < MLP_INT8_HIDDEN_DIM; ++i) {
        for (int j = 0; j < MLP_INT8_INPUT_DIM; ++j) {
            float u = (float)rand() / (float)RAND_MAX;      // [0,1]
            float r = (2.0f * u - 1.0f) * limit1;            // [-limit1, limit1]
            net->W1[i][j] = mlp_int8_float_to_q0_7(r);
        }
        net->b1[i] = 0;  /* zero bias */
    }

    /* ---- Second layer: W2 (output x hidden) ---- */
    const float fan_in2  = (float)MLP_INT8_HIDDEN_DIM;
    const float fan_out2 = (float)MLP_INT8_OUTPUT_DIM;
    float limit2 = sqrtf(6.0f / (fan_in2 + fan_out2));
    limit2 *= 0.5f;

    for (int i = 0; i < MLP_INT8_OUTPUT_DIM; ++i) {
        for (int j = 0; j < MLP_INT8_HIDDEN_DIM; ++j) {
            float u = (float)rand() / (float)RAND_MAX;      // [0,1]
            float r = (2.0f * u - 1.0f) * limit2;            // [-limit2, limit2]
            net->W2[i][j] = mlp_int8_float_to_q0_7(r);
        }
        net->b2[i] = 0;  /* zero bias */
    }
}

/**
 * @brief Serialize all trainable parameters to a flat int8 array.
 *
 * Layout (row-major):
 *   1) W1[hidden][input]  -> H * I
 *   2) b1[hidden]         -> H
 *   3) W2[output][hidden] -> O * H
 *   4) b2[output]         -> O
 *
 * @param net  Pointer to MLP.
 * @param out  Destination array, of length at least mlp_int8_param_count().
 * @return     Number of parameters written.
 */
static inline uint32_t mlp_int8_serialize_params(const MLP_INT8 *net,
                                                 int8_t *out) {
    uint32_t idx = 0;

    /* W1 */
    for (int i = 0; i < MLP_INT8_HIDDEN_DIM; ++i) {
        for (int j = 0; j < MLP_INT8_INPUT_DIM; ++j) {
            out[idx++] = net->W1[i][j];
        }
    }

    /* b1 */
    for (int i = 0; i < MLP_INT8_HIDDEN_DIM; ++i) {
        out[idx++] = net->b1[i];
    }

    /* W2 */
    for (int i = 0; i < MLP_INT8_OUTPUT_DIM; ++i) {
        for (int j = 0; j < MLP_INT8_HIDDEN_DIM; ++j) {
            out[idx++] = net->W2[i][j];
        }
    }

    /* b2 */
    for (int i = 0; i < MLP_INT8_OUTPUT_DIM; ++i) {
        out[idx++] = net->b2[i];
    }

    return idx;
}

/**
 * @brief Deserialize all trainable parameters from a flat int8 array.
 *
 * The layout must match mlp_int8_serialize_params().
 *
 * @param net  Pointer to MLP to be filled.
 * @param in   Source array, of length at least mlp_int8_param_count().
 * @return     Number of parameters read.
 */
static inline uint32_t mlp_int8_deserialize_params(MLP_INT8 *net,
                                                   const int8_t *in) {
    uint32_t idx = 0;

    /* W1 */
    for (int i = 0; i < MLP_INT8_HIDDEN_DIM; ++i) {
        for (int j = 0; j < MLP_INT8_INPUT_DIM; ++j) {
            net->W1[i][j] = in[idx++];
        }
    }

    /* b1 */
    for (int i = 0; i < MLP_INT8_HIDDEN_DIM; ++i) {
        net->b1[i] = in[idx++];
    }

    /* W2 */
    for (int i = 0; i < MLP_INT8_OUTPUT_DIM; ++i) {
        for (int j = 0; j < MLP_INT8_HIDDEN_DIM; ++j) {
            net->W2[i][j] = in[idx++];
        }
    }

    /* b2 */
    for (int i = 0; i < MLP_INT8_OUTPUT_DIM; ++i) {
        net->b2[i] = in[idx++];
    }

    return idx;
}

#endif /* MLP_INT8_H_ */

