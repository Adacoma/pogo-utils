#ifndef MLP_INT8_H_
#define MLP_INT8_H_

#include <stdint.h>

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
                                      int32_t out[MLP_INT8_OUTPUT_DIM])
{
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


#endif /* MLP_INT8_H_ */

