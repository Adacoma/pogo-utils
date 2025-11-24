#ifndef MLP_Q1_15_H_
#define MLP_Q1_15_H_

#include <stdint.h>
#include "fixp_q1_15.h"

/*
 * Compile-time configuration of the MLP dimensions.
 * You MUST define these before including this header, e.g.:
 *
 *   #define MLP_Q1_15_INPUT_DIM   4
 *   #define MLP_Q1_15_HIDDEN_DIM  8
 *   #define MLP_Q1_15_OUTPUT_DIM  2
 *   #include "MLP_Q1_15.h"
 */

#ifndef MLP_Q1_15_INPUT_DIM
# error "MLP_Q1_15_INPUT_DIM must be defined before including MLP_Q1_15.h"
#endif

#ifndef MLP_Q1_15_HIDDEN_DIM
# error "MLP_Q1_15_HIDDEN_DIM must be defined before including MLP_Q1_15.h"
#endif

#ifndef MLP_Q1_15_OUTPUT_DIM
# error "MLP_Q1_15_OUTPUT_DIM must be defined before including MLP_Q1_15.h"
#endif

/*
 * If you want the output layer to also use hard tanh,
 * define this macro before including the header:
 *
 *   #define MLP_Q1_15_OUTPUT_HARD_TANH
 */

/* Simple 1-hidden-layer MLP in Q1.15:
 *
 *   h = hard_tanh( W1 * x + b1 )
 *   y = [hard_tanh or linear]( W2 * h + b2 )
 *
 * All weights and biases are in Q1.15.
 */
typedef struct {
    q1_15_t W1[MLP_Q1_15_HIDDEN_DIM][MLP_Q1_15_INPUT_DIM];
    q1_15_t b1[MLP_Q1_15_HIDDEN_DIM];

    q1_15_t W2[MLP_Q1_15_OUTPUT_DIM][MLP_Q1_15_HIDDEN_DIM];
    q1_15_t b2[MLP_Q1_15_OUTPUT_DIM];
} MLP_Q1_15;


/*
 * Internal helper: dense layer with bias + activation.
 * We keep it file-local and specialized to Q1.15.
 */

static void dense_hidden_hard_tanh(const q1_15_t *weights,  /* [rows][cols] */
                                   const q1_15_t *biases,   /* [rows] */
                                   int rows,
                                   int cols,
                                   const q1_15_t *in,       /* [cols] */
                                   q1_15_t *out)            /* [rows] */
{
    for (int i = 0; i < rows; ++i) {
        /* Start accumulator from bias: convert Q1.15 -> Q2.30  */
        int32_t acc = ((int32_t)biases[i]) << Q1_15_FRACTIONAL_BITS;

        const q1_15_t *w_row = &weights[i * cols];

        /* Dot product: acc += sum_k w[i,k]*in[k] (Q2.30) */
        for (int k = 0; k < cols; ++k) {
            acc = q1_15_mac32(acc, w_row[k], in[k]);
        }

        /* Back to Q1.15 with rounding + saturation, then hard tanh */
        q1_15_t z = q1_15_from_acc32(acc);
        out[i] = q1_15_hard_tanh(z);
    }
}

static void dense_output_layer(const q1_15_t *weights,  /* [rows][cols] */
                               const q1_15_t *biases,   /* [rows] */
                               int rows,
                               int cols,
                               const q1_15_t *in,       /* [cols] */
                               q1_15_t *out)            /* [rows] */
{
    for (int i = 0; i < rows; ++i) {
        int32_t acc = ((int32_t)biases[i]) << Q1_15_FRACTIONAL_BITS;
        const q1_15_t *w_row = &weights[i * cols];

        for (int k = 0; k < cols; ++k) {
            acc = q1_15_mac32(acc, w_row[k], in[k]);
        }

        q1_15_t z = q1_15_from_acc32(acc);

#ifdef MLP_Q1_15_OUTPUT_HARD_TANH
        out[i] = q1_15_hard_tanh(z);
#else
        out[i] = z;  /* linear output */
#endif
    }
}


/**
 * @brief Forward pass of the Q1.15 MLP.
 *
 * @param net   Pointer to network parameters.
 * @param in    Input vector of length MLP_Q1_15_INPUT_DIM.
 * @param out   Output vector of length MLP_Q1_15_OUTPUT_DIM.
 *
 * All vectors are in Q1.15. Hidden activations always use hard tanh.
 * The output layer is:
 *   - linear if MLP_Q1_15_OUTPUT_HARD_TANH is NOT defined
 *   - hard_tanh if MLP_Q1_15_OUTPUT_HARD_TANH is defined
 */
static void mlp_q1_15_forward(const MLP_Q1_15 *net,
                       const q1_15_t in[MLP_Q1_15_INPUT_DIM],
                       q1_15_t out[MLP_Q1_15_OUTPUT_DIM]) {
    q1_15_t hidden[MLP_Q1_15_HIDDEN_DIM];

    /* First layer: input -> hidden with hard tanh */
    dense_hidden_hard_tanh(&net->W1[0][0],
                           net->b1,
                           MLP_Q1_15_HIDDEN_DIM,
                           MLP_Q1_15_INPUT_DIM,
                           in,
                           hidden);

    /* Second layer: hidden -> output (linear or hard tanh) */
    dense_output_layer(&net->W2[0][0],
                       net->b2,
                       MLP_Q1_15_OUTPUT_DIM,
                       MLP_Q1_15_HIDDEN_DIM,
                       hidden,
                       out);
}


#endif /* MLP_Q1_15_H_ */

