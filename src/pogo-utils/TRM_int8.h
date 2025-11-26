#ifndef TRM_INT8_H_
#define TRM_INT8_H_

#include <stdint.h>

/*
 * Compile-time configuration of TRM dimensions.
 * You MUST define these before including this header, e.g.:
 *
 *   #define TRM_INT8_X_DIM        16   // question embedding size
 *   #define TRM_INT8_Y_DIM        16   // answer embedding size
 *   #define TRM_INT8_Z_DIM        16   // latent state size
 *   #define TRM_INT8_HIDDEN_DIM   64   // shared trunk width
 *
 * Optional (default values given below):
 *   #define TRM_INT8_N_LATENT   6   // n latent-recursion steps
 *   #define TRM_INT8_T_STEPS    3   // T outer refinement steps
 */

#ifndef TRM_INT8_X_DIM
# error "TRM_INT8_X_DIM must be defined before including TRM_int8.h"
#endif

#ifndef TRM_INT8_Y_DIM
# error "TRM_INT8_Y_DIM must be defined before including TRM_int8.h"
#endif

#ifndef TRM_INT8_Z_DIM
# error "TRM_INT8_Z_DIM must be defined before including TRM_int8.h"
#endif

#ifndef TRM_INT8_HIDDEN_DIM
# error "TRM_INT8_HIDDEN_DIM must be defined before including TRM_int8.h"
#endif

#ifndef TRM_INT8_N_LATENT
# define TRM_INT8_N_LATENT  6
#endif

#ifndef TRM_INT8_T_STEPS
# define TRM_INT8_T_STEPS   3
#endif

#define TRM_INT8_FRAC_BITS 7  /* Q0.7 fixed-point */
#define TRM_INT8_CORE_INPUT_DIM (TRM_INT8_X_DIM + TRM_INT8_Y_DIM + TRM_INT8_Z_DIM)

/*
 * Tiny Recursive Model (TRM) in int8 Q0.7.
 *
 * We implement a 2-layer shared trunk:
 *
 *   h1 = hard_tanh( W1 * inp + b1 )
 *   h2 = hard_tanh( W2 * h1 + b2 )
 *
 * with two linear heads:
 *
 *   z_next = hard_tanh( Wz * h2 + bz )
 *   y_next = hard_tanh( Wy * h2 + by )
 *
 * The recursion follows the TRM pseudocode:
 *
 *   for step in 1..T:
 *       for i in 1..n:   # latent reasoning
 *           z = f_z(x, y, z)
 *       y = f_y(y, z)    # refine answer
 *
 * using a *single* shared trunk (W1/W2) and two small heads.
 */

typedef struct {
    /* Shared 2-layer trunk */
    int8_t W1[TRM_INT8_HIDDEN_DIM][TRM_INT8_CORE_INPUT_DIM];
    int8_t b1[TRM_INT8_HIDDEN_DIM];

    int8_t W2[TRM_INT8_HIDDEN_DIM][TRM_INT8_HIDDEN_DIM];
    int8_t b2[TRM_INT8_HIDDEN_DIM];

    /* z-head: produces next latent state */
    int8_t Wz[TRM_INT8_Z_DIM][TRM_INT8_HIDDEN_DIM];
    int8_t bz[TRM_INT8_Z_DIM];

    /* y-head: produces refined answer */
    int8_t Wy[TRM_INT8_Y_DIM][TRM_INT8_HIDDEN_DIM];
    int8_t by[TRM_INT8_Y_DIM];
} TRM_INT8;

/* Convert from 32-bit accumulator (Q0.14) back to int8 Q0.7 with rounding + saturation. */
static inline int8_t trm_int8_from_acc32(int32_t acc) {
#if TRM_INT8_FRAC_BITS > 0
    if (acc >= 0)
        acc += (1 << (TRM_INT8_FRAC_BITS - 1));
    else
        acc -= (1 << (TRM_INT8_FRAC_BITS - 1));
    acc >>= TRM_INT8_FRAC_BITS;
#endif

    if (acc > 127)   acc = 127;
    if (acc < -128)  acc = -128;

    return (int8_t)acc;
}

/* Hard tanh on Q0.7 int8 (here it is just clamping to [-128, 127]). */
static inline int8_t trm_int8_hard_tanh(int8_t x) {
    if (x > 127)   return 127;
    if (x < -128)  return -128;
    return x;
}

/* Generic dense layer with hard-tanh activation.
 * weights: [rows][cols], biases: [rows], in: [cols], out: [rows]
 */
static void trm_int8_dense_layer(const int8_t *weights,
                                 const int8_t *biases,
                                 int rows,
                                 int cols,
                                 const int8_t *in,
                                 int8_t *out)
{
    for (int i = 0; i < rows; ++i) {
        int32_t acc = ((int32_t)biases[i]) << TRM_INT8_FRAC_BITS;
        const int8_t *w_row = &weights[i * cols];

        for (int k = 0; k < cols; ++k) {
            acc += (int32_t)w_row[k] * (int32_t)in[k];   /* Q0.7 * Q0.7 => Q0.14 */
        }

        int8_t z = trm_int8_from_acc32(acc);             /* back to Q0.7 */
        out[i] = trm_int8_hard_tanh(z);                  /* hard tanh activations */
    }
}

/* Shared 2-layer trunk:
 *   inp  : [TRM_INT8_CORE_INPUT_DIM]  (x || y || z)
 *   h_out: [TRM_INT8_HIDDEN_DIM]
 */
static void trm_int8_core_forward(const TRM_INT8 *net,
                                  const int8_t inp[TRM_INT8_CORE_INPUT_DIM],
                                  int8_t h_out[TRM_INT8_HIDDEN_DIM])
{
    int8_t h1[TRM_INT8_HIDDEN_DIM];

    /* First layer: input -> hidden */
    trm_int8_dense_layer(&net->W1[0][0],
                         net->b1,
                         TRM_INT8_HIDDEN_DIM,
                         TRM_INT8_CORE_INPUT_DIM,
                         inp,
                         h1);

    /* Second layer: hidden -> hidden */
    trm_int8_dense_layer(&net->W2[0][0],
                         net->b2,
                         TRM_INT8_HIDDEN_DIM,
                         TRM_INT8_HIDDEN_DIM,
                         h1,
                         h_out);
}

/* One latent update: z' = f_z(x, y, z).
 * x : [X], y : [Y], z_in : [Z], z_out : [Z]
 */
static void trm_int8_update_z(const TRM_INT8 *net,
                              const int8_t x[TRM_INT8_X_DIM],
                              const int8_t y[TRM_INT8_Y_DIM],
                              const int8_t z_in[TRM_INT8_Z_DIM],
                              int8_t       z_out[TRM_INT8_Z_DIM])
{
    int8_t inp[TRM_INT8_CORE_INPUT_DIM];
    int8_t h[TRM_INT8_HIDDEN_DIM];

    /* Pack inp = [x || y || z_in] */
    int idx = 0;
    for (int i = 0; i < TRM_INT8_X_DIM; ++i) {
        inp[idx++] = x[i];
    }
    for (int i = 0; i < TRM_INT8_Y_DIM; ++i) {
        inp[idx++] = y[i];
    }
    for (int i = 0; i < TRM_INT8_Z_DIM; ++i) {
        inp[idx++] = z_in[i];
    }

    trm_int8_core_forward(net, inp, h);

    /* z_out = Wz * h + bz (hard tanh) */
    trm_int8_dense_layer(&net->Wz[0][0],
                         net->bz,
                         TRM_INT8_Z_DIM,
                         TRM_INT8_HIDDEN_DIM,
                         h,
                         z_out);
}

/* One answer refinement: y' = f_y(y, z).
 * We reuse the same shared trunk but zero the x part of the input.
 *
 * y_in  : [Y], z : [Z], y_out : [Y]
 */
static void trm_int8_update_y(const TRM_INT8 *net,
                              const int8_t y_in[TRM_INT8_Y_DIM],
                              const int8_t z[TRM_INT8_Z_DIM],
                              int8_t       y_out[TRM_INT8_Y_DIM])
{
    int8_t inp[TRM_INT8_CORE_INPUT_DIM];
    int8_t h[TRM_INT8_HIDDEN_DIM];

    int idx = 0;
    /* x-part = 0 (no question update here) */
    for (int i = 0; i < TRM_INT8_X_DIM; ++i) {
        inp[idx++] = 0;
    }
    /* then y and z */
    for (int i = 0; i < TRM_INT8_Y_DIM; ++i) {
        inp[idx++] = y_in[i];
    }
    for (int i = 0; i < TRM_INT8_Z_DIM; ++i) {
        inp[idx++] = z[i];
    }

    trm_int8_core_forward(net, inp, h);

    /* y_out = Wy * h + by (hard tanh) */
    trm_int8_dense_layer(&net->Wy[0][0],
                         net->by,
                         TRM_INT8_Y_DIM,
                         TRM_INT8_HIDDEN_DIM,
                         h,
                         y_out);
}

/* Latent recursion: apply n times z <- f_z(x, y, z). */
static void trm_int8_latent_recursion(const TRM_INT8 *net,
                                      const int8_t x[TRM_INT8_X_DIM],
                                      const int8_t y[TRM_INT8_Y_DIM],
                                      int8_t       z[TRM_INT8_Z_DIM])
{
    int8_t z_tmp[TRM_INT8_Z_DIM];

    for (int i = 0; i < TRM_INT8_N_LATENT; ++i) {
        trm_int8_update_z(net, x, y, z, z_tmp);
        for (int k = 0; k < TRM_INT8_Z_DIM; ++k) {
            z[k] = z_tmp[k];
        }
    }
}

/* One outer TRM step:
 *   - recurse n times on z
 *   - refine y once
 */
static void trm_int8_step(const TRM_INT8 *net,
                          const int8_t x[TRM_INT8_X_DIM],
                          int8_t       y[TRM_INT8_Y_DIM],
                          int8_t       z[TRM_INT8_Z_DIM])
{
    int8_t y_tmp[TRM_INT8_Y_DIM];

    trm_int8_latent_recursion(net, x, y, z);
    trm_int8_update_y(net, y, z, y_tmp);

    for (int k = 0; k < TRM_INT8_Y_DIM; ++k) {
        y[k] = y_tmp[k];
    }
}

/**
 * @brief Full TRM forward recursion (inference).
 *
 * Conceptually matches the TRM pseudocode:
 *
 *   for t in 1..T:
 *       for i in 1..n:
 *           z = f_z(x, y, z)
 *       y = f_y(y, z)
 *
 * @param net   Pointer to TRM parameters.
 * @param x     Question embedding (length TRM_INT8_X_DIM, Q0.7).
 * @param y_io  In/out answer embedding (length TRM_INT8_Y_DIM, Q0.7).
 *              On input: initial answer (e.g., zeros).
 *              On output: refined answer after T steps.
 * @param z_io  In/out latent state (length TRM_INT8_Z_DIM, Q0.7).
 *              On input: initial latent (e.g., zeros).
 *              On output: final latent state.
 */
static void trm_int8_forward(const TRM_INT8 *net,
                             const int8_t x[TRM_INT8_X_DIM],
                             int8_t       y_io[TRM_INT8_Y_DIM],
                             int8_t       z_io[TRM_INT8_Z_DIM])
{
    for (int t = 0; t < TRM_INT8_T_STEPS; ++t) {
        trm_int8_step(net, x, y_io, z_io);
    }
}

/**
 * @brief Convenience wrapper: run TRM starting from zero y,z.
 *
 * @param net   TRM parameters.
 * @param x     Question embedding (Q0.7).
 * @param y_out Output refined answer (Q0.7).
 */
static void trm_int8_forward_zero_state(const TRM_INT8 *net,
                                        const int8_t x[TRM_INT8_X_DIM],
                                        int8_t       y_out[TRM_INT8_Y_DIM])
{
    int8_t y[TRM_INT8_Y_DIM];
    int8_t z[TRM_INT8_Z_DIM];

    for (int i = 0; i < TRM_INT8_Y_DIM; ++i) y[i] = 0;
    for (int i = 0; i < TRM_INT8_Z_DIM; ++i) z[i] = 0;

    trm_int8_forward(net, x, y, z);

    for (int i = 0; i < TRM_INT8_Y_DIM; ++i) {
        y_out[i] = y[i];
    }
}

#endif /* TRM_INT8_H_ */

