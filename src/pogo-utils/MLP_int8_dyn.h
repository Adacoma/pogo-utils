#ifndef MLP_INT8_DYN_H_
#define MLP_INT8_DYN_H_

#include <stdint.h>

/*
 * Dynamic int8 MLP in Q0.7 fixed point.
 *
 * Architecture:
 *   - input_dim
 *   - num_hidden_layers hidden layers, all of size hidden_dim
 *   - output_dim
 *
 * All weights and biases live in a single flat int8 array owned by the user.
 * This struct only stores a pointer to that array and the dimensions; no copy
 * of the parameters is made.
 *
 * Layer layout in the flat parameter array (row-major):
 *
 *   For L = num_hidden_layers, I = input_dim, H = hidden_dim, O = output_dim:
 *
 *   1) First hidden layer: input -> hidden
 *        W0: H x I  (H * I values)
 *        b0: H      (H values)
 *
 *   2) Hidden layers 1..L-1: hidden -> hidden
 *        For each l in [1, L-1]:
 *          Wl: H x H  (H * H values)
 *          bl: H      (H values)
 *
 *   3) Output layer: hidden -> output
 *        W_out: O x H  (O * H values)
 *        b_out: O      (O values)
 *
 * Total parameter count:
 *   H*I + H                    // first hidden layer
 *   + (L-1) * (H*H + H)        // remaining hidden layers
 *   + O*H + O                  // output layer
 */

#ifndef MLP_INT8_DYN_FRAC_BITS
# define MLP_INT8_DYN_FRAC_BITS 7  /* Q0.7 fixed-point */
#endif

/* If you want the output layer to also use hard tanh,
 * define this macro before including the header:
 *
 *   #define MLP_INT8_DYN_OUTPUT_HARD_TANH
 */

typedef struct {
    uint16_t input_dim;
    uint16_t hidden_dim;
    uint16_t output_dim;
    uint8_t  num_hidden_layers;

    /* Pointer to user-owned parameter array (see layout above). */
    int8_t  *params;
} MLP_int8_dyn;

/* Convert from 32-bit accumulator (Q0.14) back to int8 Q0.7 with rounding + saturation. */
static inline int8_t mlp_int8_dyn_from_acc32(int32_t acc) {
#if MLP_INT8_DYN_FRAC_BITS > 0
    if (acc >= 0)
        acc += (1 << (MLP_INT8_DYN_FRAC_BITS - 1));
    else
        acc -= (1 << (MLP_INT8_DYN_FRAC_BITS - 1));
    acc >>= MLP_INT8_DYN_FRAC_BITS;
#endif

    if (acc > 127)   acc = 127;
    if (acc < -128)  acc = -128;

    return (int8_t)acc;
}

/* Hard tanh on Q0.7 int8. Here it's essentially just saturation in [-128, 127]. */
static inline int8_t mlp_int8_dyn_hard_tanh(int8_t x) {
    if (x > 127)   return 127;
    if (x < -128)  return -128;
    return x;
}

/**
 * @brief Return total number of parameters for the given architecture.
 *
 * The result tells you how big the flat params array must be.
 */
static inline uint32_t mlp_int8_dyn_param_count(uint16_t input_dim,
                                                uint16_t hidden_dim,
                                                uint16_t output_dim,
                                                uint8_t  num_hidden_layers) {
    uint32_t H = hidden_dim;
    uint32_t I = input_dim;
    uint32_t O = output_dim;
    uint32_t L = num_hidden_layers;

    /* Require at least one hidden layer; if not, this returns 0. */
    if (L == 0)
        return 0;

    uint32_t count = 0;

    /* First hidden layer: input -> hidden */
    count += H * I;      /* W0 */
    count += H;          /* b0 */

    /* Remaining hidden layers: hidden -> hidden */
    if (L > 1) {
        count += (L - 1u) * (H * H + H);
    }

    /* Output layer: hidden -> output */
    count += O * H;      /* W_out */
    count += O;          /* b_out */

    return count;
}

/**
 * @brief Initialize a dynamic int8 MLP descriptor.
 *
 * This does NOT allocate or copy any parameter memory. It simply stores the
 * pointer and dimensions. The user is responsible for:
 *   - allocating `params` (e.g. in .bss or with a custom allocator),
 *   - filling it with weights/biases using the layout described above.
 *
 * @param net   Pointer to MLP_int8_dyn struct to initialize.
 * @param input_dim        Number of input units.
 * @param hidden_dim       Number of units in each hidden layer.
 * @param output_dim       Number of output units.
 * @param num_hidden_layers  Number of hidden layers (>= 1).
 * @param params           Pointer to flat int8 parameter array.
 */
static inline void mlp_int8_dyn_init(MLP_int8_dyn *net,
                                     uint16_t input_dim,
                                     uint16_t hidden_dim,
                                     uint16_t output_dim,
                                     uint8_t  num_hidden_layers,
                                     int8_t  *params) {
    net->input_dim        = input_dim;
    net->hidden_dim       = hidden_dim;
    net->output_dim       = output_dim;
    net->num_hidden_layers = num_hidden_layers;
    net->params           = params;
}

/*
 * Internal helper: one dense layer with bias + hard tanh activation.
 *
 * Layout in *pp:
 *   weights: rows x cols
 *   biases : rows
 * After the call, *pp is advanced past W and b.
 */
static void mlp_int8_dyn_dense_hidden_hardtanh(const int8_t **pp,
                                               uint16_t rows,
                                               uint16_t cols,
                                               const int8_t *in,
                                               int8_t *out) {
    const int8_t *weights = *pp;
    const int8_t *biases  = weights + (uint32_t)rows * (uint32_t)cols;

    /* Advance pointer for next layer: W + b */
    *pp = biases + rows;

    for (uint16_t i = 0; i < rows; ++i) {
        int32_t acc = ((int32_t)biases[i]) << MLP_INT8_DYN_FRAC_BITS;

        const int8_t *w_row = weights + (uint32_t)i * (uint32_t)cols;
        for (uint16_t k = 0; k < cols; ++k) {
            acc += (int32_t)w_row[k] * (int32_t)in[k];
        }

        int8_t z = mlp_int8_dyn_from_acc32(acc);
        out[i] = mlp_int8_dyn_hard_tanh(z);
    }
}

/*
 * Internal helper: output layer (linear or hard-tanh depending on macro).
 *
 * Layout in *pp:
 *   weights: rows x cols
 *   biases : rows
 * After the call, *pp is advanced past W and b.
 */
static void mlp_int8_dyn_dense_output(const int8_t **pp,
                                      uint16_t rows,
                                      uint16_t cols,
                                      const int8_t *in,
                                      int8_t *out) {
    const int8_t *weights = *pp;
    const int8_t *biases  = weights + (uint32_t)rows * (uint32_t)cols;

    *pp = biases + rows;

    for (uint16_t i = 0; i < rows; ++i) {
        int32_t acc = ((int32_t)biases[i]) << MLP_INT8_DYN_FRAC_BITS;

        const int8_t *w_row = weights + (uint32_t)i * (uint32_t)cols;
        for (uint16_t k = 0; k < cols; ++k) {
            acc += (int32_t)w_row[k] * (int32_t)in[k];
        }

        int8_t z = mlp_int8_dyn_from_acc32(acc);

#ifdef MLP_INT8_DYN_OUTPUT_HARD_TANH
        out[i] = mlp_int8_dyn_hard_tanh(z);
#else
        out[i] = z;  /* linear output (still saturated to int8 range) */
#endif
    }
}

/**
 * @brief Forward pass of the dynamic int8 Q0.7 MLP.
 *
 * @param net   Pointer to network descriptor (dimensions + param pointer).
 * @param in    Input vector of length net->input_dim.
 * @param out   Output vector of length net->output_dim.
 *
 * All vectors are int8, interpreted as Q0.7. All hidden layers use hard tanh;
 * the output layer is linear or hard-tanh depending on MLP_INT8_DYN_OUTPUT_HARD_TANH.
 *
 * NOTE: this uses two temporary hidden buffers of size hidden_dim on the stack.
 * For typical small H (e.g. <= 32), this is fine on RV32IM.
 */
static void mlp_int8_dyn_forward(const MLP_int8_dyn *net,
                                 const int8_t *in,
                                 int8_t *out) {
    uint16_t I = net->input_dim;
    uint16_t H = net->hidden_dim;
    uint16_t O = net->output_dim;
    uint8_t  L = net->num_hidden_layers;

    /* Require at least one hidden layer; do nothing if invalid. */
    if (L == 0 || H == 0 || I == 0 || O == 0)
        return;

    /* Two ping-pong buffers for hidden activations. */
    int8_t h1[H];
    int8_t h2[H];

    int8_t *cur  = h1;
    int8_t *next = h2;

    const int8_t *p = net->params;

    /* First hidden layer: input -> hidden_dim */
    mlp_int8_dyn_dense_hidden_hardtanh(&p, H, I, in, cur);

    /* Remaining hidden layers (if any): hidden_dim -> hidden_dim */
    for (uint8_t layer = 1; layer < L; ++layer) {
        mlp_int8_dyn_dense_hidden_hardtanh(&p, H, H, cur, next);

        /* Swap buffers */
        int8_t *tmp = cur;
        cur = next;
        next = tmp;
    }

    /* Output layer: hidden_dim -> output_dim */
    mlp_int8_dyn_dense_output(&p, O, H, cur, out);
}

#endif /* MLP_INT8_DYN_H_ */

