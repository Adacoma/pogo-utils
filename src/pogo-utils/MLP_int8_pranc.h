/**
 * @file MLP_int8_pranc.h
 * @brief PRANC wrapper for the int8 MLP (Q0.7) on Pogobots.
 *
 * This header defines:
 *   - MLP_PRANC_SIGNATURE: compressed representation of W1
 *   - mlp_int8_pranc_fill_W1(): reconstruct W1 into an existing MLP_INT8
 *   - mlp_int8_pranc_init(): reconstruct W1 and copy dense b1/W2/b2
 *
 * The idea:
 *   W1 is not stored explicitly. Instead we store a small vector of
 *   coefficients α_k together with a PRNG seed. At initialisation time
 *   the robot recomputes W1 deterministically from (seed, α).
 *
 * The Python script train_mnist_qat_to_c.py generates:
 *   - one or more MLP_PRANC_SIGNATUREs (mnist_mlp_pranc_sigs[...])
 *   - dense b1, W2, b2 in int8 Q0.7
 * and defines PRANC_NUM_BASIS before including this header.
 */

#ifndef POGO_UTILS_MLP_INT8_PRANC_H
#define POGO_UTILS_MLP_INT8_PRANC_H

#include <stdint.h>
#include "pogo-utils/MLP_int8.h"  /* defines MLP_INT8 + MLP_INT8_INPUT_DIM/HIDDEN_DIM/OUTPUT_DIM */

#include <math.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Compile-time configuration                                                 */
/* -------------------------------------------------------------------------- */

/* Must match the `basis_scale` used in PRANCMLP() in train_mnist_qat_to_c.py */
#ifndef PRANC_BASIS_SCALE
#  define PRANC_BASIS_SCALE 0.5f
#endif

/* Size of flattened W1 (hidden_dim * input_dim). */
#define MLP_INT8_W1_SIZE  ((uint32_t)MLP_INT8_HIDDEN_DIM * (uint32_t)MLP_INT8_INPUT_DIM)

/**
 * @brief Number of PRANC basis vectors used to encode W1.
 *
 * This MUST be defined before including this header (typically by the
 * auto-generated C file from the training script).
 *
 * Example (from mnist_mlp_pranc_params.c):
 *     #define PRANC_NUM_BASIS 64
 *     #include "pogo-utils/MLP_int8_pranc.h"
 */
#ifndef PRANC_NUM_BASIS
#  define PRANC_NUM_BASIS 64
#endif

/**
 * @brief Fractional bits for the int8 fixed-point format (Q0.7).
 *
 * W1 is reconstructed in float and then quantised to int8 using this
 * number of fractional bits, to match the Python QAT export.
 */
#ifndef MLP_INT8_FRAC_BITS
#  define MLP_INT8_FRAC_BITS 7
#endif

/* -------------------------------------------------------------------------- */
/* Types                                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief Compressed representation of the first layer weights (W1).
 *
 * Conceptually:
 *   - A fixed pseudorandom basis B_k ∈ R^D is generated on the fly from
 *     `seed` (no need to store B explicitly on the robot).
 *   - The full vectorised W1 (of dimension D = HIDDEN_DIM * INPUT_DIM)
 *     is:
 *         θ = sum_k α_k * B_k
 *     and then reshaped to [HIDDEN_DIM, INPUT_DIM] and quantised to Q0.7.
 *
 * The `alpha` array has length PRANC_NUM_BASIS. The effective number of
 * active basis vectors is given by `num_basis` so you can train with a
 * smaller basis than the compiled maximum if needed.
 */
typedef struct {
    uint32_t seed;                      /**< Seed for the deterministic PRNG. */
    uint16_t num_basis;                 /**< Number of active basis vectors.  */
    float    alpha[PRANC_NUM_BASIS];    /**< Coefficients α_k (float32).      */
} MLP_PRANC_SIGNATURE;

/* -------------------------------------------------------------------------- */
/* Xorshift32 PRNG – must match Python xorshift32_step                        */
/* -------------------------------------------------------------------------- */

static inline uint32_t pranc_xorshift32(uint32_t state)
{
    state ^= state << 13;
    state ^= state >> 17;
    state ^= state << 5;
    return state;
}

/* Map uint32_t → float in [-1, 1), same as:
 *   u = (state / 2**31) - 1.0
 * in Python.
 */
static inline float pranc_uint32_to_uniform_m1_1(uint32_t state)
{
    /* 2^31 = 2147483648.0f */
    return ((float)state / 2147483648.0f) - 1.0f;
}

/*
 * NOTE ON MEMORY:
 *  - We keep a temporary float buffer of size HIDDEN * INPUT.
 *  - For MNIST (32x784) this is 32*784 = 25088 floats ≈ 100 KB.
 *  - This lives in .bss here (static array). If that’s too big for the
 *    robot, reduce HIDDEN_DIM or move to a different reconstruction
 *    strategy (hash-based per-entry basis, etc.).
 */
static float g_pranc_theta[MLP_INT8_W1_SIZE];


/* -------------------------------------------------------------------------- */
/* API                                                                        */
/* -------------------------------------------------------------------------- */

/**
 * @brief Recompute the W1 matrix of an existing int8 MLP from a PRANC signature.
 *
 * This function:
 *   - Uses `sig->seed` to drive a small PRNG that generates a reproducible
 *     random basis on the fly;
 *   - Combines this basis with the coefficients `sig->alpha[k]` to build
 *     a float W1;
 *   - Quantises W1 to int8 Q0.7 and stores it in `mlp->W1`.
 *
 * Only the first layer weights are modified. Biases and second layer are
 * untouched.
 *
 * Requirements:
 *   - MLP_INT8_INPUT_DIM, MLP_INT8_HIDDEN_DIM and MLP_INT8_OUTPUT_DIM must
 *     be defined (via MLP_int8.h).
 *   - The internal layout of MLP_INT8 must expose a W1 buffer compatible
 *     with this function (see implementation in MLP_int8_pranc.c).
 */
static void mlp_int8_pranc_fill_W1(MLP_INT8 *mlp,
                            const MLP_PRANC_SIGNATURE *sig) {
    if (!mlp || !sig) {
        return;
    }

    const uint16_t K = (sig->num_basis <= PRANC_NUM_BASIS)
                           ? sig->num_basis
                           : PRANC_NUM_BASIS;
    const uint32_t D = MLP_INT8_W1_SIZE;

    /* 1) Reset theta (vectorised W1) to 0. */
    for (uint32_t j = 0; j < D; ++j) {
        g_pranc_theta[j] = 0.0f;
    }

    /* 2) Recreate the same random basis as in Python and accumulate:
     *
     *    theta[j] = sum_{k=0..K-1} alpha[k] * B[k, j]
     *
     * where B[k,j] is generated with xorshift32 in the same order as
     * Python:
     *
     *    state = seed
     *    for k in range(K):
     *        for j in range(D):
     *            state = xorshift32_step(state)
     *            B[k, j] = ((state / 2**31) - 1.0) * basis_scale
     */
    uint32_t state = sig->seed;

    for (uint16_t k = 0; k < K; ++k) {
        const float alpha_k = sig->alpha[k];

        for (uint32_t j = 0; j < D; ++j) {
            state = pranc_xorshift32(state);
            float u = pranc_uint32_to_uniform_m1_1(state);
            float basis_val = u * PRANC_BASIS_SCALE;
            g_pranc_theta[j] += alpha_k * basis_val;
        }
    }

    /* 3) Quantise theta to int8 Q0.7 and write into mlp->W1. */
    const float qscale = (float)(1 << MLP_INT8_FRAC_BITS);

    for (uint32_t j = 0; j < D; ++j) {
        float x = g_pranc_theta[j] * qscale;

        /* Round to nearest int. */
        int32_t qi = (int32_t)lrintf(x);

        /* Saturate to int8 range. */
        if (qi > 127)  qi = 127;
        if (qi < -128) qi = -128;

        int8_t q8 = (int8_t)qi;

        /* Map flat index j back to [hidden, input]. */
        uint32_t h = j / MLP_INT8_INPUT_DIM;
        uint32_t i = j % MLP_INT8_INPUT_DIM;

        mlp->W1[h][i] = q8;
    }
}

/**
 * @brief Initialise an int8 MLP (Q0.7) from a PRANC signature + dense layers.
 *
 * This is a convenience wrapper that:
 *   1. Reconstructs W1 via mlp_int8_pranc_fill_W1();
 *   2. Copies the already-quantised dense parameters:
 *        - b1  (int8[HIDDEN_DIM])
 *        - W2  (int8[OUTPUT_DIM][HIDDEN_DIM])
 *        - b2  (int8[OUTPUT_DIM])
 *
 * It is meant to be used with the arrays exported by train_mnist_qat_to_c.py,
 * for example:
 *
 *     mlp_int8_pranc_init(&my_mlp,
 *                         &mnist_mlp_pranc_sigs[i],
 *                         mnist_b1[i],
 *                         mnist_W2[i],
 *                         mnist_b2[i]);
 */
static void mlp_int8_pranc_init(MLP_INT8 *mlp,
                         const MLP_PRANC_SIGNATURE *sig,
                         const int8_t *b1,
                         const int8_t W2[MLP_INT8_OUTPUT_DIM]
                                         [MLP_INT8_HIDDEN_DIM],
                         const int8_t *b2) {
    if (!mlp || !sig || !b1 || !W2 || !b2) {
        return;
    }

    /* Reconstruct W1 from PRANC. */
    mlp_int8_pranc_fill_W1(mlp, sig);

    /* Copy first-layer bias. */
    for (int h = 0; h < MLP_INT8_HIDDEN_DIM; ++h) {
        mlp->b1[h] = b1[h];
    }

    /* Copy second-layer weights. */
    for (int o = 0; o < MLP_INT8_OUTPUT_DIM; ++o) {
        for (int h = 0; h < MLP_INT8_HIDDEN_DIM; ++h) {
            mlp->W2[o][h] = W2[o][h];
        }
    }

    /* Copy second-layer bias. */
    for (int o = 0; o < MLP_INT8_OUTPUT_DIM; ++o) {
        mlp->b2[o] = b2[o];
    }
}


#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* POGO_UTILS_MLP_INT8_PRANC_H */

