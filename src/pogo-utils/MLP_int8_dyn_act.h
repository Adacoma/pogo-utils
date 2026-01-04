#ifndef MLP_INT8_DYN_ACT_H_
#define MLP_INT8_DYN_ACT_H_

/*
 * Dynamic int8 MLP library (RV32IM-friendly)
 * - user-owned flat genome/parameter buffer (no internal allocation, no copies)
 * - supports multiple activations per network part:
 *     (input->hidden), (hidden->hidden), (hidden->output)
 * - provides genome layout/introspection helpers so different optimizers can
 *   target different parts (weights vs biases vs shifts vs gates vs clamps)
 *
 * Numeric convention:
 * - weights, inputs, activations and biases are int8 "Q0.7-like" values
 *   (raw integers -128..127).
 * - accumulator uses int32 sum of int8*int8 products.
 * - each affine layer applies a right-shift "s" (total shift on the int32 acc)
 *   before storing back to int8.
 *   * default s = 7 gives you the usual Q0.7 -> Q0.14 -> Q0.7 path.
 *   * other s values rescale the layer (useful for int8 inference and evolution).
 */

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- Activations ---------- */

typedef enum {
    /* acc = dot(W,x) + (b<<7); y = clip((acc + round) >> s) */
    MLP_I8_ACT_LINEAR_FIXED_SHIFT = 0,

    /* same as linear, but shift "s" is stored in the genome (one byte per layer) */
    MLP_I8_ACT_LINEAR_EVOLVED_SHIFT,

    /* hard-tanh after the affine (effectively a clamp in int8) */
    MLP_I8_ACT_HARD_TANH,

    /* hard SwiGLU: y = a * hsigmoid(g) (grouped gates)
     * gate params are part of the genome (optimizable).
     */
    MLP_I8_ACT_HARD_SWIGLU,

    /* hard SwiGLU but gate params are fixed pseudo-random (non-optimizable)
     * so the optimizable param count can match hard-tanh.
     */
    MLP_I8_ACT_HARD_SWIGLU_RANDOM_GATES,
} mlp_i8_act_type_t;

/* Optional extra per-layer scalar parameters can be stored in the genome.
 * This is convenient for neuro-evolution (mutate shifts/clamps) without changing
 * the weight layout.
 */

typedef struct {
    mlp_i8_act_type_t type;

    /* --- Shift handling ---
     * "shift" is the total right-shift applied on the int32 accumulator.
     * Typical values: 6..10, default 7.
     */
    uint8_t fixed_shift;        /* used when shift is not evolved */
    bool    shift_is_evolved;   /* if true, one byte per layer in genome */

    /* --- Clamp handling ---
     * After shifting, output can be additionally clamped to [-clamp,+clamp]
     * (for clamp<127). Use clamp=127 for full int8 range.
     */
    uint8_t clamp_fixed;        /* default 127 */
    bool    clamp_is_evolved;   /* if true, one byte per layer in genome */

    /* --- Gating (SwiGLU only) ---
     * gate_count controls grouped gating (default 1).
     * For RANDOM_GATES, gate_seed drives the pseudo-random weights/biases.
     */
    uint8_t  gate_count;
    uint32_t gate_seed;
} mlp_i8_act_cfg_t;

/* ---------- Network descriptor ---------- */

typedef struct {
    uint16_t input_dim;
    uint16_t hidden_dim;
    uint16_t output_dim;
    uint8_t  num_hidden_layers; /* >= 1 */

    /* activations for the 3 parts */
    mlp_i8_act_cfg_t act_in;     /* layer 0: input -> hidden */
    mlp_i8_act_cfg_t act_hidden; /* layers 1..L-1: hidden -> hidden */
    mlp_i8_act_cfg_t act_out;    /* output layer: hidden -> output */

    /* user-owned flat genome buffer; treated as raw bytes */
    const int8_t *genome;
} mlp_i8_dyn_t;

/* ---------- Introspection: genome layout ---------- */

#define MLP_I8_OFF_NONE ((uint32_t)0xFFFFFFFFu)

typedef struct {
    /* layer index: 0..L (L is output layer) */
    uint8_t  layer;
    uint16_t in_dim;
    uint16_t out_dim;

    /* main affine */
    uint32_t w_off;   /* int8 count starts at genome[w_off] */
    uint32_t w_len;   /* number of int8 weights */
    uint32_t b_off;
    uint32_t b_len;

    /* optional per-layer scalars (stored as bytes in the genome) */
    uint32_t shift_off; /* 1 byte, or MLP_I8_OFF_NONE */
    uint32_t clamp_off; /* 1 byte, or MLP_I8_OFF_NONE */

    /* optional gate affine (SwiGLU with optimizable gates) */
    uint32_t gate_w_off;
    uint32_t gate_w_len;
    uint32_t gate_b_off;
    uint32_t gate_b_len;

    /* resolved activation for this layer */
    mlp_i8_act_cfg_t act;
} mlp_i8_layer_view_t;

/* Compute total genome size (bytes) required for this architecture + activation config. */
uint32_t mlp_i8_genome_bytes(const mlp_i8_dyn_t *net);

/* Workspace (bytes) required by mlp_i8_forward_ws.
 * Workspace is an int8 buffer partitioned as:
 *   - hidden ping-pong buffers: 2*hidden_dim
 *   - a gate buffer of size max_gate_count (can be 0)
 */
uint32_t mlp_i8_workspace_bytes(const mlp_i8_dyn_t *net);

/* Fill a layer view (offsets and sizes) for layer in [0..L].
 * Returns false if layer index is invalid.
 */
bool mlp_i8_layer_view(const mlp_i8_dyn_t *net, uint8_t layer, mlp_i8_layer_view_t *out);

/* Convenience: number of layers including output (L+1). Returns 0 if invalid net. */
static inline uint8_t mlp_i8_num_layers_total(const mlp_i8_dyn_t *net) {
    if (!net || net->num_hidden_layers == 0) return 0;
    return (uint8_t)(net->num_hidden_layers + 1u);
}

/* ---------- Init ---------- */

/* Initialize a descriptor. This does not allocate or copy the genome. */
void mlp_i8_init(mlp_i8_dyn_t *net,
                 uint16_t input_dim,
                 uint16_t hidden_dim,
                 uint16_t output_dim,
                 uint8_t  num_hidden_layers,
                 mlp_i8_act_cfg_t act_in,
                 mlp_i8_act_cfg_t act_hidden,
                 mlp_i8_act_cfg_t act_out,
                 const int8_t *genome);

/* ---------- Forward ---------- */

/* Forward using a user-provided workspace (recommended for embedded).
 * - ws must point to at least mlp_i8_workspace_bytes(net) bytes.
 */
void mlp_i8_forward_ws(const mlp_i8_dyn_t *net,
                       const int8_t *in,
                       int8_t *out,
                       int8_t *ws,
                       uint32_t ws_bytes);

/* Optional convenience wrapper using VLA on the stack.
 * Define MLP_I8_NO_VLA to disable this function.
 */
#ifndef MLP_I8_NO_VLA
void mlp_i8_forward(const mlp_i8_dyn_t *net, const int8_t *in, int8_t *out);
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* MLP_INT8_DYN_ACT_H_ */
