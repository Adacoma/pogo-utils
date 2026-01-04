#include "MLP_int8_dyn_act.h"

/* ------------------------------ Small utilities ------------------------------ */

static inline uint8_t  u8_max(uint8_t a, uint8_t b) { return (a > b) ? a : b; }

/* Symmetric clamp helper.
 * - If clamp >= 127: use full int8 range [-128, 127]
 * - Else clamp to [-clamp, +clamp]
 */
static inline int8_t clamp_i8(int32_t x, uint8_t clamp) {
    if (clamp >= 127u) {
        if (x > 127)  x = 127;
        if (x < -128) x = -128;
        return (int8_t)x;
    }
    int32_t c = (int32_t)clamp;
    if (x > c)  x = c;
    if (x < -c) x = -c;
    return (int8_t)x;
}

/* Shift + rounding + clamp.
 * shift is total right-shift applied to the int32 accumulator.
 */
static inline int8_t acc_to_i8(int32_t acc, uint8_t shift, uint8_t clamp) {
    if (shift != 0u) {
        /* round-to-nearest (ties away from 0) */
        int32_t r = (int32_t)(1u << (shift - 1u));
        if (acc >= 0) acc += r;
        else          acc -= r;
        acc >>= shift;
    }
    return clamp_i8(acc, clamp);
}

/* Hard-sigmoid in Q0.7-like int8.
 * Input: int8 in [-128, 127]
 * Output: uint8 in [0, 127]
 *   hsigmoid(x) = clamp( x/2 + 0.5, 0, 1 )
 */
static inline uint8_t hard_sigmoid_q7(int8_t x) {
    int16_t t = (int16_t)(x >> 1) + 64; /* [-64..63] + 64 -> [0..127] */
    if (t < 0)   t = 0;
    if (t > 127) t = 127;
    return (uint8_t)t;
}

static inline int8_t mul_q7(int8_t a, uint8_t b_q7) {
    /* (a * b) >> 7 with rounding.
     * b is non-negative.
     */
    int32_t prod = (int32_t)a * (int32_t)b_q7; /* Q0.7 * Q0.7 -> Q0.14 */
    prod += (prod >= 0) ? (1 << 6) : -(1 << 6);
    prod >>= 7;
    if (prod > 127)  prod = 127;
    if (prod < -128) prod = -128;
    return (int8_t)prod;
}

/* Very small hash for deterministic pseudo-random gate weights.
 * splitmix32-style.
 */
static inline uint32_t hash32(uint32_t x) {
    x += 0x9e3779b9u;
    x ^= x >> 16;
    x *= 0x85ebca6bu;
    x ^= x >> 13;
    x *= 0xc2b2ae35u;
    x ^= x >> 16;
    return x;
}

/* Generate a small int8 in roughly [-16, 15] from seed+coords.
 * (cheap scaling to reduce early saturation)
 */
static inline int8_t prand_i8_small(uint32_t seed, uint32_t a, uint32_t b) {
    uint32_t h = hash32(seed ^ (a * 0x1b873593u) ^ (b * 0x85ebca6bu));
    int8_t top = (int8_t)(h >> 24); /* -128..127 */
    return (int8_t)(top >> 3);      /* -16..15 */
}

/* Resolve gate_count to something safe.
 * - default 1
 * - clamp to [1..out_dim]
 */
static inline uint8_t resolve_gate_count(uint8_t gate_count, uint16_t out_dim) {
    if (gate_count == 0u) gate_count = 1u;
    if (gate_count > out_dim) gate_count = (uint8_t)out_dim;
    if (gate_count == 0u) gate_count = 1u;
    return gate_count;
}

static inline uint8_t gate_index_for_neuron(uint16_t i, uint16_t out_dim, uint8_t gate_count) {
    /* stable mapping even if out_dim not divisible by gate_count */
    return (uint8_t)(((uint32_t)i * (uint32_t)gate_count) / (uint32_t)out_dim);
}

/* ------------------------------ Layout helpers ------------------------------ */

static inline mlp_i8_act_cfg_t cfg_defaults(mlp_i8_act_cfg_t c) {
    if (c.fixed_shift == 0u) c.fixed_shift = 7u;
    if (c.clamp_fixed == 0u) c.clamp_fixed = 127u;
    if (c.type == MLP_I8_ACT_HARD_SWIGLU || c.type == MLP_I8_ACT_HARD_SWIGLU_RANDOM_GATES) {
        c.gate_count = resolve_gate_count(c.gate_count, 1u); /* will be re-resolved per layer */
    } else {
        c.gate_count = 0u;
        c.gate_seed = 0u;
    }
    return c;
}

void mlp_i8_init(mlp_i8_dyn_t *net,
                 uint16_t input_dim,
                 uint16_t hidden_dim,
                 uint16_t output_dim,
                 uint8_t  num_hidden_layers,
                 mlp_i8_act_cfg_t act_in,
                 mlp_i8_act_cfg_t act_hidden,
                 mlp_i8_act_cfg_t act_out,
                 const int8_t *genome) {
    if (!net) return;
    net->input_dim = input_dim;
    net->hidden_dim = hidden_dim;
    net->output_dim = output_dim;
    net->num_hidden_layers = num_hidden_layers;
    net->act_in = cfg_defaults(act_in);
    net->act_hidden = cfg_defaults(act_hidden);
    net->act_out = cfg_defaults(act_out);
    net->genome = genome;
}

/* Compute per-layer byte consumption (advancing cursor), and optionally fill view. */
static bool layer_iter(const mlp_i8_dyn_t *net, uint8_t want_layer, mlp_i8_layer_view_t *out_view,
                       uint32_t *cursor_io) {
    if (!net || net->num_hidden_layers == 0) return false;

    uint8_t L = net->num_hidden_layers;
    uint8_t total_layers = (uint8_t)(L + 1u);
    if (want_layer >= total_layers) return false;

    uint32_t cursor = 0u;
    if (cursor_io) cursor = *cursor_io;

    for (uint8_t layer = 0; layer < total_layers; ++layer) {
        mlp_i8_act_cfg_t act;
        uint16_t in_dim, out_dim;

        if (layer == 0u) {
            act = net->act_in;
            in_dim = net->input_dim;
            out_dim = net->hidden_dim;
        } else if (layer < L) {
            act = net->act_hidden;
            in_dim = net->hidden_dim;
            out_dim = net->hidden_dim;
        } else {
            act = net->act_out;
            in_dim = net->hidden_dim;
            out_dim = net->output_dim;
        }
        act = cfg_defaults(act);

        /* main W/b */
        uint32_t w_len = (uint32_t)out_dim * (uint32_t)in_dim;
        uint32_t b_len = (uint32_t)out_dim;
        uint32_t w_off = cursor;
        cursor += w_len;
        uint32_t b_off = cursor;
        cursor += b_len;

        uint32_t shift_off = MLP_I8_OFF_NONE;
        uint32_t clamp_off = MLP_I8_OFF_NONE;
        if (act.shift_is_evolved) { shift_off = cursor; cursor += 1u; }
        if (act.clamp_is_evolved) { clamp_off = cursor; cursor += 1u; }

        uint32_t gate_w_off = MLP_I8_OFF_NONE;
        uint32_t gate_b_off = MLP_I8_OFF_NONE;
        uint32_t gate_w_len = 0u;
        uint32_t gate_b_len = 0u;

        if (act.type == MLP_I8_ACT_HARD_SWIGLU) {
            uint8_t g = resolve_gate_count(act.gate_count, out_dim);
            gate_w_len = (uint32_t)g * (uint32_t)in_dim;
            gate_b_len = (uint32_t)g;
            gate_w_off = cursor; cursor += gate_w_len;
            gate_b_off = cursor; cursor += gate_b_len;
        }

        if (layer == want_layer && out_view) {
            out_view->layer = layer;
            out_view->in_dim = in_dim;
            out_view->out_dim = out_dim;
            out_view->w_off = w_off;
            out_view->w_len = w_len;
            out_view->b_off = b_off;
            out_view->b_len = b_len;
            out_view->shift_off = shift_off;
            out_view->clamp_off = clamp_off;
            out_view->gate_w_off = gate_w_off;
            out_view->gate_w_len = gate_w_len;
            out_view->gate_b_off = gate_b_off;
            out_view->gate_b_len = gate_b_len;
            out_view->act = act;
            if (cursor_io) *cursor_io = cursor;
            return true;
        }
    }

    if (cursor_io) *cursor_io = cursor;
    return false;
}

bool mlp_i8_layer_view(const mlp_i8_dyn_t *net, uint8_t layer, mlp_i8_layer_view_t *out) {
    if (!out) return false;
    uint32_t cursor = 0u;
    return layer_iter(net, layer, out, &cursor);
}

uint32_t mlp_i8_genome_bytes(const mlp_i8_dyn_t *net) {
    uint32_t cursor = 0u;
    /* Ask for last layer view only to advance cursor fully */
    mlp_i8_layer_view_t tmp;
    uint8_t n = mlp_i8_num_layers_total(net);
    if (n == 0u) return 0u;
    (void)layer_iter(net, (uint8_t)(n - 1u), &tmp, &cursor);
    return cursor;
}

uint32_t mlp_i8_workspace_bytes(const mlp_i8_dyn_t *net) {
    if (!net || net->num_hidden_layers == 0u) return 0u;
    uint8_t max_gate = 0u;

    /* gate buffers are only needed when some part uses SwiGLU */
    mlp_i8_act_cfg_t ai = cfg_defaults(net->act_in);
    mlp_i8_act_cfg_t ah = cfg_defaults(net->act_hidden);
    mlp_i8_act_cfg_t ao = cfg_defaults(net->act_out);

    if (ai.type == MLP_I8_ACT_HARD_SWIGLU || ai.type == MLP_I8_ACT_HARD_SWIGLU_RANDOM_GATES)
        max_gate = u8_max(max_gate, resolve_gate_count(ai.gate_count, net->hidden_dim));
    if (ah.type == MLP_I8_ACT_HARD_SWIGLU || ah.type == MLP_I8_ACT_HARD_SWIGLU_RANDOM_GATES)
        max_gate = u8_max(max_gate, resolve_gate_count(ah.gate_count, net->hidden_dim));
    if (ao.type == MLP_I8_ACT_HARD_SWIGLU || ao.type == MLP_I8_ACT_HARD_SWIGLU_RANDOM_GATES)
        max_gate = u8_max(max_gate, resolve_gate_count(ao.gate_count, net->output_dim));

    /* hidden ping-pong buffers + gates */
    return (uint32_t)(2u * (uint32_t)net->hidden_dim + (uint32_t)max_gate);
}

/* ------------------------------ Forward kernels ------------------------------ */

static inline uint8_t layer_shift(const mlp_i8_dyn_t *net, const mlp_i8_layer_view_t *v) {
    if (v->act.shift_is_evolved && v->shift_off != MLP_I8_OFF_NONE) {
        /* store as unsigned shift; clamp to 31 */
        uint8_t s = (uint8_t)net->genome[v->shift_off];
        if (s > 31u) s = 31u;
        return s;
    }
    return v->act.fixed_shift;
}

static inline uint8_t layer_clamp(const mlp_i8_dyn_t *net, const mlp_i8_layer_view_t *v) {
    if (v->act.clamp_is_evolved && v->clamp_off != MLP_I8_OFF_NONE) {
        uint8_t c = (uint8_t)net->genome[v->clamp_off];
        if (c == 0u) c = 127u;
        if (c > 127u) c = 127u;
        return c;
    }
    return (v->act.clamp_fixed == 0u) ? 127u : v->act.clamp_fixed;
}

static inline int32_t dot_acc_q14(const int8_t *w_row, const int8_t *in, uint16_t in_dim) {
    int32_t acc = 0;
    for (uint16_t k = 0; k < in_dim; ++k) {
        acc += (int32_t)w_row[k] * (int32_t)in[k];
    }
    return acc;
}

static void dense_linear(const mlp_i8_dyn_t *net, const mlp_i8_layer_view_t *v,
                         const int8_t *in, int8_t *out,
                         uint8_t shift, uint8_t clamp) {
    const int8_t *W = net->genome + v->w_off;
    const int8_t *B = net->genome + v->b_off;

    for (uint16_t i = 0; i < v->out_dim; ++i) {
        const int8_t *w_row = W + (uint32_t)i * (uint32_t)v->in_dim;
        int32_t acc = dot_acc_q14(w_row, in, v->in_dim);
        /* Bias is Q0.7-like, align it to the Q0.14 acc by <<7. */
        acc += ((int32_t)B[i]) << 7;
        out[i] = acc_to_i8(acc, shift, clamp);
    }
}

static void dense_hardtanh(const mlp_i8_dyn_t *net, const mlp_i8_layer_view_t *v,
                           const int8_t *in, int8_t *out,
                           uint8_t shift, uint8_t clamp) {
    /* hard-tanh is just clamping in int8 after shift */
    dense_linear(net, v, in, out, shift, clamp);
}

static void dense_hard_swiglu(const mlp_i8_dyn_t *net, const mlp_i8_layer_view_t *v,
                             const int8_t *in, int8_t *out,
                             uint8_t shift, uint8_t clamp,
                             int8_t *gate_buf, uint8_t gate_count) {
    const int8_t *W = net->genome + v->w_off;
    const int8_t *B = net->genome + v->b_off;

    /* 1) compute gates (int8 pre-activations -> hard_sigmoid) */
    if (v->act.type == MLP_I8_ACT_HARD_SWIGLU) {
        const int8_t *GW = net->genome + v->gate_w_off;
        const int8_t *GB = net->genome + v->gate_b_off;

        for (uint8_t g = 0; g < gate_count; ++g) {
            const int8_t *gw_row = GW + (uint32_t)g * (uint32_t)v->in_dim;
            int32_t accg = dot_acc_q14(gw_row, in, v->in_dim);
            accg += ((int32_t)GB[g]) << 7;
            gate_buf[g] = acc_to_i8(accg, shift, 127u);
        }
    } else {
        /* RANDOM_GATES: generate gate weights/biases on the fly */
        uint32_t seed = v->act.gate_seed;
        for (uint8_t g = 0; g < gate_count; ++g) {
            int32_t accg = 0;
            for (uint16_t k = 0; k < v->in_dim; ++k) {
                int8_t w = prand_i8_small(seed, (uint32_t)g, (uint32_t)k);
                accg += (int32_t)w * (int32_t)in[k];
            }
            int8_t bg = prand_i8_small(seed ^ 0xA5A5A5A5u, (uint32_t)g, 0xBEEF);
            accg += ((int32_t)bg) << 7;
            gate_buf[g] = acc_to_i8(accg, shift, 127u);
        }
    }

    /* 2) main affine + gating */
    for (uint16_t i = 0; i < v->out_dim; ++i) {
        const int8_t *w_row = W + (uint32_t)i * (uint32_t)v->in_dim;
        int32_t acc = dot_acc_q14(w_row, in, v->in_dim);
        acc += ((int32_t)B[i]) << 7;
        int8_t a = acc_to_i8(acc, shift, 127u);

        uint8_t gidx = gate_index_for_neuron(i, v->out_dim, gate_count);
        uint8_t gate = hard_sigmoid_q7(gate_buf[gidx]);

        int8_t y = mul_q7(a, gate);
        out[i] = clamp_i8((int32_t)y, clamp);
    }
}

void mlp_i8_forward_ws(const mlp_i8_dyn_t *net,
                       const int8_t *in,
                       int8_t *out,
                       int8_t *ws,
                       uint32_t ws_bytes) {
    if (!net || !net->genome || !in || !out || net->num_hidden_layers == 0u) return;
    if (net->input_dim == 0u || net->hidden_dim == 0u || net->output_dim == 0u) return;

    uint32_t need_ws = mlp_i8_workspace_bytes(net);
    if (!ws || ws_bytes < need_ws) return;

    /* workspace partition */
    int8_t *h1 = ws;
    int8_t *h2 = ws + net->hidden_dim;
    int8_t *gate_buf = ws + 2u * net->hidden_dim;

    int8_t *cur = h1;
    int8_t *nxt = h2;

    uint8_t total_layers = mlp_i8_num_layers_total(net);

    for (uint8_t layer = 0; layer < total_layers; ++layer) {
        mlp_i8_layer_view_t v;
        if (!mlp_i8_layer_view(net, layer, &v)) return;

        uint8_t shift = layer_shift(net, &v);
        uint8_t clamp = layer_clamp(net, &v);

        const int8_t *layer_in = (layer == 0u) ? in : cur;
        int8_t *layer_out = (layer == (uint8_t)(total_layers - 1u)) ? out : nxt;

        switch (v.act.type) {
            case MLP_I8_ACT_LINEAR_FIXED_SHIFT:
            case MLP_I8_ACT_LINEAR_EVOLVED_SHIFT:
                dense_linear(net, &v, layer_in, layer_out, shift, clamp);
                break;
            case MLP_I8_ACT_HARD_TANH:
                dense_hardtanh(net, &v, layer_in, layer_out, shift, clamp);
                break;
            case MLP_I8_ACT_HARD_SWIGLU:
            case MLP_I8_ACT_HARD_SWIGLU_RANDOM_GATES: {
                uint8_t g = resolve_gate_count(v.act.gate_count, v.out_dim);
                dense_hard_swiglu(net, &v, layer_in, layer_out, shift, clamp, gate_buf, g);
                break;
            }
            default:
                /* unsupported -> treat as linear */
                dense_linear(net, &v, layer_in, layer_out, shift, clamp);
                break;
        }

        /* advance hidden buffers if not output */
        if (layer != (uint8_t)(total_layers - 1u)) {
            int8_t *tmp = cur;
            cur = layer_out;
            nxt = (tmp == h1) ? h2 : h1;
        }
    }
}

#ifndef MLP_I8_NO_VLA
void mlp_i8_forward(const mlp_i8_dyn_t *net, const int8_t *in, int8_t *out) {
    if (!net) return;
    uint32_t ws_bytes = mlp_i8_workspace_bytes(net);
    if (ws_bytes == 0u) return;

    /* VLA workspace in int8 */
    int8_t ws[ws_bytes];
    mlp_i8_forward_ws(net, in, out, ws, ws_bytes);
}
#endif
