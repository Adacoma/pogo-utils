/**
 * @file main.c
 * @brief Pogosim/Pogobot example using a dynamic int8 MLP with selectable activations.
 *
 * The MLP implementation (see `pogo-utils/MLP_int8_dyn_act.h`) is **dynamic** (dimensions provided at init)
 * and uses a **flat "genome" buffer** (`int8_t*`) that contains all optimizable parameters (weights/biases and
 * optionally a few evolved scalars such as per-layer shifts). The genome is owned by the caller, enabling
 * mixed optimization strategies (e.g., PRANC-encoded weights, direct neuro-evolution on biases/shifts, etc.).
 *
 * @section mlp_i8_quant Fixed-point conventions
 * Inputs/hidden/output tensors are stored as signed int8 in a Q0.7-like convention (range ~[-1, 1)).
 * Dot-products accumulate into an `int32_t` accumulator:
 * \f[
 *   acc_j = \sum_i w_{j,i} x_i + (b_j \ll 7)
 * \f]
 *
 * @section mlp_i8_acts Activation functions (per part)
 * The network supports selecting different activations independently for:
 * - input \f$\rightarrow\f$ hidden(0)
 * - hidden \f$\rightarrow\f$ hidden
 * - last hidden \f$\rightarrow\f$ output
 *
 * The key activation families are:
 *
 * @subsection mlp_i8_act_linear A/B) Linear with right-shift
 * A linear layer applies an (optional) right shift \f$s\f$ before storing back to int8:
 * \f[
 *   y_j = \mathrm{sat}_{[-128,127]}\left( \left\lfloor \frac{acc_j}{2^s} \right\rfloor \right)
 * \f]
 * - **A)** `MLP_I8_ACT_LINEAR_FIXED_SHIFT`: \f$s\f$ is a fixed user parameter (`fixed_shift`).
 * - **B)** `MLP_I8_ACT_LINEAR_EVOLVED_SHIFT`: \f$s\f$ is stored in the genome (1 byte per layer).
 *
 * @subsection mlp_i8_act_htanh C) Hard-tanh (clamp)
 * A hard-tanh is implemented as "shift + clamp":
 * \f[
 *   y_j = \mathrm{clamp}\!\left(\left\lfloor \frac{acc_j}{2^s}\right\rfloor,\; -c,\; +c\right)
 * \f]
 * with \f$c\f$ typically 127 (full int8 range) and optionally evolved/stored in the genome.
 *
 * @subsection mlp_i8_act_swiglu D/E) Hard SwiGLU (gated linear unit)
 * Hard SwiGLU computes two affine transforms: a "value" branch \f$u\f$ and a "gate" branch \f$v\f$.
 * A fixed-point friendly variant is:
 * \f[
 *   u_j = \mathrm{htanh}\!\left(\left\lfloor \frac{(W_u x + b_u)_j}{2^{s_u}}\right\rfloor\right),
 *   \qquad
 *   g_j = \mathrm{hsig}\!\left(\left\lfloor \frac{(W_g x + b_g)_j}{2^{s_g}}\right\rfloor\right),
 *   \qquad
 *   y_j = \left\lfloor \frac{u_j \cdot g_j}{2^7} \right\rfloor
 * \f]
 * where a convenient hard-sigmoid in int8 is:
 * \f[
 *   \mathrm{hsig}(z) = \mathrm{clamp}\!\left(\left\lfloor \frac{z + 128}{2}\right\rfloor,\; 0,\; 127\right)
 * \f]
 *
 * **Grouped gating:** a layer can use `gate_count = G` gates, each gate controlling a block of output neurons.
 * A typical mapping is \f$g = \lfloor j \cdot G / \mathrm{out\_dim}\rfloor\f$.
 *
 * - **D)** `MLP_I8_ACT_HARD_SWIGLU`: gate parameters \f$(W_g,b_g)\f$ are stored in the genome and optimized.
 * - **E)** `MLP_I8_ACT_HARD_SWIGLU_RANDOM_GATES`: gate parameters are **not** stored; they are generated
 *   deterministically from `gate_seed` (fixed random gates), so the number of optimizable parameters can match
 *   a hard-tanh network while still benefiting from multiplicative gating.
 *
 * @section mlp_i8_layout Genome layout introspection
 * The genome is a single flat byte array. Use `mlp_i8_layer_view()` to retrieve, for each layer, the offsets
 * (indices) of:
 * - main weights and biases,
 * - optional evolved shift/clamp scalars,
 * - optional gate weights/biases (if gates are optimizable).
 *
 * @note This file is an example/benchmark harness. The core MLP implementation lives in
 *       `pogo-utils/MLP_int8_dyn_act.{h,c}`.
 */


#include "pogobase.h"
#include "pogo-utils/tiny_alloc.h"
#include "pogo-utils/MLP_int8_dyn_act.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>


#ifndef HEAP_BYTES
#define HEAP_BYTES 8192
#endif

typedef struct {
    uint8_t g_heap[HEAP_BYTES];
    tiny_alloc_t ta;
} USERDATA;
DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

#ifdef SIMULATOR
#define printf0(fmt, ...) if (pogobot_helper_getid() == 0) { printf(fmt, ##__VA_ARGS__ ); }
#else
#define printf0(fmt, ...) printf(fmt, ##__VA_ARGS__ );
#endif


/* Small-range random int8 in [-16, 15] to avoid immediate saturation */
static int8_t rand_i8_small(void) {
    return (int8_t)((rand() & 31) - 16);
}

static void fill_random_genome(const mlp_i8_dyn_t *net, int8_t *genome) {
    uint32_t n = mlp_i8_genome_bytes(net);
    for (uint32_t i = 0; i < n; ++i) genome[i] = rand_i8_small();

    /* For evolved scalars (shift/clamp), it is nicer to initialize them explicitly. */
    uint8_t total = mlp_i8_num_layers_total(net);
    for (uint8_t layer = 0; layer < total; ++layer) {
        mlp_i8_layer_view_t v;
        if (!mlp_i8_layer_view(net, layer, &v)) continue;
        if (v.shift_off != MLP_I8_OFF_NONE) genome[v.shift_off] = 7;   /* default */
        if (v.clamp_off != MLP_I8_OFF_NONE) genome[v.clamp_off] = 127; /* full range */
    }
}

void user_init(void) {
    srand(1);

    main_loop_hz = 5;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    error_codes_led_idx = 3;

    const uint16_t classes[] = { 24, 128, 512, 2048, 4096 };
    tiny_alloc_init(&mydata->ta, mydata->g_heap, sizeof(mydata->g_heap), classes, 5);

    /* Example dims: 8 -> 32 -> 2 with 2 hidden layers */
    const uint16_t I = 8, H = 32, O = 2;
    const uint8_t  L = 2;

    /* Pick your per-part activations here */
    mlp_i8_act_cfg_t act_in = {
        .type = MLP_I8_ACT_HARD_SWIGLU_RANDOM_GATES, /* E */
        .fixed_shift = 7,
        .shift_is_evolved = false,
        .clamp_fixed = 127,
        .clamp_is_evolved = false,
        .gate_count = 1,
        .gate_seed = 12345u,
    };

    mlp_i8_act_cfg_t act_hidden = {
        .type = MLP_I8_ACT_HARD_TANH, /* C */
        .fixed_shift = 7,
        .shift_is_evolved = false,
        .clamp_fixed = 127,
        .clamp_is_evolved = false,
        .gate_count = 0,
        .gate_seed = 0,
    };

    mlp_i8_act_cfg_t act_out = {
        .type = MLP_I8_ACT_LINEAR_EVOLVED_SHIFT, /* B */
        .fixed_shift = 7,
        .shift_is_evolved = true, /* shift stored in genome */
        .clamp_fixed = 127,
        .clamp_is_evolved = false,
        .gate_count = 0,
        .gate_seed = 0,
    };

    mlp_i8_dyn_t net;

    /* First init with a NULL genome so we can compute how many bytes we need */
    mlp_i8_init(&net, I, H, O, L, act_in, act_hidden, act_out, NULL);
    uint32_t genome_bytes = mlp_i8_genome_bytes(&net);
    uint32_t ws_bytes = mlp_i8_workspace_bytes(&net);
    printf0("genome_bytes=%u  ws_bytes=%u\n", genome_bytes, ws_bytes);

    int8_t *genome = (int8_t*)tiny_malloc(&mydata->ta, genome_bytes);
    int8_t *ws     = (int8_t*)tiny_malloc(&mydata->ta, ws_bytes);

    mlp_i8_init(&net, I, H, O, L, act_in, act_hidden, act_out, genome);
    fill_random_genome(&net, genome);

    /* Demonstrate genome introspection */
    for (uint8_t layer = 0; layer < mlp_i8_num_layers_total(&net); ++layer) {
        mlp_i8_layer_view_t v;
        mlp_i8_layer_view(&net, layer, &v);
        printf0("layer %u: W[%u..%u) b[%u..%u) shift=%s clamp=%s gateW=%s\n",
            (unsigned)layer,
            (unsigned)v.w_off, (unsigned)(v.w_off + v.w_len),
            (unsigned)v.b_off, (unsigned)(v.b_off + v.b_len),
            (v.shift_off == MLP_I8_OFF_NONE) ? "-" : "genome",
            (v.clamp_off == MLP_I8_OFF_NONE) ? "-" : "genome",
            (v.gate_w_off == MLP_I8_OFF_NONE) ? "-" : "genome");
    }

    /* Forward */
    int8_t x[I];
    for (uint16_t i = 0; i < I; ++i) x[i] = rand_i8_small();

    int8_t y[O];
    memset(y, 0, sizeof(y));

    mlp_i8_forward_ws(&net, x, y, ws, ws_bytes);

    printf0("y = [");
    for (uint16_t i = 0; i < O; ++i)
        printf0("%d%s", (int)y[i], (i + 1u == O) ? "" : ", ");
    printf0("]\n");

    tiny_free(&mydata->ta, ws);
    ws = NULL;
    tiny_free(&mydata->ta, genome);
    genome = NULL;
}


void user_step(void) { }


int main(void) {
    pogobot_init();
#ifndef SIMULATOR
    printf("init ok\n");
#endif
    pogobot_start(user_init, user_step);
    return 0;
}

