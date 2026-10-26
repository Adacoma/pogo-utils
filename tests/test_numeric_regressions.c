/* Independent integer oracles and small networks exercise signed rounding,
 * negative biases, conversion endpoints, deep workspace reuse and gate layout.
 * Run under UBSan as well as CTest: negative shifts used to be undefined. */
#include <assert.h>
#include <limits.h>
#include <float.h>
#include <string.h>
#include "src/pogo-utils/fixp_q16_16.h"
#include "src/pogo-utils/fixp_q8_24.h"
#include "src/pogo-utils/fixp_q6_10.h"
#include "src/pogo-utils/fixp_q1_15.h"
#define MLP_INT8_INPUT_DIM 2
#define MLP_INT8_HIDDEN_DIM 2
#define MLP_INT8_OUTPUT_DIM 2
#include "src/pogo-utils/MLP_int8.h"
#include "src/pogo-utils/MLP_int8_dyn.h"
#include "src/pogo-utils/MLP_int8_dyn_act.h"
#define MLP_Q1_15_INPUT_DIM 6
#define MLP_Q1_15_HIDDEN_DIM 6
#define MLP_Q1_15_OUTPUT_DIM 1
#include "src/pogo-utils/MLP_Q1_15.h"
#define ESN_INT8_INPUT_DIM 1
#define ESN_INT8_RESERVOIR_DIM 2
#define ESN_INT8_OUTPUT_DIM 1
#define ESN_INT8_RES_FIXED_K 2
#include "src/pogo-utils/ESN_int8.h"
#include "src/pogo-utils/ESN_int8_PRANC.h"
#define TRM_INT8_X_DIM 1
#define TRM_INT8_Y_DIM 1
#define TRM_INT8_Z_DIM 1
#define TRM_INT8_HIDDEN_DIM 1
#include "src/pogo-utils/TRM_int8.h"

static int32_t oracle(int32_t x, unsigned shift) {
    int64_t magnitude = x < 0 ? -(int64_t)x : x;
    int64_t rounded = (magnitude + (INT64_C(1) << (shift - 1u))) /
                       (INT64_C(1) << shift);
    return (int32_t)(x < 0 ? -rounded : rounded);
}

static void rounding(void) {
    const int32_t edges[] = {INT32_MIN, INT32_MAX, -128, -64, -1, 0, 1, 64, 128};
    for (unsigned shift = 1; shift <= 31; ++shift) {
        for (unsigned i = 0; i < sizeof(edges)/sizeof(edges[0]); ++i) {
            assert(pogo_round_shift_i32(edges[i], shift) == oracle(edges[i], shift));
            assert(pogo_round_shift_i64(edges[i], shift) == oracle(edges[i], shift));
        }
    }
    assert(pogo_round_shift_i64(INT64_MIN, 63) == -1);
    assert(pogo_round_shift_i64(INT64_MAX, 63) == 1);
    assert(pogo_round_shift_i32(INT32_MIN, 0) == INT32_MIN);
    for (int32_t x = INT16_MIN; x <= INT16_MAX; ++x) {
        int32_t expected = oracle(x, 7);
        int8_t clipped = expected > 127 ? 127 : expected < -128 ? -128 : (int8_t)expected;
        assert(mlp_int8_from_acc32(x) == clipped);
        assert(mlp_int8_dyn_from_acc32(x) == clipped);
        assert(esn_int8_sparse_from_acc32(x) == clipped);
        assert(esn_int8_lg_from_acc32(x) == clipped);
        assert(trm_int8_from_acc32(x) == clipped);
        assert(q1_15_from_acc32(x) == oracle(x, 15));
    }
}

static void conversions(void) {
    init_fixp(); /* Exercise the negative exponent's integer/fraction split. */
    assert(q16_16_exp(0) == Q16_16_ONE);
    assert(q16_16_exp(-Q16_16_ONE) > 0 && q16_16_exp(-Q16_16_ONE) < Q16_16_ONE);
    assert(q16_16_from_int(-1) == -65536);
    assert(q16_16_from_int(INT_MIN) == INT32_MIN);
    assert(q16_16_from_int(INT_MAX) == INT32_MAX);
    assert(q8_24_from_int(-1) == -16777216);
    assert(Q8_24_FROM_INT(INT_MIN) == INT32_MIN);
    assert(Q8_24_FROM_INT(INT_MAX) == INT32_MAX);
    assert(q1_15_from_int(-1) == INT16_MIN);
    assert(q6_10_from_int(INT_MIN) == INT16_MIN);
    assert(q6_10_from_int(INT_MAX) == INT16_MAX);
    assert(q8_24_from_q16_16(-1) == -256);
    assert(q8_24_from_q16_16(INT32_MAX) == INT32_MAX);
    assert(q8_24_from_q16_16(INT32_MIN) == INT32_MIN);
    assert(q16_16_from_q1_15(INT16_MIN) == -65536);
    assert(q8_24_from_q1_15(INT16_MIN) == -16777216);
    assert(q8_24_from_q6_10(-1) == -16384);
    assert(q16_16_from_q6_10(-1) == -64);
    assert(q1_15_from_q6_10(INT16_MIN) == INT16_MIN);
    assert(q1_15_div(-16384, 16384) == INT16_MIN);
    assert(q6_10_div(-512, 1024) == -512);
    assert(q1_15_mul(-16384, 16384) == -8192);
    /* Macro conversions remain arithmetic constant expressions in C. */
    static const q1_15_t half = Q1_15_FROM_FLOAT(0.5f);
    static const q6_10_t one = Q6_10_FROM_FLOAT(1.0f);
    assert(half == 16384 && one == 1024);
    assert(q1_15_from_float(0.99999f) == INT16_MAX);
    assert(q6_10_from_float(31.9999f) == INT16_MAX);
    assert(q1_15_from_float(nextafterf(1.0f, 0.0f)) == INT16_MAX);
    assert(q6_10_from_float(nextafterf(32.0f, 0.0f)) == INT16_MAX);
    assert(q1_15_from_float(-1.0f) == INT16_MIN);
    assert(q6_10_from_float(-32.0f) == INT16_MIN);
    assert(q1_15_from_float(NAN) == 0 && q6_10_from_float(NAN) == 0);
    assert(q8_24_from_float(NAN) == 0 && q16_16_from_float(NAN) == 0);
    assert(q1_15_from_float(INFINITY) == INT16_MAX);
    assert(q1_15_from_float(-INFINITY) == INT16_MIN);
    assert(q6_10_from_float(INFINITY) == INT16_MAX);
    assert(q6_10_from_float(-INFINITY) == INT16_MIN);
    assert(q8_24_from_float(INFINITY) == INT32_MAX);
    assert(q8_24_from_float(-INFINITY) == INT32_MIN);
    assert(q16_16_from_float(INFINITY) == INT32_MAX);
    assert(q16_16_from_float(-INFINITY) == INT32_MIN);
    assert(q8_24_from_float(FLT_MAX) == INT32_MAX);
    assert(q16_16_from_float(-FLT_MAX) == INT32_MIN);
    assert(q8_24_from_float(nextafterf(128.0f, 0.0f)) == INT32_MAX - 127);
    assert(q16_16_from_float(nextafterf(32768.0f, 0.0f)) == INT32_MAX - 127);
    /* Every int16 code is exactly representable as a float in either format. */
    for (int32_t raw = INT16_MIN; raw <= INT16_MAX; ++raw) {
        assert(q1_15_from_float((float)raw / 32768.0f) == raw);
        assert(q6_10_from_float((float)raw / 1024.0f) == raw);
    }
    assert(q1_15_mac32(INT32_MAX, 1, 1) == INT32_MAX);
    assert(q1_15_mac32(INT32_MIN, -1, 1) == INT32_MIN);
}

static void networks(void) {
    MLP_INT8 mlp = {0};
    int8_t input[2] = {0}, output[2];
    mlp.b1[0] = -1; mlp.b2[0] = -1;
    mlp_int8_forward(&mlp, input, output);
    assert(output[0] == -1 && output[1] == 0);
    MLP_Q1_15 qmlp = {0}; qmlp.b1[0] = -1; qmlp.b2[0] = -1;
    int16_t qi[6] = {0}, qo[1];
    mlp_q1_15_forward(&qmlp, qi, qo);
    assert(qo[0] == -1);
    /* Large partial sums must cancel before quantization, rather than being
     * clipped per term. The exact Q2.30 sum is 3*32768 -> output code 3. */
    memset(&qmlp, 0, sizeof(qmlp));
    for (int i = 0; i < 6; ++i) {
        qi[i] = INT16_MIN;
        qmlp.W1[0][i] = i < 3 ? INT16_MIN : INT16_MAX;
    }
    qmlp.W2[0][0] = INT16_MAX;
    mlp_q1_15_forward(&qmlp, qi, qo);
    assert(qo[0] == 3);
    int16_t weights[6], bias[1] = {0};
    for (int i = 0; i < 6; ++i) weights[i] = INT16_MIN;
    dense_hidden_hard_tanh(weights, bias, 1, 6, qi, qo);
    assert(qo[0] == INT16_MAX);
    for (int i = 0; i < 6; ++i) weights[i] = INT16_MAX;
    dense_output_layer(weights, bias, 1, 6, qi, qo);
    assert(qo[0] == INT16_MIN);
    for (int i = 0; i < 6; ++i) weights[i] = i < 3 ? INT16_MIN : INT16_MAX;
    dense_output_layer(weights, bias, 1, 6, qi, qo);
    assert(qo[0] == 3);
    ESN_INT8_SPARSE esn;
    esn_int8_sparse_init_zero(&esn);
    esn_int8_sparse_init_random_reservoir(&esn); /* K=R must terminate uniquely. */
    for (int i = 0; i < 2; ++i) assert(esn.Wres_idx[i][0] != esn.Wres_idx[i][1]);
    ESN_INT8_PRANC lg;
    esn_int8_lg_init_zero(&lg);
    esn_int8_lg_init_random_reservoir(&lg);
    for (int i = 0; i < 2; ++i) assert(lg.Wres_idx[i][0] != lg.Wres_idx[i][1]);

    /* Zero weights isolate exact negative biases at every dynamic layer. */
    mlp_i8_act_cfg_t a = {0}; a.type = MLP_I8_ACT_HARD_TANH;
    mlp_i8_dyn_t net;
    int8_t genome[256] = {0}, ws[16];
    mlp_i8_init(&net, 2, 2, 2, 3, a, a, a, genome);
    for (uint8_t layer = 0; layer < 4; ++layer) {
        mlp_i8_layer_view_t v;
        assert(mlp_i8_layer_view(&net, layer, &v));
        genome[v.b_off] = -1;
    }
    mlp_i8_forward_ws(&net, input, output, ws, sizeof(ws));
    assert(output[0] == -1 && output[1] == 0);
    /* Four permutation layers must preserve the input. An aliased hidden
     * destination instead overwrites the second neuron's source value. */
    memset(genome, 0, sizeof(genome));
    for (uint8_t layer = 0; layer < 4; ++layer) {
        mlp_i8_layer_view_t permutation;
        assert(mlp_i8_layer_view(&net, layer, &permutation));
        genome[permutation.w_off + 1] = 127;
        genome[permutation.w_off + 2] = 127;
    }
    input[0] = 10; input[1] = -20;
    mlp_i8_forward_ws(&net, input, output, ws, sizeof(ws));
    assert(output[0] == 10 && output[1] == -20);
    /* Invalid fixed shifts must leave both buffers untouched. */
    net.act_in.fixed_shift = 32;
    memset(ws, 42, sizeof(ws)); memset(output, 42, sizeof(output));
    mlp_i8_forward_ws(&net, input, output, ws, sizeof(ws));
    for (unsigned i = 0; i < sizeof(ws); ++i) assert(ws[i] == 42);
    assert(output[0] == 42 && output[1] == 42);

    /* Three gates really occupy three rows; a later narrower layer clamps it. */
    a.type = MLP_I8_ACT_HARD_SWIGLU; a.gate_count = 3;
    mlp_i8_init(&net, 2, 4, 2, 1, a, a, a, genome);
    mlp_i8_layer_view_t v;
    assert(mlp_i8_layer_view(&net, 0, &v));
    assert(v.gate_w_len == 6 && v.gate_b_len == 3);
    assert(mlp_i8_layer_view(&net, 1, &v));
    assert(v.gate_w_len == 8 && v.gate_b_len == 2);
    assert(mlp_i8_workspace_bytes(&net) == 11);
    mlp_i8_forward_ws(&net, input, output, ws, sizeof(ws));

    /* Independent gates must produce different outputs; negative SwiGLU
     * products must round symmetrically too. */
    mlp_i8_act_cfg_t linear = {0};
    mlp_i8_init(&net, 1, 3, 3, 1, a, a, linear, genome);
    memset(genome, 0, sizeof(genome));
    assert(mlp_i8_layer_view(&net, 0, &v));
    genome[v.gate_b_off] = -127;
    genome[v.gate_b_off + 2] = 127;
    for (int i = 0; i < 3; ++i) genome[v.b_off + i] = -64;
    assert(mlp_i8_layer_view(&net, 1, &v));
    for (int i = 0; i < 3; ++i) genome[v.w_off + 4*i] = 127;
    int8_t gated[3];
    mlp_i8_forward_ws(&net, input, gated, ws, sizeof(ws));
    assert(gated[0] == 0 && gated[1] == -32 && gated[2] == -64);
}

int main(void) { rounding(); conversions(); networks(); return 0; }
