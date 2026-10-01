# Tutorial: choosing and integrating an ANN

Goal: configure a quantized model, allocate correct parameters/workspace, and
run inference without accidental stack growth or buffer aliasing.
The [neural system guide](../systems/neural_networks.md) compares all variants.

## 1. Start with a shape and scaling contract

Specify input/output meaning and units, normalization/clipping, hidden width,
depth, activation, parameter format, and maximum acceptable error/latency.
Q0.7 int8 represents [-1,127/128]; +1 must clip. Outputs are not automatically
probabilities or normalized motor commands.

Choose a fixed-header MLP for one known shape, simple dynamic MLP for variable
depth with ordinary hard-tanh, or dynamic-activation MLP when explicit workspace
or evolved activation fields are needed. ESN needs recurrent state/reset policy;
TRM repeats shared computation. These are different architectures, not flags
on one common model.

## 2. Explicit-workspace dynamic MLP

This self-contained module describes I=2, H=4, O=1, L=1 with hard-tanh,
fixed shift 7 and clamp 127. Parameter bytes = 8+4+4+1 = 17, scratch = 8.
Zero weights make it a deterministic integration fixture, **not a trained model**.

```c
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "pogo-utils/MLP_int8_dyn_act.h"

typedef struct {
    mlp_i8_dyn_t net;
    int8_t genome[17];
    int8_t workspace[8];
} small_ann_t;

bool small_ann_init(small_ann_t *a) {
    memset(a, 0, sizeof(*a));
    mlp_i8_act_cfg_t act = {
        .type = MLP_I8_ACT_HARD_TANH,
        .fixed_shift = 7,
        .clamp_fixed = 127
    };
    mlp_i8_init(&a->net, 2, 4, 1, 1, act, act, act, a->genome);
    return mlp_i8_genome_bytes(&a->net) == sizeof(a->genome) &&
           mlp_i8_workspace_bytes(&a->net) <= sizeof(a->workspace);
}

int8_t small_ann_predict(small_ann_t *a, const int8_t input[2]) {
    int8_t output[1];
    mlp_i8_forward_ws(&a->net, input, output,
                     a->workspace, sizeof(a->workspace));
    return output[0];
}
```

Initialization borrows the genome; keep the whole object alive.
Put it per robot, not on a short-lived initialization stack.
Replace the zero genome with validated parameters in the exact layout.
For gates/evolved shifts/clamps ask the introspection functions for size/offsets;
17 bytes no longer applies. Do not alias genome, input, output or workspace.

With the simple [MLP_int8_dyn](../../src/pogo-utils/MLP_int8_dyn.h), parameter
ownership is similar but forward uses two hidden-width stack buffers.
With fixed [MLP_int8](../../src/pogo-utils/MLP_int8.h), define dimension macros
before including the header; the model object owns matrices.
Keep compile-time macros consistent wherever that type crosses translation units.

## 3. Train/export outside the robot

The [MNIST example](../../examples/MLP_int8_MNIST/README.md) already has generated
assets. Optional regeneration uses PyTorch/torchvision and downloads MNIST:

```sh
cd examples/MLP_int8_MNIST
python3 train_mnist_qat_to_c.py --hidden-dim 32 --output-c mnist_mlp_params.c
```

This trains and overwrites generated C data; it is **not required for a normal
example build**. Record dependencies/seeds/training settings and copy existing
assets before regeneration. QAT normalization/export rounding must match C.

For the [distributed PRANC example](../../examples/distributed_MLP_int8_MNIST/README.md),
pass `--num-basis 256` if retaining its C default; the Python default is 64.
Match hidden dimension, basis count/scale/seed convention, model count, and
exported arrays. Changing dimensions on only one side can compile and still
misread model data. PRANC signature size is not the reconstructed dense RAM size.

## 4. Add temporal models deliberately

For ESN, initialize reservoir/readout, reset state at the beginning of an
independent sequence, perform washout, and evaluate test data separate from
readout training. The Mackey–Glass examples contain floating-point ridge
training and large arrays; don't copy those training paths into a tiny mission
just because inference is int8.

For TRM, choose initial answer/latent state and recursion counts.
More recursive steps increase compute; random weights do not acquire reasoning
capability from recursion alone. Follow each header's dedicated reset/forward API.

## 5. Validate before controlling motors

```sh
make -C examples/MLP_int8_dyn_act sim POGOUTILS_INCLUDE_DIR=../../src
./examples/MLP_int8_dyn_act/MLP_int8_dyn_act -c conf/test.yaml
```

Use a tiny zero/known-weight fixture first, then compare every output to an
independent exporter/reference on boundary and representative inputs.
Check layer offsets, quantization, accumulator range, saturation fraction,
recurrent resets, scratch bounds, and multi-hidden-layer outputs (where hidden
buffer alternation matters). Measure final firmware stack, RAM and execution time.

Interpret benchmark output as a runtime check, not trained-model accuracy.
When connecting outputs to a controller, separately constrain power/heading
commands and handle invalid observations. Log inference input/output/version
without blocking the motor-control deadline.
