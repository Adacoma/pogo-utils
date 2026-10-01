# Quantized neural runtimes

The neural components are small C inference runtimes, not a unified tensor or
autodiff library. Choose a layout deliberately; dimensions, quantization,
parameter ordering, and generated assets must agree. The
[ANN tutorial](../tutorials/ann.md) starts with an explicit-workspace MLP.

## Runtime selection

| Header | Architecture / parameter ownership | Main costs and example |
| --- | --- | --- |
| [MLP_Q1_15.h](../../src/pogo-utils/MLP_Q1_15.h) | Compile-time input/hidden/output; int16 Q1.15 network object | Twice the parameter bytes of int8; [benchmark](../../examples/MLP_Q1_15/README.md) |
| [MLP_int8.h](../../src/pogo-utils/MLP_int8.h) | Compile-time single-hidden-layer int8 Q0.7 object | Dense weights, hidden scratch; [benchmark](../../examples/MLP_int8/README.md), [MNIST](../../examples/MLP_int8_MNIST/README.md) |
| [MLP_int8_dyn.h](../../src/pogo-utils/MLP_int8_dyn.h) | Runtime dimensions/depth; borrows flat int8 parameters | Two hidden-width stack buffers; [example](../../examples/MLP_int8_dyn/README.md) |
| [MLP_int8_dyn_act.h](../../src/pogo-utils/MLP_int8_dyn_act.h) | Runtime depth, configurable/evolved activations; borrows genome | Explicit workspace recommended; [example](../../examples/MLP_int8_dyn_act/README.md) |
| [MLP_int8_pranc.h](../../src/pogo-utils/MLP_int8_pranc.h) | Seeded basis signature reconstructs MLP parameters | Compact signature does not eliminate dense runtime RAM; [distributed MNIST](../../examples/distributed_MLP_int8_MNIST/README.md) |
| [ESN_int8.h](../../src/pogo-utils/ESN_int8.h) | Sparse fixed-K reservoir, dense readout, persistent int8 state | Reservoir indices/weights plus training workspace; [example](../../examples/ESN_int8/README.md) |
| [ESN_int8_PRANC.h](../../src/pogo-utils/ESN_int8_PRANC.h) | Leaky/gated reservoir, optional PRANC readout | Additional gates/basis/training costs; [example](../../examples/ESN_int8_PRANC/README.md) |
| [TRM_int8.h](../../src/pogo-utils/TRM_int8.h) | Shared trunk recursively updates answer/latent state | Recursion count multiplies inference work; [benchmark](../../examples/TRM_int8/README.md) |

Most fixed architectures are header-only; dynamic-activation functions have a
compiled implementation. Avoid assuming symbols/configuration from one variant
are usable with another.

## Quantization and dense layers

For Q0.7, a stored int8 value q represents q/128, in [-1,127/128].
Products are Q0.14; dot products accumulate in int32 with bias lifted to the
same scale. Conversion rounds/shifts back and saturates to int8. Linear output
still saturates to int8: it is not an unbounded real-valued output. Hard-tanh
on already representable Q0.7 mostly expresses the same clipping boundary.

For I inputs, H hidden units, O outputs, and L>=1 hidden layers, the simple
dynamic layout contains
`H*I + H + (L-1)*(H*H + H) + O*H + O` bytes,
ordered as each row-major W followed by its bias vector.
Initialization borrows parameters; it neither allocates nor copies them.
Keep buffers alive and nonoverlapping for inference.

Worst-case accumulator magnitude grows with fan-in. Validate dimensions and
dot-product ranges independently of final saturation: signed accumulator overflow
cannot be repaired by clipping afterward. Some legacy shifts/conversions also
rely on target C integer behavior; these headers are not a blanket portable
saturating-math guarantee. See [fixed point](fixed_point.md) and the
[historical audit](../code_audit.md).

## Dynamic activations and genome views

Use `mlp_i8_init` to describe topology/activation layout,
`mlp_i8_genome_bytes` to size the genome,
`mlp_i8_workspace_bytes` to size scratch, and `mlp_i8_layer_view` for offsets.
Activations include fixed/evolved linear shifts, hard tanh, hard SwiGLU, and
random-gated hard SwiGLU. Evolved shifts/clamps/gate choices occupy extra genome
fields, so the simple dynamic parameter-count formula is insufficient.

`mlp_i8_forward_ws` is preferred for embedded use. Workspace contains two
hidden buffers plus gating scratch as needed. Allocate once and keep it separate
from input, output, and genome. The convenience `mlp_i8_forward` uses a VLA;
`MLP_I8_NO_VLA` disables that wrapper when compiled consistently.
These are low-level routines: validate a topology before accepting arbitrary
external dimension/genome data.

## PRANC, ESN, and TRM interpretation

PRANC represents weights through a seeded pseudorandom basis and coefficients,
conceptually `W = sum(alpha_k * B_k(seed))`.
The local implementation reconstructs quantized dense weights; seed, generator,
basis scale, dimensions, and coefficient layout must match the exporter.
Compact storage/communication does not automatically mean lower inference RAM
or lower computation. See the original
[PRANC paper](https://arxiv.org/abs/2206.08464) for the method's research context;
the local headers define the actual implementation.

ESNs maintain temporal reservoir state. Sparse recurrence reduces connections
to K per reservoir row; forgetting/resetting state changes the experiment.
Readout ridge training uses floating-point linear algebra/workspaces and is much
more expensive than an int8 step. Random initialization or a nominal spectral
setting is not a proof of the echo-state property after quantization.
The examples separate washout, training, and testing on Mackey–Glass signals.

TRM reuses a trunk while alternating latent/answer updates from a fixed input.
Its x/y/z dimensions and latent/outer step counts are compile-time settings.
The example is a randomized inference benchmark, not evidence of learned
reasoning ability, accuracy, or online training.

## Generated models and deployment

The MNIST scripts use PyTorch/torchvision QAT and export C arrays plus selected
test images. Training is Python-side; the robot runs inference.
The distributed example exchanges/gossips output scores: it is not distributed
backpropagation. Compile-time hidden dimension, basis count, and exported C
metadata must agree; notably the distributed C default basis count differs
from its Python exporter's default.

Benchmark stdout measures the chosen program/toolchain, not an intrinsic
hardware-independent throughput. Simulator timing is host timing. Keep large
models/training arrays off small stacks, budget recurrent state per robot, and
record seed/assets/compiler/config for reproducibility.
