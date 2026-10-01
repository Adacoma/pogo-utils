# Distributed MNIST score voting

PRANC-encoded classifier ensemble plus IR gossip of output scores.

Entry point: [mnist.c](mnist.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/neural_networks.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/distributed_MLP_int8_MNIST sim POGOUTILS_INCLUDE_DIR=../../src
./examples/distributed_MLP_int8_MNIST/distributed_MLP_int8_MNIST -c conf/test.yaml
make -C examples/distributed_MLP_int8_MNIST bin POGOUTILS_INCLUDE_DIR=../../src
```

The last command only builds firmware; artifact:
`examples/distributed_MLP_int8_MNIST/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

The explicit source path avoids this numerical Makefile's missing-version-header
guard silently skipping the build when its inferred checkout path is wrong.

## Configuration and expected behavior

Defaults input=784,hidden=32,output=10,basis=256; match exporter flags (Python basis default=64), model count/seed/scale and generated assets.

Inspect local and collective predictions/scores. This is distributed inference/voting, not decentralized gradient training.

## Constraints and validation

No calibration needed. Homogeneous message layout and connected peers matter; reconstructed dense weights still occupy runtime RAM.

The [ensemble exporter](train_mnist_qat_to_c.py) writes
[mnist_mlp_pranc_params.c](mnist_mlp_pranc_params.c) using torch/torchvision.
Match `--hidden-dim`, `--num-basis`, `--num-models`, seed and basis scaling with
the C consumer; pass `--num-basis 256` when retaining its current default.
See the [ANN tutorial](../../docs/tutorials/ann.md).

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.
