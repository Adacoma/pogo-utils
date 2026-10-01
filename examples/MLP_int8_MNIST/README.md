# Quantized MNIST inference

Run pre-exported test images through a 784/32/10 Q0.7 classifier.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/systems/neural_networks.md) and [all examples](../../docs/examples.md).

## Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/MLP_int8_MNIST sim POGOUTILS_INCLUDE_DIR=../../src
./examples/MLP_int8_MNIST/MLP_int8_MNIST -c conf/test.yaml
make -C examples/MLP_int8_MNIST bin POGOUTILS_INCLUDE_DIR=../../src
```

The last command only builds firmware; artifact:
`examples/MLP_int8_MNIST/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

The explicit source path avoids this numerical Makefile's missing-version-header
guard silently skipping the build when its inferred checkout path is wrong.

## Configuration and expected behavior

main.c dimensions must match mnist_mlp_params.c. Optional Python QAT exporter needs torch/torchvision and downloads MNIST.

Prints image label, prediction, output scores and a summary on the exported subset, not full test-set accuracy.

## Constraints and validation

Existing generated assets suffice to build. Training/export is optional and overwrites C arrays; no robot calibration required.

The [QAT exporter](train_mnist_qat_to_c.py) needs torch/torchvision and writes
[mnist_mlp_params.c](mnist_mlp_params.c). Its `--hidden-dim`, `--num-images`,
and `--output-c` settings must agree with the consumer. See the
[ANN tutorial](../../docs/tutorials/ann.md) before regenerating assets.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.
