# pogo-utils

pogo-utils **0.1.0** is a C toolbox for embedded swarm-robotics experiments on
[Pogobot](https://pogobot.github.io/) and in [Pogosim](https://github.com/Adacoma/pogosim).
It combines heading estimation and motion control, persistent flash files and
logs, quantized neural networks, local and social optimization, and distributed
spectral estimation. It is a library plus applications, not a robot operating
system. Most modules keep state in caller-owned objects and advance through
explicit `init`, `step`, or `update` calls.

Start with [getting started](docs/getting_started.md), the
[architecture](docs/architecture.md), and the [example catalog](docs/examples.md).
The [documentation index](docs/index.md) is the compact map; headers remain the
authority for exact signatures and limits.

## Build and first experiment

Install a compatible Pogosim and its development dependencies first. Firmware
also needs the Pogobot SDK/toolchain. See [prerequisites and path
configuration](docs/getting_started.md#prerequisites). From the repository root:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
# Install to the prefix configured above; /usr/local may require permission.
cmake --install build
make -C examples/MLP_int8 sim POGOUTILS_INCLUDE_DIR=../../src
./examples/MLP_int8/MLP_int8 -c conf/test.yaml
```

The last command starts a simulator; it is not a build or a host test.
`./build.sh` combines library build, installation (using sudo when available),
and all simulator example builds. Use the explicit steps to control those
side effects. For firmware, `make -C examples/<name> bin` produces
`examples/<name>/build/bin/firmware.bin`; the
[hardware instructions](docs/getting_started.md#hardware) explain SDK paths and
uploading. Simulator binaries commonly link the **installed** library, so
reinstall after changing compiled modules.

A magnetometer mission needs calibration stored for **each** robot:

```sh
make -C examples/magnetometer_calibration sim
make -C examples/kinematics sim
./examples/magnetometer_calibration/magnetometer_calibration -c conf/magnetometer_calibration.yaml
./examples/kinematics/kinematics -c conf/magnetometer.yaml
```

Let calibration complete successfully and quit normally to export
`magnetometer.pgflash` before starting the mission. Preserve robot categories
and IDs between configurations. On hardware, run calibration firmware once,
then install a mission on the same robot. The mission loads its model; it does
not collect or fit a new one. Calibration/log creation can automatically format
missing or corrupt PFFS catalogs, losing access to other files: back up first.
See the [controller tutorial](docs/tutorials/controllers.md).

## Features and learning paths

| Feature | Public entry point | Guide / tutorial | Examples |
| --- | --- | --- | --- |
| Light start and photosensor heading | [photostart.h](src/pogo-utils/photostart.h), [heading_detection.h](src/pogo-utils/heading_detection.h) | [Sensing](docs/systems/sensing_and_calibration.md) | [photostart](examples/photostart/README.md), [heading_detection](examples/heading_detection/README.md) |
| Magnetometer calibration and flash-loaded heading | [magnetometer_heading_detection.h](src/pogo-utils/magnetometer_heading_detection.h), [magnetometer_calibration_flash.h](src/pogo-utils/magnetometer_calibration_flash.h) | [Sensing](docs/systems/sensing_and_calibration.md), [controller tutorial](docs/tutorials/controllers.md) | [calibration](examples/magnetometer_calibration/README.md), [live heading](examples/magnetometer_heading_detection/README.md) |
| Timestamped heading PID | [heading_sample.h](src/pogo-utils/heading_sample.h), [heading_PID.h](src/pogo-utils/heading_PID.h) | [PID](docs/systems/heading_pid.md), [controller tutorial](docs/tutorials/controllers.md) | [heading_PID](examples/heading_PID/README.md) |
| Calibrated motors, kinematics, IR wall avoidance | [kinematics.h](src/pogo-utils/kinematics.h), [wall_avoidance_magnetometer.h](src/pogo-utils/wall_avoidance_magnetometer.h) | [Motion](docs/systems/motion_and_avoidance.md), [controller tutorial](docs/tutorials/controllers.md) | [kinematics](examples/kinematics/README.md), [go_straight](examples/go_straight/README.md) |
| Flocking and collective turns | Application-level controllers | [Motion](docs/systems/motion_and_avoidance.md#collective-controllers) | [vicsek](examples/vicsek/README.md), [acu](examples/acu/README.md), [vicsek_u_turns](examples/vicsek_u_turns/README.md) |
| PFFS named flash files | [flash_file.h](src/pogo-utils/flash_file.h) | [PFFS guide](docs/flash_files.md), [tutorial](docs/tutorials/pffs.md) | [shell](examples/flash_file/README.md), [format](examples/flash_file_format/README.md) |
| Buffered print/CSV/binary logs | [flash_log.h](src/pogo-utils/flash_log.h) | [Logs](docs/systems/flash_logs.md), [tutorial](docs/tutorials/flash_logs.md) | [flash_log](examples/flash_log/README.md) |
| Fixed/dynamic MLP, PRANC, ESN, TRM | [MLP_int8.h](src/pogo-utils/MLP_int8.h), [MLP_int8_dyn_act.h](src/pogo-utils/MLP_int8_dyn_act.h) | [Neural networks](docs/systems/neural_networks.md), [ANN tutorial](docs/tutorials/ann.md) | [all neural examples](docs/examples.md#neural-networks) |
| Local optimizers and facade | [optim.h](src/pogo-utils/optim.h), [sep_cmaes.h](src/pogo-utils/sep_cmaes.h) | [Optimization](docs/systems/optimization.md), [tutorial](docs/tutorials/optimization.md) | [optim](examples/optim/README.md), [algorithm examples](docs/examples.md#optimization-and-memory) |
| Social learning and HIT | [social_learning.h](src/pogo-utils/social_learning.h), [hit.h](src/pogo-utils/hit.h) | [Social learning](docs/systems/social_learning.md) | [social_learning](examples/social_learning/README.md), [hit](examples/hit/README.md) |
| Swarm spectral estimation / classification | [ssr.h](src/pogo-utils/ssr.h) | [SSR](docs/systems/ssr.md) | [ssr](examples/ssr/README.md) |
| Fixed-point arithmetic | [fixp.h](src/pogo-utils/fixp.h) | [Fixed point](docs/systems/fixed_point.md), [tutorial](docs/tutorials/fixed_point.md) | [fixp](examples/fixp/README.md) |
| Bounded external-heap allocation | [tiny_alloc.h](src/pogo-utils/tiny_alloc.h) | [Allocator](docs/systems/tiny_alloc.md) | [tiny_alloc](examples/tiny_alloc/README.md) |

## Scope and limitations

The firmware targets constrained processors: code size, RAM, stack, synchronous
flash latency, and control deadlines matter. Quantized inference does not make
the entire library floating-point-free; calibration, optimizers, SSR, and model
training/setup use floating point. Sensor heading is a calibrated local frame,
not a guaranteed geographic compass. IR reception is not a distance or contact
sensor. Recovery policies cannot promise motion through a permanent obstruction.

PFFS v3 is bounded and non-transactional, with no backward-format compatibility.
The fast reader omits CRC validation; secure ordinary-file reads scan the whole
file; logs have their own per-page reader. Read the
[persistence guarantees](docs/flash_files.md) before writing valuable data.

For scientific use, record seeds, parameters, SDK/simulator revisions, sensor
calibration, objective definitions, and outcome metrics. A trajectory video or
successful build is not an experimental validation. The
[current state](docs/current_state.md) distinguishes implementation understanding
from measured evidence, and the [audit](docs/code_audit.md) is a historical
finding list, not a claim that every issue remains unfixed.

## Tests and contributions

[Getting started](docs/getting_started.md#host-tests) describes the four host
tests and their limits. [Troubleshooting](docs/troubleshooting.md) covers
build/version mismatches, calibration, motion, and flash failures.
[Extending the library](docs/extending.md) explains adding modules, heading
sources, examples, tests, serialization, and documentation without introducing
unnecessary firmware dependencies.
