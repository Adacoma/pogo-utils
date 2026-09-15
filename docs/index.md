# Project documentation

`pogo-utils` is a C11 utility library for Pogobot firmware and Pogosim
experiments. It provides robot sensing and motion components, decentralized
swarm algorithms, embedded optimization methods, fixed-point arithmetic, and
small neural-network runtimes. Applications select and compose these pieces;
the repository does not provide a single central runtime.

## Start here

- [README](../README.md) — prerequisites, build instructions, and the minimal
  usage example.
- [Current project state](current_state.md) — inspected areas, current
  understanding, open questions, scientific decisions, data limitations, and
  next tasks.
- [Static code audit](code_audit.md) — prioritized correctness, safety,
  scientific-validity, resource, and build-system findings from the 2026-09-15
  source review.
- [Public library sources](../src/pogo-utils) — headers are the canonical API
  reference.
- [Examples](../examples) — firmware and simulator entry points showing how
  the modules are composed.
- [Simulator configurations](../conf) — example Pogosim scenarios.

## Architecture map

### Sensing and motion

- [`heading_sample.h`](../src/pogo-utils/heading_sample.h) defines the common,
  sensor-independent heading sample contract.
- [`photostart`](../src/pogo-utils/photostart.h) and
  [`heading_detection`](../src/pogo-utils/heading_detection.h) implement the
  photosensor start and heading path.
- [`magnetometer_heading_detection`](../src/pogo-utils/magnetometer_heading_detection.h)
  implements filtered heading from an already-fitted model.
- [`magnetometer_calibration`](../src/pogo-utils/magnetometer_calibration.h) contains
  the separately linked collector/fitter; its
  [`flash API`](../src/pogo-utils/magnetometer_calibration_flash.h) stores and
  loads versioned, checksummed models. See the
  [`dedicated example`](../examples/magnetometer_calibration).
- [`heading_PID`](../src/pogo-utils/heading_PID.h) performs heading control
  without owning sensor, clock, or motor I/O.
- [`wall_avoidance_magnetometer`](../src/pogo-utils/wall_avoidance_magnetometer.h)
  is the current sensor-independent wall-avoidance planner despite its
  historical filename. Earlier wall-avoidance APIs remain for compatibility
  and experiments.
- [`kinematics`](../src/pogo-utils/kinematics.h) arbitrates motion and avoidance;
  [`calibrated_motors`](../src/pogo-utils/calibrated_motors.h) maps the result to
  calibrated wheel commands.

The [`kinematics`](../examples/kinematics) and
[`vicsek`](../examples/vicsek) examples are the main integration references.

### Swarm and distributed algorithms

- [`ssr`](../src/pogo-utils/ssr.h) implements Spectral Swarm Robotics diffusion,
  consensus, and spectral estimation. See the [`ssr` example](../examples/ssr).
- The [`vicsek` example](../examples/vicsek) combines neighbor alignment with
  the current sensing, PID, kinematics, and avoidance stack.
- [`distributed_MLP_int8_MNIST`](../examples/distributed_MLP_int8_MNIST) combines
  compressed local classifiers through vector push-sum consensus.

### Optimization and social learning

- Standalone ask-tell optimizers include `(1+1)-ES`, SPSA, PGPE, and separable
  CMA-ES.
- [`social_learning`](../src/pogo-utils/social_learning.h) and
  [`hit`](../src/pogo-utils/hit.h) support genome exchange between robots.
- [`optim`](../src/pogo-utils/optim.h) provides the common optimizer facade.
- [`tiny_alloc`](../src/pogo-utils/tiny_alloc.h) supplies caller-owned workspace
  allocation without assuming a conventional system heap.

Corresponding runnable examples are under [`examples`](../examples).

### Fixed-point and neural runtimes

- [`fixp_base.h`](../src/pogo-utils/fixp_base.h) and the `fixp_q*` headers provide
  saturating fixed-point arithmetic and activation functions.
- `MLP_*`, `ESN_*`, and [`TRM_int8.h`](../src/pogo-utils/TRM_int8.h) provide small
  fixed-shape or dynamic inference runtimes. Several are header-only and are
  instantiated by their applications.
- Python scripts in the relevant example directories train or export model
  parameters as generated C data; they are development tools, not a Python
  package.

## Build and execution

The root [`CMakeLists.txt`](../CMakeLists.txt) builds the reusable library.
[`build.sh`](../build.sh) configures and builds it, installs it when permitted,
and then invokes the example Makefiles. Firmware and simulation builds depend
on the external `pogobot-sdk` and `pogosim` repositories.

Pogobot applications normally allocate per-robot state, register initialization
and step callbacks with `pogobot_start`, and provide message callbacks when
communication is needed. `REAL_ROBOT` and `SIMULATOR` select platform-specific
paths in examples.
