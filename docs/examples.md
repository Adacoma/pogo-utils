# Examples and public API coverage

This catalog covers all **34** example Makefiles and **42** installed public
headers in the current checkout. Each example README explains entry point,
commands, settings, expected diagnostics, and limitations.
Read [getting started](getting_started.md) before building; configurations are
scenarios, not golden expected outcomes. Source macros need recompilation;
YAML keys only affect applications that read them.

## Suggested progression

For robot control: photostart/light or magnetometer calibration -> live heading
-> PID -> kinematics -> go_straight -> collective controllers.
For persistence: PFFS shell -> streaming files -> two independent logs.
For numerical work: fixed/simple MLP or fixed point -> explicit-workspace models
-> standalone optimizers -> facade/social protocols.

The [six tutorials](index.md#tutorials) combine systems without adding another
set of runnable applications. Code snippets are fixtures/integration modules,
not benchmark datasets.

## Sensing and motion

| Example | Entry point | Scenario | Purpose |
| --- | --- | --- | --- |
| [heading_detection](../examples/heading_detection/README.md) | [main.c](../examples/heading_detection/main.c) | [photostart.yaml](../conf/photostart.yaml) | Run-and-tumble motion with photosensor heading and optional photostart normalization. |
| [magnetometer_calibration](../examples/magnetometer_calibration/README.md) | [main.c](../examples/magnetometer_calibration/main.c) | [magnetometer_calibration.yaml](../conf/magnetometer_calibration.yaml) | Collect and fit a planar magnetometer model, then store it in the named PFFS calibration file. |
| [magnetometer_heading_detection](../examples/magnetometer_heading_detection/README.md) | [main.c](../examples/magnetometer_heading_detection/main.c) | [magnetometer.yaml](../conf/magnetometer.yaml) | Load the stored model, warm the live window, and display/log heading with optional run-and-tumble. |
| [photostart](../examples/photostart/README.md) | [main.c](../examples/photostart/main.c) | [photostart.yaml](../conf/photostart.yaml) | Demonstrate dark/bright startup transitions, sensor min/max collection, and normalized light output. |
| [heading_PID](../examples/heading_PID/README.md) | [main.c](../examples/heading_PID/main.c) | [magnetometer.yaml](../conf/magnetometer.yaml) | Hold the initial heading at 0.5 calibrated power using timestamped PID. |
| [kinematics](../examples/kinematics/README.md) | [main.c](../examples/kinematics/main.c) | [magnetometer.yaml](../conf/magnetometer.yaml) | Combine application-owned heading, PID, heading-aware wall planner, and calibrated motor ownership. |
| [go_straight](../examples/go_straight/README.md) | [main.c](../examples/go_straight/main.c) | [magnetometer.yaml](../conf/magnetometer.yaml) | Straight heading tracking with active walls, detailed diagnostics and post-start reset/reacquire recovery. |
| [wall_avoidance](../examples/wall_avoidance/README.md) | [run_and_tumble.c](../examples/wall_avoidance/run_and_tumble.c) | [test.yaml](../conf/test.yaml) | Run-and-tumble application with heading-free IR wall avoidance and direct motor execution. |
| [wall_avoidance_heading](../examples/wall_avoidance_heading/README.md) | [run_and_tumble.c](../examples/wall_avoidance_heading/run_and_tumble.c) | [photostart.yaml](../conf/photostart.yaml) | Run-and-tumble with photosensor-referenced wall targets and direct actuation. |
| [vicsek](../examples/vicsek/README.md) | [main.c](../examples/vicsek/main.c) | [magnetometer.yaml](../conf/magnetometer.yaml) | Local circular heading alignment with flash-loaded heading, PID, walls and runtime recovery. |
| [acu](../examples/acu/README.md) | [main.c](../examples/acu/main.c) | [acu.yaml](../conf/acu.yaml) | Static alignment/crowding/collective-turn law inspired by the external optimized ACU project; no optimizer runs here. |
| [vicsek_u_turns](../examples/vicsek_u_turns/README.md) | [main.c](../examples/vicsek_u_turns/main.c) | [magnetometer.yaml](../conf/magnetometer.yaml) | Retain Vicsek alignment while adding bounded collective turn events and asymmetric head-on backoff. |

## Flash persistence

| Example | Entry point | Scenario | Purpose |
| --- | --- | --- | --- |
| [flash_file](../examples/flash_file/README.md) | [main.c](../examples/flash_file/main.c) | [flash_file.yaml](../conf/flash_file.yaml) | Interactive catalog/file management on UART or simulator stdin. |
| [flash_file_format](../examples/flash_file_format/README.md) | [main.c](../examples/flash_file_format/main.c) | [flash_file.yaml](../conf/flash_file.yaml) | Deliberately recreate empty v3 catalogs without immediately creating a payload file. |
| [flash_log](../examples/flash_log/README.md) | [main.c](../examples/flash_log/main.c) | [magnetometer.yaml](../conf/magnetometer.yaml) | Separate print/CSV streams with bounded RAM caches and regular page service. |

## Neural networks

| Example | Entry point | Scenario | Purpose |
| --- | --- | --- | --- |
| [MLP_Q1_15](../examples/MLP_Q1_15/README.md) | [main.c](../examples/MLP_Q1_15/main.c) | [test.yaml](../conf/test.yaml) | Randomized fixed-shape int16 MLP inference benchmark. |
| [MLP_int8](../examples/MLP_int8/README.md) | [main.c](../examples/MLP_int8/main.c) | [test.yaml](../conf/test.yaml) | Randomized fixed-shape Q0.7 MLP inference benchmark. |
| [MLP_int8_dyn](../examples/MLP_int8_dyn/README.md) | [main.c](../examples/MLP_int8_dyn/main.c) | [test.yaml](../conf/test.yaml) | Flat borrowed parameters and dynamic shape/depth, using two hidden buffers. |
| [MLP_int8_dyn_act](../examples/MLP_int8_dyn_act/README.md) | [main.c](../examples/MLP_int8_dyn_act/main.c) | [test.yaml](../conf/test.yaml) | Dynamic genome introspection, evolved shifts/clamps and optional hard-SwiGLU gates. |
| [MLP_int8_MNIST](../examples/MLP_int8_MNIST/README.md) | [main.c](../examples/MLP_int8_MNIST/main.c) | [test.yaml](../conf/test.yaml) | Run pre-exported test images through a 784/32/10 Q0.7 classifier. |
| [distributed_MLP_int8_MNIST](../examples/distributed_MLP_int8_MNIST/README.md) | [mnist.c](../examples/distributed_MLP_int8_MNIST/mnist.c) | [test.yaml](../conf/test.yaml) | PRANC-encoded classifier ensemble plus IR gossip of output scores. |
| [ESN_int8](../examples/ESN_int8/README.md) | [main.c](../examples/ESN_int8/main.c) | [test.yaml](../conf/test.yaml) | Mackey–Glass washout, floating-point ridge readout training and int8 recurrent prediction. |
| [ESN_int8_PRANC](../examples/ESN_int8_PRANC/README.md) | [main.c](../examples/ESN_int8_PRANC/main.c) | [test.yaml](../conf/test.yaml) | Compare dense and PRANC-compressed readout on Mackey–Glass using a leaky/gated sparse reservoir. |
| [TRM_int8](../examples/TRM_int8/README.md) | [main.c](../examples/TRM_int8/main.c) | [test.yaml](../conf/test.yaml) | Random shared-trunk recursive inference with answer and latent state. |

## Optimization and memory

| Example | Entry point | Scenario | Purpose |
| --- | --- | --- | --- |
| [oneplusone_es](../examples/oneplusone_es/README.md) | [main.c](../examples/oneplusone_es/main.c) | [test.yaml](../conf/test.yaml) | Standalone ES initialization, bounded mutation, initial fitness and candidate evaluation. |
| [spsa](../examples/spsa/README.md) | [main.c](../examples/spsa/main.c) | [test.yaml](../conf/test.yaml) | Standalone simultaneous-perturbation optimization with plus/minus evaluations. |
| [pgpe](../examples/pgpe/README.md) | [main.c](../examples/pgpe/main.c) | [test.yaml](../conf/test.yaml) | Standalone antithetic sampling and adaptation of mean/sigma. |
| [sep_cmaes](../examples/sep_cmaes/README.md) | [main.c](../examples/sep_cmaes/main.c) | [test.yaml](../conf/test.yaml) | Evaluate one candidate at a time and update a diagonal distribution by generation. |
| [optim](../examples/optim/README.md) | [main.c](../examples/optim/main.c) | [test.yaml](../conf/test.yaml) | Switch local/social algorithms through tiny_alloc-backed opt_t on a sphere objective. |
| [social_learning](../examples/social_learning/README.md) | [main.c](../examples/social_learning/main.c) | [test.yaml](../conf/test.yaml) | Episode-style social selection/mutation with an IR neighbor repository. |
| [hit](../examples/hit/README.md) | [main.c](../examples/hit/main.c) | [test.yaml](../conf/test.yaml) | Sliding-window reward maturation, adaptive transfer and blockwise IR observations. |
| [tiny_alloc](../examples/tiny_alloc/README.md) | [main.c](../examples/tiny_alloc/main.c) | [test.yaml](../conf/test.yaml) | Exercise small allocate/free/realloc lifetimes and report arena accounting. |

## Spectral and arithmetic

| Example | Entry point | Scenario | Purpose |
| --- | --- | --- | --- |
| [ssr](../examples/ssr/README.md) | [main.c](../examples/ssr/main.c) | [ssr.yaml](../conf/ssr.yaml) | Distributed diffusion/consensus/decay fitting/classification with application-owned random motility. |
| [fixp](../examples/fixp/README.md) | [bench_fixp.c](../examples/fixp/bench_fixp.c) | [test.yaml](../conf/test.yaml) | Compare selected fixed-point operations with float/double and print correctness/timing diagnostics. |

## Public header coverage

Installed does not mean every header has the same API stability or validation
level. Exact function contracts live in headers; internal headers are excluded
by CMake installation. Version metadata links to architecture because it has
no independent runtime subsystem.

| Header | Guide | Integration example | Responsibility |
| --- | --- | --- | --- |
| [photostart.h](../src/pogo-utils/photostart.h) | [Guide](systems/sensing_and_calibration.md) | [photostart](../examples/photostart/README.md) | Light start and sensor normalization |
| [heading_detection.h](../src/pogo-utils/heading_detection.h) | [Guide](systems/sensing_and_calibration.md) | [heading_detection](../examples/heading_detection/README.md) | Light-gradient heading |
| [magnetometer_heading_detection.h](../src/pogo-utils/magnetometer_heading_detection.h) | [Guide](systems/sensing_and_calibration.md) | [magnetometer_heading_detection](../examples/magnetometer_heading_detection/README.md) | Runtime model/filter and shared calibration types |
| [magnetometer_calibration.h](../src/pogo-utils/magnetometer_calibration.h) | [Guide](systems/sensing_and_calibration.md) | [magnetometer_calibration](../examples/magnetometer_calibration/README.md) | Dedicated collection/fitting/sign routine |
| [magnetometer_calibration_flash.h](../src/pogo-utils/magnetometer_calibration_flash.h) | [Guide](systems/sensing_and_calibration.md) | [magnetometer_calibration](../examples/magnetometer_calibration/README.md) | Named-model persistence |
| [heading_sample.h](../src/pogo-utils/heading_sample.h) | [Guide](systems/heading_pid.md) | [kinematics](../examples/kinematics/README.md) | Sensor-independent timestamp/reference contract |
| [heading_sample_photosensors.h](../src/pogo-utils/heading_sample_photosensors.h) | [Guide](systems/sensing_and_calibration.md) | [heading_PID](../examples/heading_PID/README.md) | Light acquisition adapter |
| [heading_sample_magnetometer.h](../src/pogo-utils/heading_sample_magnetometer.h) | [Guide](systems/sensing_and_calibration.md) | [kinematics](../examples/kinematics/README.md) | Runtime snapshot adapter |
| [heading_PID.h](../src/pogo-utils/heading_PID.h) | [Guide](systems/heading_pid.md) | [heading_PID](../examples/heading_PID/README.md) | Circular PID |
| [calibrated_motors.h](../src/pogo-utils/calibrated_motors.h) | [Guide](systems/motion_and_avoidance.md) | [kinematics](../examples/kinematics/README.md) | Signed/calibrated motor mapping |
| [kinematics.h](../src/pogo-utils/kinematics.h) | [Guide](systems/motion_and_avoidance.md) | [kinematics](../examples/kinematics/README.md) | Single actuation coordinator |
| [wall_avoidance.h](../src/pogo-utils/wall_avoidance.h) | [Guide](systems/motion_and_avoidance.md) | [wall_avoidance](../examples/wall_avoidance/README.md) | Legacy direct face-based avoidance |
| [wall_avoidance_heading.h](../src/pogo-utils/wall_avoidance_heading.h) | [Guide](systems/motion_and_avoidance.md) | [wall_avoidance_heading](../examples/wall_avoidance_heading/README.md) | Legacy light-heading direct avoidance |
| [wall_avoidance_magnetometer.h](../src/pogo-utils/wall_avoidance_magnetometer.h) | [Guide](systems/motion_and_avoidance.md) | [go_straight](../examples/go_straight/README.md) | Sensor-neutral heading-aware planner |
| [flash_file.h](../src/pogo-utils/flash_file.h) | [Guide](flash_files.md) | [flash_file](../examples/flash_file/README.md) | PFFS catalog/read/write/defrag |
| [flash_log.h](../src/pogo-utils/flash_log.h) | [Guide](systems/flash_logs.md) | [flash_log](../examples/flash_log/README.md) | Buffered append streams |
| [MLP_Q1_15.h](../src/pogo-utils/MLP_Q1_15.h) | [Guide](systems/neural_networks.md) | [MLP_Q1_15](../examples/MLP_Q1_15/README.md) | Fixed int16 MLP |
| [MLP_int8.h](../src/pogo-utils/MLP_int8.h) | [Guide](systems/neural_networks.md) | [MLP_int8](../examples/MLP_int8/README.md) | Fixed int8 MLP |
| [MLP_int8_dyn.h](../src/pogo-utils/MLP_int8_dyn.h) | [Guide](systems/neural_networks.md) | [MLP_int8_dyn](../examples/MLP_int8_dyn/README.md) | Flat-parameter dynamic MLP |
| [MLP_int8_dyn_act.h](../src/pogo-utils/MLP_int8_dyn_act.h) | [Guide](systems/neural_networks.md) | [MLP_int8_dyn_act](../examples/MLP_int8_dyn_act/README.md) | Genome/activation/workspace MLP |
| [MLP_int8_pranc.h](../src/pogo-utils/MLP_int8_pranc.h) | [Guide](systems/neural_networks.md) | [distributed_MLP_int8_MNIST](../examples/distributed_MLP_int8_MNIST/README.md) | Seeded basis model compaction |
| [ESN_int8.h](../src/pogo-utils/ESN_int8.h) | [Guide](systems/neural_networks.md) | [ESN_int8](../examples/ESN_int8/README.md) | Sparse reservoir |
| [ESN_int8_PRANC.h](../src/pogo-utils/ESN_int8_PRANC.h) | [Guide](systems/neural_networks.md) | [ESN_int8_PRANC](../examples/ESN_int8_PRANC/README.md) | Leaky/gated reservoir and basis readout |
| [TRM_int8.h](../src/pogo-utils/TRM_int8.h) | [Guide](systems/neural_networks.md) | [TRM_int8](../examples/TRM_int8/README.md) | Recursive shared-trunk inference |
| [oneplusone_es.h](../src/pogo-utils/oneplusone_es.h) | [Guide](systems/optimization.md) | [oneplusone_es](../examples/oneplusone_es/README.md) | Standalone ES |
| [spsa.h](../src/pogo-utils/spsa.h) | [Guide](systems/optimization.md) | [spsa](../examples/spsa/README.md) | Paired stochastic gradient probes |
| [pgpe.h](../src/pogo-utils/pgpe.h) | [Guide](systems/optimization.md) | [pgpe](../examples/pgpe/README.md) | Antithetic distribution learning |
| [sep_cmaes.h](../src/pogo-utils/sep_cmaes.h) | [Guide](systems/optimization.md) | [sep_cmaes](../examples/sep_cmaes/README.md) | Bounded diagonal evolution strategy |
| [optim.h](../src/pogo-utils/optim.h) | [Guide](systems/optimization.md) | [optim](../examples/optim/README.md) | Allocator-backed optimizer facade |
| [social_learning.h](../src/pogo-utils/social_learning.h) | [Guide](systems/social_learning.md) | [social_learning](../examples/social_learning/README.md) | Repository selection/mutation |
| [hit.h](../src/pogo-utils/hit.h) | [Guide](systems/social_learning.md) | [hit](../examples/hit/README.md) | Sliding reward/maturation and transfer |
| [tiny_alloc.h](../src/pogo-utils/tiny_alloc.h) | [Guide](systems/tiny_alloc.md) | [tiny_alloc](../examples/tiny_alloc/README.md) | Bounded caller-owned arena |
| [ssr.h](../src/pogo-utils/ssr.h) | [Guide](systems/ssr.md) | [ssr](../examples/ssr/README.md) | Distributed spectral state machine |
| [ssr_utils.h](../src/pogo-utils/ssr_utils.h) | [Guide](systems/ssr.md) | [ssr](../examples/ssr/README.md) | Behavior names/utilities |
| [ssr_colors.h](../src/pogo-utils/ssr_colors.h) | [Guide](systems/ssr.md) | [ssr](../examples/ssr/README.md) | Classification colors |
| [fixp.h](../src/pogo-utils/fixp.h) | [Guide](systems/fixed_point.md) | [fixp](../examples/fixp/README.md) | All-format umbrella |
| [fixp_base.h](../src/pogo-utils/fixp_base.h) | [Guide](systems/fixed_point.md) | [fixp](../examples/fixp/README.md) | Types/constants/setup/formatting |
| [fixp_q1_15.h](../src/pogo-utils/fixp_q1_15.h) | [Guide](systems/fixed_point.md) | [fixp](../examples/fixp/README.md) | Q1.15 primitives |
| [fixp_q6_10.h](../src/pogo-utils/fixp_q6_10.h) | [Guide](systems/fixed_point.md) | [fixp](../examples/fixp/README.md) | Q6.10 primitives |
| [fixp_q8_24.h](../src/pogo-utils/fixp_q8_24.h) | [Guide](systems/fixed_point.md) | [fixp](../examples/fixp/README.md) | Q8.24 primitives |
| [fixp_q16_16.h](../src/pogo-utils/fixp_q16_16.h) | [Guide](systems/fixed_point.md) | [fixp](../examples/fixp/README.md) | Q16.16 primitives |
| [version.h](../src/pogo-utils/version.h) | [Guide](architecture.md) | [photostart](../examples/photostart/README.md) | Release identification |

## Generated assets and Python tools

[MNIST QAT exporter](../examples/MLP_int8_MNIST/train_mnist_qat_to_c.py) writes
[MNIST arrays](../examples/MLP_int8_MNIST/mnist_mlp_params.c).
[PRANC ensemble exporter](../examples/distributed_MLP_int8_MNIST/train_mnist_qat_to_c.py)
writes [PRANC arrays](../examples/distributed_MLP_int8_MNIST/mnist_mlp_pranc_params.c).
They require torch/torchvision and can download data/overwrite assets; ordinary
builds use the existing exported arrays.
[Fixed-point plotting](../examples/fixp/plot.py) uses pandas/Matplotlib and a
hard-coded dataset, not the latest stdout.
Neither Python training nor plotting is a prerequisite for all library users.

## Validation scope

The host suite covers selected PFFS/log/calibration-storage invariants.
Benchmark/assertion examples provide additional observations but are not a
complete regression suite. Commands here have not all been run as experiments;
hardware latency, motion recovery, numerical accuracy and learning outcomes need
independent validation. See [current state](current_state.md).
