# Swarm spectral estimation and classification

[ssr.h](../../src/pogo-utils/ssr.h) owns distributed diffusion, consensus,
decay fitting, and classification state. [ssr_utils.h](../../src/pogo-utils/ssr_utils.h)
provides behavior names/utilities and
[ssr_colors.h](../../src/pogo-utils/ssr_colors.h) supplies color support.
The [SSR example](../../examples/ssr/README.md) combines this system with
simple application-owned random motility, platform callbacks, and simulator
exports. Its current motility does not use the heading coordinator or require
magnetometer calibration.

## Model and interpretation

Neighbor diffusion approximates a graph process: conceptually
`s_next = s - tau*L*s`, where L is the interaction-graph Laplacian.
The selected kernel/normalization changes the operator; do not compare lambda
across kernels as if units and spectra were identical.
After appropriate centering/consensus, fitting a decaying signal uses the
relationship `log(abs(s)) approximately intercept - lambda*t`.
The module tracks multiple diffusion channels, fit quality, and an aggregate
estimate, then applies configured classification thresholds.

Finite windows, small amplitudes, noise, changing graph topology, and incomplete
consensus affect the estimate. A fitted decay is not automatically a unique
geometric signature or a statistically validated classifier. Thresholds need
calibration for the particular scenario and operator.

## Lifecycle and callbacks

Call `ssr_config_init_default`, adjust validated experiment settings, and
`ssr_init(state,&config)`. Enable/disable or explicitly trigger startup as
needed. Deliver incoming messages through `ssr_process_message`; offer
outgoing work through `ssr_send_message`; call `ssr_step` regularly.

Use behavior/phase accessors and `ssr_step_result_t` to integrate the
application. `ssr_is_motility_phase` and `ssr_is_measurement_phase` indicate
policy phases; they do not move motors. The application must keep measurement
motion consistent with the experimental protocol. `ssr_reset` starts fresh
state; it does not erase external flash or recalibrate headings.

Diagnostics include neighbor counts (buffered versus last update), behavior
elapsed time, iteration, diffusion validity, result readiness, per-channel s,
lambda/average lambda, diffusion time/tau, best MSE, and fitted point count.
Read readiness/validity before interpreting a numeric result.

## Bounds and resources

Default compile-time capacities are 20 neighbors, 3 diffusion channels,
15 fit-window samples, and 10 classes. Changing those macros affects structure
sizes and protocol consistency; rebuild all corresponding library/application
objects. State, neighbor arrays, and fit buffers belong per robot, not in one
shared simulator global.

The implementation uses floating-point math and serialized application-level
messages. Several protocol paths rely on native float/struct conventions, so
heterogeneous architectures/untrusted packets need a separately specified
wire format and validation. The library is not a generic robust transport layer.

Measure processing/fitting/logging time against the control loop. Faster
diffusion sampling is not necessarily better if packet reception cannot supply
fresh neighborhood data. Capacity truncation and packet loss change the graph
being estimated.

## Example and validation

`conf/ssr.yaml` and the SSR example expose phase schedules, diffusion settings,
classification, logging, and exports. Source switches include always-moving
versus phase-controlled motility and log decimation. Defaults are scenario
choices, not a universal classification model.

First verify stable neighbor exchange and phase transitions, then fit validity
and MSE, then repeat across seeds/graphs and independent labels.
Record robot count/density, arena, communication model, movement schedule,
kernel/tau, thresholds, fit windows, and valid/invalid results.
Do not infer shape-recognition accuracy from a single color pattern.
See [extending](../extending.md) for protocol/state changes.
