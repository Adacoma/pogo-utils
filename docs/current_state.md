# Current project state

Last updated: 2026-09-15.

This file is a concise engineering and scientific handoff. Statements under
"Understood" describe the current implementation, not guarantees established
by hardware or experimental validation.

## Inspected

- Repository layout, tracked-file inventory, build scripts, installation rules,
  version metadata, and recent project history.
- Public headers and representative implementations for sensing, heading
  control, motor calibration, wall avoidance, kinematics, SSR, optimization,
  social learning, fixed-point arithmetic, and neural runtimes.
- A repository-scale static code audit covering correctness, memory safety,
  numerical behavior, embedded resources, protocols, scientific assumptions,
  examples, and build/packaging infrastructure. Findings are recorded in
  `docs/code_audit.md`.
- All example categories and the available Pogosim configuration files, with
  closer inspection of the current kinematics, Vicsek, SSR, and distributed
  MNIST integrations.
- Existing build artifacts and repository cleanliness. No fresh build,
  simulator run, automated test run, or physical-robot experiment has yet been
  performed for this assessment.

## Understood

- The project is an embedded swarm-robotics research toolbox for both Pogobot
  firmware and Pogosim, rather than a standalone application or middleware.
- CMake package metadata and the public version header identify the current
  release as `0.1.0`.
- Modules generally use caller-owned state and explicit `init`, `update`, or
  `step` calls. Examples provide the application lifecycle, communication
  callbacks, experiment policy, and per-robot state.
- The current motion path separates heading acquisition, PID control, wall
  planning, kinematic arbitration, and calibrated motor actuation. Legacy
  wall-avoidance paths with direct motor ownership remain in the tree.
- SSR and Vicsek cover two distinct collective-control directions: distributed
  spectral estimation/classification and local heading alignment.
- Optimization is exposed through standalone strict ask-tell implementations
  and a common facade. Caller-provided workspaces and `tiny_alloc` reduce
  reliance on a platform heap.
- Fixed-point and int8 components target constrained processors, but the whole
  repository is not floating-point-free: calibration, training, several
  optimizers, SSR, and some model setup paths use floating point.
- Examples are currently the main integration documentation and also contain
  most of the available assertions and benchmarks.
- The dynamic int8 MLP workspace alternation and Q16.16 saturating addition,
  subtraction, and absolute value have been corrected locally. The changes
  have received only source-level verification because compilation and
  execution were excluded.
- `tiny_alloc` now checks size/address arithmetic, class ordering and slot-size
  representation, validates exact slot boundaries and allocation state, and
  prevents API-level double-free cycles. Invalid inputs fail closed without a
  public error-reporting channel.
- SEP-CMA-ES now has explicit population limits, checked workspace arithmetic,
  numeric parameter validation, and strict one-ask/one-tell sequencing. Its
  generation update reuses caller workspace rather than consuming a
  dimension-dependent stack allocation.
- In the Vicsek application, wall-avoidance faults reached after successful
  startup now trigger a local state reset instead of permanently latching
  the coordinator in STOP. Calibration and its heading reference are retained;
  the coordinator, avoidance state, and heading median window are reset.

## What remains unknown

- Behavior, timing margins, RAM/stack use, and numerical accuracy on each
  supported physical Pogobot revision.
- Whether a clean checkout builds against the current `pogobot-sdk` and
  `pogosim` revisions on all intended toolchains.
- Empirical robustness of magnetometer calibration, wall recovery, collective
  convergence, and learned controllers across arenas and robot populations.
- Which legacy APIs are intentionally supported and which are retained only for
  old experiments.
- Intended release policy, compatibility guarantees, and distribution terms;
  no repository license is currently visible.

## Currently working analyses

- The repository-level structure, dependency map, and broad static code audit
  are complete.
- The first audit remediation milestone corrected the dynamic MLP buffer alias
  and Q16.16 add/subtract/absolute-value saturation logic.
- The second milestone hardened `tiny_alloc`; its exact boundary checks trade
  constant-time pointer operations for O(number of slots) worst-case walks.
- The SEP-CMA-ES milestone rejects invalid or oversized populations, exposes
  initialization status without changing the legacy initializer signature,
  and makes partial unified-factory allocation failure recoverable.
- A physical-robot report of permanent violet stops is consistent with the
  runtime avoidance-fault latch. The Vicsek application now records the
  cause/count and shows amber while reacquiring a heading; hardware validation
  remains pending.
- Build validation, resource profiling, behavioral regression testing, and
  scientific result reproduction have not begun.

## Current scientific decisions

The following choices are encoded in the current implementation:

- Heading consumers use an explicit sample with timestamp, validity, and
  reference identity so recalibration or source changes can invalidate control
  history safely.
- Current motion code separates sensing and planning from final motor ownership;
  kinematics is the normal arbitration point.
- Swarm methods rely on local communication and per-robot state rather than a
  centralized controller.
- Embedded optimization uses ask-tell interfaces so objective evaluation remains
  under application control.
- Low-precision inference and optional fixed-point heading estimation are used
  where useful, while validation gates or floating-point paths remain where the
  implementation needs them.

These are implementation decisions, not yet documented experimental findings.

## Known data limitations

- No tracked experimental dataset, golden simulator trace, or hardware
  calibration corpus was found.
- Pogosim YAML files define scenarios but do not establish expected outcomes or
  acceptance tolerances.
- Generated neural parameters and test images are tracked for some examples,
  but end-to-end provenance and reproducibility are not standardized.
- Papers and run outputs present in ignored `doc`, `frames`, or `tmp` paths are
  not guaranteed to be available in another checkout.
- Example assertions and timing prints do not constitute a repeatable regression
  suite.

## Next concrete tasks

1. Add targeted tests for the corrected MLP, Q16.16, and `tiny_alloc` normal,
   boundary, overflow, invalid-pointer, and double-free cases, then run them
   when compilation is permitted.
2. Enforce the remaining unified optimizer default/override contract, then
   harden the SSR and distributed-MNIST message protocols.
3. Perform a clean library and example build against pinned SDK and simulator
   revisions; record toolchains, warnings, binary sizes, RAM, and stack use.
4. Define the supported public API surface and document migration from the
   legacy motion and wall-avoidance modules.
5. Add host-side tests for platform-neutral numerics, angle/time wraparound,
   PID state transitions, optimizer invariants, and serialization boundaries.
6. Add deterministic simulator regressions for heading calibration, avoidance,
   Vicsek alignment, and SSR phase/convergence behavior.
7. Clarify licensing, compatibility guarantees, and the intended
   install/package interface.
8. Record experimental hypotheses, metrics, datasets, configurations, and
   acceptance criteria before drawing scientific conclusions from simulations
   or robot runs.
9. Flash the updated Vicsek firmware on representative robots and verify that
   induced heading dropouts and difficult wall escapes recover within the
   intended one-to-three-second interval without permanent violet stops.
