# Extending pogo-utils

Prefer one focused module with explicit ownership and a small example.
Preserve existing behavior/APIs unless a migration is intended.
Do not add platform, heap, POSIX or floating-point requirements casually.

## Add a compiled module

Place `new_system.h` and `new_system.c` in
[src/pogo-utils](../src/pogo-utils/). Use include guards, required standard
includes, C++ extern-C guards for compiled C functions, and a documented public
state/config/status interface. Header comments should state purpose, units,
bounds, ownership, pointer lifetimes, valid call order, failure behavior,
timing/allocation/I/O side effects and concurrency assumptions.

Prefer caller-owned state, explicit init/step and bounded scratch:
no shared mutable simulator globals or hidden acquisition in a consumer.
Keep internal declarations in `*_internal.h`; CMake excludes them from installed
headers. Avoid raw persisted/wire structs, silent truncation and uncontrolled
dimension-dependent stack use.

CMake globs compiled source with CONFIGURE_DEPENDS and installs public headers.
It does not generate a complete imported-target package config merely by
installing a ConfigVersion file. Reconfigure/build/test after adding files and
inspect installation explicitly rather than assuming consumers' include paths.

Header-only code is useful for fixed-size primitives/configurable neural
architectures, but can duplicate code and couples dimension macros/types to
each translation unit. Compiled modules give stable entrypoints and cleaner
separate reader/writer linkage. Measure linked firmware to choose, not source
line count alone.

## Add a heading source or motion behavior

Emit `heading_sample_t` with finite wrapped angle, true original acquisition
time, explicit validity and reference identity.
A cached read does not become new acquisition. Change reference for source,
calibration/chirality/offset changes; document units/handedness/quality.
Acquire once in application code and keep PID/planner sensor-independent.

Bind the source through a small adapter, following the photosensor/magnetometer
headers. Keep startup/readout recovery with the application and preserve invalid
samples so downstream watchdogs advance. Do not infer confidence from finite
angle alone.

For new coordinated motion, extend an explicit command/policy layer with one
motor owner. Avoid calling legacy direct-motor executors alongside kinematics.
Document priority relative to wall avoidance, heading unavailable, STOP and
latched faults. Distinguish requested/applied commands from measured movement.
Add tests for reference/time wrap, held samples, direction sign, recovery and
bounded progress before relying on a new field.

## Add persistence or serialization

Define your payload schema, byte order, lengths, finite/range validation and
checksum policy. Use a free stable ID or exact optional name; there is no reserved
calibration ID. Version payload separately from PFFS catalog format.
Do not persist C padding or pointers.

Use one-page/streaming PFFS APIs depending on size. Fixed size is immutable:
replacement rewrites the extent; resize needs recreation.
Read-only loaders must not silently format damaged metadata.
If creation autoformats, expose/report that destructive behavior.
Design power-failure/error handling knowing replacement/catalog edits/overlapping
defrag are not atomic.

For high-frequency values use buffered logs rather than rewriting an ordinary
file per record. Budget cache/pending bytes, accepted-length backpressure,
checkpoint policy, flash latency and wear. CRC detects accidental damage;
it does not establish authenticity or transactional safety.

## Add neural or optimizer variants

For ANN, specify shape and quantization/layout first, borrow parameters only
with clear lifetime, and expose parameter/workspace sizing/introspection.
Use distinct scratch buffers for successive dense layers. Provide tiny
known-weight fixtures and independent reference comparisons.
Training/export provenance and generated dimensions must match the C consumer.

For optimizers, distinguish candidate/workspace/current/best pointers and one
outstanding evaluation. Define finite-objective and invalid-input behavior,
bounds lifetime, seed semantics and accepted-update count. Validate size arithmetic,
population parameters, buffer overlap assumptions and memory requirements.
Standalone static buffers and facade allocation are separate integration paths.

If extending `opt_algo_t`, update default config, allocation, initialization,
ask/tell/readiness/accessors, destruction and remote hooks together.
Account for actual slot sizes/counts in tiny_alloc; no general malloc fallback.
Do not quietly unify algorithms with incompatible reward/maturity semantics.

For fixed-point primitives document range/rounding/saturation/domain, widen
before arithmetic, avoid negative signed left shifts/overflow, and test extremes.
State table/init or floating-point setup dependencies explicitly.

## Add a swarm protocol

Choose a distinct tag/version and bound payload/capacity.
Validate payload length before reading fields; encode integers and floats
explicitly when portability matters. Reject nonfinite values, invalid block
offsets/lengths and incompatible dimensions. Define repeated/stale epoch behavior,
expiry, ID handling and truncated repositories.

Several existing research examples use packed native structs. Copying that
pattern does not establish a robust interoperable wire schema.
For SSR extensions keep application motility separate from estimator phase logic;
for learning protocols align fitness duration/units/sign across robots.
Use per-robot state and capped work per callback.

## Add an example and tests

Create `examples/<name>/main.c` (or a clearly documented entry point),
a compatible Makefile and README. Copy the relevant **current** Makefile:
older numerical examples and modern storage examples do not share every
local-source rule. Check simulator installed-library linkage versus firmware
source compilation, source exclusions and SDK paths.
Register USERDATA once, initialize callbacks, and use platform startup as the
existing example does. For walls register the relevant wall application.

Provide the minimal scenario under conf only when needed; explain any archive
import/export and destructive commands. List compile switches separately from
YAML parameters, expected diagnostics, and validation limits.

Add deterministic host tests in tests and CMake/CTest.
Use stubs for peripheral boundaries and strict NOR behavior for writes.
Do not place setup/mutations inside assert: NDEBUG removes them.
Test bad inputs, boundary/overflow, ordering, corruption, aliasing, timestamp
wrap, and failure recovery as appropriate. Existing tests do not cover every
system; an example timing print is not a regression test.

## Documentation checklist

1. Update the [README feature map](../README.md) and [compact index](index.md).
2. Add/update a system guide and relevant tutorial; link public headers and example.
3. Update [examples/API coverage](examples.md) for every new public header/example.
4. Give each example build/run/config/output/constraints instructions.
5. Check relative links, headings, actual symbols and self-contained snippets.
6. Build/test without running destructive hardware/simulations as a side effect.
7. Update [current state](current_state.md) with what was validated and still unknown.

Keep mathematical claims separate from implementation choices and experimental
results. Prefer primary references when citing algorithms. Record toolchains,
seeds/assets/configs and measurable acceptance criteria for scientific work.
