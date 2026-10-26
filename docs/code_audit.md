# Static code audit

Audit date: 2026-09-15.

This audit covers the tracked, in-repository C, header, Python, example, and
build files. Linked repositories and nested Git content were excluded. No
Pogosim simulation, compilation, recompilation, executable, or physical-robot
experiment was run. A source-only static analysis pass was used to corroborate
selected manual findings.

The findings distinguish confirmed implementation defects from configuration,
hardware, and scientific-validity risks. The latter need targeted validation
before they should be treated as behavioral defects.

## Critical findings

The sections below preserve the original audit. The
[2026-10-26 follow-up](#2026-10-26-follow-up) records the later fixes and their
validation; it does not mark every historical risk as resolved.

### Dynamic int8 MLP corrupts multi-hidden-layer inference

Status: fixed locally on 2026-09-15; not compiled or executed as part of this
audit.

The ping-pong update in `MLP_int8_dyn_act.c` chooses the next buffer from the
previous value of `cur`. After the first hidden layer, this makes `cur` and
`nxt` point to the same workspace buffer. A network with two or more hidden
layers consequently overwrites activations while they are still being read.

The implementation also accepts an unchecked fixed shift. A shift of 32 or
more invokes undefined behavior. Its convenience forward wrapper uses a
dimension-dependent VLA on the embedded stack.

### Q16.16 saturating addition and subtraction are incorrect

Status: fixed locally on 2026-09-15; not compiled or executed as part of this
audit.

`q16_16_add()` and `q16_16_sub()` perform a signed overflow before attempting
to detect it. Signed overflow is undefined in C. On ordinary two's-complement
wrapping, the saturation direction is also reversed: positive overflow can
return `Q16_16_MIN`, while negative overflow can return `Q16_16_MAX`.

The branch-free Q16.16 absolute-value routine also overflows for `INT32_MIN`.

### Tiny allocator can underallocate or corrupt free lists

Status: fixed locally on 2026-09-15; not compiled or executed as part of this
audit. The hardened implementation fails closed on invalid layouts, validates
exact slot boundaries and allocation state, rejects `calloc` multiplication
overflow, and counts physical free slots without following free-list links.

`tiny_alloc.c` has several memory-safety hazards:

- Large aligned slot sizes are truncated to 65,535 bytes and can become
  smaller than the advertised payload.
- `count * size` is unchecked in `tiny_calloc()`.
- `tiny_free()` does not validate slot boundaries or allocation state, so
  interior pointers and double frees can corrupt the free lists.
- A double free can form a cyclic list, leading to duplicate live allocations
  or an infinite free-list traversal.
- Class ordering is documented as ascending but is not validated. An unsorted
  class list can return a block smaller than the request.

## High-priority findings

### Optimizer configuration contract is not implemented

`optim.h` states that `use_defaults` applies algorithm defaults before user
overrides. `opt_create()` copies a supplied configuration verbatim and never
reads that flag. Partially initialized configurations can therefore pass zero
or invalid parameters to optimizer backends.

An invalid optimizer enum is initialized through the ES1P1 switch default but
remains stored as the invalid value. Later dispatch and destruction no longer
match the initialized object.

The optimizer example's allocator class array contains a 4,096-byte class but
passes a class count of five, excluding it. When the example is configured for
SEP-CMA-ES, creation can fail on its larger workspace request, after which the
example continues and dereferences the null optimizer.

### SEP-CMA-ES lacks robust parameter and resource bounds

Status: fixed locally on 2026-09-15; not compiled or executed as part of this
audit. Population sizes, workspace-size arithmetic, numeric parameters,
bounds, and weights are now validated. The fixed cache limits are explicit,
ask--tell ordering is enforced, and generation recombination reuses a caller
workspace instead of allocating dimension-dependent stack memory.

The direct SEP-CMA-ES API does not safely handle `lambda == 0`. This produces
invalid weight calculations and later writes `fit_buf[0]`. Values of `mu`
above 32 are only partially supported: some operations use the configured
value while ranking and weights silently use 32.

Generation updates always reserve a 256-float stack buffer and use `alloca()`
for larger dimensions. Parameter validation is otherwise weak, and the API
does not enforce one outstanding ask before each tell.

### SSR messages can alter the global application clock

When synchronization is enabled, an accepted SSR packet sufficiently ahead of
local time directly replaces `_current_time_milliseconds` and resets the
global timer. There is no maximum jump, epoch, protocol version, checksum, or
trusted-source mechanism. A malformed, stale, or colliding packet can skip
phases and disturb unrelated application timing.

The packed SSR wire structure has no magic or version field, assumes native
float representation and endianness, and places its first float at byte offset
10 despite claiming float alignment. Custom phase-duration sums can also wrap
`uint32_t`.

### SSR Metropolis diffusion may not contract as intended

The Metropolis kernel uses `1 / max(degree_i, degree_j)`. Standard lazy
Metropolis consensus normally includes an additional self-weight term in the
denominator. For two degree-one nodes and `tau = 1`, the current update swaps
values instead of contracting them. This is a strong mathematical concern;
the intended SSR formulation must be confirmed before changing it.

Invalid diffusion values are stored as NaN. In the optional color-from-`s`
mode, that NaN can reach a float-to-`uint8_t` conversion, which is undefined.

### Distributed MNIST voting has timing and consensus defects

The nominal 50 Hz gossip period is calculated as `500 / 50 = 10 ms`, producing
100 Hz. The push-sum state is halved before invoking best-effort IR
transmission, so failed or lost messages permanently destroy consensus mass.

Messages contain no protocol identifier or image/round epoch. Delayed packets
from a previous image can contaminate the next vote after robots reset their
local state. Received floats are accepted without finite-value validation.

### Legacy wall avoidance has timing and motor-safety risks

The older wall-avoidance implementations compare absolute deadlines directly
and fail around the 32-bit clock wrap. The heading variant's angle-wrapping
loops do not reject infinity, allowing an infinite loop. Legacy motor paths
can command nonzero power before changing direction, potentially reversing a
loaded motor.

The current magnetometer-based controller is substantially safer. Its forward
commit state nevertheless suppresses front-beacon replanning for the configured
duration. Without a bumper or ranging sensor, this remains a physical
experiment risk rather than a hidden implementation defect.

### Photosensor APIs and implementations disagree

Documented photostart defaults do not match the implementation. EWMA is
documented as disabled but enabled in code, normalization is documented as
clamped but is not clamped, and timing and threshold defaults differ.

`PHOTOSTART_NSENS` appears configurable, but the implementation hardcodes
three sensor indices. NaN EWMA coefficients and additive-threshold overflow
are not rejected. In heading detection, normalized floating-point readings
pass through an `int16_t` helper and lose fractional precision. Flat or
degenerate sensor geometry returns a valid-looking zero heading without a
confidence indicator.

## Additional engineering risks

- Fixed-point headers contain several left shifts of negative signed values,
  signed-overflow cases, and implementation-defined right shifts. Float-to-
  fixed conversions do not consistently reject NaN.
- Fixed-point formatting uses mismatched `printf` argument types and does not
  correctly parse standard length modifiers such as `%ld`.
- ESN reservoir initialization can loop forever if requested connectivity
  exceeds reservoir dimension. Neural headers lack several useful compile-time
  dimension and accumulator-bound checks.
- PRANC MLP expansion can require a static float buffer of roughly 100 KiB for
  a 32 by 784 layer, before the rest of the model and application state.
- Optimizers and neural initializers share libc `rand()`, coupling
  reproducibility and reentrancy to global call order.
- HIT documents `eval_T = 200`, defaults to 5, and has a compiled maximum of
  32. Its block-transfer documentation specifies `round(alpha*n)`, whereas the
  implementation uses `round(alpha*len)`. HIT also uses dimension-dependent
  VLAs and does not use sender/epoch fields to reject stale exchanges.
- The photostart example uses `POGO_UTILS_VERSION_STR`, while the repository
  defines only `POGO_UTILS_VERSION`.
- CMake installs a package version file but no package configuration or
  exported target. It also does not formally declare SDK dependencies used by
  some library sources.
- `build.sh` automatically invokes `sudo make install`, lacks fail-fast
  handling, and assumes `nproc`.
- Most example Makefiles are duplicated legacy variants, creating
  configuration drift and fragile object-name collision risks.
- No automated unit/regression suite or CI configuration covers arithmetic,
  allocation, protocol parsing, or optimizer state machines.

## Stronger areas

The current magnetometer wall-avoidance stack, calibrated motor helper,
heading PID, kinematics, and Vicsek example are more defensive. They generally
use finite-value checks, wrap-safe timing, bounded state, local random
generators, explicit packet framing, and stop-before-direction-change behavior.

The magnetometer fitting code is careful about arithmetic, but its normal-
equation conic fit is not robust by design. Acceptance lacks a residual and
coverage quality criterion. Its synchronous execution cost and several-
kilobyte state require measurement on target hardware.

## Recommended remediation order

1. Verify the locally fixed dynamic MLP buffer alias and Q16.16 saturation with
   targeted host-side tests when compilation is permitted.
2. Verify the locally hardened `tiny_alloc` with targeted overflow, invalid
   layout, boundary, double-free, exhaustion, and reallocation tests when
   compilation is permitted.
3. Enforce the remaining unified optimizer configuration/default contract and
   validate SEP-CMA-ES changes with targeted tests.
4. Version and validate network messages; bound or remove SSR clock correction.
5. Correct distributed MNIST timing, epoch handling, and mass-loss behavior.
6. Replace remaining undefined fixed-point operations and unbounded VLAs.
7. Add host-side unit tests for arithmetic, allocation, optimizers, and
   protocol parsing.
8. Validate SSR convergence, magnetometer calibration quality, and wall
   behavior using recorded datasets and supervised hardware trials.

## 2026-10-26 follow-up

The latest review's confirmed defects are addressed in the source:

- The optimizer example enables all six arena classes and stops initialization
  and callbacks after creation failure. Every backend records partial buffer
  ownership before checking allocation success, so failure cleanup frees it.
- The factory rejects invalid dispatch enums and invalid bounds. Complete
  default configurations remain editable, including zero/false overrides;
  `use_defaults=1` now merges nonzero fields of a partial configuration.
- Quantized ANN and Q1.15 rounding uses unsigned magnitudes and ties away from
  zero. Negative exact multiples remain exact, and accumulator endpoints do
  not overflow during rounding. Signed negative scaling uses multiplication;
  affected range-reducing conversions and integer conversions saturate.
- Dynamic MLP inference rejects fixed shifts above 31 before writing buffers.
  SwiGLU gate counts survive default normalization and are bounded per layer.
  Existing multi-gate genome layouts and numerical baselines need revalidation.
- Both sparse ESN variants reject impossible connectivity or reservoirs larger
  than their byte-sized indices can represent at compile time.
- HIT and SL reject nonfinite remote fitness/genomes before state changes.
  HIT block-length clamping avoids signed addition overflow.
- SSR packets carry a protocol tag. Synchronization uses its own offset,
  bounded to 1,000 ms, without changing the robot clock. Neighbor expiry keeps
  raw time. Default Metropolis weights retain a self weight and tau is capped
  at one, eliminating the degree-one two-node swap.
- Distributed MNIST uses epoch/image-tagged, unique-origin vote relaying.
  Broadcasts/retries cannot multiply a vote, failed sends remove no votes,
  and the nominal 50 Hz period is 20 ms. The cache defaults to 64 origins;
  incomplete delivery and capacity limits can still yield different tallies.
- Trimmed UART downloads validate the full stream but return the CRC of the
  saved prefix, consistent with their returned byte count.

All 12 Debug CTest tests pass, including a build with AddressSanitizer and
UndefinedBehaviorSanitizer. New fixtures cover numerical endpoints, negative
biases, deep MLP buffer alternation, separate gates, allocation failure cleanup,
invalid remote values, sparse ESN configuration rejection, SSR clock wrap and
two-node contraction, vote relay/de-duplication and trimmed-download digests.
All six optimizer configurations, full/block HIT and distributed MNIST pass
strict host syntax checks. Existing header warnings concern redundant int8
range checks and unused header-only functions in the broad numeric fixture.

No simulation, robot program, firmware build, training, installation or linked
repository inspection was performed for this follow-up. SSR and MNIST require
matching updated peers. Their experiment performance, ANN accuracy and target
firmware timing/size/RAM remain unmeasured. Other historical concerns, including
legacy controllers, photosensors, unbounded optional stack workspaces and
remaining numerical/protocol assumptions, need separate work.

## High-priority fixes after follow-up review

The six high-priority findings from the next review are fixed:

| Finding | Resulting behavior |
| --- | --- |
| Cached log addresses survive file movement/reuse | Log I/O validates the catalog entry fingerprint and CRC. Commits also require an erased destination; stale handles fail before programming. Clear validates ownership before erasing. |
| Legacy forward speed is ignored | Both legacy executors scale each calibrated motor by the configured ratio, including zero, and remove power before direction changes. |
| Q1.15 neural accumulation overflows | Both layers accumulate in int64 and saturate after the complete dot product. The int32 MAC helper now saturates each addition. |
| Float conversion overflows near int16 endpoints | All four formats have defined endpoint/NaN/infinity conversion behavior. Q1.15/Q6.10 macros retain C static-initializer support. |
| Smaller compile-time capacities corrupt arrays | SSR copies only the defaults that fit in 1..255 classes. Photostart requires its implemented three sensors at compile time. |
| Legacy heading wrapping can hang and deadlines fail at rollover | Large finite angles use bounded reduction. Invalid headings/targets produce a recoverable STOP, and deadlines use modular comparisons with bounded intervals. |

The log checks retain RAM-only append and one-page service programming. They
add four bytes per handle plus catalog CRC/read and erased-page read costs at
commit. Reopen after filesystem movement or destructive operations; entry
fingerprints are not globally unique lifetime identifiers. On-flash formats
are unchanged, while the public log struct and legacy action enum require
rebuilding applications. The Q1.15 MLP trades extra integer instructions for
defined full-sum arithmetic; previous overflowed outputs will change. Float
conversion uses float arithmetic, with NaN mapped to zero and infinities
saturated by sign.

All 15 Debug CTest tests pass normally and with AddressSanitizer,
UndefinedBehaviorSanitizer and float-cast-overflow instrumentation. Tests include
stale-handle/new-owner isolation with zero rejected writes, competing log handles,
catalog damage, calibrated speed ratios, invalid-heading recovery, huge finite
angles, deadline rollover, large dot-product cancellation, conversion endpoints,
exhaustive int16-code round trips, small SSR arrays and compile-time rejection.
No simulation, hardware run, firmware build or linked-repository inspection was
performed. Target code size, stack, timing and physical motor behavior remain
unmeasured.

The medium-priority findings from that review remain: reciprocal-based division
accuracy/INT32_MIN handling, local optimizer parameter/fitness validation, SPSA
probe bounds, photosensor float-to-int narrowing, NaN in SSR's optional signal
colormap, and photostart threshold addition overflow. These fixes do not resolve
every historical audit concern or establish swarm performance.
