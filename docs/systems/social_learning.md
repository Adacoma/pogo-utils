# Social learning and HIT

[social_learning.h](../../src/pogo-utils/social_learning.h) and
[hit.h](../../src/pogo-utils/hit.h) implement decentralized controller exchange
and adaptation. They accept observations from neighbors; message framing,
serialization, transmission schedule, and objective measurement are application
responsibilities. [optim.h](../../src/pogo-utils/optim.h) exposes them through
the common facade.

## Two different evaluation contracts

Social learning keeps a bounded repository of local/remote controllers and
advertised fitness. Roulette-style selection plus mutation generates trials;
mutation scaling and comparison mode are configurable.
Initialize caller buffers/repository, provide initial fitness, ask a candidate
with the scale argument, evaluate it for a comparable interval, then tell.
Remote `from_id`/epoch updates should replace/update a neighbor's stored entry
rather than multiply its selection weight by every retransmission.

HIT follows an instantaneous-reward/sliding-window design. It reports a
current controller each tick, accumulates rewards over a maturation window, and
permits adoption/mutation according to maturity and remote information.
Adaptive alpha and sigma logic belong to that algorithm, not a generic
fitness-vector averaging rule. `HIT_MAX_T=32` bounds stored window length.
Do not pass a whole-episode cumulative reward repeatedly as if it were an
instantaneous tick reward.

“Best fitness,” current window average, last advertised score, and readiness
have distinct meanings. HIT's `get_f`, `get_f_sum`, epoch, alpha, and ready
accessors support inspection; they do not standardize every optimizer's notion
of iteration. Match the exact reward/time semantics of the selected mode.

## Ownership and memory

Standalone initialization borrows vector/bounds/workspaces and the bounded
repository. Keep all arrays alive and nonoverlapping. Capacity times dimension
is a real RAM cost; remote adoption also costs copying/mutation.
Facade creation uses a supplied tiny_alloc arena and can fail even when its
total free-byte count looks large.

Use `sl_observe_remote` or `hit_observe_remote` with valid dimensions,
identity, epoch, advertised score, and controller values. HIT also accepts
contiguous block observations; full and block messages are different contracts.
A block update preserves coordinates outside that block. Validate offset/length
and the epoch/controller consistency required by the application. Blockwise
exchange reduces one packet's size but can produce partially updated
controllers, not an atomic genome transaction.

## Transport integration

The [social_learning](../../examples/social_learning/README.md),
[hit](../../examples/hit/README.md), and
[optim](../../examples/optim/README.md) applications illustrate bounded IR
exchange, callbacks, and per-robot repositories. The HIT example defaults to
four-coordinate blocks in a 16-dimensional controller.

Several learning examples use packed native C structs with float fields.
These demonstrate the current homogeneous-toolchain protocol; they are not
a portable byte-order/float-encoding schema or a hardened untrusted-input
interface. Keep all peers on matching dimension/tag/layout, enforce payload
lengths and finite values, and avoid assuming raw casts provide alignment.
The historical [audit](../code_audit.md) identifies protocol follow-ups.

Serialize transport separately from optimization; the library has no network
stack or ACK/reliable delivery mechanism. Repeated epochs, loss, stale neighbors,
delayed scores, and changing objectives are experimental variables.

## Scientific interpretation

Remote fitness is comparable only if robots share objective sign, scale,
duration, operating conditions, and maturity semantics. Neighbor controller
quality in one location may not transfer to another. Record adoption counts,
source IDs, epochs, repository occupancy, diversity, maturity, rewards, and
communication loss alongside aggregate objective curves.

The [ACU example](../../examples/acu/README.md) is a static controller and
does **not** use these optimizers despite its inspiration from an external
optimized-controller project. The local headers/examples are the authority
for this library's behavior. See [optimization tutorial](../tutorials/optimization.md).
