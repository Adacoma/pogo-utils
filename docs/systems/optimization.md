# Local optimization and the common facade

The standalone algorithms use caller-owned float vectors/workspaces and explicit
candidate evaluation. [optim.h](../../src/pogo-utils/optim.h) adds a common
interface and allocation from [tiny_alloc](tiny_alloc.md). It does not make
algorithm iteration counts, fitness semantics, or memory needs identical.

## Choosing an algorithm

| Algorithm / header | Update mechanism | Evaluation and resource implications |
| --- | --- | --- |
| [(1+1)-ES](../../src/pogo-utils/oneplusone_es.h) | Gaussian mutation; parent selection; success-driven sigma adaptation | One candidate per tell, two n-vectors; explicit initial fitness |
| [SPSA](../../src/pogo-utils/spsa.h) | Random simultaneous plus/minus probes approximate a gradient | Two objective evaluations per parameter update; O(n) vectors |
| [PGPE](../../src/pogo-utils/pgpe.h) | Antithetic parameter samples adapt Gaussian mean and per-coordinate sigma | Pair of evaluations per update; baseline and several n-vectors |
| [SEP-CMA-ES](../../src/pogo-utils/sep_cmaes.h) | Generation ranking, diagonal covariance and evolution paths | lambda evaluations per generation; O(mu*n + lambda + n) workspace |
| [Social learning/HIT](social_learning.md) | Remote controller repository, mutation/adoption | Message bandwidth, evaluation policy, and repository memory are additional costs |

Use minimization/maximization modes consistently. Bounds are borrowed by
standalone states; keep them alive and choose finite ordered limits.
Clipping changes the sampled search distribution near a boundary, so broad
algorithm guarantees do not automatically carry over.

## Mathematical picture

For (1+1)-ES, trial `x' = clip(x + sigma*z)`, with standard-normal z;
selection compares objective values in the requested mode. An EWMA success
fraction adapts sigma toward a target success rate.

SPSA uses independent signs delta_i in {-1,+1} and two probes at
`x +/- c_k*delta`. Its coordinate estimate is
`g_i = (f_plus-f_minus)/(2*c_k*delta_i)`,
with diminishing a_k and c_k schedules. The update moves against/with the
gradient according to minimize/maximize mode. Comparing probes under different
environmental conditions biases the estimate. See
[Spall's primary overview](https://www.jhuapl.edu/spsa/pdf-spsa/spall_an_overview.pdf).

PGPE samples `mu +/- sigma*epsilon`; the pair's reward difference informs
mean adaptation, while centered reward informs exploration-scale adaptation.
The mean, latest candidate, and best evaluated controller are different objects.

SEP-CMA-ES samples with diagonal variances, ranks a generation, and updates its
mean, paths, covariance diagonal, and global scale. It omits full off-diagonal
covariance, trading cross-coordinate modeling for bounded memory.
[Ros and Hansen's publication listing](https://www.cmap.polytechnique.fr/~nikolaus.hansen/publications.html)
identifies the separable-CMA research formulation; inspect this implementation
for its specific bounded sequential-generation behavior.

## Ask–tell ordering and ownership

Initialize buffers and algorithm settings before asking.
(1+1)-ES and SEP require `tell_initial` before the first ask.
SPSA/PGPE pair their probes through internal phases; use their own documented
initialization contract rather than inventing an initial-evaluation call.
Never overwrite a returned candidate while its evaluation is outstanding.
Its pointer is borrowed workspace, not an owned allocation or permanent history.

Each successful ask must get the corresponding tell. In strict implementations,
asking again before tell returns NULL; tell without a matching candidate does
not advance the intended evaluation sequence. Do not count calls as accepted
updates. Validate every objective as finite and correctly scaled before telling;
robust invalid-fitness handling is not uniform across all legacy algorithms.

For embodied evaluations, hold a candidate over a defined time interval,
accumulate the agreed objective, then tell once. A single control tick and an
optimization evaluation are often different durations. Fair comparison requires
consistent initial state, elapsed time, and disturbance exposure.

## SEP-CMA-ES limits

The current header bounds lambda at 256 and mu at 32, with
`1 <= mu <= lambda`. Allocate n-length x, trial, paths and sampling buffers,
mu*n stored selected steps, lambda fitness entries, and lambda indices as
documented. An optional covariance diagonal can be NULL; other required buffers
must exist and not overlap. Parameter/bounds/weight validation and checked size
arithmetic reject invalid configurations, but C cannot infer actual buffer length
from a pointer.

The legacy init returns void: call `sep_cmaes_initialized` to verify success.
Invalid initialization leaves an inert state; it is not a usable zero controller.
Generation updates reuse caller workspaces rather than dimension-dependent VLA
scratch. A nonfinite fitness consumes the outstanding candidate without updating
the distribution or accepted-evaluation count; the caller can then ask a new
candidate. See [example](../../examples/sep_cmaes/README.md).

## Facade and allocation

Start with `opt_default_cfg(algo,n)`, edit the algorithm union and sizing
fields deliberately, then `opt_create(&handle,ta,n,algo,mode,lo,hi,&cfg)`.
Check its success result. NULL bounds request allocated [-1,+1] vectors.
External bounds and allocator storage must outlive the handle.
`opt_destroy` releases the facade's owned blocks, not external buffers.

Set/randomize x before initial fitness; call `opt_tell_initial` according to
the selected algorithm's semantics, then `opt_ask(handle,aux_scale)` and
`opt_tell`. The auxiliary scale matters for social learning rather than all
algorithms. Inspect `opt_ready`, `opt_iterations`, and `opt_get_x` with the
selected algorithm in mind. Remote-observation calls matter only for social
algorithms.

Allocator total free bytes alone do not guarantee success: each requested block
needs a sufficiently large free slot, including the opaque optimizer handle,
population/workspace arrays, and repository blocks. Default tiny_alloc classes
only reach 192 bytes; use appropriately sized custom classes or standalone
caller buffers for larger problems. Increasing dimension is not free.

The `use_defaults` field's header commentary should not be treated as a
universal default-merging contract; current creation copies the supplied config
rather than applying arbitrary partial-config defaults. A zeroed config is not
a substitute for `opt_default_cfg`. The audit records remaining facade-contract
work separately from hardened standalone SEP.

## Experiments and debugging

Use the [optimization tutorial](../tutorials/optimization.md) and the
[individual examples](../examples.md#optimization-and-memory).
Record algorithm, seed, bounds, objective, evaluation duration, accepted
evaluations versus updates/generations, exploration parameters, best/current
vectors, and memory failures. Analytic test-function improvement is not proof
of successful decentralized robot learning.
