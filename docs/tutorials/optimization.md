# Tutorial: an optimization experiment

Goal: define a reproducible evaluation and obey ask–tell lifetime/order.
Start with a standalone algorithm; adopt the common facade only when its
allocation/switching benefits are useful. Read
[optimization](../systems/optimization.md) and
[social learning](../systems/social_learning.md).

## 1. Define the objective and evaluation boundary

Document n, units, finite bounds, minimize/maximize, initial state, seed,
evaluation duration, and reward/loss definition. A robot controller usually needs
many control ticks per candidate; an analytic function may evaluate immediately.
Compare candidates under matched conditions, not merely the same function name.

Separate number of objective calls, successful parameter updates, and generations.
SPSA/PGPE use pairs; SEP uses a lambda-sized generation; HIT uses reward windows.
Never use one “iteration” axis without specifying which quantity it means.

## 2. Standalone (1+1)-ES module

This self-contained module minimizes a two-dimensional sphere.
Caller-owned arrays avoid any allocator dependency.

```c
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include "pogo-utils/oneplusone_es.h"

typedef struct {
    es1p1_t es;
    float x[2], trial[2], lo[2], hi[2];
} experiment_t;

static float loss(const float x[2]) {
    return x[0]*x[0] + x[1]*x[1];
}

void experiment_init(experiment_t *e) {
    e->x[0] = 0.8f; e->x[1] = -0.6f;
    for (unsigned i = 0; i < 2; ++i) {
        e->lo[i] = -1.0f; e->hi[i] = 1.0f;
    }
    es1p1_params_t p = {
        .mode = ES1P1_MINIMIZE,
        .sigma0 = 0.2f, .sigma_min = 1e-5f, .sigma_max = 0.8f,
        .s_target = 0.2f, .s_alpha = 0.2f, .c_sigma = 0.0f
    };
    es1p1_init(&e->es, 2, e->x, e->trial, e->lo, e->hi, &p);
    es1p1_tell_initial(&e->es, loss(e->x)); /* Required before asking. */
}

bool experiment_evaluate_one(experiment_t *e) {
    const float *candidate = es1p1_ask(&e->es);
    if (candidate == NULL) return false;
    float f = loss(candidate);
    if (!isfinite(f)) return false; /* Caller must resolve pending evaluation. */
    (void)es1p1_tell(&e->es, f);
    return true;
}
```

The candidate is borrowed and reused; copy it if you need an archive.
The tiny analytic example cannot produce a nonfinite loss within bounds.
For an embodied objective, keep the candidate outstanding until a valid result
or an explicitly designed failure/restart policy exists. Do not invent an
unrelated tell or repeatedly ask on a failed evaluation.

Seed the application's RNG as the existing platform examples do and record
that seed; this module does not silently select one. For stationary minimization
the parent selection provides a useful best/current distinction; it is not a
guarantee under noisy changing rewards.

## 3. Replace the algorithm, not its semantics

SPSA initializes its own buffers/params and alternates positive/negative probes.
One coordinate update needs both evaluations; do not overwrite its delta/work
vector between them. PGPE adapts distribution mean/sigma after an antithetic pair.
Its mean is not necessarily the latest tested controller.

For SEP allocate all documented work arrays before init, with lambda<=256 and
mu<=min(lambda,32). Check `sep_cmaes_initialized`, provide initial fitness,
then one ask/tell at a time. A complete generation updates the distribution;
one offspring tell is not a generation. Dimensions/population limits do not
validate actual C buffer lengths for you.

[Individual example READMEs](../examples.md#optimization-and-memory) link each
ready-to-build integration and diagnostic output.

## 4. Common facade and allocator sizing

`opt_default_cfg(algo,n)` yields a baseline config; set its specific fields and
sizing values before `opt_create`. Do not pass an arbitrary zero-filled partial
config or depend on undocumented default merging.
Use `opt_set_x`/`opt_randomize_x` before initial fitness.
Check creation, every ask result, and algorithm readiness.

The allocator needs slots for the opaque handle **and** each vector/repository
block. Default tiny_alloc largest payload is 192 bytes, often insufficient.
Use the [optim example](../../examples/optim/README.md)'s custom classes as a
starting point, then compute actual requested sizes/counts for your dimension.
Total arena bytes alone do not ensure an appropriate free class exists.
Keep external bounds/arena alive and call `opt_destroy` before recycling them.

## 5. Social learning and HIT

Define a wire tag/layout, validate payload length/values, and deliver remote
ID/epoch/fitness/genome observations to the selected algorithm.
Use matching dimensions and objective/maturity conventions across robots.
Social learning's episode-style tell and HIT's instantaneous reward window are
not interchangeable. Block observations reduce one packet's size but need a
consistent epoch/block policy.

The supplied learning examples demonstrate native packed protocols; they are
not a portable or fully hardened network schema. Use explicit serialization for
new interoperable protocols and reserve a distinct tag.

## 6. Validate and report

```sh
make -C examples/oneplusone_es sim
./examples/oneplusone_es/oneplusone_es -c conf/test.yaml
```

Check objective sign, bounded candidate values, accepted-evaluation counts,
sigma/learning-rate behavior, initialization failure, outstanding-candidate
ordering, allocation exhaustion, and multiple seeds.
For robots additionally log evaluation duration, resets, motion interruptions,
neighbor adoption/epochs, repository occupancy and communication rate.
Keep CSV/log commits outside the reward/control timing assumptions.
A toy sphere benchmark does not establish controller-learning performance.
