# Tutorial: fixed-point range and precision

Goal: choose a format for an actual bounded computation and understand what
saturating arithmetic does—and does not—protect.
See [the format guide](../systems/fixed_point.md).

## 1. Start with units and intermediates

For Qm.n, the raw value q represents q/2^n. The sign is included in m.
Choose storage from the maximum intermediate and acceptable error.
Q1.15 is good for normalized fractions but cannot store +1 exactly;
Q16.16 accommodates much larger values at lower fractional resolution than Q8.24.

Products double fractional-bit count before rescaling. Dot products accumulate
many products: output clipping cannot repair signed intermediate overflow.
Document a range bound for every stage before substituting fixed for float.

## 2. Bounded Q16.16 example

This complete C function adds two finite, clamped values with saturating
Q16.16 addition and returns a float diagnostic result.
It avoids unsafe signed integer-left-shift conversion paths.

```c
#include <math.h>
#include "pogo-utils/fixp_q16_16.h"

float bounded_sum(float a, float b) {
    if (!isfinite(a) || !isfinite(b)) return NAN;
    /* Domain well inside Q16.16 conversion/float precision boundaries. */
    if (a > 100.0f) a = 100.0f;
    if (a < -100.0f) a = -100.0f;
    if (b > 100.0f) b = 100.0f;
    if (b < -100.0f) b = -100.0f;
    q16_16_t qa = q16_16_from_float(a);
    q16_16_t qb = q16_16_from_float(b);
    return q16_16_to_float(q16_16_add(qa, qb));
}
```

A float boundary here is useful for testing, not automatically optimal firmware.
If your input is already scaled integer sensor data, define explicit scaling
with widened arithmetic and range checks rather than repeatedly converting
through float.

The addition/subtraction/absolute-value Q16.16 helpers saturate. Other helpers
and formats need their own review. Do not assume that
`q16_16_from_int(-1)` or a cross-format signed left shift is portable merely
because the target happens to produce expected bits. Reject NaN/Infinity before
float conversion and avoid side effects in conversion macros.

## 3. Quantify error instead of guessing

For this example, compare against clamped float a+b and allow quantization error
from both conversions, not just one. Test raw extremes separately:
max+one saturates, min-one saturates, abs(min) becomes max.
For multiply/divide, check rounding of negative values, zero divisor policy,
widened intermediate range, and saturation boundaries.

For exp/log/tanh/reciprocal, test the documented domain and worst error,
including near zero and range limits. Approximation error can dominate the
format's least-significant step. Saturation is often a useful control safeguard
but should be counted if it signifies loss of information.

## 4. Initialize only the paths that need it

Call `init_fixp` once before table-dependent Q16.16 approximations.
Current table setup costs 1,280 bytes of int32 storage and uses float math.
The simple addition function above needs no tables.
For compact telemetry use scaled-integer
[log serialization](flash_logs.md) rather than float printf if appropriate.

## 5. Benchmark on the intended platform

```sh
make -C examples/fixp sim POGOUTILS_INCLUDE_DIR=../../src
./examples/fixp/fixp -c conf/test.yaml
```

The example compares selected functions and prints timings/assertion diagnostics.
Different tests use different tolerances; read
[bench_fixp.c](../../examples/fixp/bench_fixp.c).
Build firmware to measure the real processor, compiler settings, linked code
size, stack and RAM. Host timing is not a firmware speedup claim.

[plot.py](../../examples/fixp/plot.py) uses pandas/Matplotlib and currently
contains **hard-coded timing data**. Replace its dataset deliberately with your
measured results before presenting its plot as a new experiment; it does not
automatically parse the latest run.

## 6. Extend cautiously

Use a widened intermediate, documented rounding, explicit saturation/domain
policy, and known-value/boundary tests for a new primitive. Avoid negative signed
left shifts and unchecked signed overflow. Measure portability on the actual
toolchains. See [extending](../extending.md) before changing a public type or
arithmetic convention.
