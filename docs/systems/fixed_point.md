# Fixed-point arithmetic

[fixp.h](../../src/pogo-utils/fixp.h) includes the format-specific APIs.
[fixp_base.h](../../src/pogo-utils/fixp_base.h) defines types/constants,
`init_fixp`, and `printf_fixp`. A stored integer q represents
`q / 2^fractional_bits`; the named integer-bit count includes the sign.

| Header / type | Storage | Representable range | Step |
| --- | --- | --- | --- |
| [fixp_q1_15.h](../../src/pogo-utils/fixp_q1_15.h), q1_15_t | int16 | -1 to 1-2^-15 | 2^-15 |
| [fixp_q6_10.h](../../src/pogo-utils/fixp_q6_10.h), q6_10_t | int16 | -32 to 32-2^-10 | 2^-10 |
| [fixp_q8_24.h](../../src/pogo-utils/fixp_q8_24.h), q8_24_t | int32 | -128 to 128-2^-24 | 2^-24 |
| [fixp_q16_16.h](../../src/pogo-utils/fixp_q16_16.h), q16_16_t | int32 | -32768 to 32768-2^-16 | 2^-16 |

More fractional bits improve resolution while reducing headroom. Formats are
not interchangeable aliases: adding a Q1.15 raw value to Q16.16 raw storage
without rescaling is a unit error.

## Operations and domains

Each format supplies conversions, arithmetic, and selected approximations.
Multiplication needs a widened product and fractional rescaling; division or
reciprocal needs an explicit zero policy and a range budget.
Logarithms require a positive domain; exponentials and activations have
implementation-specific clamps/approximations. Read each chosen function's
domain and rounding behavior rather than assuming exact floating-point math.

Q16.16 addition, subtraction, and absolute value deliberately saturate,
including abs(INT32_MIN)->INT32_MAX, using widened arithmetic where needed.
This does **not** mean every conversion, shift, nonlinear function, or operation
in all formats is equally hardened. Some conversion helpers use signed shifts;
out-of-range inputs and negative left shifts can invoke undefined or
implementation-defined C behavior. Avoid those paths or validate their exact
target behavior before deploying. The [audit](../code_audit.md) records remaining
numerical risks.

Integer-to-fixed shifts do not universally clamp. Float conversion requires
finite inputs; NaN is not a valid fixed number. Macro conversions can evaluate
arguments repeatedly: do not pass side-effecting expressions.
Cross-format conversion must check destination range as well as lost precision.

## Initialization, code size, and portability

Call `init_fixp` once before using table-dependent Q16.16 approximations.
Current setup fills a 256-entry reciprocal table and a 64-entry exp2 table,
both int32 (1,280 bytes together), using floating-point setup math.
Thus fixed-point **runtime arithmetic** does not imply a float-free binary or
zero initialization cost. Stateless arithmetic such as saturating addition
does not need those tables.

Most operations are inline/header-only; include only needed format headers
when helpful. Actual linked code size depends on optimizer, inlining, LTO,
referenced helpers, printf formatting, and table usage. Measure the final
firmware/map, not header length. A 64-bit intermediate can cost multiple
instructions on a 32-bit processor without wide arithmetic hardware.

`printf_fixp` is platform-oriented formatting, not a general serialization
schema. For compact CSV consider the integer/scaled helpers in
[flash logs](flash_logs.md).

## Error budgeting and testing

Quantization contributes about half a step under suitable rounding, while
saturation, biased shifts, accumulated dot products, and nonlinear approximations
add separate errors. Choose a format from the **largest intermediate**, not
only input/output range. Decide whether saturation is an intended safety limit
or an error worth reporting.

[fixp example](../../examples/fixp/README.md) benchmarks operations and compares
against float; individual tolerances differ. Host/simulator timing cannot
predict firmware speed. Test zero, one-step values, bounds, saturation, negative
values, domain limits, and reference errors on the target compiler.
The [tutorial](../tutorials/fixed_point.md) shows a small bounded computation
and explains conversion pitfalls.
