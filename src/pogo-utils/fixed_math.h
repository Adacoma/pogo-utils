/** @file fixed_math.h
 * Defined signed scaling/rounding for quantized kernels. No heap, tables or
 * floating point. The 32-bit path also avoids widened runtime arithmetic.
 * Rounding is nearest, with ties away from zero, including negative values.
 */
#ifndef POGO_UTILS_FIXED_MATH_H
#define POGO_UTILS_FIXED_MATH_H
#include <stdint.h>
#include <limits.h>

static inline int32_t pogo_round_shift_i32(int32_t x, unsigned shift) {
    if (shift == 0u) return x;
    if (shift > 31u) return 0; /* Callers with configurable shifts must reject. */
    uint32_t magnitude = x < 0 ? 0u - (uint32_t)x : (uint32_t)x;
    uint32_t rounded = (magnitude >> shift) +
        ((magnitude & ((UINT32_C(1) << shift) - 1u)) >= (UINT32_C(1) << (shift - 1u)));
    return x < 0 ? -(int32_t)rounded : (int32_t)rounded;
}

static inline int64_t pogo_round_shift_i64(int64_t x, unsigned shift) {
    if (shift == 0u) return x;
    if (shift > 63u) return 0;
    uint64_t magnitude = x < 0 ? UINT64_C(0) - (uint64_t)x : (uint64_t)x;
    uint64_t rounded = (magnitude >> shift) +
        ((magnitude & ((UINT64_C(1) << shift) - 1u)) >= (UINT64_C(1) << (shift - 1u)));
    return x < 0 ? -(int64_t)rounded : (int64_t)rounded;
}

/** Multiply by 2^shift with saturation, without left-shifting signed values. */
static inline int32_t pogo_scale_i32_sat(int32_t x, unsigned shift) {
    if (shift == 0u || x == 0) return x;
    if (shift >= 31u) return x < 0 ? INT32_MIN : INT32_MAX;
    int32_t factor = (int32_t)(UINT32_C(1) << shift);
    if (x > INT32_MAX / factor) return INT32_MAX;
    if (x < INT32_MIN / factor) return INT32_MIN;
    return x * factor;
}

static inline int16_t pogo_sat_i16(int32_t x) {
    return x > INT16_MAX ? INT16_MAX : x < INT16_MIN ? INT16_MIN : (int16_t)x;
}
#endif
