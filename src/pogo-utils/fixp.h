
#ifndef FIXP_H_
#define FIXP_H_

#include <stdint.h>

/* ==== Q8.24 Fixed-Point Definitions and Functions ==== */

#define Q8_24_FRACTIONAL_BITS 24
//#define Q8_24_ONE (1 << Q8_24_FRACTIONAL_BITS)
// Q8.24: 32-bit with 24 fractional bits
#define Q8_24_ONE      ((q8_24_t)(1UL << Q8_24_FRACTIONAL_BITS))         // 1.0 in Q8.24 (16777216)
#define Q8_24_NEG_ONE  ((q8_24_t)(- (1UL << Q8_24_FRACTIONAL_BITS)))     // -1.0 in Q8.24 (-16777216)

#define Q8_24_FROM_INT(x) ((q8_24_t)((x) << 24)) /* In Q8.24, converting an int x means computing x << 24.*/

// Precomputed constant macro for ln(2) in Q8.24.
// For Q8.24, 1.0 = (1UL << 24) = 16777216, so ln(2) ≈ 0.693147 * 16777216 ≈ 11639491.
#define Q8_24_LN2_CONST     11639491       // Precomputed: 0.693147 * 16777216

/*
 * In Q8.24 the maximum representable value is (2^31 - 1) scaled by 2^-24,
 * which is approximately 127.99999994, and the minimum is -128.0.
 */
#define Q8_24_MAX ((q8_24_t)0x7FFFFFFF)
#define Q8_24_MIN ((q8_24_t)0x80000000)



typedef int32_t q8_24_t;

/* Convert an integer to Q8.24 fixed point with saturation. */
static inline q8_24_t q8_24_from_int(int x) {
    if (x > 127)
        return Q8_24_MAX;
    if (x < -128)
        return Q8_24_MIN;
    return (q8_24_t)(x << Q8_24_FRACTIONAL_BITS);
}

/* Convert Q8.24 fixed point to an integer (truncates fractional part). */
static inline int q8_24_to_int(q8_24_t x) {
    return x >> Q8_24_FRACTIONAL_BITS;
}

/* Convert a float to Q8.24 fixed point with saturation. */
static inline q8_24_t q8_24_from_float(float x) {
    if (x >= 128.0f)
        return Q8_24_MAX;
    if (x < -128.0f)
        return Q8_24_MIN;
    return (q8_24_t)(x * Q8_24_ONE);
}

/* Convert Q8.24 fixed point to a float. */
static inline float q8_24_to_float(q8_24_t x) {
    return (float)x / Q8_24_ONE;
}

/* Addition with saturation. */
static inline q8_24_t q8_24_add(q8_24_t a, q8_24_t b) {
    int64_t sum = (int64_t)a + b;
    if (sum > Q8_24_MAX)
        return Q8_24_MAX;
    if (sum < Q8_24_MIN)
        return Q8_24_MIN;
    return (q8_24_t)sum;
}

/* Subtraction with saturation. */
static inline q8_24_t q8_24_sub(q8_24_t a, q8_24_t b) {
    int64_t diff = (int64_t)a - b;
    if (diff > Q8_24_MAX)
        return Q8_24_MAX;
    if (diff < Q8_24_MIN)
        return Q8_24_MIN;
    return (q8_24_t)diff;
}

/* Multiplication with saturation.
 * Multiply two Q8.24 numbers, using a 64-bit intermediate product, then shift back.
 */
static inline q8_24_t q8_24_mul(q8_24_t a, q8_24_t b) {
    int64_t prod = ((int64_t)a * b) >> Q8_24_FRACTIONAL_BITS;
    if (prod > Q8_24_MAX)
        return Q8_24_MAX;
    if (prod < Q8_24_MIN)
        return Q8_24_MIN;
    return (q8_24_t)prod;
}

/* Division with saturation.
 * If dividing by zero, the function returns Q8_24_MAX for a non-negative numerator
 * and Q8_24_MIN for a negative numerator.
 */
static inline q8_24_t q8_24_div(q8_24_t a, q8_24_t b) {
    if (b == 0) {
        return (a >= 0) ? Q8_24_MAX : Q8_24_MIN;
    }
    int64_t res = ((int64_t)a << Q8_24_FRACTIONAL_BITS) / b;
    if (res > Q8_24_MAX)
        return Q8_24_MAX;
    if (res < Q8_24_MIN)
        return Q8_24_MIN;
    return (q8_24_t)res;
}

/* Absolute value with saturation.
 * Since abs(Q8_24_MIN) cannot be represented in two's complement, we saturate to Q8_24_MAX.
 */
static inline q8_24_t q8_24_abs(q8_24_t x) {
    if (x == Q8_24_MIN)
        return Q8_24_MAX;
    return (x < 0) ? -x : x;
}

/*
 * Check if a Q8.24 value is saturated.
 * Returns 1 if x is equal to Q8_24_MAX or Q8_24_MIN, otherwise returns 0.
 */
static inline int q8_24_is_saturated(q8_24_t x) {
    return (x == Q8_24_MAX || x == Q8_24_MIN);
}


/*
 * q8_24_exp:
 * Computes e^x for a Q8.24 number x.
 * Uses range reduction: x = n * ln2 + r, with r in [0, ln2)
 * so that e^x = 2^n * e^r.
 *
 * e^r is approximated by a 7-term Taylor series:
 *    e^r ≈ 1 + r + r^2/2! + r^3/3! + r^4/4! + r^5/5!  (we use 5 additional terms)
 *
 * All intermediate calculations are done in 64-bit arithmetic,
 * with Q8_24_ONE (16777216) representing 1.0.
 *
 * For saturation, if x is greater than Q8_24_EXP_MAX_ARG_CONST (≈4.85203) then the result
 * is saturated to Q8_24_MAX; if x is less than Q8_24_EXP_MIN_ARG_CONST (≈-16.632) then 0 is returned.
 */
#define Q8_24_EXP_MAX_ARG_CONST  81368978  // Precomputed: 4.85203 * 16777216
#define Q8_24_EXP_MIN_ARG_CONST  (-279035456) // Precomputed: -16.632 * 16777216
static inline q8_24_t q8_24_exp(q8_24_t x) {
    const q8_24_t max_arg = Q8_24_EXP_MAX_ARG_CONST;
    const q8_24_t min_arg = Q8_24_EXP_MIN_ARG_CONST;
    if (x > max_arg)
        return Q8_24_MAX;
    if (x < min_arg)
        return 0;

    // Use precomputed ln2.
    const q8_24_t ln2_fixed = Q8_24_LN2_CONST;

    // Range reduction: find integer n such that x = n * ln2 + r, with r in [0, ln2).
    int64_t x64 = x;
    int n = (int)(x64 / ln2_fixed);
    if (x64 < 0 && (x64 % ln2_fixed) != 0)
        n--;

    q8_24_t n_ln2 = n * ln2_fixed;  // Multiply in integer arithmetic.
    q8_24_t r = q8_24_sub(x, n_ln2);

    // Compute exp(r) using a Taylor series:
    // exp(r) ≈ 1 + r + r^2/2! + r^3/3! + r^4/4! + r^5/5!
    int64_t term = Q8_24_ONE;  // term0 = 1.0 in Q8_24
    int64_t sum  = Q8_24_ONE;  // initialize sum with 1.0
    for (int i = 1; i <= 5; i++) {
        term = (term * r) >> 24;  // Q8_24_FRACTIONAL_BITS = 24
        term = term / i;
        sum += term;
    }

    // Scale the series result by 2^n.
    int64_t result = sum;
    if (n >= 0) {
        for (int i = 0; i < n; i++) {
            result = result << 1;
            if (result > Q8_24_MAX) { result = Q8_24_MAX; break; }
        }
    } else {
        for (int i = 0; i < -n; i++) {
            result = result >> 1;
        }
    }
    if (result > Q8_24_MAX)
        result = Q8_24_MAX;
    if (result < Q8_24_MIN)
        result = Q8_24_MIN;
    return (q8_24_t) result;
}


/*
 * _q8_24_log_base:
 * Computes the natural logarithm (ln) of x, for x > 0, in Q8.24.
 * It normalizes x by expressing it as:
 *
 *    x = m * 2^n,  with m in [Q8_24_ONE, 2*Q8_24_ONE)   (i.e. [16777216, 33554432))
 *
 * Then, letting t = m - Q8_24_ONE, it approximates ln(1+t) using a 7-term alternating series:
 *
 *    ln(1+t) ≈ t - t^2/2 + t^3/3 - t^4/4 + t^5/5 - t^6/6 + t^7/7.
 *
 * Finally, ln(x) = ln(m) + n*ln2, where ln2 is given by Q8_24_LN2_CONST.
 */
static inline q8_24_t _q8_24_log_base(q8_24_t x) {
    if (x <= 0)
        return Q8_24_MIN;  // Saturate for nonpositive x.

    int n = 0;
    q8_24_t m = x;
    while (m < Q8_24_ONE) {
        // Multiply m by 2 using the macro to convert 2 to Q8.24.
        m = q8_24_mul(m, Q8_24_FROM_INT(2));
        n--;
    }
    while (m >= (Q8_24_ONE << 1)) {  // while m >= 2.0
        m = q8_24_div(m, Q8_24_FROM_INT(2));
        n++;
    }

    // t = m - Q8_24_ONE, so that m = 1+t (with t in Q8.24 units)
    q8_24_t t = q8_24_sub(m, Q8_24_ONE);

    // Compute ln(1+t) using a 7-term alternating series.
    q8_24_t term = t;
    q8_24_t sum = t;
    for (int i = 2; i <= 5; i++) {
        term = q8_24_mul(term, t);
        q8_24_t frac = q8_24_div(term, Q8_24_FROM_INT(i));
        if (i % 2 == 0)
            sum = q8_24_sub(sum, frac);
        else
            sum = q8_24_add(sum, frac);
    }

    // Use the macro-defined ln2 constant.
    q8_24_t n_ln2 = q8_24_mul(Q8_24_FROM_INT(n), Q8_24_LN2_CONST);
    return q8_24_add(sum, n_ln2);
}


/*
 * q8_24_log
 * Computes a refined approximation of ln(x) for a Q8.24 number x > 0.
 * It first obtains an initial approximation y0 using the series-based
 * q8_24_log function. Then one Newton–Raphson iteration is applied:
 *
 *    y_new = y_old - (exp(y_old) - x) / exp(y_old)
 *
 * This iteration tends to reduce the error in the initial approximation.
 *
 * Note: All arithmetic uses our Q8.24 functions.
 */
static inline q8_24_t q8_24_log(q8_24_t x) {
    // Initial approximation using the series-based function.
    q8_24_t y = _q8_24_log_base(x);

    // Compute exp(y) using our Q16.16 exp function.
    q8_24_t exp_y = q8_24_exp(y);

    // Compute the error: error = exp(y) - x.
    q8_24_t error = q8_24_sub(exp_y, x);

    // Compute the correction: correction = error / exp(y)
    q8_24_t correction = q8_24_div(error, exp_y);

    // Refine: y_new = y - correction.
    q8_24_t y_new = q8_24_sub(y, correction);

    return y_new;
}

// Returns the integer part of a Q8.24 number.
static inline int32_t q8_24_get_int(q8_24_t x) {
    return x >> Q8_24_FRACTIONAL_BITS;
}

// Returns the fractional part of a Q8.24 number as an unsigned 32-bit value.
static inline uint32_t q8_24_get_frac(q8_24_t x) {
    uint32_t mask = (1UL << Q8_24_FRACTIONAL_BITS) - 1;
    // Use the absolute value of the lower bits.
    return (x >= 0) ? ((uint32_t)x & mask) : (((uint32_t)(-x)) & mask);
}



/* ==== Q1.15 Fixed-Point Definitions and Conversion Functions ==== */

/*
 * In Q1.15, a 16-bit signed fixed-point format, 15 bits represent the fractional part.
 * The representable range is approximately [-1.0, 0.99997].
 */
#define Q1_15_FRACTIONAL_BITS 15
// XXX
#define Q1_15_ONE 32768L // (1 << Q1_15_FRACTIONAL_BITS)
//#define Q1_15_ONE (q1_15_t)(1 << Q1_15_FRACTIONAL_BITS)
//#define Q1_15_NEG_ONE  ((q1_15_t)(- (1 << Q1_15_FRACTIONAL_BITS)))        // -1.0 in Q1.15 (-32768)

#define Q1_15_MAX ((q1_15_t)0x7FFF)
#define Q1_15_MIN ((q1_15_t)0x8000)
#define Q1_15_SCALE (1 << Q1_15_FRACTIONAL_BITS)  // 32768
#define Q1_15_LN2_CONST 22713  // ln(2) ≈ 0.693147 * 32768


typedef int16_t q1_15_t;

/* Convert an integer to Q1.15 fixed-point with saturation.
   Only values in the range [-1, 1) are representable.
   For any integer x >= 1, the result saturates to Q1_15_MAX.
   For any x < -1, it saturates to Q1_15_MIN.
*/
static inline q1_15_t q1_15_from_int(int x) {
    if (x >= 1)
        return Q1_15_MAX;
    if (x < -1)
        return Q1_15_MIN;
    return (q1_15_t)(x << Q1_15_FRACTIONAL_BITS);
}

/* Convert Q1.15 fixed-point to an integer (truncating the fractional part). */
static inline int q1_15_to_int(q1_15_t x) {
    return x >> Q1_15_FRACTIONAL_BITS;
}

/*
 * Convert from Q1.15 to Q8.24.
 * Since Q8.24 has a wider range, conversion is done by shifting left 9 bits.
 */
static inline q8_24_t q8_24_from_q1_15(q1_15_t x) {
    return ((q8_24_t)x) << (Q8_24_FRACTIONAL_BITS - Q1_15_FRACTIONAL_BITS);
}

/* Convert a float to Q1.15 fixed-point with saturation.
   Valid input range is [-1.0, 1.0); values outside saturate.
*/
static inline q1_15_t q1_15_from_float(float x) {
    if (x >= 1.0f)
        return Q1_15_MAX;
    if (x < -1.0f)
        return Q1_15_MIN;
    return (q1_15_t)(x * Q1_15_ONE);
    //volatile float res = x * 128.0f; // XXX evil hack to avoid double promotion
    //return (q1_15_t)(res * 256.0f);
}

/* Convert Q1.15 fixed-point to a float. */
static inline float q1_15_to_float(q1_15_t x) {
    return (float)x / Q1_15_ONE;
    //float tmp = (float)x * 1.52587890625e-01f;
    ////for (uint8_t i = 0; i < 1; i++)
    //    tmp /= 10.0f;
    ////volatile float res = (float)x / 128.0f;
    ////return (float)tmp * 1.52587890625e-05f;
    //return (float)tmp;
}

/* ==== Conversion Between Q8.24 and Q1.15 ==== */

/*
 * Convert from Q8.24 to Q1.15.
 * The conversion is achieved by shifting right (24-15 = 9) bits.
 * Since Q1.15 can only represent numbers in the range [-1.0, 1.0),
 * values in Q8.24 outside [-1.0, 1.0) saturate to Q1_15_MIN or Q1_15_MAX.
 */
static inline q1_15_t q1_15_from_q8_24(q8_24_t x) {
    /* Q8.24 representing 1.0 is (1 << 24) and -1.0 is (-1 << 24). */
    if (x >= (1 << Q8_24_FRACTIONAL_BITS))
        return Q1_15_MAX;
    if (x < -(1 << Q8_24_FRACTIONAL_BITS))
        return Q1_15_MIN;
    return (q1_15_t)(x >> (Q8_24_FRACTIONAL_BITS - Q1_15_FRACTIONAL_BITS));
}


/* ==== Q1.15 Arithmetic Functions ==== */

/* Addition with saturation for Q1.15 */
static inline q1_15_t q1_15_add(q1_15_t a, q1_15_t b) {
    int32_t sum = (int32_t)a + b;
    if (sum > Q1_15_MAX)
        return Q1_15_MAX;
    if (sum < Q1_15_MIN)
        return Q1_15_MIN;
    return (q1_15_t)sum;
}

/* Subtraction with saturation for Q1.15 */
static inline q1_15_t q1_15_sub(q1_15_t a, q1_15_t b) {
    int32_t diff = (int32_t)a - b;
    if (diff > Q1_15_MAX)
        return Q1_15_MAX;
    if (diff < Q1_15_MIN)
        return Q1_15_MIN;
    return (q1_15_t)diff;
}

/* Multiplication with saturation for Q1.15 */
static inline q1_15_t q1_15_mul(q1_15_t a, q1_15_t b) {
    int32_t prod = ((int32_t)a * b) >> Q1_15_FRACTIONAL_BITS;
    if (prod > Q1_15_MAX)
        return Q1_15_MAX;
    if (prod < Q1_15_MIN)
        return Q1_15_MIN;
    return (q1_15_t)prod;
}

/* Division with saturation for Q1.15 */
static inline q1_15_t q1_15_div(q1_15_t a, q1_15_t b) {
    if (b == 0)
        return (a >= 0) ? Q1_15_MAX : Q1_15_MIN;
    int32_t res = (((int32_t)a) << Q1_15_FRACTIONAL_BITS) / b;
    if (res > Q1_15_MAX)
        return Q1_15_MAX;
    if (res < Q1_15_MIN)
        return Q1_15_MIN;
    return (q1_15_t)res;
}

/* Absolute value for Q1.15 with saturation */
static inline q1_15_t q1_15_abs(q1_15_t x) {
    if (x == Q1_15_MIN)
        return Q1_15_MAX;
    return (x < 0) ? -x : x;
}

/* Returns 1 if x equals Q1_15_MAX or Q1_15_MIN, otherwise 0. */
static inline int q1_15_is_saturated(q1_15_t x) {
    return (x == Q1_15_MAX || x == Q1_15_MIN);
}

/*
 * q1_15_exp:
 * Computes exp(x) for a Q1.15 number x.
 * Since Q1.15 can represent only numbers in approximately [-1.0, 0.99997]
 * and exp(x) >= 1 for x >= 0, we saturate nonnegative x to Q1_15_MAX.
 * For negative x we use range reduction:
 *
 *     x = n * ln2 + r,   with r in [0, ln2)
 *
 * Then exp(x) = 2^n * exp(r), where exp(r) is approximated by a 7-term series:
 *
 *     exp(r) ≈ 1 + r + r^2/2! + ... + r^6/6!
 *
 * All calculations for the series are done in 32-bit arithmetic with Q1_15_SCALE
 * (32768) representing 1.0.
 */
static inline q1_15_t q1_15_exp(q1_15_t x) {
    // Use the macro for ln2 in Q1.15.
    const q1_15_t ln2_fixed = Q1_15_LN2_CONST;  // ~22713

    // For any nonnegative input, exp(x) is >= 1.0; saturate to Q1_15_MAX.
    if (x >= 0)
        return Q1_15_MAX;

    /* Compute n = floor(x/ln2) using 32-bit arithmetic.
       Let L = Q1_15_ONE * ln2 ≈ 32768 * 0.693147 ≈ 22713 in Q1.15 units.
    */
    int32_t x32 = (int32_t)x;  // e.g., for x = -1.0, x32 = -32768.
    int32_t L = Q1_15_LN2_CONST;
    int n = x32 / L;         // C division truncates toward 0.
    if (x32 % L != 0)
        n--;  // Adjust for negative x to get the floor.

    // Compute n * ln2 in Q1.15.
    int32_t ln2_32 = ln2_fixed;
    int32_t n_ln2 = n * ln2_32;

    // Compute remainder r = x - (n * ln2) in 32-bit arithmetic.
    int32_t r32 = x32 - n_ln2;
    q1_15_t r = (q1_15_t) r32;

    /* Compute exp(r) using a 7-term series in 32-bit arithmetic.
       We work with 32-bit values where Q1_15_ONE (32768) represents 1.0.
       Here we use 5 terms (term0 plus terms for i = 1 to 5).
    */
    int32_t term32 = Q1_15_ONE;  // term0 = 1.0
    int32_t sum32  = Q1_15_ONE;  // series sum starts at 1.0
    for (int i = 1; i <= 5; i++) {
        int32_t prod = ((int32_t)term32 * (int32_t)r) >> Q1_15_FRACTIONAL_BITS;
        term32 = prod / i;  // plain 32-bit division (non-saturating)
        sum32 += term32;
    }

    // Multiply the series result by 2^n.
    int32_t result32 = sum32;
    if (n < 0) {
        for (int i = 0; i < -n; i++)
            result32 >>= 1;  // Divide by 2 for negative n.
    } else {
        for (int i = 0; i < n; i++) {
            result32 <<= 1;  // Multiply by 2 for positive n.
            if (result32 >= Q1_15_ONE)
                result32 = Q1_15_ONE - 1;  // Saturate.
        }
    }
    if (result32 >= Q1_15_ONE)
         result32 = Q1_15_ONE - 1;
    return (q1_15_t) result32;
}

/*
 * q1_15_log:
 * Computes the natural logarithm (ln) for a Q1.15 number x > 0.
 * Q1.15 represents numbers in approximately [-1.0, 0.99997],
 * so the ideal value 1.0 is not exactly representable (since Q1_15_MAX is 32767,
 * while the ideal 1.0 would be 32768). To work around this,
 * we perform normalization in 32-bit arithmetic.
 *
 * We normalize x as m * 2^n (with m in [Q1_15_SCALE, 2*Q1_15_SCALE), where
 * Q1_15_SCALE is 1<<15 (32768)). Then we set t = m - Q1_15_SCALE and approximate
 * ln(1+t) with a 7-term alternating series. Finally, ln(x) = ln(m) + n * ln2.
 *
 * This revised implementation computes everything in a 32-bit variable and then
 * saturates the final result to the valid Q1.15 range [-32768, 32767].
 */
static inline q1_15_t q1_15_log(q1_15_t x) {
    if (x <= 0)
        return Q1_15_MIN;  // undefined for nonpositive x, so saturate

    int n = 0;
    // Use a 32-bit integer for normalization.
    // If x equals Q1_15_MAX (32767), treat it as the ideal 1.0 (32768)
    int32_t m32 = (x == Q1_15_MAX) ? (1 << Q1_15_FRACTIONAL_BITS) : (int32_t)x;

    // Normalize m32 so that it lies in [32768, 65536)
    while (m32 < (1 << Q1_15_FRACTIONAL_BITS)) {
        m32 *= 2;
        n--;
    }
    while (m32 >= ((1 << Q1_15_FRACTIONAL_BITS) << 1)) {
        m32 /= 2;
        n++;
    }

    // t = m32 - 32768, which represents m - 1.0
    int32_t t_int = m32 - (1 << Q1_15_FRACTIONAL_BITS);

    // Compute ln(1+t) using a 7-term alternating series in 32-bit arithmetic.
    int32_t term = t_int;  // first term is t
    int32_t sum  = t_int;
    for (int i = 2; i <= 5; i++) {
        int32_t prod = (term * t_int) >> Q1_15_FRACTIONAL_BITS; // fixed-point multiplication
        term = prod / i;  // plain 32-bit division (non-saturating)
        if (i % 2 == 0)
            sum -= term;
        else
            sum += term;
    }

    // ln2 in Q1.15: ideal ln2 ≈ 0.693147, which in Q1.15 is about 22713.
    int32_t ln2_32 = 22713;
    int32_t n_ln2 = n * ln2_32;

    int32_t ln_x = sum + n_ln2;

    // Saturate ln_x to the valid Q1.15 range: [-32768, 32767]
    if (ln_x < -32768)
        ln_x = -32768;
    if (ln_x > 32767)
        ln_x = 32767;

    return (q1_15_t) ln_x;
}

// Returns the integer part of a Q1.15 number.
static inline int16_t q1_15_get_int(q1_15_t x) {
    return x >> Q1_15_FRACTIONAL_BITS;
}

// Returns the fractional part of a Q1.15 number as an unsigned 16-bit value.
static inline uint16_t q1_15_get_frac(q1_15_t x) {
    uint16_t mask = (1 << Q1_15_FRACTIONAL_BITS) - 1;
    return (x >= 0) ? ((uint16_t)x & mask) : (((uint16_t)(-x)) & mask);
}


/* === Q16.16 Fixed-Point Definitions === */
/*
 * Q16.16 uses 32 bits: 16 bits for the integer part and 16 bits for the fractional part.
 * The scaling factor is 2^16 = 65536.
 */
#define Q16_16_FRACTIONAL_BITS 16
#define Q16_16_ONE     ((q16_16_t)(1 << Q16_16_FRACTIONAL_BITS))           // 1.0 in Q16.16 (65536)
#define Q16_16_NEG_ONE ((q16_16_t)(- (1 << Q16_16_FRACTIONAL_BITS)))       // -1.0 in Q16.16 (-65536)

#define Q16_16_LN2_CONST 45426          // ln2 ≈ 0.693147 * 65536

/*
 * Q16.16 uses a signed 32-bit integer.
 * The maximum representable value is 0x7FFFFFFF and the minimum is 0x80000000.
 * (These macros are used for saturation in arithmetic operations.)
 */
#define Q16_16_MAX ((q16_16_t)0x7FFFFFFF)
#define Q16_16_MIN ((q16_16_t)0x80000000)

typedef int32_t q16_16_t;

/* --- Conversions Between int and Q16.16 --- */
/* Convert int to Q16.16 (by shifting left 16 bits) */
static inline q16_16_t q16_16_from_int(int x) {
    return (q16_16_t)x << Q16_16_FRACTIONAL_BITS;
}

/* Convert Q16.16 to int (by shifting right 16 bits) */
static inline int q16_16_to_int(q16_16_t x) {
    return x >> Q16_16_FRACTIONAL_BITS;
}

/* --- Conversions Between float and Q16.16 --- */
/* Convert float to Q16.16 */
static inline q16_16_t q16_16_from_float(float x) {
    return (q16_16_t)(x * Q16_16_ONE);
}

/* Convert Q16.16 to float */
static inline float q16_16_to_float(q16_16_t x) {
    return (float)x / Q16_16_ONE;
}

/* --- Conversions Between Q8.24 and Q16.16 --- */
/*
   Q8.24 is a 32-bit fixed-point format with 24 fractional bits.
   To convert from Q8.24 to Q16.16, shift right by (24 - 16)=8 bits.
*/
static inline q16_16_t q16_16_from_q8_24(int32_t q8_24) {
    return (q16_16_t)(q8_24 >> (24 - Q16_16_FRACTIONAL_BITS));
}

/* Convert Q16.16 to Q8.24 by shifting left by 8 bits */
static inline int32_t q8_24_from_q16_16(q16_16_t x) {
    return ((int32_t)x) << (24 - Q16_16_FRACTIONAL_BITS);
}

/* --- Conversions Between Q1.15 and Q16.16 --- */
/*
   Q1.15 is a 16-bit fixed-point format with 15 fractional bits.
   To convert Q1.15 to Q16.16, we first extend to 32 bits and then shift left
   by (16-15)=1 bit.
*/
static inline q16_16_t q16_16_from_q1_15(int16_t q1_15) {
    return ((q16_16_t)q1_15) << (Q16_16_FRACTIONAL_BITS - 15);
}

/* Convert Q16.16 to Q1.15 by shifting right by 1 bit.
   (Note: This conversion may involve rounding/truncation.)
*/
static inline int16_t q1_15_from_q16_16(q16_16_t x) {
    return (int16_t)(x >> (Q16_16_FRACTIONAL_BITS - 15));
}

/*
 * Q16.16 Addition with Saturation.
 * The operation is performed in 64-bit to detect overflow.
 */
static inline q16_16_t q16_16_add(q16_16_t a, q16_16_t b) {
    int64_t sum = (int64_t)a + (int64_t)b;
    if(sum > Q16_16_MAX)
        return Q16_16_MAX;
    if(sum < Q16_16_MIN)
        return Q16_16_MIN;
    return (q16_16_t)sum;
}

/*
 * Q16.16 Subtraction with Saturation.
 */
static inline q16_16_t q16_16_sub(q16_16_t a, q16_16_t b) {
    int64_t diff = (int64_t)a - (int64_t)b;
    if(diff > Q16_16_MAX)
        return Q16_16_MAX;
    if(diff < Q16_16_MIN)
        return Q16_16_MIN;
    return (q16_16_t)diff;
}

/*
 * Q16.16 Multiplication with Saturation.
 * Multiply two Q16.16 numbers. The 64-bit product is shifted right by 16 bits.
 */
static inline q16_16_t q16_16_mul(q16_16_t a, q16_16_t b) {
    int64_t prod = (int64_t)a * (int64_t)b;
    prod = prod >> Q16_16_FRACTIONAL_BITS;
    if(prod > Q16_16_MAX)
        return Q16_16_MAX;
    if(prod < Q16_16_MIN)
        return Q16_16_MIN;
    return (q16_16_t)prod;
}

/*
 * Q16.16 Division with Saturation.
 * Divides a Q16.16 number by another.
 * The numerator is shifted left by 16 bits to maintain precision.
 * Division by zero is handled by returning Q16_16_MAX (if numerator is nonnegative)
 * or Q16_16_MIN (if numerator is negative).
 */
static inline q16_16_t q16_16_div(q16_16_t a, q16_16_t b) {
    if(b == 0)
        return (a >= 0) ? Q16_16_MAX : Q16_16_MIN;
    int64_t res = (((int64_t)a) << Q16_16_FRACTIONAL_BITS) / b;
    if(res > Q16_16_MAX)
        return Q16_16_MAX;
    if(res < Q16_16_MIN)
        return Q16_16_MIN;
    return (q16_16_t)res;
}

/*
 * Q16.16 Absolute Value with Saturation.
 * If x is Q16_16_MIN (which cannot be negated in two's complement), return Q16_16_MAX.
 */
static inline q16_16_t q16_16_abs(q16_16_t x) {
    if(x == Q16_16_MIN)
        return Q16_16_MAX;
    return (x < 0) ? -x : x;
}

static inline int q16_16_is_saturated(q16_16_t x) {
    return (x == Q16_16_MAX || x == Q16_16_MIN);
}


/*
 * q16_16_exp:
 * Computes e^x for a Q16.16 number x.
 * Uses range reduction: x = n * ln2 + r, with r in [0, ln2),
 * so that e^x = 2^n * e^r.
 *
 * e^r is approximated by a 7-term Taylor series:
 *    e^r ≈ 1 + r + r^2/2! + r^3/3! + ... + r^5/5!
 *
 * All series computations are done in 64-bit arithmetic,
 * with Q16_16_ONE (65536) representing 1.0.
 *
 * For saturation, if x ≥ 10.397 (≈681360 in Q16.16) the function returns Q16_16_MAX,
 * and if x < -16.0 (i.e. below -1048576) it returns 0.
 */
#define Q16_16_EXP_MAX_ARG ((q16_16_t)681360)  // 10.397 * 65536, approximate
#define Q16_16_EXP_MIN_ARG ((q16_16_t)(-16 * Q16_16_ONE)) // -16.0 in Q16.16
static inline q16_16_t q16_16_exp(q16_16_t x) {
    const q16_16_t max_arg = Q16_16_EXP_MAX_ARG;
    const q16_16_t min_arg = Q16_16_EXP_MIN_ARG;
    if (x >= max_arg)
        return Q16_16_MAX;
    if (x < min_arg)
        return 0;

    // Use the precomputed ln2 constant.
    const q16_16_t ln2_fixed = Q16_16_LN2_CONST;  // ~45426

    // Range reduction: write x = n * ln2 + r.
    int64_t x64 = x;
    int n = (int)(x64 / ln2_fixed);
    if (x64 < 0 && (x64 % ln2_fixed) != 0)
        n--;  // adjust for floor for negative x

    // Compute n * ln2 in Q16.16.
    q16_16_t n_ln2 = q16_16_mul(q16_16_from_int(n), ln2_fixed);
    // Compute remainder: r = x - n*ln2.
    q16_16_t r = q16_16_sub(x, n_ln2);

    // Compute e^r using a 7-term Taylor series.
    // Let scale = Q16_16_ONE (65536) represent 1.0.
    int64_t term = Q16_16_ONE;  // term_0 = 1.0
    int64_t sum  = Q16_16_ONE;  // initialize series sum with 1.0
    for (int i = 1; i <= 5; i++) {  // Using 5 terms in the series
        term = (term * r) >> 16;  // Q16_16_FRACTIONAL_BITS = 16
        term = term / i;
        sum += term;
    }

    // Multiply the series result by 2^n.
    int64_t result = sum;
    if (n < 0) {
        for (int i = 0; i < -n; i++)
            result >>= 1;
    } else {
        for (int i = 0; i < n; i++) {
            result <<= 1;
            if (result > Q16_16_MAX) { result = Q16_16_MAX; break; }
        }
    }
    if (result > Q16_16_MAX)
        result = Q16_16_MAX;
    if (result < Q16_16_MIN)
        result = Q16_16_MIN;
    return (q16_16_t) result;
}

/*
 * q16_16_log:
 * Computes the natural logarithm (ln) for a Q16.16 number x > 0.
 * Q16.16 uses 32 bits with 16 fractional bits (so 1.0 = 65536).
 *
 * First, x is normalized by writing it as:
 *
 *      x = m * 2^n,    with m in [Q16_16_ONE, 2*Q16_16_ONE)
 *
 * (Q16_16_ONE is defined as 1 << 16, i.e. 65536.)
 *
 * Then, letting t = m - Q16_16_ONE, we approximate ln(1+t)
 * via an alternating series:
 *
 *      ln(1+t) ≈ t - t^2/2 + t^3/3 - t^4/4 + ...
 *
 * In this revised version, we use terms for i = 2 up through 9 (i.e. 8 terms after the initial t)
 * and we use rounding in each multiplication and division step.
 *
 * Finally, ln(x) is computed as:
 *
 *      ln(x) = ln(m) + n * ln2,
 *
 * where ln2 is approximated in Q16.16 (ln2 ≈ 0.693147, represented as ~45426).
 *
 * All intermediate calculations are done in 64-bit arithmetic, and the final
 * result is saturated to the Q16.16 range if necessary.
 */
static inline q16_16_t _q16_16_log_base(q16_16_t x) {
    if (x <= 0)
        return Q16_16_MIN;  // log undefined for nonpositive x

    int n = 0;
    // Use 64-bit integer for normalization.
    // If x equals Q16_16_MAX (which is 0x7FFFFFFF, but note that the ideal 1.0 is 65536),
    // treat it as the ideal 1.0 (65536) for normalization purposes.
    int64_t m = (x == Q16_16_MAX) ? (1LL << Q16_16_FRACTIONAL_BITS) : (int64_t)x;

    // Normalize m so that m is in [65536, 131072)
    while (m < (1LL << Q16_16_FRACTIONAL_BITS)) {
        m *= 2;
        n--;
    }
    while (m >= ((int64_t)1 << (Q16_16_FRACTIONAL_BITS + 1))) {
        m /= 2;
        n++;
    }

    // Let t = m - 65536. (t represents m - 1.0 in Q16.16.)
    int64_t t = m - (1LL << Q16_16_FRACTIONAL_BITS);

    // Compute ln(1+t) using an alternating series.
    // We'll compute terms for i = 2 through 9 (8 terms) for improved accuracy.
    int64_t term = t;  // first term is t
    int64_t sum = t;   // initialize sum with t
    for (int i = 2; i <= 5; i++) {
        // Multiply the previous term by t in Q16.16 arithmetic.
        // Use rounding: add half the divisor before shifting.
        int64_t prod = term * t;
        prod = (prod + (1LL << (Q16_16_FRACTIONAL_BITS - 1))) >> Q16_16_FRACTIONAL_BITS;
        // Now perform division by i with rounding.
        term = (prod + i/2) / i;
        if (i % 2 == 0)
            sum -= term;
        else
            sum += term;
    }

    // ln2 in Q16.16: ideally, ln2 ≈ 0.693147, which is approximately 45426 in Q16.16.
    q16_16_t ln2_fixed = Q16_16_LN2_CONST;
    int64_t n_ln2 = n * (int64_t)ln2_fixed;

    int64_t ln_x = sum + n_ln2;

    // Saturate the final result to the Q16.16 range.
    if (ln_x > Q16_16_MAX)
        ln_x = Q16_16_MAX;
    if (ln_x < Q16_16_MIN)
        ln_x = Q16_16_MIN;

    return (q16_16_t)ln_x;
}

/*
 * q16_16_log:
 * Computes a refined approximation of ln(x) for a Q16.16 number x > 0.
 * It first obtains an initial approximation y0 using the series-based
 * q16_16_log function. Then one Newton–Raphson iteration is applied:
 *
 *    y_new = y_old - (exp(y_old) - x) / exp(y_old)
 *
 * This iteration tends to reduce the error in the initial approximation.
 *
 * Note: All arithmetic uses our Q16.16 functions.
 */
static inline q16_16_t q16_16_log(q16_16_t x) {
    // Initial approximation using the series-based function.
    q16_16_t y = _q16_16_log_base(x);

    // Compute exp(y) using our Q16.16 exp function.
    q16_16_t exp_y = q16_16_exp(y);

    // Compute the error: error = exp(y) - x.
    q16_16_t error = q16_16_sub(exp_y, x);

    // Compute the correction: correction = error / exp(y)
    q16_16_t correction = q16_16_div(error, exp_y);

    // Refine: y_new = y - correction.
    q16_16_t y_new = q16_16_sub(y, correction);

    return y_new;
}

// Returns the integer part of a Q16.16 number.
static inline int32_t q16_16_get_int(q16_16_t x) {
    return x >> Q16_16_FRACTIONAL_BITS;
}

// Returns the fractional part of a Q16.16 number as an unsigned 16-bit value.
static inline uint16_t q16_16_get_frac(q16_16_t x) {
    uint16_t mask = (1 << Q16_16_FRACTIONAL_BITS) - 1;
    return (x >= 0) ? ((uint16_t)x & mask) : (((uint16_t)(-x)) & mask);
}



/* Q6.10 Fixed–Point Definitions */
#define Q6_10_FRACTIONAL_BITS 10
#define Q6_10_ONE      ((q6_10_t)(1 << Q6_10_FRACTIONAL_BITS))            // 1.0 in Q6.10 (1024)
#define Q6_10_NEG_ONE  ((q6_10_t)(- (1 << Q6_10_FRACTIONAL_BITS)))        // -1.0 in Q6.10 (-1024)

/*
 * For a 16‐bit signed integer, the maximum representable value is 0x7FFF
 * (32767) and the minimum is 0x8000 (which is interpreted as –32768 in two’s complement).
 * In Q6.10, these correspond roughly to +31.9990 and –32.0.
 */
#define Q6_10_MAX ((q6_10_t)0x7FFF)
#define Q6_10_MIN ((q6_10_t)0x8000)


typedef int16_t q6_10_t;

/*--------------------*/
/* Int <-> Q6.10      */
/*--------------------*/

/* Convert an int to Q6.10 by multiplying by 1024 */
static inline q6_10_t q6_10_from_int(int x) {
    return (q6_10_t)(x * Q6_10_ONE);
}

/* Convert a Q6.10 number to an int by shifting right 10 bits */
static inline int q6_10_to_int(q6_10_t x) {
    return x >> Q6_10_FRACTIONAL_BITS;
}

/*--------------------*/
/* Float <-> Q6.10    */
/*--------------------*/

/* Convert a float to Q6.10 */
static inline q6_10_t q6_10_from_float(float x) {
    return (q6_10_t)(x * Q6_10_ONE);
}

/* Convert a Q6.10 number to float */
static inline float q6_10_to_float(q6_10_t x) {
    return (float)x / Q6_10_ONE;
}

/*-------------------------------*/
/* Q8.24 <-> Q6.10 Conversions    */
/*-------------------------------*/
/*
   Q8.24 is typically represented as a 32-bit integer with 24 fractional bits.
   To convert from Q8.24 to Q6.10, we shift right by (24 - 10) = 14 bits.
*/
static inline q6_10_t q6_10_from_q8_24(int32_t x) {
    return (q6_10_t)(x >> (24 - Q6_10_FRACTIONAL_BITS));
}

/* Convert from Q6.10 to Q8.24 by shifting left by 14 bits */
static inline int32_t q8_24_from_q6_10(q6_10_t x) {
    return ((int32_t)x) << (24 - Q6_10_FRACTIONAL_BITS);
}

/*-------------------------------*/
/* Q1.15 <-> Q6.10 Conversions    */
/*-------------------------------*/
/*
   Q1.15 is a 16-bit format with 15 fractional bits.
   To convert from Q1.15 to Q6.10, shift right by (15 - 10) = 5 bits.
*/
static inline q6_10_t q6_10_from_q1_15(int16_t x) {
    return (q6_10_t)(x >> (15 - Q6_10_FRACTIONAL_BITS));
}

/* Convert from Q6.10 to Q1.15 by shifting left by 5 bits */
static inline int16_t q1_15_from_q6_10(q6_10_t x) {
    return (int16_t)(x << (15 - Q6_10_FRACTIONAL_BITS));
}

/*-------------------------------*/
/* Q16.16 <-> Q6.10 Conversions   */
/*-------------------------------*/
/*
   Q16.16 is a 32-bit format with 16 fractional bits.
   To convert from Q16.16 to Q6.10, shift right by (16 - 10) = 6 bits.
*/
static inline q6_10_t q6_10_from_q16_16(int32_t x) {
    return (q6_10_t)(x >> (16 - Q6_10_FRACTIONAL_BITS));
}

/* Convert from Q6.10 to Q16.16 by shifting left by 6 bits */
static inline int32_t q16_16_from_q6_10(q6_10_t x) {
    return ((int32_t)x) << (16 - Q6_10_FRACTIONAL_BITS);
}


/*
 * q6_10_add:
 * Adds two Q6.10 numbers using 32-bit arithmetic to check for overflow,
 * then saturates the result if necessary.
 */
static inline q6_10_t q6_10_add(q6_10_t a, q6_10_t b) {
    int32_t sum = (int32_t)a + (int32_t)b;
    if (sum > Q6_10_MAX)
        return Q6_10_MAX;
    if (sum < Q6_10_MIN)
        return Q6_10_MIN;
    return (q6_10_t)sum;
}

/*
 * q6_10_sub:
 * Subtracts b from a using 32-bit arithmetic and saturates the result.
 */
static inline q6_10_t q6_10_sub(q6_10_t a, q6_10_t b) {
    int32_t diff = (int32_t)a - (int32_t)b;
    if (diff > Q6_10_MAX)
        return Q6_10_MAX;
    if (diff < Q6_10_MIN)
        return Q6_10_MIN;
    return (q6_10_t)diff;
}

/*
 * q6_10_mul:
 * Multiplies two Q6.10 numbers. The 32-bit product is shifted right by 10 bits,
 * then saturated to the Q6.10 range.
 */
static inline q6_10_t q6_10_mul(q6_10_t a, q6_10_t b) {
    int32_t prod = (int32_t)a * (int32_t)b;
    prod = prod >> Q6_10_FRACTIONAL_BITS;
    if (prod > Q6_10_MAX)
        return Q6_10_MAX;
    if (prod < Q6_10_MIN)
        return Q6_10_MIN;
    return (q6_10_t)prod;
}

/*
 * q6_10_div:
 * Divides Q6.10 number a by Q6.10 number b.
 * The numerator is shifted left by 10 bits to maintain precision.
 * Division by zero returns Q6_10_MAX if a is nonnegative, or Q6_10_MIN otherwise.
 */
static inline q6_10_t q6_10_div(q6_10_t a, q6_10_t b) {
    if (b == 0)
        return (a >= 0) ? Q6_10_MAX : Q6_10_MIN;
    int32_t res = (((int32_t)a) << Q6_10_FRACTIONAL_BITS) / b;
    if (res > Q6_10_MAX)
        return Q6_10_MAX;
    if (res < Q6_10_MIN)
        return Q6_10_MIN;
    return (q6_10_t)res;
}

/*
 * q6_10_abs:
 * Returns the absolute value of a Q6.10 number.
 * Since Q6.10_MIN (–32768) cannot be negated in two’s complement,
 * we return Q6_10_MAX in that case.
 */
static inline q6_10_t q6_10_abs(q6_10_t x) {
    if (x == Q6_10_MIN)
        return Q6_10_MAX;
    return (x < 0) ? -x : x;
}

/*
 * q6_10_is_saturated:
 * Returns 1 if the Q6.10 number is saturated (i.e. equals Q6_10_MAX or Q6_10_MIN), 0 otherwise.
 */
static inline int q6_10_is_saturated(q6_10_t x) {
    return (x == Q6_10_MAX || x == Q6_10_MIN);
}

/*
 * q6_10_exp:
 * Computes exp(x) for a Q6.10 number x.
 * Uses range reduction: x = n * ln2 + r, with r in [0, ln2)
 * so that exp(x) = 2^n * exp(r).
 * Approximates exp(r) with a 5-term Taylor series:
 *    exp(r) ≈ 1 + r + r^2/2 + r^3/6 + r^4/24.
 *
 * Calculations use 64-bit arithmetic with Q6_10_ONE (1024) as 1.0.
 * If x ≥ 3.5 (real) we saturate to Q6_10_MAX, and if x < -10.0 we return 0.
 */
#define Q6_10_EXP_MAX_ARG 3584    // Represents 3.5 in Q6.10.
#define Q6_10_EXP_MIN_ARG (-10240) // Represents -10.0 in Q6.10.
#define Q6_10_LN2_CONST 709       // Represents ln2 (0.693147) in Q6.10.
static inline q6_10_t q6_10_exp(q6_10_t x) {
    // Use precomputed constants:
    q6_10_t max_arg = Q6_10_EXP_MAX_ARG;
    q6_10_t min_arg = Q6_10_EXP_MIN_ARG;
    if (x >= max_arg)
        return Q6_10_MAX;
    if (x < min_arg)
        return 0;

    // Use precomputed ln2 constant.
    q6_10_t ln2 = Q6_10_LN2_CONST;
    int32_t x32 = x;
    int n = x32 / ln2;
    if (x32 % ln2 != 0)
        n--;  // floor for negative x
    int32_t n_ln2 = n * ln2;
    q6_10_t r = (q6_10_t)(x32 - n_ln2);

    // Compute exp(r) using a 5-term Taylor series:
    int64_t term = Q6_10_ONE;  // 1.0 in Q6.10 (1024)
    int64_t sum  = Q6_10_ONE;
    for (int i = 1; i <= 4; i++) {
        term = (term * r) >> Q6_10_FRACTIONAL_BITS;
        term = term / i;
        sum += term;
    }

    int64_t result = sum;
    if (n < 0) {
        for (int i = 0; i < -n; i++)
            result >>= 1;
    } else {
        for (int i = 0; i < n; i++) {
            result <<= 1;
            if (result > Q6_10_MAX) { result = Q6_10_MAX; break; }
        }
    }
    if (result > Q6_10_MAX)
        result = Q6_10_MAX;
    if (result < 0)
        result = 0;
    return (q6_10_t)result;
}

/*
 * q6_10_log:
 * Computes ln(x) for a Q6.10 number x > 0.
 * First, normalize x by writing it as:
 *    x = m * 2^n,  with m in [Q6_10_ONE, 2*Q6_10_ONE)  (i.e. [1024, 2048))
 * Then set t = m - Q6_10_ONE. (t/1024 approximates m - 1.0.)
 * Approximate ln(1+u) with u = t/1024 using a 5-term series:
 *    ln(1+u) ≈ u - u^2/2 + u^3/3 - u^4/4.
 * Multiply the result by 1024 to get Q6_10 units.
 * Finally, ln(x) = [ln(m) + n*ln2] (with ln2 ≈ 0.693147, i.e. ~709 in Q6_10).
 */
#define Q6_10_LN2_CONST 709 /* Define ln(2) in Q6.10 as a macro. 0.693147 * 1024 ≈ 709. */
static inline q6_10_t q6_10_log(q6_10_t x) {
    if (x <= 0)
        return Q6_10_MIN;  // undefined for nonpositive x

    int n = 0;
    int32_t m = x;
    while (m < Q6_10_ONE) {
        m *= 2;
        n--;
    }
    while (m >= (Q6_10_ONE << 1)) {
        m /= 2;
        n++;
    }
    int32_t t = m - Q6_10_ONE;  // t in Q6.10 units.

    // Compute series for ln(1+u) with u = t/1024.
    // 5-term series: u - u^2/2 + u^3/3 - u^4/4.
    // Multiply each term by 1024 to have result in Q6.10 units.
    int64_t term0 = t;  // represents u*1024
    int64_t term1 = ((int64_t)t * t) / (2 * Q6_10_ONE);
    int64_t term2 = ((int64_t)t * t * t) / (3 * ((int64_t)Q6_10_ONE * Q6_10_ONE));
    int64_t term3 = ((int64_t)t * t * t * t) / (4 * ((int64_t)Q6_10_ONE * Q6_10_ONE * Q6_10_ONE));
    int64_t series = term0 - term1 + term2 - term3;
    // series approximates ln(m)*1024 in Q6.10 units.
    // ln2 in Q6.10: 0.693147 * 1024 ≈ 709.
    int32_t ln2_val = Q6_10_LN2_CONST;
    int64_t ln_x = series + n * ln2_val;
    if (ln_x > Q6_10_MAX)
        ln_x = Q6_10_MAX;
    if (ln_x < Q6_10_MIN)
        ln_x = Q6_10_MIN;
    return (q6_10_t)ln_x;
}

// Returns the integer part of a Q6.10 number.
static inline int16_t q6_10_get_int(q6_10_t x) {
    return x >> Q6_10_FRACTIONAL_BITS;
}

// Returns the fractional part of a Q6.10 number as an unsigned 16-bit value.
static inline uint16_t q6_10_get_frac(q6_10_t x) {
    uint16_t mask = (1 << Q6_10_FRACTIONAL_BITS) - 1;
    return (x >= 0) ? ((uint16_t)x & mask) : (((uint16_t)(-x)) & mask);
}


#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
