
#ifndef FIXP_Q6_10_H_
#define FIXP_Q6_10_H_

#include <stdint.h>
#include <limits.h>
#include <stdbool.h>
#include <math.h>

#include "fixp_base.h"



/////////////////////////////////////////////////////////////////
///                      Q6.10 FUNCTIONS                      /// {{{1
/////////////////////////////////////////////////////////////////

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

/* Convert a Q6.10 number to float */
#define Q6_10_FLOAT_RECIP    (1.0f / (float)Q6_10_ONE)                  // Reciprocal constant
static inline float q6_10_to_float(q6_10_t x) {
    return (float)x * Q6_10_FLOAT_RECIP;
}

/* Convert a Q6.10 number to double */
#define Q6_10_DOUBLE_RECIP    (1.0 / (double)Q6_10_ONE)                  // Reciprocal constant
static inline double q6_10_to_double(q6_10_t x) {
    return (double)x * Q6_10_FLOAT_RECIP;
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

#define Q6_10_LN2_CONST 709 /* Define ln(2) in Q6.10 as a macro. 0.693147 * 1024 ≈ 709. */
#define Q6_10_EXP_MAX_ARG 3584     // Represents 3.5 in Q6.10.
#define Q6_10_EXP_MIN_ARG (-10240)  // Represents -10.0 in Q6.10.

/*
 * q6_10_exp:
 * Computes exp(x) for a Q6.10 number x using range reduction and an unrolled
 * Taylor series for exp(r). The algorithm uses:
 *
 *  1. Early saturation: if x ≥ Q6_10_EXP_MAX_ARG, return Q6_10_MAX;
 *     if x < Q6_10_EXP_MIN_ARG, return 0.
 *
 *  2. Range reduction: express x = n * ln2 + r with r in [0, ln2) so that:
 *         exp(x) = 2^n * exp(r)
 *
 *  3. Taylor series for exp(r) is approximated as:
 *         exp(r) ≈ 1 + r + r²/2 + r³/6 + r⁴/24
 *
 *     The division by 2 is done by a right-shift by 1.
 *     The division by 6 is replaced by a multiplication by RECIP_6 (in Q32) and a >>32.
 *     The division by 24 is replaced by a multiplication by RECIP_24 (in Q32) and a >>32.
 *
 *  4. Finally, the result is scaled by 2^n using shifts (with overflow checks) and
 *     saturated to the Q6.10 range.
 */
static inline q6_10_t q6_10_exp(q6_10_t x) {
    // Early saturation.
    if (x >= Q6_10_EXP_MAX_ARG)
        return Q6_10_MAX;
    if (x < Q6_10_EXP_MIN_ARG)
        return 0;

    // Range reduction:
    //   Write x = n * ln2 + r, where ln2 is given in Q6.10.
    q6_10_t ln2 = Q6_10_LN2_CONST;  // 709 in Q6.10
    int32_t x32 = x;
    int n = x32 / ln2;
    if (x32 % ln2 != 0)
        n--;  // For negative x, adjust to floor division.
    int32_t n_ln2 = n * ln2;
    q6_10_t r = (q6_10_t)(x32 - n_ln2);

    // Compute exp(r) using an unrolled 5-term Taylor series.
    // All arithmetic is performed in 64-bit integers with Q6.10_ONE (1024) as 1.0.
    int64_t one   = Q6_10_ONE;  // 1024
    int64_t term0 = one;        // 1.0
    int64_t term1 = r;          // r

    // Compute r^2 in Q6.10.
    int64_t r2 = (((int64_t)r * r) >> Q6_10_FRACTIONAL_BITS);
    // term2 = r^2/2; division by 2 via right shift.
    int64_t term2 = r2 >> 1;

    // Compute r^3 in Q6.10.
    int64_t r3 = (r2 * r) >> Q6_10_FRACTIONAL_BITS;
    // Use RECIP_6 to compute r^3/6 without a division.
    const int64_t RECIP_6 = 715827882LL;  // floor((1<<32)/6)
    int64_t term3 = (r3 * RECIP_6) >> 32;

    // Compute r^4 in Q6.10.
    int64_t r4 = (r3 * r) >> Q6_10_FRACTIONAL_BITS;
    // Use RECIP_24 to compute r^4/24.
    const int64_t RECIP_24 = 178956970LL;  // floor((1<<32)/24)
    int64_t term4 = (r4 * RECIP_24) >> 32;

    int64_t series = term0 + term1 + term2 + term3 + term4;

    // Scale the series result by 2^n.
    int64_t result = series;
    if (n < 0) {
        result = result >> (-n);
    } else {
        if (n >= 32) {  // A left shift by 32 or more would overflow.
            result = Q6_10_MAX;
        } else {
            result = result << n;
            if (result > Q6_10_MAX)
                result = Q6_10_MAX;
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
