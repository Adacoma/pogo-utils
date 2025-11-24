
#ifndef FIXP_Q8_24_H_
#define FIXP_Q8_24_H_

#include <stdint.h>
#include <limits.h>
#include <stdbool.h>
#include <math.h>

#include "fixp_base.h"



/////////////////////////////////////////////////////////////////
///                      Q8.24 FUNCTIONS                      /// {{{1
/////////////////////////////////////////////////////////////////

static inline q8_24_t q8_24_from_float(float x) {
    if (x >= 128.0f)  return Q8_24_MAX;
    if (x <  -128.0f) return Q8_24_MIN;
    float y = x * 16777216.0f + (x >= 0 ? 0.5f : -0.5f);
    return (q8_24_t)y;
}


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
//#define q8_24_from_float(x) Q8_24_FROM_FLOAT(x) 
//static inline q8_24_t q8_24_from_float(float x) {
//    if (x >= 128.0f)
//        return Q8_24_MAX;
//    if (x < -128.0f)
//        return Q8_24_MIN;
//    return (q8_24_t)(x * Q8_24_ONE);
//}

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

/*
 * q8_24_approximate_reciprocal:
 *
 * Computes an approximate reciprocal 1/b for a Q8.24 number b.
 *
 * The method:
 * 1. Handles negative inputs and asserts if b is zero.
 * 2. Normalizes b to lie in the range [0.5, 1.0] in Q8.24, while tracking a shift factor.
 * 3. Uses a linear approximation:
 *        r0 = (48/17) - (32/17) * norm
 *    (with constants scaled to Q8.24)
 * 4. Applies three iterations of the Newton–Raphson update:
 *        rₙ₊₁ = rₙ * (2 - norm * rₙ)
 * 5. Adjusts for the normalization: if b was scaled so that b = norm * 2^(shift),
 *    then 1/b = (1/norm) * 2^(-shift).
 * 6. Finally, saturates the result to Q8_24_MIN (-128.0) and Q8_24_MAX (~127.99999994).
 */
static inline q8_24_t q8_24_approximate_reciprocal(q8_24_t b) {
    // b must not be zero.
    if (b == 0) {
        return Q8_24_MAX;
    }

    // Handle negative numbers.
    int sign = 1;
    if (b < 0) {
        sign = -1;
        b = -b;
    }

    // Early saturation: In Q8.24, 0.0078125 (which is 1/128) is represented as 131072.
    // For any b <= 131072, the reciprocal 1/b would be >= 128,
    // which exceeds the representable maximum (~127.99999994).
    if (b <= (1 << 17)) {  // (1 << 17) == 131072
        return (sign > 0 ? Q8_24_MAX : Q8_24_MIN);
    }

    // Normalize b: we want norm in [Q8_24_ONE/2, Q8_24_ONE] (i.e., roughly [0.5, 1.0]).
    int shift = 0;
    q8_24_t norm = b;
    while (norm < (Q8_24_ONE >> 1)) {
        norm <<= 1;
        shift--;  // b was too small; will later multiply reciprocal by 2^(-shift)
    }
    while (norm > Q8_24_ONE) {
        norm >>= 1;
        shift++;  // b was too large
    }

    // Use a linear approximation to get a better initial guess for 1/norm.
    // For x in [0.5,1], a good approximation is:
    //      r0 = (48/17) - (32/17)*x
    // with constants represented in Q8.24.
    const int32_t K = (int32_t)((48.0 / 17.0) * Q8_24_ONE + 0.5);
    const int32_t L = (int32_t)((32.0 / 17.0) * Q8_24_ONE + 0.5);
    int32_t r = K - (int32_t)(((int64_t)L * norm) >> Q8_24_FRACTIONAL_BITS);

    // Refine the initial guess with three iterations of Newton–Raphson:
    //     r = r * (2 - norm * r)
    for (int i = 0; i < 3; i++) {
        int64_t prod = ((int64_t)norm * r) >> Q8_24_FRACTIONAL_BITS;
        int64_t diff = (2LL * Q8_24_ONE) - prod;
        r = (int32_t)(((int64_t)r * diff) >> Q8_24_FRACTIONAL_BITS);
    }

    // Adjust for the normalization:
    // If b was scaled as b = norm * 2^(shift), then 1/b = (1/norm) * 2^(-shift)
    if (shift > 0) {
        r >>= shift;
    } else if (shift < 0) {
        // Before left-shifting, check for potential overflow.
        if (r > (Q8_24_MAX >> (-shift))) {
            r = Q8_24_MAX;
        } else {
            r <<= -shift;
        }
    }

    // Apply the sign.
    int32_t result = sign * r;

    // Saturate the result to the representable Q8.24 range.
    if (result > Q8_24_MAX)
        result = Q8_24_MAX;
    else if (result < Q8_24_MIN)
        result = Q8_24_MIN;

    return (q8_24_t)result;
}


/* Division with saturation.
 * If dividing by zero, the function returns Q8_24_MAX for a non-negative numerator
 * and Q8_24_MIN for a negative numerator.
 */
static inline q8_24_t q8_24_div(q8_24_t a, q8_24_t b) {
    if (b == 0) {
        return (a >= 0) ? Q8_24_MAX : Q8_24_MIN;
    }

    // Division by using multiplication of 1/b
    q8_24_t const reciprocal = q8_24_approximate_reciprocal(b);        // Compute reciprocal of b in Q8.24
    int64_t res = ((int64_t)a * reciprocal) >> Q8_24_FRACTIONAL_BITS;  // Multiply a by the reciprocal
    if (res > Q8_24_MAX)
        return Q8_24_MAX;
    if (res < Q8_24_MIN)
        return Q8_24_MIN;
    return (q8_24_t)res;

    // Slow division
//    int64_t res = ((int64_t)a << Q8_24_FRACTIONAL_BITS) / b;
//    if (res > Q8_24_MAX)
//        return Q8_24_MAX;
//    if (res < Q8_24_MIN)
//        return Q8_24_MIN;
//    return (q8_24_t)res;
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
//static inline q8_24_t q8_24_exp(q8_24_t x) {
//    const q8_24_t max_arg = Q8_24_EXP_MAX_ARG_CONST;
//    const q8_24_t min_arg = Q8_24_EXP_MIN_ARG_CONST;
//    if (x > max_arg)
//        return Q8_24_MAX;
//    if (x < min_arg)
//        return 0;
//
//    // Use precomputed ln2.
//    const q8_24_t ln2_fixed = Q8_24_LN2_CONST;
//
//    // Range reduction: find integer n such that x = n * ln2 + r, with r in [0, ln2).
//    int64_t x64 = x;
//    int n = (int)(x64 / ln2_fixed);
//    if (x64 < 0 && (x64 % ln2_fixed) != 0)
//        n--;
//
//    q8_24_t n_ln2 = n * ln2_fixed;  // Multiply in integer arithmetic.
//    q8_24_t r = q8_24_sub(x, n_ln2);
//
//    // Compute exp(r) using a Taylor series:
//    // exp(r) ≈ 1 + r + r^2/2! + r^3/3! + r^4/4! + r^5/5!
//    int64_t term = Q8_24_ONE;  // term0 = 1.0 in Q8_24
//    int64_t sum  = Q8_24_ONE;  // initialize sum with 1.0
//    for (int i = 1; i <= 5; i++) {
//        term = (term * r) >> 24;  // Q8_24_FRACTIONAL_BITS = 24
//        term = term / i;
//        sum += term;
//    }
//
//    // Scale the series result by 2^n.
//    int64_t result = sum;
//    if (n >= 0) {
//        for (int i = 0; i < n; i++) {
//            result = result << 1;
//            if (result > Q8_24_MAX) { result = Q8_24_MAX; break; }
//        }
//    } else {
//        for (int i = 0; i < -n; i++) {
//            result = result >> 1;
//        }
//    }
//    if (result > Q8_24_MAX)
//        result = Q8_24_MAX;
//    if (result < Q8_24_MIN)
//        result = Q8_24_MIN;
//    return (q8_24_t) result;
//}

/*
 * q8_24_exp:
 * Computes e^x for a Q8.24 number x using range reduction and a 7‑term Taylor series expansion,
 * but avoids slow 64‑bit divisions by replacing them with multiplications by precomputed reciprocal constants.
 *
 * The algorithm is:
 * 1. Early saturation: if x > Q8_24_EXP_MAX_ARG_CONST, return Q8_24_MAX; if x < Q8_24_EXP_MIN_ARG_CONST, return 0.
 *
 * 2. Range reduction: Express x = n * ln2 + r, with r in [0, ln2).
 *    (n is computed via integer division in 32 bits.)
 *
 * 3. Compute e^r using the unrolled Taylor series:
 *      e^r ≈ 1 + r + r^2/2! + r^3/3! + r^4/4! + r^5/5!
 *
 *    Here, instead of dividing by 2,6,24,120, we use:
 *      - r^2/2 is computed as (r^2 >> 1)
 *      - r^3/6  is computed as (r^3 * RECIP_6)  >> 32, where RECIP_6  ≈ (1<<32)/6
 *      - r^4/24 is computed as (r^4 * RECIP_24) >> 32, where RECIP_24 ≈ (1<<32)/24
 *      - r^5/120 is computed as (r^5 * RECIP_120) >> 32, where RECIP_120 ≈ (1<<32)/120
 *
 * 4. Scale the result by 2^n via shifting.
 *
 * 5. Saturate the final result to the Q8.24 range.
 */
static inline q8_24_t q8_24_exp(q8_24_t x) {
    // Early saturation.
    if (x > Q8_24_EXP_MAX_ARG_CONST)
        return Q8_24_MAX;
    if (x < Q8_24_EXP_MIN_ARG_CONST)
        return 0;

    // Precomputed ln2 in Q8.24.
    const q8_24_t ln2 = Q8_24_LN2_CONST;

    // Range reduction: find n such that x = n*ln2 + r with r in [0, ln2).
    int n = (int)(x / ln2);
    if (x < 0 && (x % ln2) != 0)
        n--;
    q8_24_t r = x - n * ln2;


    // Compute Taylor series for e^r.
    // All intermediate terms are kept in 64-bit arithmetic (with Q8.24 scaling).
    int64_t one   = Q8_24_ONE;     // 1.0 in Q8.24.
    int64_t term1 = r;             // r.
    int64_t term2 = (((int64_t)r * r) >> 24);  // r^2 in Q8.24.
    int64_t term3 = ((term2 * r) >> 24);         // r^3 in Q8.24.
    int64_t term4 = ((term3 * r) >> 24);         // r^4 in Q8.24.
    int64_t term5 = ((term4 * r) >> 24);         // r^5 in Q8.24.

    // Precomputed reciprocals in Q32 format.
    // (1 << 32) / 6   ≈ 715827883
    // (1 << 32) / 24  ≈ 178956971
    // (1 << 32) / 120 ≈ 35791394
    const int64_t RECIP_6   = 715827883LL;
    const int64_t RECIP_24  = 178956971LL;
    const int64_t RECIP_120 = 35791394LL;

    int64_t series = one
                     + term1
                     + (term2 >> 1)                      // r^2/2! (division by 2 using shift)
                     + ((term3 * RECIP_6) >> 32)           // r^3/3!
                     + ((term4 * RECIP_24) >> 32)          // r^4/4!
                     + ((term5 * RECIP_120) >> 32);        // r^5/5!

    // Scale the series result by 2^n.
    int64_t result = series;
    if (n >= 0) {
        if (n >= 31) { // If n is very high, saturate.
            result = Q8_24_MAX;
        } else {
            result = result << n;
            if (result > Q8_24_MAX)
                result = Q8_24_MAX;
        }
    } else {
        result = result >> (-n);
    }

    // Final saturation.
    if (result > Q8_24_MAX)
        result = Q8_24_MAX;
    if (result < Q8_24_MIN)
        result = Q8_24_MIN;

    return (q8_24_t)result;
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
    // For nonpositive x, return a saturated value.
    if (x <= 0)
        return Q8_24_LN2_CONST * (-100); // or Q8_24_MIN; choose an appropriate saturation.

    // ----- Step 1: Normalize x using bit-level operations -----
    // We want to represent x as:
    //       x = m * 2^n,
    // where m is in [Q8_24_ONE, 2*Q8_24_ONE) (i.e. [1.0, 2.0)).
    // Compute the index of the highest set bit. For positive x:
    int p = 31 - __builtin_clz((unsigned int)x); // floor(log2(x))
    // The constant Q8_24_ONE is 1 << 24. Thus, we want:
    int n = p - Q8_24_FRACTIONAL_BITS;  // n = floor_log2(x) - 24.
    
    // Normalize x: if n >= 0, then m = x >> n; otherwise, m = x << (-n).
    q8_24_t m;
    if (n >= 0)
        m = x >> n;
    else
        m = x << (-n);
    // Now m is in [Q8_24_ONE, 2*Q8_24_ONE).
    
    // ----- Step 2: Compute the series for ln(1+t) -----
    // Let t = m - Q8_24_ONE (so that m = 1 + t in Q8.24).
    q8_24_t t = m - Q8_24_ONE;
    // We approximate ln(1+t) by:
    //    ln(1+t) ≈ t - t^2/2 + t^3/3 - t^4/4 + t^5/5.
    // Perform all multiplications in 64-bit arithmetic.
    int64_t t64 = t;  // t in Q8.24.
    
    // Compute powers of t in Q8.24:
    int64_t t2 = (t64 * t64) >> Q8_24_FRACTIONAL_BITS;      // t^2 in Q8.24.
    int64_t t3 = (t2 * t64) >> Q8_24_FRACTIONAL_BITS;       // t^3 in Q8.24.
    int64_t t4 = (t3 * t64) >> Q8_24_FRACTIONAL_BITS;       // t^4 in Q8.24.
    int64_t t5 = (t4 * t64) >> Q8_24_FRACTIONAL_BITS;       // t^5 in Q8.24.
    
    // Replace divisions by constants:
    // Division by 2: use a right shift by 1.
    int64_t term2 = t2 >> 1;
    // Division by 3: use multiplication by reciprocal RECIP_3 in Q32.
    // (1<<32) / 3 ≈ 1431655765.
    const uint32_t RECIP_3 = 1431655765U;
    int64_t term3 = (t3 * RECIP_3) >> 32;
    // Division by 4: use a right shift by 2.
    int64_t term4 = t4 >> 2;
    // Division by 5: use multiplication by reciprocal RECIP_5 in Q32.
    // (1<<32) / 5 ≈ 858993459.
    const uint32_t RECIP_5 = 858993459U;
    int64_t term5 = (t5 * RECIP_5) >> 32;
    
    // Combine terms with alternating signs:
    // ln(1+t) ≈ t - t^2/2 + t^3/3 - t^4/4 + t^5/5.
    int64_t series = t64 - term2 + term3 - term4 + term5;
    // series is in Q8.24.
    
    // ----- Step 3: Reconstruct ln(x) -----
    // We have:
    //      ln(x) = ln(m) + n * ln2,
    // and we computed ln(m) ≈ series.
    int64_t n_ln2 = (int64_t)n * Q8_24_LN2_CONST;
    int64_t result = series + n_ln2;
    
    return (q8_24_t)result;
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

#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
