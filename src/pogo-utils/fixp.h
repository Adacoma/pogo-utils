
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
//#define Q8_24_LN2_CONST     11629080       // Precomputed: 0.693147 * 16777216
#define Q8_24_INV_LN2       24204406       // 1/ln2 in Q8.24 (≈1.442695 * 2^24)


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
 *
 * For x >= 0, exp(x) is at least 1.0 and we saturate to Q1_15_MAX.
 *
 * For negative x, we use range reduction:
 *      x = n * ln2 + r, with r in [0, ln2),
 * so that exp(x) = 2^n * exp(r).
 *
 * Then exp(r) is approximated by the unrolled Taylor series:
 *      exp(r) ≈ 1 + r + r²/2 + r³/3 + r⁴/4 + r⁵/5
 *
 * To avoid slow 32-bit divisions we:
 *   - Replace division by 2 with a right shift by 1.
 *   - Replace division by 4 with a right shift by 2.
 *   - Replace division by 3 with a multiplication by RECIP_3 (≈ (1<<32)/3)
 *   - Replace division by 5 with a multiplication by RECIP_5 (≈ (1<<32)/5)
 *
 * All intermediate arithmetic is done in 32- or 64-bit integers.
 */
static inline q1_15_t q1_15_exp(q1_15_t x) {
    // For any nonnegative input, exp(x) >= 1.0; saturate to Q1_15_MAX.
    if (x >= 0)
        return Q1_15_MAX;

    // Range reduction: write x = n * ln2 + r, with r in [0, ln2)
    // x and ln2 are in Q1.15. Since x is negative, we compute floor(x/ln2).
    int32_t x32 = (int32_t)x;             // e.g., for x = -1.0, x32 = -32768.
    int32_t L = Q1_15_LN2_CONST;          // ~22713 in Q1.15 units.
    int n = x32 / L;                     // C division truncates toward 0.
    if (x32 % L != 0)
        n--;                           // Adjust to get the mathematical floor for negative x.
    int32_t n_ln2 = n * L;
    int32_t r32 = x32 - n_ln2;           // Remainder r in Q1.15.
    q1_15_t r = (q1_15_t) r32;
    
    // Prepare the constant representing 1.0 in Q1.15.
    int32_t one = Q1_15_ONE;             // 32768

    // Unroll the Taylor series for exp(r):
    // exp(r) ≈ 1 + r + r²/2 + r³/3 + r⁴/4 + r⁵/5
    int32_t sum = one;                   // term0 = 1.0

    // term1 = r
    int32_t term1 = r;
    sum += term1;

    // term2 = (term1 * r) >> 15, then divided by 2 (division by 2 is a right shift by 1)
    int64_t prod = (int64_t)term1 * r;
    int32_t term2 = (int32_t)(prod >> (Q1_15_FRACTIONAL_BITS + 1)); // shift by (15 + 1)
    sum += term2;

    // term3 = (term2 * r) >> 15, then divided by 3.
    prod = (int64_t)term2 * r;
    int32_t term3_intermediate = (int32_t)(prod >> Q1_15_FRACTIONAL_BITS);
    // Precomputed reciprocal for 3 in Q32: RECIP_3 = floor((1<<32)/3) ≈ 1431655765 (0x55555555).
    const uint32_t RECIP_3 = 1431655765U;
    int32_t term3 = (int32_t)(((int64_t)term3_intermediate * RECIP_3) >> 32);
    sum += term3;

    // term4 = (term3 * r) >> 15, then divided by 4 (division by 4 is a right shift by 2).
    prod = (int64_t)term3 * r;
    int32_t term4 = (int32_t)(prod >> (Q1_15_FRACTIONAL_BITS + 2)); // shift by (15 + 2)
    sum += term4;

    // term5 = (term4 * r) >> 15, then divided by 5.
    prod = (int64_t)term4 * r;
    int32_t term5_intermediate = (int32_t)(prod >> Q1_15_FRACTIONAL_BITS);
    // Precomputed reciprocal for 5 in Q32: RECIP_5 = floor((1<<32)/5) ≈ 858993459 (0x33333333).
    const uint32_t RECIP_5 = 858993459U;
    int32_t term5 = (int32_t)(((int64_t)term5_intermediate * RECIP_5) >> 32);
    sum += term5;

    // Now sum holds the series approximation for exp(r) in Q1.15.

    // Scale the result by 2^n, i.e. compute result = exp(r) * 2^n.
    int32_t result = sum;
    if (n < 0) {
        // For negative n, perform a right shift.
        result = result >> (-n);
    } else {
        // For positive n, left-shift; if n is large, saturate.
        if (n >= 16) {  // 2^16 would already overflow Q1.15.
            result = one - 1;  // Saturate to Q1_15_MAX.
        } else {
            result = result << n;
            if (result >= one)
                result = one - 1;
        }
    }

    return (q1_15_t) result;
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
 * q16_16_approximate_reciprocal:
 *
 * Computes an approximate reciprocal 1/b for a Q16.16 number b.
 *
 * The method:
 * 1. Checks that b is nonzero and handles negative inputs.
 * 2. Performs an early saturation check: in Q16.16, the smallest positive number
 *    that can be inverted without overflow is about 1/32768 (0.000030517578125).
 *    In fixed-point that value is Q16_16_ONE/32768, i.e. 65536/32768 = 2.
 *    If |b| <= 2, the reciprocal exceeds the representable range so the function
 *    immediately returns Q16_16_MAX (or Q16_16_MIN for negative b).
 * 3. Normalizes b so that its absolute value (norm) lies in [Q16_16_ONE/2, Q16_16_ONE],
 *    which corresponds roughly to the real interval [0.5, 1.0]. The number of shifts is
 *    recorded in the variable "shift".
 * 4. Computes an initial guess for 1/norm using a linear approximation:
 *
 *       r0 = (48/17) - (32/17) * norm_real
 *
 *    The constants are scaled into Q16.16 format.
 * 5. Refines the guess with three iterations of the Newton–Raphson update:
 *
 *       rₙ₊₁ = rₙ * (2 - norm * rₙ)
 *
 * 6. Adjusts the result back to the original scaling (undoing the normalization),
 *    taking care to check for overflow when left-shifting.
 * 7. Applies the original sign and finally saturates the result to Q16_16_MAX or Q16_16_MIN.
 */
static inline q16_16_t q16_16_approximate_reciprocal(q16_16_t b) {
    // b must not be zero.
    if (b == 0) {
        return Q16_16_MAX;
    }

    // Work with the absolute value; record the sign.
    int sign = 1;
    if (b < 0) {
        sign = -1;
        b = -b;
    }

    // Early saturation:
    // In Q16.16 the smallest positive value that can be inverted without overflow is:
    //    1/32768 ≈ 0.000030517578125.
    // In Q16.16, 0.000030517578125 is represented as:
    //    0.000030517578125 * 65536 = 2.
    // Thus, if b <= 2, 1/b would be >= 32768 (beyond the Q16.16 maximum ~32767.99998).
    if (b <= 2) {
        return (sign > 0 ? Q16_16_MAX : Q16_16_MIN);
    }

    // Normalize b so that norm is in [Q16_16_ONE/2, Q16_16_ONE] (i.e. [32768, 65536]).
    int shift = 0;
    q16_16_t norm = b;
    while (norm < (Q16_16_ONE >> 1)) {
        norm <<= 1;
        shift--;
    }
    while (norm > Q16_16_ONE) {
        norm >>= 1;
        shift++;
    }

    // Use a linear approximation for the initial guess on the interval [0.5,1].
    // In real arithmetic for x in [0.5,1], one effective approximation for 1/x is:
    //     r0 = (48/17) - (32/17)*x.
    // Here, x = norm / Q16_16_ONE. We precompute the constants in Q16.16:
    const int32_t K = (int32_t)((48.0 / 17.0) * Q16_16_ONE + 0.5);
    const int32_t L = (int32_t)((32.0 / 17.0) * Q16_16_ONE + 0.5);
    int32_t r = K - (int32_t)(((int64_t)L * norm) >> Q16_16_FRACTIONAL_BITS);

    // Refine the approximation with three Newton–Raphson iterations:
    //    r = r * (2 - norm * r)
    for (int i = 0; i < 3; i++) {
        int64_t prod = ((int64_t)norm * r) >> Q16_16_FRACTIONAL_BITS;
        int64_t diff = (2LL * Q16_16_ONE) - prod;
        r = (int32_t)(((int64_t)r * diff) >> Q16_16_FRACTIONAL_BITS);
    }

    // Adjust for the normalization.
    // Since we wrote b as: b = norm * 2^(shift), it follows that 1/b = (1/norm) * 2^(-shift).
    if (shift > 0) {
        r >>= shift;
    } else if (shift < 0) {
        // When left-shifting, check for potential overflow.
        if (r > (Q16_16_MAX >> (-shift)))
            r = Q16_16_MAX;
        else
            r <<= -shift;
    }

    int32_t result = sign * r;

    // Final saturation to the representable Q16.16 range.
    if (result > Q16_16_MAX)
        result = Q16_16_MAX;
    else if (result < Q16_16_MIN)
        result = Q16_16_MIN;

    return (q16_16_t)result;
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

    // Division by using multiplication of 1/b
    q16_16_t const reciprocal = q16_16_approximate_reciprocal(b);        // Compute reciprocal of b in Q8.24
    int64_t res = ((int64_t)a * reciprocal) >> Q16_16_FRACTIONAL_BITS;  // Multiply a by the reciprocal
    if (res > Q16_16_MAX)
        return Q16_16_MAX;
    if (res < Q16_16_MIN)
        return Q16_16_MIN;
    return (q16_16_t)res;

//    // Slow division
//    int64_t res = (((int64_t)a) << Q16_16_FRACTIONAL_BITS) / b;
//    if(res > Q16_16_MAX)
//        return Q16_16_MAX;
//    if(res < Q16_16_MIN)
//        return Q16_16_MIN;
//    return (q16_16_t)res;
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


// Saturation arguments (given in Q16.16):
#define Q16_16_EXP_MAX_ARG ((q16_16_t)681360)           // ~10.397 in Q16.16
#define Q16_16_EXP_MIN_ARG ((q16_16_t)(-16 * Q16_16_ONE)) // -16.0 in Q16.16

/*
 * q16_16_exp:
 *
 * Computes e^x for a Q16.16 number x using range reduction and an unrolled Taylor
 * series for e^r, avoiding costly 64-bit divisions.
 *
 * 1. Early saturation:
 *      - if x ≥ Q16_16_EXP_MAX_ARG, returns Q16_16_MAX.
 *      - if x < Q16_16_EXP_MIN_ARG, returns 0.
 *
 * 2. Range reduction:
 *      x = n * ln2 + r,  with r in [0, ln2)
 *   so that e^x = 2^n * e^r.
 *
 * 3. The Taylor series for e^r is approximated as:
 *      e^r ≈ 1 + r + r^2/2 + r^3/6 + r^4/24 + r^5/120
 *
 *    We compute r^2, r^3, r^4, and r^5 in 64-bit arithmetic (with Q16.16 scaling) and
 *    replace the divisions by 6, 24, and 120 with multiplications by precomputed reciprocals.
 *
 *    Precomputed reciprocals in Q32 (i.e., representing 1/i as floor((1<<32)/i)):
 *         RECIP_6   ≈ 715827882    (for division by 6)
 *         RECIP_24  ≈ 178956970    (for division by 24)
 *         RECIP_120 ≈ 35791394     (for division by 120)
 *
 * 4. The result is then scaled by 2^n using shifts.
 *
 * 5. Finally, the result is saturated to the Q16.16 representable range.
 */
static inline q16_16_t q16_16_exp(q16_16_t x) {
    // Early saturation.
    if (x >= Q16_16_EXP_MAX_ARG)
        return Q16_16_MAX;
    if (x < Q16_16_EXP_MIN_ARG)
        return 0;

    // Use the precomputed ln2 constant.
    const q16_16_t ln2 = Q16_16_LN2_CONST;  // ~45426 in Q16.16

    // Range reduction: find n such that x = n * ln2 + r, with r in [0, ln2).
    // We use 64-bit arithmetic for the division.
    int64_t x64 = x;
    int n = (int)(x64 / ln2);
    if (x64 < 0 && (x64 % ln2) != 0)
        n--;  // Adjust for floor when x is negative.

    // Compute n * ln2 in Q16.16.
    int64_t n_ln2 = (int64_t)n * ln2;
    // Compute the remainder: r = x - n * ln2.
    q16_16_t r = x - (q16_16_t)n_ln2;

    // Compute the Taylor series for e^r:
    // e^r ≈ 1 + r + r^2/2 + r^3/6 + r^4/24 + r^5/120
    int64_t one = Q16_16_ONE;  // 65536 represents 1.0 in Q16.16

    // Compute powers of r in Q16.16:
    int64_t r2 = (((int64_t)r * r) >> 16);  // r^2 in Q16.16
    int64_t r3 = (((int64_t)r2 * r) >> 16);   // r^3 in Q16.16
    int64_t r4 = (((int64_t)r3 * r) >> 16);   // r^4 in Q16.16
    int64_t r5 = (((int64_t)r4 * r) >> 16);   // r^5 in Q16.16

    // Precomputed reciprocals in Q32.
    const int64_t RECIP_6   = 715827882LL;   // ≈ (1<<32)/6
    const int64_t RECIP_24  = 178956970LL;    // ≈ (1<<32)/24
    const int64_t RECIP_120 = 35791394LL;     // ≈ (1<<32)/120

    int64_t term0 = one;              // 1.0
    int64_t term1 = r;                // r
    int64_t term2 = r2 >> 1;          // r^2/2 (division by 2 using a right shift)
    int64_t term3 = (r3 * RECIP_6) >> 32;   // r^3/6
    int64_t term4 = (r4 * RECIP_24) >> 32;  // r^4/24
    int64_t term5 = (r5 * RECIP_120) >> 32; // r^5/120

    int64_t series = term0 + term1 + term2 + term3 + term4 + term5;

    // Scale the series result by 2^n.
    int64_t result = series;
    if (n < 0) {
        result = result >> (-n);
    } else {
        if (n >= 32) {  // A left shift of 32 or more would overflow.
            result = Q16_16_MAX;
        } else {
            result = result << n;
            if (result > Q16_16_MAX)
                result = Q16_16_MAX;
        }
    }

    // Final saturation.
    if (result > Q16_16_MAX)
        result = Q16_16_MAX;
    if (result < Q16_16_MIN)
        result = Q16_16_MIN;

    return (q16_16_t)result;
}


/*
 * _q16_16_log_base:
 * Computes the natural logarithm of x (x > 0) in Q16.16 by first normalizing
 * x to the form:   x = m * 2^n,   with m in [Q16_16_ONE, 2*Q16_16_ONE)
 * Then, letting t = m - Q16_16_ONE (so that m = 1+t), we approximate:
 *
 *   ln(1+t) ≈ t - t^2/2 + t^3/3 - t^4/4 + t^5/5 - t^6/6 + t^7/7 - t^8/8 + t^9/9.
 *
 * In this version the normalization is done via bit–level operations (using __builtin_clz)
 * and the series is unrolled. Divisions by powers of two are replaced with right shifts,
 * while divisions by 3, 5, 7, and 9 are replaced by multiplications by precomputed reciprocals
 * in Q32 with rounding.
 */
static inline q16_16_t _q16_16_log_base(q16_16_t x) {
    if (x <= 0)
        return Q16_16_MIN;  // Log undefined for nonpositive x; saturate as needed.

    // ----- Step 1: Normalize x using bit-level operations -----
    // Represent x in Q16.16 as:  x = m * 2^n,  with m in [65536, 131072).
    int p = 31 - __builtin_clz((unsigned int)x);  // floor(log2(x))
    int n = p - Q16_16_FRACTIONAL_BITS;            // n = floor(log2(x)) - 16
    uint32_t m;
    if (n >= 0)
        m = x >> n;
    else
        m = x << (-n);
    // m is now the normalized mantissa (in Q16.16) satisfying: 65536 <= m < 131072.

    // ----- Step 2: Compute ln(1+t) via an unrolled alternating series -----
    // Let t = m - Q16_16_ONE, so that m = 1+t in Q16.16.
    int32_t t = (int32_t)m - Q16_16_ONE;
    // Our series is:
    //   ln(1+t) ≈ t - t^2/2 + t^3/3 - t^4/4 + t^5/5 - t^6/6 + t^7/7 - t^8/8 + t^9/9.
    // All terms are computed in 64-bit arithmetic in Q16.16.
    int64_t t64 = t;    // Q16.16
    int64_t sum  = t64;  // First term: t.
    int64_t term = t64;  // Current term; will be updated iteratively.

    // Precomputed reciprocals in Q32 (i.e. floor((1<<32)/d)):
    const uint32_t RECIP_3 = 1431655765U;   // ≈ (1<<32)/3
    const uint32_t RECIP_5 = 858993459U;      // ≈ (1<<32)/5
    const uint32_t RECIP_7 = 613566756U;      // ≈ (1<<32)/7
    const uint32_t RECIP_9 = 477218588U;      // ≈ (1<<32)/9

    // For each term i = 2 to 9, update term = term * t (with rounding) then divide by i.
    // Use a right-shift with rounding for multiplications (round by adding 1<<(16-1)).
    // Alternate subtracting and adding according to the series sign.

    // i = 2: term = (t^2) / 2.
    term = ((term * t64) + (1LL << (Q16_16_FRACTIONAL_BITS - 1))) >> Q16_16_FRACTIONAL_BITS;
    term = term >> 1;  // Division by 2 (power of two).
    sum -= term;       // Even term: subtract.

    // i = 3: term = (t^3) / 3.
    term = ((term * t64) + (1LL << (Q16_16_FRACTIONAL_BITS - 1))) >> Q16_16_FRACTIONAL_BITS;
    term = (term * RECIP_3 + (1LL << 31)) >> 32;  // Division by 3.
    sum += term;       // Odd term: add.

    // i = 4: term = (t^4) / 4.
    term = ((term * t64) + (1LL << (Q16_16_FRACTIONAL_BITS - 1))) >> Q16_16_FRACTIONAL_BITS;
    term = term >> 2;  // Division by 4.
    sum -= term;       // Even: subtract.

    // i = 5: term = (t^5) / 5.
    term = ((term * t64) + (1LL << (Q16_16_FRACTIONAL_BITS - 1))) >> Q16_16_FRACTIONAL_BITS;
    term = (term * RECIP_5 + (1LL << 31)) >> 32;  // Division by 5.
    sum += term;       // Odd: add.

    // i = 6: term = (t^6) / 6.
    term = ((term * t64) + (1LL << (Q16_16_FRACTIONAL_BITS - 1))) >> Q16_16_FRACTIONAL_BITS;
    // Division by 6: since 6 is not a power of two, use reciprocal.
    // (1<<32)/6 = 715827882.
    term = (term * 715827882LL + (1LL << 31)) >> 32;
    sum -= term;       // Even: subtract.

    // i = 7: term = (t^7) / 7.
    term = ((term * t64) + (1LL << (Q16_16_FRACTIONAL_BITS - 1))) >> Q16_16_FRACTIONAL_BITS;
    term = (term * RECIP_7 + (1LL << 31)) >> 32;  // Division by 7.
    sum += term;       // Odd: add.

    // i = 8: term = (t^8) / 8.
    term = ((term * t64) + (1LL << (Q16_16_FRACTIONAL_BITS - 1))) >> Q16_16_FRACTIONAL_BITS;
    term = term >> 3;  // Division by 8.
    sum -= term;       // Even: subtract.

    // i = 9: term = (t^9) / 9.
    term = ((term * t64) + (1LL << (Q16_16_FRACTIONAL_BITS - 1))) >> Q16_16_FRACTIONAL_BITS;
    term = (term * RECIP_9 + (1LL << 31)) >> 32;  // Division by 9.
    sum += term;       // Odd: add.

    // ----- Step 3: Reconstruct ln(x) -----
    // We have normalized x as:  x = m * 2^n, where m = 1+t.
    // Thus, ln(x) = ln(m) + n * ln2, and ln(m) ≈ series = sum.
    int64_t n_ln2 = (int64_t)n * Q16_16_LN2_CONST;
    int64_t ln_val = sum + n_ln2;
    
    // Saturate result if needed.
    if (ln_val > Q16_16_MAX)
        ln_val = Q16_16_MAX;
    if (ln_val < Q16_16_MIN)
        ln_val = Q16_16_MIN;
    
    return (q16_16_t)ln_val;
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
