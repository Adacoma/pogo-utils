
#ifndef FIXP_H_
#define FIXP_H_

#include <stdint.h>
#include <limits.h>
#include <stdbool.h>
#include <math.h>


void init_fixp(void);


/////////////////////////////////////////////////////////////////
///                      Q8.24 FUNCTIONS                      /// {{{1
/////////////////////////////////////////////////////////////////

/* ==== Q8.24 Fixed-Point Definitions and Functions ==== */

typedef int32_t q8_24_t;

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
#define Q8_24_MAX ((q8_24_t)INT32_MAX)
#define Q8_24_MIN ((q8_24_t)INT32_MIN)

static inline q8_24_t q8_24_from_float(float x) {
    if (x >= 128.0f)  return Q8_24_MAX;
    if (x <  -128.0f) return Q8_24_MIN;
    float y = x * 16777216.0f + (x >= 0 ? 0.5f : -0.5f);
    return (q8_24_t)y;
}
#define Q8_24_FROM_FLOAT(x)  q8_24_from_float(x)


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



/////////////////////////////////////////////////////////////////
///                      Q1.15 FUNCTIONS                      /// {{{1
/////////////////////////////////////////////////////////////////
/* ==== Q1.15 Fixed-Point Definitions and Conversion Functions ==== */

typedef int16_t q1_15_t;

/*
 * In Q1.15, a 16-bit signed fixed-point format, 15 bits represent the fractional part.
 * The representable range is approximately [-1.0, 0.99997].
 */
#define Q1_15_FRACTIONAL_BITS 15
#define Q1_15_SCALE (1 << Q1_15_FRACTIONAL_BITS)  // 32768
#define Q1_15_ONE ((int32_t)Q1_15_SCALE) // (1 << Q1_15_FRACTIONAL_BITS)
//#define Q1_15_ONE (q1_15_t)(1 << Q1_15_FRACTIONAL_BITS)
//#define Q1_15_NEG_ONE  ((q1_15_t)(- (1 << Q1_15_FRACTIONAL_BITS)))        // -1.0 in Q1.15 (-32768)

#define Q1_15_MAX ((q1_15_t)0x7FFF)
#define Q1_15_MIN ((q1_15_t)0x8000)

/* ln(2) in Q1.15: round(0.6931471805599453 * 32768) = 22713 */
#define Q1_15_LN2_CONST 22713

// Q1.15: 1.0 = 2^15 = 32768.0f; saturate if x >= 1.0 or x < -1.0.
#define Q1_15_FROM_FLOAT(x)  ((q1_15_t)(((x) >= 1.0f) ? Q1_15_MAX : \
                                ((x) < -1.0f) ? Q1_15_MIN : \
                                (((x) * 32768.0f) + ((x) >= 0 ? 0.5f : -0.5f))))



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
#define q1_15_from_float(x) Q1_15_FROM_FLOAT(x)    // Always use the macro instead of the inline function
//static inline q1_15_t q1_15_from_float(float x) {
//    if (x >= 1.0f)
//        return Q1_15_MAX;
//    if (x < -1.0f)
//        return Q1_15_MIN;
//    return (q1_15_t)(x * Q1_15_ONE);
//    //volatile float res = x * 128.0f; // XXX evil hack to avoid double promotion
//    //return (q1_15_t)(res * 256.0f);
//}

/* Convert Q1.15 fixed-point to a float. */
static inline float q1_15_to_float(q1_15_t x) {
    return (float)x / (float)Q1_15_SCALE;
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

/* Rounded multiply: returns (a*b)>>15 with rounding, saturated to q1_15_t range */
static inline q1_15_t q1_15_mul(q1_15_t a, q1_15_t b) {
    int32_t p = (int32_t)a * (int32_t)b;               /* Q2.30 */
    /* add 0.5 ulp for rounding toward nearest */
    p += (p >= 0 ? (1 << (Q1_15_FRACTIONAL_BITS - 1)) : -(1 << (Q1_15_FRACTIONAL_BITS - 1)));
    int32_t r = p >> Q1_15_FRACTIONAL_BITS;            /* back to Q1.15 */
    if (r > Q1_15_MAX) r = Q1_15_MAX;
    if (r < Q1_15_MIN) r = Q1_15_MIN;
    return (q1_15_t)r;
}

/* Multiply two Q1.15 values, but keep full int32 in Q1.15 for intermediate work (with rounding). */
static inline int32_t q1_15_mul32_r(int32_t a, int32_t b) {
    int64_t p = (int64_t)a * (int64_t)b;               /* Q2.30 */
    p += (p >= 0 ? (1LL << (Q1_15_FRACTIONAL_BITS - 1)) : -(1LL << (Q1_15_FRACTIONAL_BITS - 1)));
    return (int32_t)(p >> Q1_15_FRACTIONAL_BITS);
}

/* Divide by small integer with rounding using 32-bit “magic reciprocals”. */
static inline int32_t div3_r32(int32_t x){ return (int32_t)(((int64_t)x * 1431655766LL + (x>=0?1: -1)) >> 32); } /* round(2^32/3) */
static inline int32_t div5_r32(int32_t x){ return (int32_t)(((int64_t)x *  858993459LL + (x>=0?1: -1)) >> 32); } /* round(2^32/5) */

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


static inline q1_15_t q1_15_exp(q1_15_t x) {
    /* For x >= 0, exp(x) >= 1 and Q1.15 cannot represent >1. Saturate. */
    if (x >= 0) return Q1_15_MAX;

    /* Range reduction: x = n*ln2 + r, with r in [0, ln2) */
    int32_t x32 = (int32_t)x;           /* Q1.15 */
    int32_t L   = Q1_15_LN2_CONST;      /* Q1.15 */
    int n = x32 / L;                    /* trunc toward 0 */
    if (x32 % L) n--;                   /* floor for negatives */
    int32_t r = x32 - n * L;            /* Q1.15, 0 <= r < ln2 */

    /* exp(r) via 5th-order (tightly rounded) Taylor in Q1.15, Horner form:
       exp(r) ≈ 1 + r * (1 + r*(1/2 + r*(1/6 + r*(1/24 + r*(1/120)))))
    */
    int32_t one = Q1_15_ONE;            /* 32768 */

    /* Fold from smallest term up, rounding each step */
    /* c5 = 1/120 */
    //int32_t t = q1_15_mul32_r(one, div5_r32(div5_r32( one ))); /* 1/25? No—do it explicitly: */
    /* Better: constants directly in Q1.15 to avoid compounding div rounding */
    const int32_t C2 = (int32_t)((1.0/2.0)*Q1_15_SCALE + 0.5);   /* 16384 */
    const int32_t C3 = (int32_t)((1.0/6.0)*Q1_15_SCALE + 0.5);   /*  5461 */
    const int32_t C4 = (int32_t)((1.0/24.0)*Q1_15_SCALE + 0.5);  /*  1365 */
    const int32_t C5 = (int32_t)((1.0/120.0)*Q1_15_SCALE + 0.5); /*   273 */

    int32_t y = C5;                                     /* Q1.15 */
    y = q1_15_mul32_r(y, r) + C4;
    y = q1_15_mul32_r(y, r) + C3;
    y = q1_15_mul32_r(y, r) + C2;
    y = q1_15_mul32_r(y, r) + one;                      /* now y ≈ 1 + r/.. chain */
    int32_t exp_r = q1_15_mul32_r(y, r) + one;          /* 1 + r*(...) */

    /* Scale by 2^n (n ≤ 0). For n<0, right-shift with rounding each step. */
    int32_t result = exp_r;
    int sh = -n;
    while (sh-- > 0) {
        /* round: add sign*0.5 ulp before >>1 */
        result += (result >= 0 ? 1 : -1) * (1);
        result >>= 1;
    }

    /* Saturate to Q1.15 (should be in (0,1]) */
    if (result >= Q1_15_ONE) result = Q1_15_ONE - 1;  /* max representable < 1 */
    if (result < Q1_15_MIN)  result = Q1_15_MIN;      /* shouldn’t happen here */
    return (q1_15_t)result;
}


static inline q1_15_t q1_15_log(q1_15_t x) {
    if (x <= 0) return Q1_15_MIN;

    /* Normalize x to m in [1,2): x = m * 2^n. We do it in Q1.15. */
    int32_t m = (int32_t)x;
    int      n = 0;

    /* Treat Q1_15_MAX as exact 1.0 for stability */
    if (m == Q1_15_MAX) m = Q1_15_ONE;

    while (m < Q1_15_ONE) { m <<= 1; n--; }              /* scale up */
    while (m >= (Q1_15_ONE << 1)) { m >>= 1; n++; }      /* scale down */

    /* y = (m - 1) / (m + 1) in Q1.15 with rounding */
    int32_t num = m - Q1_15_ONE;                         /* Q1.15 */
    int32_t den = m + Q1_15_ONE;                         /* Q1.15 */
    /* y = (num / den). Do a Q1.15 division with rounding: (num<<15)/den */
    int64_t y64 = ((int64_t)num << Q1_15_FRACTIONAL_BITS);
    if ( (y64 ^ den) >= 0 ) y64 += (den/2); else y64 -= (den/2);
    int32_t y = (int32_t)(y64 / den);                    /* Q1.15, |y| <= ~0.333 */

    /* S = y + y^3/3 + y^5/5 (+ y^7/7 if you want more accuracy) */
    int32_t y2 = q1_15_mul32_r(y, y);                    /* y^2 */
    int32_t y3 = q1_15_mul32_r(y2, y);                   /* y^3 */
    int32_t y5 = q1_15_mul32_r(y3, y2);                  /* y^5 */
    int32_t term3 = div3_r32(y3);
    int32_t term5 = div5_r32(y5);
    int32_t S = y + term3 + term5;                       /* Q1.15 */

    /* ln(m) ≈ 2*S */
    int32_t ln_m = (S << 1);                             /* Q1.15 */

    /* ln(x) = ln(m) + n*ln2 */
    int32_t ln_x = ln_m + n * Q1_15_LN2_CONST;

    /* Saturate */
    if (ln_x > Q1_15_MAX) ln_x = Q1_15_MAX;
    if (ln_x < Q1_15_MIN) ln_x = Q1_15_MIN;
    return (q1_15_t)ln_x;
}


/* Signed, human-readable decomposition */
static inline int16_t q1_15_get_int(q1_15_t x) {
    int16_t s = (x < 0);
    int32_t ax = s ? -(int32_t)x : (int32_t)x;
    int16_t ip = (int16_t)(ax >> Q1_15_FRACTIONAL_BITS);
    return s ? -ip : ip;
}

static inline uint16_t q1_15_get_frac(q1_15_t x) {
    int32_t ax = (x < 0) ? -(int32_t)x : (int32_t)x;
    return (uint16_t)(ax & (Q1_15_SCALE - 1));
}



/////////////////////////////////////////////////////////////////
///                     Q16.16 FUNCTIONS                      /// {{{1
/////////////////////////////////////////////////////////////////
/* === Q16.16 Fixed-Point Definitions === */
typedef int32_t q16_16_t;

extern bool q16_16_inited;

/*
 * Q16.16 uses 32 bits: 16 bits for the integer part and 16 bits for the fractional part.
 * The scaling factor is 2^16 = 65536.
 */
#define Q16_16_FRACTIONAL_BITS 16
#define Q16_16_ONE     ((q16_16_t)(1 << Q16_16_FRACTIONAL_BITS))           // 1.0 in Q16.16 (65536)
#define Q16_16_NEG_ONE ((q16_16_t)(- (1 << Q16_16_FRACTIONAL_BITS)))       // -1.0 in Q16.16 (-65536)

/*
 * Q16.16 uses a signed 32-bit integer.
 * The maximum representable value is 0x7FFFFFFF and the minimum is 0x80000000.
 * (These macros are used for saturation in arithmetic operations.)
 */
#define Q16_16_MAX ((q16_16_t)INT32_MAX)
#define Q16_16_MIN ((q16_16_t)INT32_MIN)

/* Useful naturals in Q16.16 */
#define Q16_16_LN2     ((q16_16_t)45426)   /* ln(2)  ≈ 0.693147 * 65536 */
#define Q16_16_LOG2E  ((q16_16_t)94548) /* log2(e)=1/ln2 ≈ 1.442695 * 65536 */

/* Recommended exp() safe range (≈ [-16, +10.397]) */
#define Q16_16_EXP_MAX_ARG ((q16_16_t)681360)              /* 10.397 * 65536 */
#define Q16_16_EXP_MIN_ARG ((q16_16_t)(-16 * Q16_16_ONE))  /* -16 */


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

/* Convert Q16.16 to float */
static inline float q16_16_to_float(q16_16_t x) {
    return (float)x / Q16_16_ONE;
}

static inline q16_16_t q16_16_from_float(float x) {
    if (x >= 32767.99998f) return Q16_16_MAX;
    if (x <= -32768.0f)  return Q16_16_MIN;   // saturate lower
    float xf = x * 65536.0f;
    /* round half away from zero */
    xf += (xf >= 0.0f) ? 0.5f : -0.5f;        // round half away from zero
    return (q16_16_t)(int32_t)xf;
}

#define Q16_16_FROM_FLOAT(x) q16_16_from_float(x)


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
    int32_t s = (int32_t)(a + b);
    /* if overflow: signs of (s^a) and (s^b) differ at sign bit */
    int32_t ov = ((s ^ a) & (s ^ b)) >> 31;
    if (ov) return (s < 0) ? Q16_16_MIN : Q16_16_MAX; /* saturated */
    return s;

}

/*
 * Q16.16 Subtraction with Saturation.
 */
static inline q16_16_t q16_16_sub(q16_16_t a, q16_16_t b) {
    int32_t s = (int32_t)(a - b);
    int32_t ov = ((s ^ a) & (s ^ ~b)) >> 31;
    if (ov) return (s < 0) ? Q16_16_MIN : Q16_16_MAX;
    return s;
}

/* --- Multiply: one 64-bit product, then down-shift and saturate --- */
static inline q16_16_t q16_16_mul(q16_16_t a, q16_16_t b) {
    /* 32x32->64 is cheap on RV32IM (MUL + MULH), the rest stays 32-bit. */
    int64_t p = (int64_t)a * (int64_t)b;
    /* arithmetic shift down to Q16.16 */
    int64_t v = p >> Q16_16_FRACTIONAL_BITS;
    if (v > Q16_16_MAX) return Q16_16_MAX;
    if (v < Q16_16_MIN) return Q16_16_MIN;
    return (q16_16_t)v;
}

/* --- Branch-free saturating abs (handles INT32_MIN -> Q16_16_MAX) --- */
static inline q16_16_t q16_16_abs(q16_16_t x) {
    /* Standard branch-free abs, then fix INT_MIN to saturate */
    int32_t m = x >> 31;                 /* all 1s if negative, else 0 */
    int32_t v = (x ^ m) - m;             /* abs w/o branch */
    /* If x == INT_MIN, v overflows back to INT_MIN => saturate to MAX */
    return (v == Q16_16_MIN) ? Q16_16_MAX : v;
}

#define Q16_16_RECIP_TABLE_SIZE 256
// Global lookup table for approximate reciprocals of normalized Q16.16 numbers.
// The table covers the range [Q16_16_ONE/2, Q16_16_ONE) i.e. [32768, 65536).
// Each table entry is a Q16.16 reciprocal approximation.
extern q16_16_t q16_16_recip_table[Q16_16_RECIP_TABLE_SIZE];

void _init_q16_16(void);

/* --- msb_index: index of highest set bit in 32-bit x (0..31). x!=0.
 * Few predictable branches; no loops; no CLZ dependency.
 */
static inline int msb_index(uint32_t x)
{
    int n = 0;
    if (x >= (1u << 16)) { x >>= 16; n += 16; }
    if (x >= (1u <<  8)) { x >>=  8; n +=  8; }
    if (x >= (1u <<  4)) { x >>=  4; n +=  4; }
    if (x >= (1u <<  2)) { x >>=  2; n +=  2; }
    if (x >= (1u <<  1)) {            n +=  1; }
    return n;
}

/* ~16b accuracy, 1 NR step */
/* --- approximate reciprocal 1/b (Q16.16) using:
 *  1) sign & early saturation
 *  2) normalize: b = m * 2^k, with m in [1,2)
 *  3) LUT 1/m (8 bits) + one Newton step r = r*(2 - m*r)
 *  4) scale back by 2^(-k)
 */
static inline q16_16_t q16_16_approximate_reciprocal(q16_16_t b) {
    if (b == 0) return Q16_16_MAX;
    if (!q16_16_inited) _init_q16_16();

    int sign = 1;
    uint32_t ub = (uint32_t)b;
    if (b < 0) { sign = -1; ub = (uint32_t)(-b); }

    /* If |b| <= 2/65536, reciprocal would overflow in Q16.16 */
    if (ub <= 2u) return (sign > 0) ? Q16_16_MAX : Q16_16_MIN;

    /* Normalize: b = m * 2^k, with m in [1,2) in Q16.16 */
    int p = msb_index(ub);
    int k = p - 16;
    uint32_t m;
    if (k >= 0) m = ub >> k; else m = ub << (-k);
    /* m is now in [65536, 131072) */

    /* LUT index for m in [1,2): take top 8 fractional bits: (m - 1.0) / (1/256) */
    uint32_t frac = (m - (uint32_t)Q16_16_ONE) >> 8;   /* 0..255 */
    q16_16_t r = q16_16_recip_table[frac];                     /* ~1/m */

    /* One Newton step in Q16.16: r <- r*(2 - m*r) */
    int32_t mr   = (int32_t)(((int64_t)m * r) >> 16);
    int32_t two  = (2 * Q16_16_ONE);
    int32_t diff = (int32_t)(two - mr);
    r = (q16_16_t)(((int64_t)r * diff) >> 16);

    /* Scale by 2^(-k) */
    if (k > 0)      r >>= k;
    else if (k < 0) {
        int sh = -k;
        if (r > (Q16_16_MAX >> sh)) r = Q16_16_MAX;
        else                        r <<= sh;
    }

    int32_t res = (sign > 0) ? r : -r;
    if (res > Q16_16_MAX) res = Q16_16_MAX;
    if (res < Q16_16_MIN) res = Q16_16_MIN;
    return (q16_16_t)res;
}


/* saturating, uses reciprocal */
static inline q16_16_t q16_16_div(q16_16_t a, q16_16_t b) {
    if (b == 0) return (a >= 0) ? Q16_16_MAX : Q16_16_MIN;
    q16_16_t r = q16_16_approximate_reciprocal(b);
    int64_t v = ((int64_t)a * (int64_t)r) >> 16;
    if (v > Q16_16_MAX) return Q16_16_MAX;
    if (v < Q16_16_MIN) return Q16_16_MIN;
    return (q16_16_t)v;
}


static inline int q16_16_is_saturated(q16_16_t x) {
    return (x == Q16_16_MAX || x == Q16_16_MIN);
}

/* 2^(i/64) table in Q16.16 (64 entries, 256 bytes) */
extern q16_16_t q16_16_exp2_table[64];

/* exp(x) using exp2 range reduction with 64-bin table + quartic on residual
 * y = x * log2(e) = k + f,  f in [0,1)
 * Let i = floor(f*64),  g = f - i/64  (so g in [0,1/64))
 * 2^f = 2^(i/64) * 2^g, with 2^g ≈ 1 + c1 g + c2 g^2 + c3 g^3 + c4 g^4
 */
static inline q16_16_t q16_16_exp(q16_16_t x) {
    if (x >= Q16_16_EXP_MAX_ARG) return Q16_16_MAX;
    if (x <= Q16_16_EXP_MIN_ARG) return 0;
    if (!q16_16_inited) _init_q16_16();

    /* y = x * log2(e) in Q16.16 */
    int64_t y = ((int64_t)x * (int64_t)Q16_16_LOG2E) >> 16;

    /* k = floor(y),  f = y - k  (Q16.16) */
    int32_t k = (int32_t)(y >> 16);
    q16_16_t f = (q16_16_t)(y - ((int64_t)k << 16)); /* 0..65535 */

    /* i = floor(f*64)  => exact: (f * 64) >> 16 == f >> 10  */
    int32_t i = ((int32_t)f) >> 10;  /* 0..63 */
    if (i > 63) i = 63;

    /* g = f - i/64  -> i/64 in Q16.16 is (i << 10) */
    q16_16_t g = (q16_16_t)((int32_t)f - (i << 10)); /* g in [0, 2^16/64) */

    /* base = 2^(i/64) from LUT */
    q16_16_t base = q16_16_exp2_table[i];

    /* Quartic for 2^g on g ∈ [0, 1/64).
     * Coeffs are Taylor of e^{ln2*g} (Q16.16), rounded:
     *   c1 = ln2
     *   c2 = (ln2^2)/2
     *   c3 = (ln2^3)/6
     *   c4 = (ln2^4)/24
     */
    const q16_16_t c1 = (q16_16_t)45426; /* ln2 */
    const q16_16_t c2 = (q16_16_t)15743; /* 0.240226506959... * 65536 */
    const q16_16_t c3 = (q16_16_t) 3638; /* 0.055504108664... * 65536 */
    const q16_16_t c4 = (q16_16_t)  630; /* 0.009618129107... * 65536 */

    /* Horner: (((c4*g + c3)*g + c2)*g + c1)*g + 1 */
    int32_t t  = c4;
    t  = (int32_t)(((int64_t)t * g) >> 16) + c3;
    t  = (int32_t)(((int64_t)t * g) >> 16) + c2;
    t  = (int32_t)(((int64_t)t * g) >> 16) + c1;
    int32_t two_pow_g = (int32_t)(((int64_t)t * g) >> 16) + Q16_16_ONE;

    /* 2^f = base * 2^g */
    int64_t two_pow_f = ((int64_t)base * (int64_t)two_pow_g) >> 16;

    /* exp(x) = 2^k * 2^f */
    int64_t res;
    if (k >= 0) {
        if (k >= 31) return Q16_16_MAX;
        res = two_pow_f << k;
        if (res > Q16_16_MAX) res = Q16_16_MAX;
    } else {
        res = two_pow_f >> (-k);
    }
    if (res < 0) res = 0;
    return (q16_16_t)res;
}

/* --- log via normalization + one Newton refinement ---
 * x = m * 2^k, ln(x) = ln(m) + k*ln2, with m in [1,2).
 * Initial ln(m) uses cubic log1p; then one Newton step refines it.
 */
static inline q16_16_t q16_16_log(q16_16_t x) {
    if (x <= 0) return Q16_16_MIN; /* saturate for invalid */

    /* ---- normalization: x = m * 2^k, with m in [1,2) ---- */
    uint32_t ux = (uint32_t)x;
    /* branch-light msb */
    int p = 0;
    if (ux >= (1u << 16)) { ux >>= 16; p += 16; }
    if (ux >= (1u <<  8)) { ux >>=  8; p +=  8; }
    if (ux >= (1u <<  4)) { ux >>=  4; p +=  4; }
    if (ux >= (1u <<  2)) { ux >>=  2; p +=  2; }
    if (ux >= (1u <<  1)) {            p +=  1; }
    int k = p - 16;

    uint32_t m;
    if (k >= 0) m = (uint32_t)x >> k; else m = (uint32_t)x << (-k); /* m in [1,2) Q16.16 */

    /* ---- cubic log1p on t = m - 1 ----
       ln(1+t) ≈  t - t^2/2 + t^3/3
    */
    int32_t t  = (int32_t)m - Q16_16_ONE;          /* Q16.16 */
    int64_t t2 = ((int64_t)t * t) >> 16;
    int64_t t3 = (t2 * t) >> 16;

    int64_t ln_m = (int64_t)t
                 - (t2 >> 1)
                 + ( (t3 * 1431655765LL) >> 32 ); /* 1/3 in Q32 */

    /* initial y0 = ln(m) + k*ln2 */
    q16_16_t y = (q16_16_t)ln_m;
    y = (q16_16_t)q16_16_add(y, (q16_16_t)((int64_t)k * (int64_t)Q16_16_LN2));

    /* ---- one Newton refinement: y <- y - (exp(y) - x)/exp(y) ---- */
    q16_16_t exp_y = q16_16_exp(y);
    /* guard against tiny exp_y (shouldn't happen for valid y) */
    if (exp_y != 0) {
        q16_16_t num = q16_16_sub(exp_y, x);          /* exp(y) - x */
        q16_16_t den = exp_y;
        q16_16_t corr = q16_16_div(num, den);         /* (exp(y)-x)/exp(y) */
        y = q16_16_sub(y, corr);
    }

    /* clamp to range */
    if (y > Q16_16_MAX) return Q16_16_MAX;
    if (y < Q16_16_MIN) return Q16_16_MIN;
    return y;
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



/////////////////////////////////////////////////////////////////
///                      Q6.10 FUNCTIONS                      /// {{{1
/////////////////////////////////////////////////////////////////
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

// Q6.10: 1.0 = 2^10 = 1024.0f; saturate if x >= 32.0 or x < -32.0.
#define Q6_10_FROM_FLOAT(x)  ((q6_10_t)(((x) >= 32.0f) ? Q6_10_MAX : \
                                ((x) < -32.0f) ? Q6_10_MIN : \
                                (((x) * 1024.0f) + ((x) >= 0 ? 0.5f : -0.5f))))


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
#define q6_10_from_float(x) Q6_10_FROM_FLOAT(x)    // Always use the macro instead of the inline function
//static inline q6_10_t q6_10_from_float(float x) {
//    return (q6_10_t)(x * Q6_10_ONE);
//}

//static inline float q6_10_to_float(q6_10_t x) {
//    return (float)x / Q6_10_ONE;
//}
//
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



/////////////////////////////////////////////////////////////////
///                     GENERIC FUNCTIONS                     /// {{{1
/////////////////////////////////////////////////////////////////
void printf_fixp(const char *format, ...);


#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
