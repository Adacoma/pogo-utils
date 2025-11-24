
#ifndef FIXP_Q1_15_H_
#define FIXP_Q1_15_H_

#include <stdint.h>
#include <limits.h>
#include <stdbool.h>
#include <math.h>

#include "fixp_base.h"


/////////////////////////////////////////////////////////////////
///                      Q1.15 FUNCTIONS                      /// {{{1
/////////////////////////////////////////////////////////////////

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


/* ==== Q1.15 Activation Functions: tanh, sigmoid, ReLU ==== */

/*
 * Fast Q1.15 tanh approximation.
 *
 * For x in [-1, 1), we use a 5th–order odd polynomial:
 *    tanh(x) ≈ x - x^3/3 + 2x^5/15
 *
 * Implemented entirely in Q1.15 using q1_15_mul32_r and precomputed
 * Q1.15 coefficients:
 *    1/3   ≈ 10923 / 2^15
 *    2/15  ≈  4369 / 2^15
 *
 * On [-0.5, 0.5] the max absolute error is about 4e-4; on [-1, 1] it
 * stays below ~0.04, which is usually sufficient for small MLP/GRU
 * controllers.
 */
static inline q1_15_t q1_15_tanh(q1_15_t x) {
    int32_t x32 = (int32_t)x;  /* promote to 32 bits for intermediate math */

    /* x^2, x^3, x^5 in Q1.15 */
    int32_t x2 = q1_15_mul32_r(x32, x32);
    int32_t x3 = q1_15_mul32_r(x2,  x32);
    int32_t x5 = q1_15_mul32_r(x3,  x2);

    /* Q1.15 constants: 1/3 and 2/15 */
    const int32_t C_INV3   = 10923; /* round((1.0/3.0)  * 32768) */
    const int32_t C_2DIV15 =  4369; /* round((2.0/15.0) * 32768) */

    /* term3 = x^3 / 3, term5 = 2x^5 / 15 */
    int32_t term3 = q1_15_mul32_r(x3, C_INV3);
    int32_t term5 = q1_15_mul32_r(x5, C_2DIV15);

    int32_t y = x32 - term3 + term5;

    /* Saturate to Q1.15 range, though for |x|<=1 this should not hit. */
    if (y > Q1_15_MAX) y = Q1_15_MAX;
    if (y < Q1_15_MIN) y = Q1_15_MIN;

    return (q1_15_t)y;
}

/*
 * Fast Q1.15 sigmoid approximation.
 *
 * We use the identity:
 *    sigmoid(x) = 0.5 * (tanh(x/2) + 1)
 *
 * For x in [-1, 1), x/2 is in [-0.5, 0.5], where the tanh polynomial
 * is highly accurate (max error ~4e-4). We do all math in Q1.15 and
 * clamp the result to [0, 1).
 */
static inline q1_15_t q1_15_sigmoid(q1_15_t x) {
    /* x_half = x / 2 in Q1.15 (arithmetic shift) */
    q1_15_t x_half = (q1_15_t)(x >> 1);

    q1_15_t t = q1_15_tanh(x_half);      /* tanh(x/2) in Q1.15 */

    /* Compute 0.5 * (t + 1) in Q1.15.
     *
     * 1.0 is represented as Q1_15_ONE (32768) in 32-bit space; final
     * result must be clamped into [0, Q1_15_MAX].
     */
    int32_t tmp = (int32_t)t + Q1_15_ONE; /* t + 1.0, in Q1.15+scale */
    int32_t y   = tmp >> 1;               /* divide by 2 -> 0.5*(t+1) */

    /* Clamp to [0, Q1_15_MAX] to enforce sigmoid range [0,1). */
    if (y < 0)         y = 0;
    if (y > Q1_15_MAX) y = Q1_15_MAX;

    return (q1_15_t)y;
}

/*
 * Q1.15 ReLU: max(0, x)
 *
 * Fully branch–predictable on RV32IM core and essentially free
 * compared to multiplies.
 */
static inline q1_15_t q1_15_relu(q1_15_t x) {
    return (x > 0) ? x : (q1_15_t)0;
}


// acc is Q2.30
static inline int32_t q1_15_mac32(int32_t acc, q1_15_t a, q1_15_t b) {
    acc += (int32_t)a * (int32_t)b;   // Q2.30, no sat
    return acc;
}

// convert 32-bit Q2.30 accumulator to Q1.15 with rounding & saturation
static inline q1_15_t q1_15_from_acc32(int32_t acc) {
    // round: shift from Q2.30 -> Q1.15
    acc += (acc >= 0 ? (1 << (Q1_15_FRACTIONAL_BITS - 1))
                     : -(1 << (Q1_15_FRACTIONAL_BITS - 1)));
    acc >>= Q1_15_FRACTIONAL_BITS;

    if (acc > Q1_15_MAX) acc = Q1_15_MAX;
    if (acc < Q1_15_MIN) acc = Q1_15_MIN;

    return (q1_15_t)acc;
}

/*
 * Q1.15 hard tanh:
 *   hard_tanh(x) = -1      if x <= -1
 *                   x      if -1 < x <  1
 *                   +1     if x >=  1
 *
 * In Q1.15 we approximate +1 by Q1_15_MAX (0x7FFF) and -1 by Q1_15_MIN (0x8000).
 */
static inline q1_15_t q1_15_hard_tanh(q1_15_t x) {
//    if (x <= Q1_15_MIN)
//        return Q1_15_MIN;
//    if (x >= Q1_15_MAX)
//        return Q1_15_MAX;
//    return x;
    /* In Q1.15 we are already in [-1, 1), so hard tanh is just identity. */
    return x;
}


#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
