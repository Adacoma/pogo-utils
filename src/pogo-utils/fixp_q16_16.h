
#ifndef FIXP_Q16_16_H_
#define FIXP_Q16_16_H_

#include <stdint.h>
#include <limits.h>
#include <stdbool.h>
#include <math.h>

#include "fixp_base.h"



/////////////////////////////////////////////////////////////////
///                     Q16.16 FUNCTIONS                      /// {{{1
/////////////////////////////////////////////////////////////////

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


#endif

/////////////////////////////////////////////////////////////////
// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
