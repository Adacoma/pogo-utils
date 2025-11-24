
#ifndef FIXP_BASE_H_
#define FIXP_BASE_H_

#include <stdint.h>
#include <limits.h>
#include <stdbool.h>
#include <math.h>

/////////////////////////////////////////////////////////////////
///                     BASE TYPES                     /// {{{1
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

/* Convert a float to Q1.15 fixed-point with saturation.
   Valid input range is [-1.0, 1.0); values outside saturate.
*/
#define q1_15_from_float(x) Q1_15_FROM_FLOAT(x)    // Always use the macro instead of the inline function



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

#define Q8_24_FROM_FLOAT(x)  q8_24_from_float(x)



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

#define Q16_16_FROM_FLOAT(x) q16_16_from_float(x)


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

/* Convert a float to Q6.10 */
#define q6_10_from_float(x) Q6_10_FROM_FLOAT(x)    // Always use the macro instead of the inline function


typedef int16_t q6_10_t;

/////////////////////////////////////////////////////////////////
///                     GENERIC FUNCTIONS                     /// {{{1
/////////////////////////////////////////////////////////////////

void init_fixp(void);
void printf_fixp(const char *format, ...);


#endif

// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
