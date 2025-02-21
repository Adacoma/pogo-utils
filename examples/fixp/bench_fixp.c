/* TODO
 */

#include "pogobase.h"
#include "pogo-utils/fixp.h"
#include "pogo-utils/version.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>
#include <stdlib.h>

#define BENCH_RUNS 1000
#define TOLERANCE 1e-4f

typedef struct {
    time_reference_t timer_it;
} USERDATA;

//extern USERDATA *mydata;
REGISTER_USERDATA(USERDATA)

#ifdef SIMULATOR
#define printf0(fmt, ...) if (pogobot_helper_getid() == 0) { printf(fmt, ##__VA_ARGS__ ); }
#define printf_fixp0(fmt, ...) if (pogobot_helper_getid() == 0) { printf_fixp(fmt, ##__VA_ARGS__ ); }
#else
#define printf0(fmt, ...) printf(fmt, ##__VA_ARGS__ );
#define printf_fixp0(fmt, ...) printf_fixp(fmt, ##__VA_ARGS__ );
#endif

void test_fixp_functions(void);
void bench_fixp_functions(void);


void test_fixp_functions(void) {
#ifdef SIMULATOR
    printf0("==============================================\n");
    printf0("=== POGO-UTILS " POGO_UTILS_VERSION " - FIXED-POINT TESTS  ===\n");
    printf0("==============================================\n");

    printf0("=== Q8.24 Conversion Tests ===\n");
    // Test conversion from int
    {
        int a = 42;
        q8_24_t qa = q8_24_from_int(a);
        assert(q8_24_to_int(qa) == a);
        printf0("int %d -> fixed -> int %d\n", a, q8_24_to_int(qa));

        int b = 200; // Out of range: should saturate to 127.
        q8_24_t qb = q8_24_from_int(b);
        printf0("int %d -> fixed -> int %d (expected 127)\n", b, q8_24_to_int(qb));
        assert(q8_24_to_int(qb) == 127);

        int c = -200; // Out of range: should saturate to -128.
        q8_24_t qc = q8_24_from_int(c);
        printf0("int %d -> fixed -> int %d (expected -128)\n", c, q8_24_to_int(qc));
        assert(q8_24_to_int(qc) == -128);
    }

    // Test conversion from float
    {
        float f = 3.14f;
        q8_24_t qf = q8_24_from_float(f);
        float f_back = q8_24_to_float(qf);
        printf0("float %f -> fixed -> float %f\n", f, f_back);
        assert(fabsf(f - f_back) < TOLERANCE);

        float f2 = 200.0f; // Out of range: should saturate.
        q8_24_t qf2 = q8_24_from_float(f2);
        float f2_back = q8_24_to_float(qf2);
        printf0("float %f -> fixed -> float %f (expected ~127.9999)\n", f2, f2_back);
        assert(q8_24_to_int(qf2) == 127);

        float f3 = -200.0f; // Out of range: should saturate.
        q8_24_t qf3 = q8_24_from_float(f3);
        float f3_back = q8_24_to_float(qf3);
        printf0("float %f -> fixed -> float %f (expected -128.0)\n", f3, f3_back);
        assert(q8_24_to_int(qf3) == -128);
    }

    printf0("\n");
    printf0("=== Q8.24 approximate_reciprocal Tests ===\n");

    // Test 1: Reciprocal of 1.0 should be approximately 1.0.
    {
        q8_24_t b = q8_24_from_float(1.0f);
        q8_24_t recip = q8_24_approximate_reciprocal(b);
        float f_recip = q8_24_to_float(recip);
        printf0("Reciprocal of 1.0: %f (expected ~1.0)\n", f_recip);
        assert(fabs(f_recip - 1.0f) < TOLERANCE);
    }

    // Test 2: Reciprocal of 2.0 should be approximately 0.5.
    {
        q8_24_t b = q8_24_from_float(2.0f);
        q8_24_t recip = q8_24_approximate_reciprocal(b);
        float f_recip = q8_24_to_float(recip);
        printf0("Reciprocal of 2.0: %f (expected ~0.5)\n", f_recip);
        assert(fabs(f_recip - 0.5f) < TOLERANCE);
    }

    // Test 3: Reciprocal of 0.5 should be approximately 2.0.
    {
        q8_24_t b = q8_24_from_float(0.5f);
        q8_24_t recip = q8_24_approximate_reciprocal(b);
        float f_recip = q8_24_to_float(recip);
        printf0("Reciprocal of 0.5: %f (expected ~2.0)\n", f_recip);
        assert(fabs(f_recip - 2.0f) < TOLERANCE);
    }

    // Test 4: Reciprocal of -1.0 should be approximately -1.0.
    {
        q8_24_t b = q8_24_from_float(-1.0f);
        q8_24_t recip = q8_24_approximate_reciprocal(b);
        float f_recip = q8_24_to_float(recip);
        printf0("Reciprocal of -1.0: %f (expected ~-1.0)\n", f_recip);
        assert(fabs(f_recip + 1.0f) < TOLERANCE);
    }

    // Test 5: Reciprocal of -2.0 should be approximately -0.5.
    {
        q8_24_t b = q8_24_from_float(-2.0f);
        q8_24_t recip = q8_24_approximate_reciprocal(b);
        float f_recip = q8_24_to_float(recip);
        printf0("Reciprocal of -2.0: %f (expected ~-0.5)\n", f_recip);
        assert(fabs(f_recip + 0.5f) < TOLERANCE);
    }

    // Test 6: Reciprocal of 0.001 should saturate to Q8_24_MAX (~127.99999994).
    {
        q8_24_t b = q8_24_from_float(0.001f);
        q8_24_t recip = q8_24_approximate_reciprocal(b);
        float f_recip = q8_24_to_float(recip);
        printf0("Reciprocal of 0.001: %f (expected saturated to Q8_24_MAX ~127.99999994)\n", f_recip);
        // Check that the result is exactly Q8_24_MAX.
        assert(recip == Q8_24_MAX);
    }

    // Test 7: Reciprocal of -0.001 should saturate to Q8_24_MIN (-128.0).
    {
        q8_24_t b = q8_24_from_float(-0.001f);
        q8_24_t recip = q8_24_approximate_reciprocal(b);
        float f_recip = q8_24_to_float(recip);
        printf0("Reciprocal of -0.001: %f (expected saturated to Q8_24_MIN -128.0)\n", f_recip);
        // Check that the result is exactly Q8_24_MIN.
        assert(recip == Q8_24_MIN);
    }

    printf0("\n");
    printf0("=== Q8.24 Arithmetic Tests with Saturation ===\n");
    // Addition saturation test:
    {
        q8_24_t max = Q8_24_MAX;
        q8_24_t one = q8_24_from_float(1.0f);
        q8_24_t add_overflow = q8_24_add(max, one);
        printf0("Addition: MAX + 1.0 = %f (expected saturation to MAX: %f)\n",
               q8_24_to_float(add_overflow), q8_24_to_float(Q8_24_MAX));
        assert(add_overflow == Q8_24_MAX);
    }

    // Subtraction saturation test:
    {
        q8_24_t min = Q8_24_MIN;
        q8_24_t one = q8_24_from_float(1.0f);
        q8_24_t sub_overflow = q8_24_sub(min, one);
        printf0("Subtraction: MIN - 1.0 = %f (expected saturation to MIN: %f)\n",
               q8_24_to_float(sub_overflow), q8_24_to_float(Q8_24_MIN));
        assert(sub_overflow == Q8_24_MIN);

        q8_24_t a = q8_24_from_int(127);
        q8_24_t b = q8_24_from_int(-1);
        q8_24_t sub_over = q8_24_sub(a, b); // 127 - (-1) = 128, out-of-range.
        printf0("Subtraction: 127 - (-1) = %f (expected saturation to MAX: %f)\n",
               q8_24_to_float(sub_over), q8_24_to_float(Q8_24_MAX));
        assert(sub_over == Q8_24_MAX);
    }

    // Multiplication saturation test:
    {
        q8_24_t hundred = q8_24_from_int(100);
        q8_24_t prod = q8_24_mul(hundred, hundred); // 100*100 = 10000, out-of-range.
        printf0("Multiplication: 100 * 100 = %f (expected saturation to MAX: %f)\n",
               q8_24_to_float(prod), q8_24_to_float(Q8_24_MAX));
        assert(prod == Q8_24_MAX);

        q8_24_t neg_hundred = q8_24_from_int(-100);
        q8_24_t prod_neg = q8_24_mul(neg_hundred, hundred); // -100*100 = -10000, out-of-range.
        printf0("Multiplication: -100 * 100 = %f (expected saturation to MIN: %f)\n",
               q8_24_to_float(prod_neg), q8_24_to_float(Q8_24_MIN));
        assert(prod_neg == Q8_24_MIN);
    }

    // Division saturation tests:
    {
        // Division by zero (positive numerator)
        q8_24_t pos = q8_24_from_int(10);
        q8_24_t div_zero = q8_24_div(pos, 0);
        printf0("Division: 10 / 0 = %f (expected saturation to MAX: %f)\n",
               q8_24_to_float(div_zero), q8_24_to_float(Q8_24_MAX));
        assert(div_zero == Q8_24_MAX);

        // Division by zero (negative numerator)
        q8_24_t neg = q8_24_from_int(-10);
        q8_24_t div_zero_neg = q8_24_div(neg, 0);
        printf0("Division: -10 / 0 = %f (expected saturation to MIN: %f)\n",
               q8_24_to_float(div_zero_neg), q8_24_to_float(Q8_24_MIN));
        assert(div_zero_neg == Q8_24_MIN);

        // Division overflow: dividing MAX by a very small positive number.
        q8_24_t tiny = q8_24_from_float(0.000001f);
        q8_24_t div_over = q8_24_div(Q8_24_MAX, tiny);
        printf0("Division: MAX / 0.000001 = %f (expected saturation to MAX: %f)\n",
               q8_24_to_float(div_over), q8_24_to_float(Q8_24_MAX));
        assert(div_over == Q8_24_MAX);

        // Division
        q8_24_t a = q8_24_from_float(42.f);
        q8_24_t b = q8_24_from_float(23.f);
        q8_24_t c = q8_24_div(a, b);
        printf0("Division: 42.0 / 23.0 = %f (expected: %f)\n",
               q8_24_to_float(c), 42.f/23.f);
        assert(fabsf(q8_24_to_float(c) - 42.f/23.f) < TOLERANCE);
    }

    // Absolute value tests with saturation:
    {
        q8_24_t pos = q8_24_from_int(50);
        q8_24_t abs_pos = q8_24_abs(pos);
        printf0("Absolute: abs(50) = %f (expected 50.0)\n", q8_24_to_float(abs_pos));
        assert(abs_pos == pos);

        // Special case: abs(Q8_24_MIN) saturates to Q8_24_MAX.
        q8_24_t abs_min = q8_24_abs(Q8_24_MIN);
        printf0("Absolute: abs(MIN) = %f (expected saturation to MAX: %f)\n",
               q8_24_to_float(abs_min), q8_24_to_float(Q8_24_MAX));
        assert(abs_min == Q8_24_MAX);
    }

    // Saturation Detection Tests:
    {
        printf0("\n");
        printf0("=== Q8.24 Saturation Detection Test ===\n");
        q8_24_t x1 = q8_24_from_float(3.14f); // Not saturated.
        q8_24_t x2 = Q8_24_MAX;               // Saturated at max.
        q8_24_t x3 = Q8_24_MIN;               // Saturated at min.

        printf0("Value %f is saturated? %d (expected 0)\n", q8_24_to_float(x1), q8_24_is_saturated(x1));
        printf0("Value %f is saturated? %d (expected 1)\n", q8_24_to_float(x2), q8_24_is_saturated(x2));
        printf0("Value %f is saturated? %d (expected 1)\n", q8_24_to_float(x3), q8_24_is_saturated(x3));

        assert(q8_24_is_saturated(x1) == 0);
        assert(q8_24_is_saturated(x2) == 1);
        assert(q8_24_is_saturated(x3) == 1);
    }



    printf0("\n");
    printf0("=== Q8.24 exp and log Tests ===\n");
    {
        q8_24_t exp1 = q8_24_exp(q8_24_from_float(1.0f));
        //printf0("exp(1) = %li.%lu (expected ~2.71828)\n", q8_24_get_int(exp1), q8_24_get_frac(exp1));
        printf0("exp(1) = %f (expected ~2.71828)\n", q8_24_to_float(exp1));
        assert(fabsf(q8_24_to_float(exp1) - 2.71828f) < 0.01f);
    }
    {
        q8_24_t log_e = q8_24_log(q8_24_from_float(2.71828f));
        //printf0("log(2.71828) = %li.%lu (expected ~1.0)\n", q8_24_get_int(log_e), q8_24_get_frac(log_e));
        printf0("log(2.71828) = %f (expected ~1.0)\n", q8_24_to_float(log_e));
        assert(fabsf(q8_24_to_float(log_e) - 1.0f) < 0.01f);
    }
    {
        q8_24_t three = q8_24_from_float(3.0f);
        q8_24_t log3 = q8_24_log(three);
        q8_24_t exp_log3 = q8_24_exp(log3);
        //printf0("exp(log(3.0)) = %li.%lu (expected ~3.0)\n", q8_24_get_int(exp_log3), q8_24_get_frac(exp_log3));
        printf0("exp(log(3.0)) = %f (expected ~3.0)\n", q8_24_to_float(exp_log3));
        assert(fabsf(q8_24_to_float(exp_log3) - 3.0f) < 0.01f);
    }
    {
        q8_24_t two = q8_24_from_float(2.0f);
        q8_24_t log_exp2 = q8_24_log(q8_24_exp(two));
        //printf0("log(exp(2.0)) = %li.%lu (expected ~2.0)\n", q8_24_get_int(log_exp2), q8_24_get_frac(log_exp2));
        printf0("log(exp(2.0)) = %f (expected ~2.0)\n", q8_24_to_float(log_exp2));
        assert(fabsf(q8_24_to_float(log_exp2) - 2.0f) < 0.01f);
    }
    {
        q8_24_t a = q8_24_from_float(-10.0f);
        q8_24_t exp_a = q8_24_exp(a);
        //printf0("exp(-10.0) = %li.%lu \n", q8_24_get_int(exp_a), q8_24_get_frac(exp_a));
        printf0("exp(-10.0) = %f \n", q8_24_to_float(exp_a));
    }


    /* --- Tests for Q1.15 Conversions --- */
    printf0("\n");
    printf0("=== Q1.15 Conversion Tests ===\n");
    {
        // Test conversion from int:
        int a = 0;
        q1_15_t qa = q1_15_from_int(a);
        printf0("int %d -> Q1.15 -> int %d\n", a, q1_15_to_int(qa));
        assert(q1_15_to_int(qa) == a);

        int b = 1;
        q1_15_t qb = q1_15_from_int(b);
        printf0("int %d -> Q1.15 -> float %f (expected saturation to nearly 1.0)\n",
               b, q1_15_to_float(qb));
        /* 1.0 is not representable exactly; the maximum is slightly less than 1.0. */
        assert(q1_15_to_float(qb) < 1.0f);

        int c = -1;
        q1_15_t qc = q1_15_from_int(c);
        printf0("int %d -> Q1.15 -> int %d (expected -1)\n", c, q1_15_to_int(qc));
        assert(q1_15_to_int(qc) == c);
    }

    {
        // Test conversion from float:
        float f = 0.5f;
        q1_15_t qf = q1_15_from_float(f);
        float f_back = q1_15_to_float(qf);
        printf0("float %f -> Q1.15 -> float %f\n", f, f_back);
        assert(fabsf(f - f_back) < TOLERANCE);

        float f2 = 1.2f; // Out of range: should saturate.
        q1_15_t qf2 = q1_15_from_float(f2);
        float f2_back = q1_15_to_float(qf2);
        printf0("float %f -> Q1.15 -> float %f (expected saturation to <1.0)\n", f2, f2_back);
        assert(f2_back < 1.0f);

        float f3 = -1.5f; // Out of range: should saturate.
        q1_15_t qf3 = q1_15_from_float(f3);
        float f3_back = q1_15_to_float(qf3);
        printf0("float %f -> Q1.15 (%d) -> float %f (expected saturation to -1.0)\n", f3, qf3, f3_back);
        assert(f3_back <= -1.0f);
    }


    printf0("\n");
    printf0("=== Q8.24 <-> Q1.15 Conversion Tests ===\n");
    // Test 1: Convert a Q8.24 value representing 0.5 to Q1.15 and back.
    {
        float original = 0.5f;
        q8_24_t fixed8 = q8_24_from_float(original);
        q1_15_t conv15 = q1_15_from_q8_24(fixed8);
        float conv15_f = q1_15_to_float(conv15);
        q8_24_t back8 = q8_24_from_q1_15(conv15);
        float back_f = q8_24_to_float(back8);
        printf0("0.5 Test:\n  Original: %f\n  Q8.24: %f\n  Converted to Q1.15: %f\n  Back to Q8.24: %f\n",
               original, q8_24_to_float(fixed8), conv15_f, back_f);
        assert(fabsf(conv15_f - original) < 0.001f);
        assert(fabsf(back_f - original) < 0.001f);
    }

    // Test 2: Convert a Q8.24 value representing -0.5 to Q1.15 and back.
    {
        float original = -0.5f;
        q8_24_t fixed8 = q8_24_from_float(original);
        q1_15_t conv15 = q1_15_from_q8_24(fixed8);
        float conv15_f = q1_15_to_float(conv15);
        q8_24_t back8 = q8_24_from_q1_15(conv15);
        float back_f = q8_24_to_float(back8);
        printf0(" -0.5 Test:\n  Original: %f\n  Q8.24: %f\n  Converted to Q1.15: %f\n  Back to Q8.24: %f\n",
               original, q8_24_to_float(fixed8), conv15_f, back_f);
        assert(fabsf(conv15_f - original) < 0.001f);
        assert(fabsf(back_f - original) < 0.001f);
    }

    // Test 3: Convert a Q8.24 value representing 1.0 (boundary case)
    // Q1.15 cannot represent 1.0; it saturates to Q1_15_MAX.
    {
        float original = 1.0f;
        q8_24_t fixed8 = q8_24_from_float(original);
        q1_15_t conv15 = q1_15_from_q8_24(fixed8);
        float conv15_f = q1_15_to_float(conv15);
        q8_24_t back8 = q8_24_from_q1_15(conv15);
        float back_f = q8_24_to_float(back8);
        printf0("1.0 Test (Saturation Expected):\n  Original: %f\n  Q8.24: %f\n  Converted to Q1.15: %f\n  Back to Q8.24: %f\n",
               original, q8_24_to_float(fixed8), conv15_f, back_f);
        // The converted Q1.15 value should be less than 1.0.
        assert(conv15_f < 1.0f);
    }

    // Test 4: Convert a Q8.24 value representing 2.0 (out-of-range, should saturate)
    {
        float original = 2.0f;
        q8_24_t fixed8 = q8_24_from_float(original);
        q1_15_t conv15 = q1_15_from_q8_24(fixed8);
        float conv15_f = q1_15_to_float(conv15);
        printf0("2.0 Test (Saturation Expected):\n  Original: %f\n  Q8.24: %f\n  Converted to Q1.15: %f\n",
               original, q8_24_to_float(fixed8), conv15_f);
        // Conversion should saturate to a value below 1.0.
        assert(conv15_f < 1.0f);
    }

    // Test 5: Convert a Q8.24 value representing -2.0 (out-of-range, should saturate)
    {
        float const original = -2.0f;
        q8_24_t const fixed8 = q8_24_from_float(original);
        q1_15_t const conv15 = q1_15_from_q8_24(fixed8);
        float const conv15_f = q1_15_to_float(conv15);
        printf0("-2.0 Test (Saturation Expected):\n  Original: %f\n  Q8.24: %li.%lu\n  Converted to Q1.15: %f\n",
               original, q8_24_get_int(fixed8), q8_24_get_frac(fixed8), conv15_f);
        // For negative values, saturation means it should be no less than -1.0.
        assert(conv15_f <= -1.0f);
    }

    // Test 6: Convert from Q1.15 to Q8.24 for a mid-range positive value.
    {
        float original = 0.75f;
        q1_15_t fixed15 = q1_15_from_float(original);
        q8_24_t conv8 = q8_24_from_q1_15(fixed15);
        float conv8_f = q8_24_to_float(conv8);
        printf0("Q1.15 -> Q8.24 Test (0.75):\n  Original Q1.15: %f\n  Converted to Q8.24: %f\n",
               q1_15_to_float(fixed15), conv8_f);
        assert(fabsf(conv8_f - original) < 0.001f);
    }

    // Test 7: Convert from Q1.15 to Q8.24 for a mid-range negative value.
    {
        float original = -0.75f;
        q1_15_t fixed15 = q1_15_from_float(original);
        q8_24_t conv8 = q8_24_from_q1_15(fixed15);
        float conv8_f = q8_24_to_float(conv8);
        printf0("Q1.15 -> Q8.24 Test (-0.75):\n  Original Q1.15: %f\n  Converted to Q8.24: %f\n",
               q1_15_to_float(fixed15), conv8_f);
        assert(fabsf(conv8_f - original) < 0.001f);
    }


    printf0("\n");
    printf0("=== Q1.15 Arithmetic Tests ===\n");
    // Test addition: 0.3 + 0.4 = 0.7
    {
        float a = 0.3f, b = 0.4f;
        q1_15_t qa = q1_15_from_float(a);
        q1_15_t qb = q1_15_from_float(b);
        q1_15_t qsum = q1_15_add(qa, qb);
        float sum_f = q1_15_to_float(qsum);
        printf0("Addition: %f + %f = %f (expected ~0.7)\n", a, b, sum_f);
        assert(fabsf(sum_f - 0.7f) < 0.01f);

        // Saturation test: MAX + a small positive value
        q1_15_t sum_sat = q1_15_add(Q1_15_MAX, q1_15_from_float(0.01f));
        printf0("Addition saturation: MAX + 0.01 = %f (expected saturation to MAX)\n", q1_15_to_float(sum_sat));
        assert(sum_sat == Q1_15_MAX);
    }

    // Test subtraction: 0.7 - 0.4 = 0.3
    {
        float a = 0.7f, b = 0.4f;
        q1_15_t qa = q1_15_from_float(a);
        q1_15_t qb = q1_15_from_float(b);
        q1_15_t qdiff = q1_15_sub(qa, qb);
        float diff_f = q1_15_to_float(qdiff);
        printf0("Subtraction: %f - %f = %f (expected ~0.3)\n", a, b, diff_f);
        assert(fabsf(diff_f - 0.3f) < 0.01f);

        // Saturation test: MIN - a small positive value
        q1_15_t sub_sat = q1_15_sub(Q1_15_MIN, q1_15_from_float(0.01f));
        printf0("Subtraction saturation: MIN - 0.01 = %f (expected saturation to MIN)\n", q1_15_to_float(sub_sat));
        assert(sub_sat == Q1_15_MIN);
    }

    // Test multiplication: 0.5 * 0.5 = 0.25
    {
        float a = 0.5f, b = 0.5f;
        q1_15_t qa = q1_15_from_float(a);
        q1_15_t qb = q1_15_from_float(b);
        q1_15_t qprod = q1_15_mul(qa, qb);
        float prod_f = q1_15_to_float(qprod);
        printf0("Multiplication: %f * %f = %f (expected ~0.25)\n", a, b, prod_f);
        assert(fabsf(prod_f - 0.25f) < 0.01f);

        // Saturation test: multiplying near maximum values should saturate.
        float c = 0.99f;
        q1_15_t qc = q1_15_from_float(c);
        q1_15_t qprod_sat = q1_15_mul(qc, qc);
        float prod_sat_f = q1_15_to_float(qprod_sat);
        printf0("Multiplication saturation: %f * %f = %f (expected saturation < 1.0)\n", c, c, prod_sat_f);
        assert(prod_sat_f < 1.0f);
    }

    // Test division:
    {
        // 0.5 / 0.5 should yield 1.0, but since 1.0 is not representable, it saturates to MAX.
        float a = 0.5f, b = 0.5f;
        q1_15_t qa = q1_15_from_float(a);
        q1_15_t qb = q1_15_from_float(b);
        q1_15_t qdiv = q1_15_div(qa, qb);
        float div_f = q1_15_to_float(qdiv);
        printf0("Division: %f / %f = %f (expected saturation to MAX ~0.99997)\n", a, b, div_f);
        assert(qdiv == Q1_15_MAX);

        // 0.25 / 0.5 = 0.5
        a = 0.25f; b = 0.5f;
        qa = q1_15_from_float(a);
        qb = q1_15_from_float(b);
        qdiv = q1_15_div(qa, qb);
        div_f = q1_15_to_float(qdiv);
        printf0("Division: %f / %f = %f (expected ~0.5)\n", a, b, div_f);
        assert(fabsf(div_f - 0.5f) < 0.01f);

        // Division by zero test.
        qa = q1_15_from_float(0.5f);
        qb = 0;
        qdiv = q1_15_div(qa, qb);
        printf0("Division by zero: 0.5 / 0 = %f (expected saturation to MAX)\n", q1_15_to_float(qdiv));
        assert(qdiv == Q1_15_MAX);
    }

    // Test absolute value:
    {
        float a = -0.75f;
        q1_15_t qa = q1_15_from_float(a);
        q1_15_t qabs = q1_15_abs(qa);
        float abs_f = q1_15_to_float(qabs);
        printf0("Absolute: abs(%f) = %f (expected ~0.75)\n", a, abs_f);
        assert(fabsf(abs_f - 0.75f) < 0.01f);

        // Special case: abs(Q1_15_MIN) should saturate to Q1_15_MAX.
        qabs = q1_15_abs(Q1_15_MIN);
        printf0("Absolute: abs(MIN) = %f (expected saturation to MAX)\n", q1_15_to_float(qabs));
        assert(qabs == Q1_15_MAX);
    }

    /* --- Q1.15 Saturation Detection Tests --- */
    printf0("\n");
    printf0("=== Q1.15 Saturation Detection Tests ===\n");
    {
        // A value that is not saturated.
        q1_15_t a = q1_15_from_float(0.5f);
        printf0("Value %f is saturated? %d (expected 0)\n", q1_15_to_float(a), q1_15_is_saturated(a));
        assert(q1_15_is_saturated(a) == 0);
    }
    {
        // A value saturated at maximum.
        q1_15_t b = Q1_15_MAX;
        printf0("Value %f is saturated? %d (expected 1)\n", q1_15_to_float(b), q1_15_is_saturated(b));
        assert(q1_15_is_saturated(b) == 1);
    }
    {
        // A value saturated at minimum.
        q1_15_t c = Q1_15_MIN;
        printf0("Value %f is saturated? %d (expected 1)\n", q1_15_to_float(c), q1_15_is_saturated(c));
        assert(q1_15_is_saturated(c) == 1);
    }


    printf0("\n");
    printf0("=== Q1.15 exp and log Tests ===\n");
    /* q1_15_exp tests */
    {
        // Test 1: For nonnegative input, q1_15_exp should saturate to Q1_15_MAX.
        q1_15_t exp0 = q1_15_exp(q1_15_from_float(0.0f));
        float exp0_f = q1_15_to_float(exp0);
        printf0("q1_15_exp(0.0) = %f (expected ~1.0, saturated to max)\n", exp0_f);
        assert(exp0 == Q1_15_MAX);
    }
    {
        // Test 2: exp(-ln2) should be about 0.5.
        q1_15_t exp_neg_ln2 = q1_15_exp(q1_15_from_float(-0.693147f));
        float exp_neg_ln2_f = q1_15_to_float(exp_neg_ln2);
        printf0("q1_15_exp(-0.693147) = %f (expected ~0.5)\n", exp_neg_ln2_f);
        assert(fabsf(exp_neg_ln2_f - 0.5f) < TOLERANCE);
    }
    {
        // Test 3: exp(-1.0) should be close to exp(-1) ≈ 0.3679.
        q1_15_t const exp_neg1 = q1_15_exp(q1_15_from_float(-1.0f));
        float const exp_neg1_f = q1_15_to_float(exp_neg1);
        printf0("q1_15_exp(-1.0) = %d.%u (expected ~0.3679)\n", q1_15_get_int(exp_neg1), q1_15_get_frac(exp_neg1));
        assert(fabsf(exp_neg1_f - 0.3679f) < TOLERANCE);
    }

    /* q1_15_log tests */
    {
        // Test 5: log(1.0) should be 0.
        q1_15_t log1 = q1_15_log(q1_15_from_float(1.0f));
        float log1_f = q1_15_to_float(log1);
        printf0("q1_15_log(1.0) = %f (expected 0)\n", log1_f);
        assert(fabsf(log1_f) < TOLERANCE);
    }
    // XXX
    {
        // Test 6: log(0.5) should be about ln(0.5) ≈ -0.693147.
        q1_15_t const log05 = q1_15_log(q1_15_from_float(0.5f));
        float const log05_f = q1_15_to_float(log05);
        printf0("q1_15_log(0.5) = %d.%u (expected ~ -0.693147)\n", (int16_t)q1_15_get_int(log05), (uint16_t)q1_15_get_frac(log05));
        assert(fabsf(log05_f + 0.693147f) < TOLERANCE);
    }
    // XXX
    {
        // Test 7: log(0.3679) should be about ln(0.3679) ≈ -1.0.
        q1_15_t log037 = q1_15_log(q1_15_from_float(0.3679f));
        float log037_f = q1_15_to_float(log037);
        printf0("q1_15_log(0.3679) = %f (expected ~ -1.0)\n", log037_f);
        assert(fabsf(log037_f + 1.0f) < TOLERANCE);
    }
    {
        // Test 8: log(-0.5) is undefined; the function should return Q1_15_MIN.
        q1_15_t log_neg = q1_15_log(q1_15_from_float(-0.5f));
        float log_neg_f = q1_15_to_float(log_neg);
        printf0("q1_15_log(-0.5) = %f (expected saturation to MIN)\n", log_neg_f);
        assert(log_neg == Q1_15_MIN);
    }


    printf0("\n");
    printf0("=== Q16.16 Conversion Tests ===\n");
    /* Test int <-> Q16.16 */
    {
        int a = 42;
        q16_16_t qa = q16_16_from_int(a);
        int a_back = q16_16_to_int(qa);
        printf0("int %d -> Q16.16 -> int %d\n", a, a_back);
        assert(a == a_back);
    }

    /* Test float <-> Q16.16 */
    {
        float f = 3.14159f;
        q16_16_t qf = q16_16_from_float(f);
        float f_back = q16_16_to_float(qf);
        printf0("float %f -> Q16.16 -> float %f\n", f, f_back);
        assert(fabsf(f - f_back) < TOLERANCE);
    }

    /* Test Q8.24 <-> Q16.16 */
    {
        float f_val = 2.71828f;
        int32_t q8_val = q8_24_from_float(f_val);
        q16_16_t q16_val = q16_16_from_q8_24(q8_val);
        int32_t q8_back = q8_24_from_q16_16(q16_val);
        float f_q8 = q8_24_to_float(q8_val);
        float f_q8_back = q8_24_to_float(q8_back);
        printf0("Q8.24 %f -> Q16.16 -> Q8.24 %f\n", f_q8, f_q8_back);
        assert(fabsf(f_q8 - f_q8_back) < TOLERANCE);
    }

    /* Test Q1.15 <-> Q16.16 */
    {
        float f_val = -0.5f;
        int16_t q1_val = q1_15_from_float(f_val);
        q16_16_t q16_from_q1 = q16_16_from_q1_15(q1_val);
        int16_t q1_back = q1_15_from_q16_16(q16_from_q1);
        float f_q1 = q1_15_to_float(q1_val);
        float f_q1_back = q1_15_to_float(q1_back);
        printf0("Q1.15 %f -> Q16.16 -> Q1.15 %f\n", f_q1, f_q1_back);
        assert(fabsf(f_q1 - f_q1_back) < TOLERANCE);
    }

    printf0("\n");
    printf0("=== Q16.16 approximate_reciprocal Tests ===\n");

    // Test 1: Reciprocal of 1.0 should be approximately 1.0.
    {
        q16_16_t b = q16_16_from_float(1.0f);
        q16_16_t recip = q16_16_approximate_reciprocal(b);
        float f_recip = q16_16_to_float(recip);
        printf0("Reciprocal of 1.0: %f (expected ~1.0)\n", f_recip);
        assert(fabsf(f_recip - 1.0f) < TOLERANCE);
    }

    // Test 2: Reciprocal of 2.0 should be approximately 0.5.
    {
        q16_16_t b = q16_16_from_float(2.0f);
        q16_16_t recip = q16_16_approximate_reciprocal(b);
        float f_recip = q16_16_to_float(recip);
        printf0("Reciprocal of 2.0: %f (expected ~0.5)\n", f_recip);
        assert(fabsf(f_recip - 0.5f) < TOLERANCE);
    }

    // Test 3: Reciprocal of 0.5 should be approximately 2.0.
    {
        q16_16_t b = q16_16_from_float(0.5f);
        q16_16_t recip = q16_16_approximate_reciprocal(b);
        float f_recip = q16_16_to_float(recip);
        printf0("Reciprocal of 0.5: %f (expected ~2.0)\n", f_recip);
        assert(fabsf(f_recip - 2.0f) < TOLERANCE);
    }

    // Test 4: Reciprocal of -1.0 should be approximately -1.0.
    {
        q16_16_t b = q16_16_from_float(-1.0f);
        q16_16_t recip = q16_16_approximate_reciprocal(b);
        float f_recip = q16_16_to_float(recip);
        printf0("Reciprocal of -1.0: %f (expected ~-1.0)\n", f_recip);
        assert(fabsf(f_recip + 1.0f) < TOLERANCE);
    }

    // Test 5: Reciprocal of -2.0 should be approximately -0.5.
    {
        q16_16_t b = q16_16_from_float(-2.0f);
        q16_16_t recip = q16_16_approximate_reciprocal(b);
        float f_recip = q16_16_to_float(recip);
        printf0("Reciprocal of -2.0: %f (expected ~-0.5)\n", f_recip);
        assert(fabsf(f_recip + 0.5f) < TOLERANCE);
    }

    // Saturation Tests:
    // Test 6: For a very small positive value b, 1/b exceeds the representable range.
    // In Q16.16, the smallest b that can be inverted without saturation is 2 (i.e. 2/65536 ≈ 0.000030517578125).
    // Hence, for any b <= 2 (in Q16.16 representation), the reciprocal should be saturated to Q16_16_MAX.
    {
        // 0.00003 * 65536 ≈ 1.96608, which is below the threshold.
        q16_16_t b = q16_16_from_float(0.00003f);
        q16_16_t recip = q16_16_approximate_reciprocal(b);
        float f_recip = q16_16_to_float(recip);
        printf0("Reciprocal of 0.00003: %f (expected saturated to Q16_16_MAX ~32767.99998)\n", f_recip);
        assert(recip == Q16_16_MAX);
    }

    // Test 7: For a very small negative value b, the reciprocal should be saturated to Q16_16_MIN.
    {
        q16_16_t b = q16_16_from_float(-0.00003f);
        q16_16_t recip = q16_16_approximate_reciprocal(b);
        float f_recip = q16_16_to_float(recip);
        printf0("Reciprocal of -0.00003: %f (expected saturated to Q16_16_MIN -32768.0)\n", f_recip);
        assert(recip == Q16_16_MIN);
    }


    printf0("\n");
    printf0("=== Q16.16 Arithmetic Tests ===\n");

    /* Test int <-> Q16.16 conversions */
    {
        int a = 42;
        q16_16_t qa = q16_16_from_int(a);
        int a_back = q16_16_to_int(qa);
        printf0("int %d -> Q16.16 -> int %d\n", a, a_back);
        assert(a == a_back);
    }

    /* Test float <-> Q16.16 conversions */
    {
        float f = 3.14159f;
        q16_16_t qf = q16_16_from_float(f);
        float f_back = q16_16_to_float(qf);
        printf0("float %f -> Q16.16 -> float %f\n", f, f_back);
        assert(fabsf(f - f_back) < TOLERANCE);
    }

    /* Test addition */
    {
        q16_16_t x = q16_16_from_float(1.5f);
        q16_16_t y = q16_16_from_float(2.25f);
        q16_16_t sum = q16_16_add(x, y);
        float sum_f = q16_16_to_float(sum);
        printf0("Addition: 1.5 + 2.25 = %f (expected 3.75)\n", sum_f);
        assert(fabsf(sum_f - 3.75f) < TOLERANCE);
    }

    /* Test subtraction */
    {
        q16_16_t x = q16_16_from_float(5.0f);
        q16_16_t y = q16_16_from_float(2.0f);
        q16_16_t diff = q16_16_sub(x, y);
        float diff_f = q16_16_to_float(diff);
        printf0("Subtraction: 5.0 - 2.0 = %f (expected 3.0)\n", diff_f);
        assert(fabsf(diff_f - 3.0f) < TOLERANCE);
    }

    /* Test multiplication */
    {
        q16_16_t x = q16_16_from_float(2.0f);
        q16_16_t y = q16_16_from_float(3.5f);
        q16_16_t prod = q16_16_mul(x, y);
        float prod_f = q16_16_to_float(prod);
        printf0("Multiplication: 2.0 * 3.5 = %f (expected 7.0)\n", prod_f);
        assert(fabsf(prod_f - 7.0f) < TOLERANCE);
    }

#undef TOLERANCE
#define TOLERANCE 1e-1f

    /* Test division */
    {
        q16_16_t x = q16_16_from_float(7.0f);
        q16_16_t y = q16_16_from_float(2.0f);
        q16_16_t quot = q16_16_div(x, y);
        float quot_f = q16_16_to_float(quot);
        printf0("Division: 7.0 / 2.0 = %f (expected 3.5)\n", quot_f);
        assert(fabsf(quot_f - 3.5f) < TOLERANCE);
    }

    /* Test division by zero */
    {
        q16_16_t x = q16_16_from_float(1.0f);
        q16_16_t quot = q16_16_div(x, 0);
        float quot_f = q16_16_to_float(quot);
        printf0("Division: 1.0 / 0 = %f (expected saturation to max ~32767.9999... scaled, here Q16.16_MAX)\n", quot_f);
        // Here we simply check that the result equals Q16_16_MAX (if x is nonnegative)
        assert(quot == Q16_16_MAX);
    }

#undef TOLERANCE
#define TOLERANCE 1e-2f

    /* Test absolute value */
    {
        q16_16_t pos = q16_16_from_float(3.0f);
        q16_16_t abs_pos = q16_16_abs(pos);
        float abs_pos_f = q16_16_to_float(abs_pos);
        printf0("Absolute: abs(3.0) = %f (expected 3.0)\n", abs_pos_f);
        assert(fabsf(abs_pos_f - 3.0f) < TOLERANCE);

        q16_16_t neg = q16_16_from_float(-4.5f);
        q16_16_t abs_neg = q16_16_abs(neg);
        float abs_neg_f = q16_16_to_float(abs_neg);
        printf0("Absolute: abs(-4.5) = %f (expected 4.5)\n", abs_neg_f);
        assert(fabsf(abs_neg_f - 4.5f) < TOLERANCE);
    }

    // XXX
    // Test exp
    {
        q16_16_t zero = q16_16_from_float(0.0f);
        q16_16_t exp0 = q16_16_exp(zero);
        float f_exp0 = q16_16_to_float(exp0);
        printf0("q16_16_exp(0.0) = %f (expected 1.0)\n", f_exp0);
        assert(fabsf(f_exp0 - 1.0f) < TOLERANCE);
    }
    // XXX
    {
        q16_16_t neg_ln2 = q16_16_from_float(-0.693147f);
        q16_16_t exp_neg_ln2 = q16_16_exp(neg_ln2);
        float f_exp_neg_ln2 = q16_16_to_float(exp_neg_ln2);
        printf0("q16_16_exp(-0.693147) = %f (expected ~0.5)\n", f_exp_neg_ln2);
        assert(fabsf(f_exp_neg_ln2 - 0.5f) < TOLERANCE);
    }
    // XXX
    {
        q16_16_t const neg1 = Q16_16_NEG_ONE;
        q16_16_t const exp_neg1 = q16_16_exp(neg1);
        float const f_exp_neg1 = q16_16_to_float(exp_neg1);
        printf0("q16_16_exp(-1.0) = %f (expected ~0.3679)\n", f_exp_neg1);
        assert(fabsf((float)f_exp_neg1 - 0.3679f) < TOLERANCE);
    }

    printf0("\n");
    printf0("=== Q16.16 exp and refined log Tests ===\n");

    {
        // Test exp(-1.0)
        q16_16_t neg1 = q16_16_from_float(-1.0f);
        q16_16_t exp_neg1 = q16_16_exp(neg1);
        float f_exp_neg1 = q16_16_to_float(exp_neg1);
        printf0("q16_16_exp(-1.0) = %f (expected ~0.3679)\n", f_exp_neg1);
        assert(fabsf(f_exp_neg1 - 0.3679f) < TOLERANCE);
    }
    {
        // Test log(1.0)
        q16_16_t one = q16_16_from_float(1.0f);
        q16_16_t log_one = q16_16_log(one);
        float f_log_one = q16_16_to_float(log_one);
        printf0("q16_16_log(1.0) = %f (expected 0.0)\n", f_log_one);
        assert(fabsf(f_log_one - 0.0f) < TOLERANCE);
    }
    {
        // Test refined log(exp(-1.0)) should be near -1.0.
        q16_16_t neg1 = q16_16_from_float(-1.0f);
        q16_16_t exp_neg1 = q16_16_exp(neg1);
        q16_16_t log_exp_neg1 = q16_16_log(exp_neg1);
        float f_log_exp_neg1 = q16_16_to_float(log_exp_neg1);
        printf0("q16_16_log(exp(-1.0)) = %f (expected ~ -1.0)\n", f_log_exp_neg1);
        assert(fabsf(f_log_exp_neg1 + 1.0f) < TOLERANCE);
    }

    // Test 1: exp(0.0) should equal 1.0.
    {
        q16_16_t zero = q16_16_from_float(0.0f);
        q16_16_t exp_zero = q16_16_exp(zero);
        float f_exp_zero = q16_16_to_float(exp_zero);
        printf0("q16_16_exp(0.0) = %f (expected ~1.0)\n", f_exp_zero);
        assert(fabsf(f_exp_zero - 1.0f) < TOLERANCE);
    }

    // Test 2: exp(1.0) should equal approximately 2.71828.
    {
        q16_16_t one = q16_16_from_float(1.0f);
        q16_16_t exp_one = q16_16_exp(one);
        float f_exp_one = q16_16_to_float(exp_one);
        printf0("q16_16_exp(1.0) = %f (expected ~2.71828)\n", f_exp_one);
        assert(fabsf(f_exp_one - 2.71828f) < TOLERANCE);
    }

    // Test 3: exp(2.0) should be approximately 7.38906.
    {
        q16_16_t two = q16_16_from_float(2.0f);
        q16_16_t exp_two = q16_16_exp(two);
        float f_exp_two = q16_16_to_float(exp_two);
        printf0("q16_16_exp(2.0) = %f (expected ~7.38906)\n", f_exp_two);
        assert(fabsf(f_exp_two - 7.38906f) < TOLERANCE);
    }

    // Test 4: log(exp(0.5)) should be approximately 0.5.
    {
        q16_16_t half = q16_16_from_float(0.5f);
        q16_16_t exp_half = q16_16_exp(half);
        q16_16_t log_exp_half = q16_16_log(exp_half);
        float f_log_exp_half = q16_16_to_float(log_exp_half);
        printf0("q16_16_log(exp(0.5)) = %f (expected ~0.5)\n", f_log_exp_half);
        assert(fabsf(f_log_exp_half - 0.5f) < TOLERANCE);
    }

    // Test 5: log(exp(2.5)) should be approximately 2.5.
    {
        q16_16_t twoPoint5 = q16_16_from_float(2.5f);
        q16_16_t exp_twoPoint5 = q16_16_exp(twoPoint5);
        q16_16_t log_exp_twoPoint5 = q16_16_log(exp_twoPoint5);
        float f_log_exp_twoPoint5 = q16_16_to_float(log_exp_twoPoint5);
        printf0("q16_16_log(exp(2.5)) = %f (expected ~2.5)\n", f_log_exp_twoPoint5);
        assert(fabsf(f_log_exp_twoPoint5 - 2.5f) < TOLERANCE);
    }

    // Test 6: log(2.71828) should be approximately 1.0.
    {
        q16_16_t eVal = q16_16_from_float(2.71828f);
        q16_16_t log_eVal = q16_16_log(eVal);
        float f_log_eVal = q16_16_to_float(log_eVal);
        printf0("q16_16_log(2.71828) = %f (expected ~1.0)\n", f_log_eVal);
        assert(fabsf(f_log_eVal - 1.0f) < TOLERANCE);
    }

    // Test 7: exp(log(5.0)) should be approximately 5.0.
    {
        q16_16_t five = q16_16_from_float(5.0f);
        q16_16_t log_five = q16_16_log(five);
        q16_16_t exp_log_five = q16_16_exp(log_five);
        float f_exp_log_five = q16_16_to_float(exp_log_five);
        printf0("q16_16_exp(log(5.0)) = %f (expected ~5.0)\n", f_exp_log_five);
        assert(fabsf(f_exp_log_five - 5.0f) < TOLERANCE);
    }

    // Test 8: log(exp(-2.0)) should be approximately -2.0.
    {
        q16_16_t neg2 = q16_16_from_float(-2.0f);
        q16_16_t exp_neg2 = q16_16_exp(neg2);
        q16_16_t log_exp_neg2 = q16_16_log(exp_neg2);
        float f_log_exp_neg2 = q16_16_to_float(log_exp_neg2);
        printf0("q16_16_log(exp(-2.0)) = %f (expected ~ -2.0)\n", f_log_exp_neg2);
        assert(fabsf(f_log_exp_neg2 + 2.0f) < TOLERANCE);
    }

    // Test 9: exp(log(0.1)) should be approximately 0.1.
    {
        q16_16_t oneTenth = q16_16_from_float(0.1f);
        q16_16_t log_oneTenth = q16_16_log(oneTenth);
        q16_16_t exp_log_oneTenth = q16_16_exp(log_oneTenth);
        float f_exp_log_oneTenth = q16_16_to_float(exp_log_oneTenth);
        printf0("q16_16_exp(log(0.1)) = %f (expected ~0.1)\n", f_exp_log_oneTenth);
        assert(fabsf(f_exp_log_oneTenth - 0.1f) < TOLERANCE);
    }

    // Test 10: log(exp(10.0)) should be approximately 10.0.
    {
        q16_16_t ten = q16_16_from_float(10.0f);
        q16_16_t exp_ten = q16_16_exp(ten);
        q16_16_t log_exp_ten = q16_16_log(exp_ten);
        float f_log_exp_ten = q16_16_to_float(log_exp_ten);
        printf0("q16_16_log(exp(10.0)) = %f (expected ~10.0)\n", f_log_exp_ten);
        assert(fabsf(f_log_exp_ten - 10.0f) < TOLERANCE);
    }

    // Test 11: log(exp(-1.0)) from the original test (repeated here for completeness).
    {
        q16_16_t neg1 = q16_16_from_float(-1.0f);
        q16_16_t exp_neg1 = q16_16_exp(neg1);
        q16_16_t log_exp_neg1 = q16_16_log(exp_neg1);
        float f_log_exp_neg1 = q16_16_to_float(log_exp_neg1);
        printf0("q16_16_log(exp(-1.0)) = %f (expected ~ -1.0)\n", f_log_exp_neg1);
        assert(fabsf(f_log_exp_neg1 + 1.0f) < TOLERANCE);
    }

    // Test 12: exp(log(exp(3.0))) should return exp(3.0) (i.e. composition recovers the value).
    {
        q16_16_t three = q16_16_from_float(3.0f);
        q16_16_t exp_three = q16_16_exp(three);
        q16_16_t log_exp_three = q16_16_log(exp_three);
        q16_16_t exp_log_exp_three = q16_16_exp(log_exp_three);
        float f_exp_log_exp_three = q16_16_to_float(exp_log_exp_three);
        float f_exp_three = q16_16_to_float(exp_three);
        printf0("q16_16_exp(log(exp(3.0))) = %f (expected ~ %f)\n", f_exp_log_exp_three, f_exp_three);
        assert(fabsf(f_exp_log_exp_three - f_exp_three) < TOLERANCE);
    }



#undef TOLERANCE
#define TOLERANCE 1e-2f

    printf0("\n");
    printf0("=== Q6.10 Conversion Tests ===\n");

    /* Test int <-> Q6.10 */
    {
        int a = 7;
        q6_10_t qa = q6_10_from_int(a);
        int a_back = q6_10_to_int(qa);
        printf0("int %d -> Q6.10 -> int %d\n", a, a_back);
        assert(a == a_back);
    }

    /* Test float <-> Q6.10 */
    {
        float f = 3.14159f;
        q6_10_t qf = q6_10_from_float(f);
        float f_back = q6_10_to_float(qf);
        printf0("float %f -> Q6.10 -> float %f\n", f, f_back);
        assert(fabsf(f - f_back) < TOLERANCE);
    }
    /* Test float <-> Q6.10 */
    {
        float f = -1.1f;
        q6_10_t qf = q6_10_from_float(f);
        float f_back = q6_10_to_float(qf);
        printf0("float %f -> Q6.10 (%hu) -> float %f\n", f, qf, f_back);
        assert(fabsf(f - f_back) < TOLERANCE);
    }

    /* Test Q8.24 <-> Q6.10 */
    {
        // For Q8.24, the scaling factor is 2^24.
        float f_val = 2.71828f;
        int32_t q8_val = (int32_t)(f_val * (1 << 24));
        q6_10_t q6_from_q8 = q6_10_from_q8_24(q8_val);
        int32_t q8_back = q8_24_from_q6_10(q6_from_q8);
        float f_q8 = (float)q8_val / (1 << 24);
        float f_q8_back = (float)q8_back / (1 << 24);
        printf0("Q8.24 %f -> Q6.10 -> Q8.24 %f\n", f_q8, f_q8_back);
        assert(fabsf(f_q8 - f_q8_back) < TOLERANCE);
    }

    /* Test Q1.15 <-> Q6.10 */
    {
        // For Q1.15, the scaling factor is 2^15.
        float f_val = 0.75f;
        int16_t q1_val = (int16_t)(f_val * (1 << 15));
        q6_10_t q6_from_q1 = q6_10_from_q1_15(q1_val);
        int16_t q1_back = q1_15_from_q6_10(q6_from_q1);
        float f_q1 = (float)q1_val / (1 << 15);
        float f_q1_back = (float)q1_back / (1 << 15);
        printf0("Q1.15 %f -> Q6.10 -> Q1.15 %f\n", f_q1, f_q1_back);
        assert(fabsf(f_q1 - f_q1_back) < TOLERANCE);
    }

    /* Test Q16.16 <-> Q6.10 */
    {
        // For Q16.16, the scaling factor is 2^16.
        float f_val = 1.2345f;
        int32_t q16_val = (int32_t)(f_val * (1 << 16));
        q6_10_t q6_from_q16 = q6_10_from_q16_16(q16_val);
        int32_t q16_back = q16_16_from_q6_10(q6_from_q16);
        float f_q16 = (float)q16_val / (1 << 16);
        float f_q16_back = (float)q16_back / (1 << 16);
        printf0("Q16.16 %f -> Q6.10 -> Q16.16 %f\n", f_q16, f_q16_back);
        assert(fabsf(f_q16 - f_q16_back) < TOLERANCE);
    }

    printf0("\n");
    printf0("=== Q6.10 Arithmetic Tests ===\n");

    /* Test Addition: 1.5 + 2.25 = 3.75 */
    {
        q6_10_t a = q6_10_from_float(1.5f);
        q6_10_t b = q6_10_from_float(2.25f);
        q6_10_t sum = q6_10_add(a, b);
        float f_sum = q6_10_to_float(sum);
        printf0("Addition: 1.5 + 2.25 = %f (expected 3.75)\n", f_sum);
        assert(fabsf(f_sum - 3.75f) < TOLERANCE);
    }

    /* Test Addition Saturation: 20.0 + 20.0 exceeds the maximum (~32.0) */
    {
        q6_10_t c = q6_10_from_float(20.0f);
        q6_10_t d = q6_10_from_float(20.0f);
        q6_10_t sat_sum = q6_10_add(c, d);
        float f_sat_sum = q6_10_to_float(sat_sum);
        printf0("Addition (saturation): 20.0 + 20.0 = %f (expected ~31.999)\n", f_sat_sum);
        // Q6_10_MAX is 0x7FFF = 32767; 32767/1024 ≈ 31.9990.
        assert(fabsf(f_sat_sum - 31.9990f) < TOLERANCE);
    }

    /* Test Subtraction: 10.0 - 3.5 = 6.5 */
    {
        q6_10_t e = q6_10_from_float(10.0f);
        q6_10_t f = q6_10_from_float(3.5f);
        q6_10_t diff = q6_10_sub(e, f);
        float f_diff = q6_10_to_float(diff);
        printf0("Subtraction: 10.0 - 3.5 = %f (expected 6.5)\n", f_diff);
        assert(fabsf(f_diff - 6.5f) < TOLERANCE);
    }

    /* Test Subtraction Saturation: -20.0 - 20.0 = -40, but minimum is -32.0 */
    {
        q6_10_t g = q6_10_from_float(-20.0f);
        q6_10_t h = q6_10_from_float(20.0f);
        q6_10_t sat_diff = q6_10_sub(g, h);
        float f_sat_diff = q6_10_to_float(sat_diff);
        printf0("Subtraction (saturation): -20.0 - 20.0 = %f (expected -32.0)\n", f_sat_diff);
        assert(fabsf(f_sat_diff + 32.0f) < TOLERANCE);
    }

    /* Test Multiplication: 2.0 * 3.5 = 7.0 */
    {
        q6_10_t i = q6_10_from_float(2.0f);
        q6_10_t j = q6_10_from_float(3.5f);
        q6_10_t prod = q6_10_mul(i, j);
        float f_prod = q6_10_to_float(prod);
        printf0("Multiplication: 2.0 * 3.5 = %f (expected 7.0)\n", f_prod);
        assert(fabsf(f_prod - 7.0f) < TOLERANCE);
    }

    /* Test Multiplication Saturation: 20.0 * 2.0 = 40, saturates to ~31.999 */
    {
        q6_10_t k = q6_10_from_float(20.0f);
        q6_10_t l = q6_10_from_float(2.0f);
        q6_10_t sat_prod = q6_10_mul(k, l);
        float f_sat_prod = q6_10_to_float(sat_prod);
        printf0("Multiplication (saturation): 20.0 * 2.0 = %f (expected ~31.999)\n", f_sat_prod);
        assert(fabsf(f_sat_prod - 31.9990f) < TOLERANCE);
    }

    /* Test Division: 7.0 / 2.0 = 3.5 */
    {
        q6_10_t m = q6_10_from_float(7.0f);
        q6_10_t n = q6_10_from_float(2.0f);
        q6_10_t quot = q6_10_div(m, n);
        float f_quot = q6_10_to_float(quot);
        printf0("Division: 7.0 / 2.0 = %f (expected 3.5)\n", f_quot);
        assert(fabsf(f_quot - 3.5f) < TOLERANCE);
    }

    /* Test Division by Zero: 1.0 / 0 should saturate */
    {
        q6_10_t one_val = q6_10_from_float(1.0f);
        q6_10_t div0 = q6_10_div(one_val, 0);
        float f_div0 = q6_10_to_float(div0);
        printf0("Division by zero: 1.0 / 0 = %f (expected saturation to Q6_10_MAX, ~31.999)\n", f_div0);
        assert(div0 == Q6_10_MAX);
    }

    /* Test Absolute Value */
    {
        q6_10_t pos_val = q6_10_from_float(3.0f);
        q6_10_t abs_pos = q6_10_abs(pos_val);
        float f_abs_pos = q6_10_to_float(abs_pos);
        printf0("Absolute: abs(3.0) = %f (expected 3.0)\n", f_abs_pos);
        assert(fabsf(f_abs_pos - 3.0f) < TOLERANCE);

        q6_10_t neg_val = q6_10_from_float(-4.5f);
        q6_10_t abs_neg = q6_10_abs(neg_val);
        float f_abs_neg = q6_10_to_float(abs_neg);
        printf0("Absolute: abs(-4.5) = %f (expected 4.5)\n", f_abs_neg);
        assert(fabsf(f_abs_neg - 4.5f) < TOLERANCE);
    }

    /* Test Saturation Detection */
    {
        q6_10_t sat_max = Q6_10_MAX;
        q6_10_t sat_min = Q6_10_MIN;
        printf0("Saturation check: Q6_10_MAX is saturated? %d (expected 1)\n", q6_10_is_saturated(sat_max));
        printf0("Saturation check: Q6_10_MIN is saturated? %d (expected 1)\n", q6_10_is_saturated(sat_min));
        assert(q6_10_is_saturated(sat_max) == 1);
        assert(q6_10_is_saturated(sat_min) == 1);

        q6_10_t non_sat = q6_10_from_float(5.0f);
        printf0("Saturation check: 5.0 in Q6.10 is saturated? %d (expected 0)\n", q6_10_is_saturated(non_sat));
        assert(q6_10_is_saturated(non_sat) == 0);
    }


    printf0("\n");
    printf0("=== Q6.10 Exp and Log Tests ===\n");

    // XXX
    /* Test exp */
    {
        q6_10_t zero = q6_10_from_float(0.0f);
        q6_10_t exp0 = q6_10_exp(zero);
        float f_exp0 = q6_10_to_float(exp0);
        printf0("q6_10_exp(0.0) = %f (expected 1.0)\n", f_exp0);
        assert(fabsf(f_exp0 - 1.0f) < TOLERANCE);
    }
    // XXX
    {
        q6_10_t neg_ln2 = q6_10_from_float(-0.693147f);
        q6_10_t exp_neg_ln2 = q6_10_exp(neg_ln2);
        float f_exp_neg_ln2 = q6_10_to_float(exp_neg_ln2);
        printf0("q6_10_exp(-0.693147) = %f (expected ~0.5)\n", f_exp_neg_ln2);
        assert(fabsf(f_exp_neg_ln2 - 0.5f) < TOLERANCE);
    }
    // XXX
    {
        q6_10_t neg1 = q6_10_from_float(-1.0f);
        q6_10_t exp_neg1 = q6_10_exp(neg1);
        float f_exp_neg1 = q6_10_to_float(exp_neg1);
        printf0("q6_10_exp(-1.0) = %f (expected ~0.3679)\n", f_exp_neg1);
        assert(fabsf(f_exp_neg1 - 0.3679f) < TOLERANCE);
    }
    // XXX

    /* Test log */
    {
        q6_10_t one = q6_10_from_float(1.0f);
        q6_10_t log1 = q6_10_log(one);
        float f_log1 = q6_10_to_float(log1);
        printf0("q6_10_log(1.0) = %f (expected 0.0)\n", f_log1);
        assert(fabsf(f_log1 - 0.0f) < TOLERANCE);
    }
    // XXX
    {
        q6_10_t half = q6_10_from_float(0.5f);
        q6_10_t log_half = q6_10_log(half);
        float f_log_half = q6_10_to_float(log_half);
        printf0("q6_10_log(0.5) = %f (expected ~ -0.693147)\n", f_log_half);
        assert(fabsf(f_log_half + 0.693147f) < TOLERANCE);
    }
    // XXX
    {
        q6_10_t half = q6_10_from_float(0.366211f);
        q6_10_t log_half = q6_10_log(half);
        float f_log_half = q6_10_to_float(log_half);
        printf0("q6_10_log(0.366211) = %f (expected ~ -1.0)\n", f_log_half);
        assert(fabsf(f_log_half + 1.0f) < TOLERANCE);
    }
    // XXX
    {
        q6_10_t neg1 = q6_10_from_float(-1.0f);
        q6_10_t exp_neg1_val = q6_10_exp(neg1);  // ~0.3679
        q6_10_t log_exp_neg1 = q6_10_log(exp_neg1_val);
        float f_log_exp_neg1 = q6_10_to_float(log_exp_neg1);
        printf0("q6_10_log(exp(-1.0)) = %f (expected ~ -1.0)\n", f_log_exp_neg1);
        assert(fabsf(f_log_exp_neg1 + 1.0f) < TOLERANCE);
    }


    printf0("\n");
    printf0("=== Fixed-Point Comparison Tests ===\n");

    /* --- Q8.24 Comparisons --- */
    {
        // Create some Q8.24 values.
        q8_24_t a = q8_24_from_float(3.14159f);
        q8_24_t b = q8_24_from_float(3.14159f);
        q8_24_t c = q8_24_from_float(2.71828f);
        q8_24_t d = q8_24_from_float(4.0f);

        // Check equality.
        assert(a == b);
        // Check ordering.
        assert(c < a);
        assert(a < d);
        // Also check the inverse.
        assert(d > a);

        printf0("Q8.24: %f == %f, %f < %f, %f < %f\n",
               q8_24_to_float(a), q8_24_to_float(b),
               q8_24_to_float(c), q8_24_to_float(a),
               q8_24_to_float(a), q8_24_to_float(d));
        printf0("Q8.24 comparisons passed.\n\n");
    }

    /* --- Q1.15 Comparisons --- */
    {
        q1_15_t a = q1_15_from_float(0.5f);
        q1_15_t b = q1_15_from_float(0.5f);
        q1_15_t c = q1_15_from_float(-0.25f);
        q1_15_t d = q1_15_from_float(0.75f);

        assert(a == b);
        assert(c < a);
        assert(a < d);

        printf0("Q1.15: %f == %f, %f < %f, %f < %f\n",
               q1_15_to_float(a), q1_15_to_float(b),
               q1_15_to_float(c), q1_15_to_float(a),
               q1_15_to_float(a), q1_15_to_float(d));
        printf0("Q1.15 comparisons passed.\n\n");
    }

    /* --- Q16.16 Comparisons --- */
    {
        q16_16_t a = q16_16_from_float(1.2345f);
        q16_16_t b = q16_16_from_float(1.2345f);
        q16_16_t c = q16_16_from_float(1.0f);
        q16_16_t d = q16_16_from_float(2.0f);

        assert(a == b);
        assert(c < a);
        assert(a < d);

        printf0("Q16.16: %f == %f, %f < %f, %f < %f\n",
               q16_16_to_float(a), q16_16_to_float(b),
               q16_16_to_float(c), q16_16_to_float(a),
               q16_16_to_float(a), q16_16_to_float(d));
        printf0("Q16.16 comparisons passed.\n\n");
    }

    /* --- Q6.10 Comparisons --- */
    {
        q6_10_t a = q6_10_from_float(3.0f);
        q6_10_t b = q6_10_from_float(3.0f);
        q6_10_t c = q6_10_from_float(2.5f);
        q6_10_t d = q6_10_from_float(3.5f);

        assert(a == b);
        assert(c < a);
        assert(a < d);

        printf0("Q6.10: %f == %f, %f < %f, %f < %f\n",
               q6_10_to_float(a), q6_10_to_float(b),
               q6_10_to_float(c), q6_10_to_float(a),
               q6_10_to_float(a), q6_10_to_float(d));
        printf0("Q6.10 comparisons passed.\n\n");
    }

    printf0("\n");
    printf0("=== printf_fixp tests ===\n");
    {
        q8_24_t a = q8_24_from_float(42.5f);
        q1_15_t b = q1_15_from_float(0.25f);
        q16_16_t c = q16_16_from_float(123.1f);
        q6_10_t d = q6_10_from_float(7.5f);
        printf_fixp0("Q8.24: %Q8.24\n", a);
        printf_fixp0("Q1.15: %Q1.15\n", b);
        printf_fixp0("Q16.16: %Q16.16\n", c);
        printf_fixp0("Q6.10: %Q6.10\n", d);
        printf_fixp0("float values: %f %f %f %f\n", q8_24_to_float(a), q1_15_to_float(b), q16_16_to_float(c), q6_10_to_float(d));
    }
    {
        q8_24_t a = q8_24_from_float(-42.5f);
        q1_15_t b = q1_15_from_float(-0.25f);
        q16_16_t c = q16_16_from_float(-123.1f);
        q6_10_t d = q6_10_from_float(-7.5f);
        printf_fixp0("Q8.24: %Q8.24\n", a);
        printf_fixp0("Q1.15: %Q1.15\n", b);
        printf_fixp0("Q16.16: %Q16.16\n", c);
        printf_fixp0("Q6.10: %Q6.10\n", d);
        printf_fixp0("float values: %f %f %f %f\n", q8_24_to_float(a), q1_15_to_float(b), q16_16_to_float(c), q6_10_to_float(d));
    }

    {
        // Test with one set of values.
        q8_24_t a = Q8_24_FROM_FLOAT(42.5f);
        q1_15_t b = Q1_15_FROM_FLOAT(-0.25f);
        q16_16_t c = Q16_16_FROM_FLOAT(123.1f);
        q6_10_t d = Q6_10_FROM_FLOAT(7.5f);

        printf0("Float -> Fixed -> Float conversions:\n");
        printf0("Q8.24: %f\n", q8_24_to_float(a));
        printf0("Q1.15: %f\n", q1_15_to_float(b));
        printf0("Q16.16: %f\n", q16_16_to_float(c));
        printf0("Q6.10: %f\n", q6_10_to_float(d));

        // Additional tests with a variety of values.
        float testValues[] = { 0.0f, 1.0f, -1.0f, 3.14159f, -2.71828f, 0.12345f, -0.98765f };
        int numTests = sizeof(testValues) / sizeof(testValues[0]);
        for (int i = 0; i < numTests; i++) {
            float f = testValues[i];
            q8_24_t qa = Q8_24_FROM_FLOAT(f);
            q1_15_t qb = Q1_15_FROM_FLOAT(f);
            q16_16_t qc = Q16_16_FROM_FLOAT(f);
            q6_10_t qd = Q6_10_FROM_FLOAT(f);
            printf0("Original: %f -> Q8.24: %f, Q1.15: %f, Q16.16: %f, Q6.10: %f\n",
                f, q8_24_to_float(qa), q1_15_to_float(qb), q16_16_to_float(qc), q6_10_to_float(qd));
        }
    }
    // Test Q8.24 conversion.
    {
        q8_24_t a = Q8_24_FROM_FLOAT(42.5f);
        float fa = q8_24_to_float(a);
        printf0("Q8.24: %f (expected 42.5)\n", fa);
        assert(fabsf(fa - 42.5f) < TOLERANCE);
    }

    // Test Q1.15 conversion.
    {
        q1_15_t b = Q1_15_FROM_FLOAT(-0.25f);
        float fb = q1_15_to_float(b);
        printf0("Q1.15: %f (expected -0.25)\n", fb);
        assert(fabsf(fb - (-0.25f)) < TOLERANCE);
    }

    // Test Q16.16 conversion.
    {
        q16_16_t c = Q16_16_FROM_FLOAT(123.1f);
        float fc = q16_16_to_float(c);
        printf0("Q16.16: %f (expected 123.1)\n", fc);
        assert(fabsf(fc - 123.1f) < TOLERANCE);
    }

    // Test Q6.10 conversion.
    {
        q6_10_t d = Q6_10_FROM_FLOAT(7.5f);
        float fd = q6_10_to_float(d);
        printf0("Q6.10: %f (expected 7.5)\n", fd);
        assert(fabsf(fd - 7.5f) < TOLERANCE);
    }

    // Test normal conversions (within range).
    {
        q8_24_t a = Q8_24_FROM_FLOAT(42.5f);
        assert(fabsf(q8_24_to_float(a) - 42.5f) < TOLERANCE);

        q1_15_t b = Q1_15_FROM_FLOAT(-0.25f);
        assert(fabsf(q1_15_to_float(b) - (-0.25f)) < TOLERANCE);

        q16_16_t c = Q16_16_FROM_FLOAT(123.1f);
        assert(fabsf(q16_16_to_float(c) - 123.1f) < TOLERANCE);

        q6_10_t d = Q6_10_FROM_FLOAT(7.5f);
        assert(fabsf(q6_10_to_float(d) - 7.5f) < TOLERANCE);
    }

    // Test saturation behavior:
    // For Q8.24, any x >= 128.0f should produce Q8_24_MAX; any x < -128.0f should produce Q8_24_MIN.
    {
        q8_24_t a1 = Q8_24_FROM_FLOAT(128.0f);
        assert(a1 == Q8_24_MAX);
        q8_24_t a2 = Q8_24_FROM_FLOAT(-130.0f);
        assert(a2 == Q8_24_MIN);
    }

    // For Q1.15, any x >= 1.0f should saturate.
    {
        q1_15_t b1 = Q1_15_FROM_FLOAT(1.0f);
        assert(b1 == Q1_15_MAX);
        q1_15_t b2 = Q1_15_FROM_FLOAT(-2.0f);
        assert(b2 == Q1_15_MIN);
    }

    // For Q16.16, any x >= 32768.0f should saturate.
    {
        q16_16_t c1 = Q16_16_FROM_FLOAT(32768.0f);
        assert(c1 == Q16_16_MAX);
        q16_16_t c2 = Q16_16_FROM_FLOAT(-40000.0f);
        assert(c2 == Q16_16_MIN);
    }

    // For Q6.10, any x >= 32.0f should saturate.
    {
        q6_10_t d1 = Q6_10_FROM_FLOAT(32.0f);
        assert(d1 == Q6_10_MAX);
        q6_10_t d2 = Q6_10_FROM_FLOAT(-33.0f);
        assert(d2 == Q6_10_MIN);
    }

    printf0("\n");
    printf0("All tests passed successfully.\n");
#endif
}


void bench_fixp_functions(void) {
    printf0("==============================================\n");
    printf0("=== POGO-UTILS " POGO_UTILS_VERSION " - FIXED-POINT BENCHS  ===\n");
    printf0("==============================================\n");

    printf0("\n");
    printf0("CSV Table of benchmark results over 1000 runs:\n");
    printf0("\n");

    printf0("fn,float,double,Q8.24,Q1.15,Q16.16,Q6.10\n");
    uint32_t elapsed_float = 0;
    uint32_t elapsed_double = 0;
    uint32_t elapsed_q8_24 = 0;
    uint32_t elapsed_q1_15 = 0;
    uint32_t elapsed_q16_16 = 0;
    uint32_t elapsed_q6_10 = 0;

    float final_float = 0.f;
    double final_double = 0.0;
    q8_24_t final_q8_24 = 0;
    q1_15_t final_q1_15 = 0;
    q16_16_t final_q16_16 = 0;
    q6_10_t final_q6_10 = 0;

    // Add
    {
        volatile float val = -23.04435f;
        volatile float val2 = 0.01f;
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val += val2;
        }
        elapsed_float = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_float += val;
    }
    {
        volatile double val = -23.04435;
        volatile double val2 = 0.01;
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val += val2;
        }
        elapsed_double = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_double += val;
    }
    {
        volatile q8_24_t val = q8_24_from_float(-23.0876f);
        volatile q8_24_t val2 = q8_24_from_float(0.01f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q8_24_add(val, val2);
        }
        elapsed_q8_24 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q8_24 += val;
    }
    {
        volatile q1_15_t val = q1_15_from_float(0.0876f);
        volatile q1_15_t val2 = q1_15_from_float(0.001f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q1_15_add(val, val2);
        }
        elapsed_q1_15 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q1_15 += val;
    }
    {
        volatile q16_16_t val = q16_16_from_float(-23.0876f);
        volatile q16_16_t val2 = q16_16_from_float(0.001f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q16_16_add(val, val2);
        }
        elapsed_q16_16 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q16_16 += val;
    }
    {
        volatile q6_10_t val = q6_10_from_float(-23.0876f);
        volatile q6_10_t val2 = q6_10_from_float(0.001f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q6_10_add(val, val2);
        }
        elapsed_q6_10 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q6_10 += val;
    }
    printf0("add,%lu,%lu,%lu,%lu,%lu,%lu\n", elapsed_float, elapsed_double, elapsed_q8_24, elapsed_q1_15, elapsed_q16_16, elapsed_q6_10);

    // Sub
    {
        volatile float val = -23.04435f;
        volatile float val2 = 0.01f;
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val -= val2;
        }
        elapsed_float = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_float += val;
    }
    {
        volatile double val = -23.04435;
        volatile double val2 = 0.01f;
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val -= val2;
        }
        elapsed_double = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_double += val;
    }
    {
        volatile q8_24_t val = q8_24_from_float(-23.0876f);
        volatile q8_24_t val2 = q8_24_from_float(0.01f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q8_24_sub(val, val2);
        }
        elapsed_q8_24 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q8_24 += val;
    }
    {
        volatile q1_15_t val = q1_15_from_float(0.0876f);
        volatile q1_15_t val2 = q1_15_from_float(0.001f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q1_15_sub(val, val2);
        }
        elapsed_q1_15 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q1_15 += val;
    }
    {
        volatile q16_16_t val = q16_16_from_float(-23.0876f);
        volatile q16_16_t val2 = q16_16_from_float(0.001f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q16_16_sub(val, val2);
        }
        elapsed_q16_16 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q16_16 += val;
    }
    {
        volatile q6_10_t val = q6_10_from_float(-23.0876f);
        volatile q6_10_t val2 = q6_10_from_float(0.001f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q6_10_sub(val, val2);
        }
        elapsed_q6_10 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q6_10 += val;
    }
    printf0("sub,%lu,%lu,%lu,%lu,%lu,%lu\n", elapsed_float, elapsed_double, elapsed_q8_24, elapsed_q1_15, elapsed_q16_16, elapsed_q6_10);

    // Mul
    {
        volatile float val = 23.04435f;
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val *= 42.f;
        }
        elapsed_float = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_float += val;
    }
    {
        volatile double val = 23.04435;
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val *= 42.0;
        }
        elapsed_double = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_double += val;
    }
    {
        volatile q8_24_t val = q8_24_from_float(-23.0876f);
        volatile q8_24_t val2 = q8_24_from_float(0.01f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q8_24_mul(val, val2);
        }
        elapsed_q8_24 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q8_24 += val;
    }
    {
        volatile q1_15_t val = q1_15_from_float(0.0876f);
        volatile q1_15_t val2 = q1_15_from_float(0.001f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q1_15_mul(val, val2);
        }
        elapsed_q1_15 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q1_15 += val;
    }
    {
        volatile q16_16_t val = q16_16_from_float(-23.0876f);
        volatile q16_16_t val2 = q16_16_from_float(0.001f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q16_16_mul(val, val2);
        }
        elapsed_q16_16 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q16_16 += val;
    }
    {
        volatile q6_10_t val = q6_10_from_float(-23.0876f);
        volatile q6_10_t val2 = q6_10_from_float(0.001f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q6_10_mul(val, val2);
        }
        elapsed_q6_10 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q6_10 += val;
    }
    printf0("mul,%lu,%lu,%lu,%lu,%lu,%lu\n", elapsed_float, elapsed_double, elapsed_q8_24, elapsed_q1_15, elapsed_q16_16, elapsed_q6_10);

    // Div
    {
        volatile float val = 0.0876f;
        volatile float val2 = 0.001f;
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = val / val2;
        }
        elapsed_float = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_float += val;
    }
    {
        volatile double val = 0.0876;
        volatile double val2 = 0.001;
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = val / val2;
        }
        elapsed_double = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_double += val;
    }
    {
        volatile q8_24_t val = q8_24_from_float(0.0876);
        volatile q8_24_t val2 = q8_24_from_float(0.001f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q8_24_div(val, val2);
        }
        elapsed_q8_24 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q8_24 += val;
    }
    {
        volatile q1_15_t val = q1_15_from_float(0.0876f);
        volatile q1_15_t val2 = q1_15_from_float(0.001f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q1_15_div(val, val2);
        }
        elapsed_q1_15 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q1_15 += val;
    }
    {
        volatile q16_16_t val = q16_16_from_float(0.0876f);
        volatile q16_16_t val2 = q16_16_from_float(0.001f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q16_16_div(val, val2);
        }
        elapsed_q16_16 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q16_16 += val;
    }
    {
        volatile q6_10_t val = q6_10_from_float(0.0876f);
        volatile q6_10_t val2 = q6_10_from_float(0.001f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q6_10_div(val, val2);
        }
        elapsed_q6_10 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q6_10 += val;
    }
    printf0("div,%lu,%lu,%lu,%lu,%lu,%lu\n", elapsed_float, elapsed_double, elapsed_q8_24, elapsed_q1_15, elapsed_q16_16, elapsed_q6_10);

    // Abs
    {
        volatile float val = -23.04435f;
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = fabsf(val);
        }
        elapsed_float = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_float += val;
    }
    {
        volatile double val = -23.04435;
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = abs(val);
        }
        elapsed_double = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_double += val;
    }
    {
        volatile q8_24_t val = q8_24_from_float(-23.0876f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q8_24_abs(val);
        }
        elapsed_q8_24 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q8_24 += val;
    }
    {
        volatile q1_15_t val = q1_15_from_float(0.0876f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q1_15_abs(val);
        }
        elapsed_q1_15 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q1_15 += val;
    }
    {
        volatile q16_16_t val = q16_16_from_float(-23.0876f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q16_16_abs(val);
        }
        elapsed_q16_16 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q16_16 += val;
    }
    {
        volatile q6_10_t val = q6_10_from_float(-23.0876f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q6_10_abs(val);
        }
        elapsed_q6_10 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q6_10 += val;
    }
    printf0("abs,%lu,%lu,%lu,%lu,%lu,%lu\n", elapsed_float, elapsed_double, elapsed_q8_24, elapsed_q1_15, elapsed_q16_16, elapsed_q6_10);

    // Exp
    {
        volatile float val = 0.0876f;
        volatile float val2 = -0.3f;
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = expf(val2);
        }
        elapsed_float = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_float += val;
    }
    {
        volatile double val = 0.0876;
        volatile double val2 = -0.3;
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = exp(val2);
        }
        elapsed_double = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_double += val;
    }
    {
        volatile q8_24_t val = q8_24_from_float(0.0876f);
        volatile q8_24_t val2 = q8_24_from_float(-0.3f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q8_24_exp(val2);
        }
        elapsed_q8_24 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q8_24 += val;
    }
    {
        volatile q1_15_t val = q1_15_from_float(0.0876f);
        volatile q1_15_t val2 = q1_15_from_float(-0.3f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q1_15_exp(val2);
        }
        elapsed_q1_15 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q1_15 += val;
    }
    {
        volatile q16_16_t val = q16_16_from_float(0.0876f);
        volatile q16_16_t val2 = q16_16_from_float(-0.3f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q16_16_exp(val2);
        }
        elapsed_q16_16 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q16_16 += val;
    }
    {
        volatile q6_10_t val = q6_10_from_float(0.0876f);
        volatile q6_10_t val2 = q6_10_from_float(-0.3f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q6_10_exp(val2);
        }
        elapsed_q6_10 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q6_10 += val;
    }
    printf0("exp,%lu,%lu,%lu,%lu,%lu,%lu\n", elapsed_float, elapsed_double, elapsed_q8_24, elapsed_q1_15, elapsed_q16_16, elapsed_q6_10);

    // Log
    {
        volatile float val = 0.0f;
        volatile float val2 = 0.6f;
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = logf(val2);
        }
        elapsed_float = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_float += val;
    }
    {
        volatile double val = 0.0;
        volatile double val2 = 0.6;
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = log(val2);
        }
        elapsed_double = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_double += val;
    }
    {
        volatile q8_24_t val = q8_24_from_float(0.0f);
        volatile q8_24_t val2 = q8_24_from_float(0.6f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q8_24_log(val2);
        }
        elapsed_q8_24 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q8_24 += val;
    }
    {
        volatile q1_15_t val = q1_15_from_float(0.0f);
        volatile q1_15_t val2 = q1_15_from_float(0.6f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q1_15_log(val2);
        }
        elapsed_q1_15 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q1_15 += val;
    }
    {
        volatile q16_16_t val = q16_16_from_float(0.0f);
        volatile q16_16_t val2 = q16_16_from_float(0.6f);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q16_16_log(val2);
        }
        elapsed_q16_16 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q16_16 += val;
    }
    {
        volatile q6_10_t val = q6_10_from_float(0.0f);
        volatile q6_10_t val2 = q6_10_from_float(0.6);
        pogobot_stopwatch_reset(&mydata->timer_it);
        for (uint16_t i = 0; i < BENCH_RUNS; i++) {
            val = q6_10_log(val2);
        }
        elapsed_q6_10 = pogobot_stopwatch_get_elapsed_microseconds(&mydata->timer_it);
        final_q6_10 += val;
    }
    printf0("log,%lu,%lu,%lu,%lu,%lu,%lu\n", elapsed_float, elapsed_double, elapsed_q8_24, elapsed_q1_15, elapsed_q16_16, elapsed_q6_10);

    printf0("\n");
    printf_fixp0("Final vals: %Q16.16 %Q16.16 %Q8.24 %Q1.15 %Q16.16 %Q6.10\n", q16_16_from_float(final_float), q16_16_from_float(final_double), final_q8_24, final_q1_15, final_q16_16, final_q6_10);
}


void user_init(void) {
#ifndef SIMULATOR
    printf0("setup ok\n");
#endif
    // Set main loop frequency, message sending frequency, message processing frequency
    main_loop_hz = 20;
    max_nb_processed_msg_per_tick = 0;
    // Specify functions to send/transmit messages
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;

    // Set led index to show error codes
    error_codes_led_idx = 3; // Default value, negative values to disable

    test_fixp_functions();
    bench_fixp_functions();
}


void user_step(void) {
    // ...
}


int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    return 0;
}



// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
