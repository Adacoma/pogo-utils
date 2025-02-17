/* TODO
 */

#include "fixp.h"

//#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#include "pogosim/pogosim.h"

/*
 * print_fixed_q8_24:
 *
 * This function prints a Q8.24 fixed–point number as a human–readable decimal value.
 * Q8.24 numbers are stored in 32 bits with 24 fractional bits, so 1.0 is represented as (1 << 24).
 *
 * The function works as follows:
 * 1. Determine if the number is negative. If so, work with its absolute value.
 * 2. Extract the integer part by shifting right by 24 bits.
 * 3. Extract the fractional part by masking out the lower 24 bits.
 * 4. Convert the fractional part into a decimal fraction with 6 digits by multiplying by 1,000,000,
 *    adding half the scaling factor for rounding, and shifting right by 24 bits.
 * 5. Print the result with a fixed–width format for the fractional part.
 */
static void print_fixed_q8_24(q8_24_t x) {
    int negative = (x < 0);
    // Work with the absolute value.
    uint32_t absx = negative ? (uint32_t)(-x) : (uint32_t)x;
    // Extract the integer part by shifting right by 24 bits.
    int32_t integerPart = absx >> 24;
    // Extract the fractional part by masking the lower 24 bits.
    uint32_t fracPart = absx & ((1UL << 24) - 1);
    // Convert the fractional part to a decimal value with 6 digits, with rounding.
    uint32_t fracDecimal = (uint32_t)(((uint64_t)fracPart * 1000000 + (1UL << 23)) >> 24);
    if (negative)
        printf("-%ld.%06lu", integerPart, fracDecimal);
    else
        printf("%ld.%06lu", integerPart, fracDecimal);
}

/*
 * print_fixed_q1_15:
 *
 * This function prints a Q1.15 fixed–point number as a human–readable decimal value.
 * Q1.15 numbers are stored in 16 bits with 15 fractional bits, so 1.0 is represented as (1 << 15).
 *
 * The function works as follows:
 * 1. Determine if the number is negative and compute the absolute value.
 * 2. Extract the integer part by shifting right by 15 bits.
 * 3. Extract the fractional part by masking the lower 15 bits.
 * 4. Convert the fractional part to a decimal fraction with 4 digits by multiplying by 10,000,
 *    adding half of the scaling factor (1 << 14) for rounding, and shifting right by 15 bits.
 * 5. Print the result with a fixed format, including a minus sign for negative values.
 */
static void print_fixed_q1_15(q1_15_t x) {
    int negative = (x < 0);
    // Compute the absolute value.
    uint16_t absx = negative ? (uint16_t)(-x) : (uint16_t)x;
    // Extract the integer part (1 << 15 represents 1.0).
    int32_t integerPart = absx >> 15;
    // Extract the fractional part by masking the lower 15 bits.
    uint16_t fracPart = absx & ((1U << 15) - 1);
    // Convert the fractional part to 4 decimal digits.
    uint32_t fracDecimal = (uint32_t)(((uint32_t)fracPart * 10000 + (1U << 14)) >> 15);
    if (negative)
        printf("-%ld.%04lu", integerPart, fracDecimal);
    else
        printf("%ld.%04lu", integerPart, fracDecimal);
}

/*
 * print_fixed_q16_16:
 *
 * This function prints a Q16.16 fixed–point number as a human–readable decimal value.
 * Q16.16 numbers are stored in 32 bits with 16 fractional bits, so 1.0 is represented as 65536.
 *
 * The function works as follows:
 * 1. Check for a negative value and work with its absolute value.
 * 2. Extract the integer part by performing an integer division by Q16_16_ONE (65536).
 * 3. Extract the fractional part using the modulus operator with Q16_16_ONE.
 * 4. Convert the fractional part to a decimal fraction with 6 digits by multiplying by 1,000,000,
 *    adding half of Q16_16_ONE for rounding, and then dividing by Q16_16_ONE.
 * 5. Print the final value with a fixed format, including a '-' sign for negative numbers.
 */
static void print_fixed_q16_16(q16_16_t x) {
    int negative = (x < 0);
    // Compute the absolute value.
    uint32_t absx = negative ? (uint32_t)(-x) : (uint32_t)x;
    // Extract the integer part by division.
    int32_t integerPart = absx / (1 << 16);  // Q16_16_ONE = 65536
    // Extract the fractional part using modulus.
    uint32_t fracPart = absx % (1 << 16);
    // Convert the fractional part to 6 decimal digits with rounding.
    uint32_t fracDecimal = (uint32_t)(((uint64_t)fracPart * 1000000 + ((1 << 16) / 2)) / (1 << 16));
    if (negative)
        printf("-%ld.%06lu", integerPart, fracDecimal);
    else
        printf("%ld.%06lu", integerPart, fracDecimal);
}

/*
 * print_fixed_q6_10:
 *
 * This function prints a Q6.10 fixed–point number as a human–readable decimal value.
 * Q6.10 numbers are stored in 16 bits with 10 fractional bits, so 1.0 is represented as 1024.
 *
 * The function works as follows:
 * 1. Check if the value is negative and work with its absolute value.
 * 2. Extract the integer part by shifting right by 10 bits.
 * 3. Extract the fractional part by masking the lower 10 bits.
 * 4. Convert the fractional part into a decimal fraction with 3 digits by multiplying by 1000,
 *    adding half of the scaling factor (1 << 9) for rounding, and then shifting right by 10 bits.
 * 5. Print the result in a fixed–width format, including a '-' sign if needed.
 */
static void print_fixed_q6_10(q6_10_t x) {
    int negative = (x < 0);
    // Use the absolute value for extraction.
    uint16_t absx = negative ? (uint16_t)(-x) : (uint16_t)x;
    // For Q6.10, 1.0 is represented as 1024 (1 << 10).
    int32_t integerPart = absx >> 10;
    // Extract the fractional part by masking the lower 10 bits.
    uint16_t fracPart = absx & ((1U << 10) - 1);
    // Convert the fractional part to a decimal fraction with 3 digits.
    uint32_t fracDecimal = (uint32_t)(((uint32_t)fracPart * 1000 + (1U << 9)) >> 10);
    if (negative)
        printf("-%ld.%03lu", integerPart, fracDecimal);
    else
        printf("%ld.%03lu", integerPart, fracDecimal);
}


/*
 * printf_fixp:
 *
 * A custom printf–like function that supports all standard conversion specifiers
 * (with length–field and type–field details) as well as the following additional
 * conversion specifiers for fixed–point numbers:
 *
 *    %Q8.24  – prints a q8_24_t fixed–point value.
 *    %Q1.15  – prints a q1_15_t fixed–point value.
 *    %Q16.16 – prints a q16_16_t fixed–point value.
 *    %Q6.10  – prints a q6_10_t fixed–point value.
 *
 * The function scans the format string character–by–character. When it encounters a '%',
 * it checks if the conversion is one of our custom ones (i.e. starts with "Q"). If so,
 * it parses the fixed–point specifier (e.g. "8.24") and calls the appropriate helper.
 *
 * Otherwise, it collects the entire conversion specifier (including any flags, width,
 * precision, and length fields) into a temporary buffer. Then, based on the final
 * conversion character, it extracts the corresponding argument from the variable–argument
 * list and calls printf() with that temporary format string.
 */
void printf_fixp(const char *format, ...) {
    va_list args;
    va_start(args, format);

    const char *p = format;
    while (*p) {
        if (*p == '%') {
            //const char *start = p; // remember beginning of conversion
            p++; // skip '%'
            // Handle "%%" (literal '%')
            if (*p == '%') {
                putchar('%');
                p++;
                continue;
            }
            // Check for our custom fixed–point conversion specifier.
            if (*p == 'Q') {
                p++; // skip 'Q'
                // Check for each supported custom specifier.
                if (strncmp(p, "8.24", 4) == 0) {
                    p += 4;
                    q8_24_t num = va_arg(args, int32_t);
                    print_fixed_q8_24(num);
                    continue;
                } else if (strncmp(p, "1.15", 4) == 0) {
                    p += 4;
                    // q1_15_t is promoted to int in varargs.
                    q1_15_t num = (q1_15_t)va_arg(args, int);
                    print_fixed_q1_15(num);
                    continue;
                } else if (strncmp(p, "16.16", 5) == 0) {
                    p += 5;
                    q16_16_t num = va_arg(args, int32_t);
                    print_fixed_q16_16(num);
                    continue;
                } else if (strncmp(p, "6.10", 4) == 0) {
                    p += 4;
                    q6_10_t num = (q6_10_t)va_arg(args, int);
                    print_fixed_q6_10(num);
                    continue;
                } else {
                    // Unrecognized fixed–point specifier: print "%Q" literally.
                    printf("%s", "%Q");
                    continue;
                }
            } else {
                // For standard conversion specifiers, copy the entire specifier into a temporary buffer.
                char fmt[128];
                int i = 0;
                fmt[i++] = '%';
                // Copy flags, width, precision, and length fields.
                // We copy until we hit a conversion specifier letter (a–z or A–Z).
                while (*p && i < (int)(sizeof(fmt) - 1)) {
                    fmt[i++] = *p;
                    if ((*p >= 'a' && *p <= 'z') || (*p >= 'A' && *p <= 'Z')) {
                        p++;
                        break;
                    }
                    p++;
                }
                fmt[i] = '\0';
                // Determine the conversion type (the last character in fmt).
                char conv = fmt[i - 1];
                // Based on the conversion character, extract one argument and print it.
                // (This simple implementation assumes one argument per conversion.)
                if (conv == 'd' || conv == 'i') {
                    int arg = va_arg(args, int);
                    printf(fmt, arg);
                } else if (conv == 'u' || conv == 'o' || conv == 'x' || conv == 'X') {
                    unsigned int arg = va_arg(args, unsigned int);
                    printf(fmt, arg);
                } else if (conv == 'f' || conv == 'F' || conv == 'e' || conv == 'E' ||
                           conv == 'g' || conv == 'G') {
                    double arg = va_arg(args, double);
                    printf(fmt, arg);
                } else if (conv == 'c') {
                    int arg = va_arg(args, int);
                    printf(fmt, arg);
                } else if (conv == 's') {
                    char *arg = va_arg(args, char*);
                    printf(fmt, arg);
                } else if (conv == 'p') {
                    void *arg = va_arg(args, void*);
                    printf(fmt, arg);
                } else {
                    // Fallback: print the conversion specifier literally.
                    printf("%s", fmt);
                }
                continue;
            }
        } else {
            putchar(*p);
            p++;
        }
    }

    va_end(args);
}



// MODELINE "{{{1
// vim:expandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
// vim:foldmethod=marker
