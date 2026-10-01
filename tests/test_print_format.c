/** Host formatter regression: boundaries, syntax, bounded writes, and a
 * widened integer oracle. Production formatting itself uses no 64-bit math. */
#include "src/pogo-utils/print_format.h"
#include "src/pogo-utils/fixp_base.h"

#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

static char terminal[128];
static size_t terminal_length;

/** Replace the terminal adapter's transport without starting a simulator. */
int putchar(int c) {
    assert(terminal_length < sizeof(terminal));
    terminal[terminal_length++] = (char)c;
    return (unsigned char)c;
}

static void expect(const char *expected, const char *format, ...) {
    char text[128];
    size_t written = SIZE_MAX;
    va_list args;
    va_start(args, format);
    assert(pogo_vsnprintf(text, sizeof(text), &written, format, args) == POGO_FORMAT_OK);
    va_end(args);
    assert(written == strlen(expected));
    assert(strcmp(text, expected) == 0);
}

/** Reference rounds the exact raw integer magnitude, not an approximate
 * floating-point value; negative zero intentionally retains its sign. */
static void compare_fixed(int32_t raw, unsigned bits, unsigned precision) {
    uint32_t magnitude = raw < 0 ? 0u - (uint32_t)raw : (uint32_t)raw;
    uint64_t scale = 1u;
    for (unsigned i = 0; i < precision; ++i) scale *= 10u;
    uint64_t rounded = ((uint64_t)magnitude * scale + (UINT64_C(1) << (bits - 1u))) >> bits;
    char expected[64], format[32], actual[64];
    if (precision == 0u) {
        snprintf(expected, sizeof(expected), "%s%" PRIu64, raw < 0 ? "-" : "", rounded);
    } else {
        snprintf(expected, sizeof(expected), "%s%" PRIu64 ".%0*" PRIu64,
                 raw < 0 ? "-" : "", rounded / scale, (int)precision, rounded % scale);
    }
    const char *q = bits == 24u ? "8.24" : bits == 16u ? "16.16" :
                    bits == 15u ? "1.15" : "6.10";
    snprintf(format, sizeof(format), "%%.%uQ%s", precision, q);
    size_t length;
    pogo_format_status_t status = bits == 15u || bits == 10u ?
        pogo_snprintf(actual, sizeof(actual), &length, format, (int)(int16_t)raw) :
        pogo_snprintf(actual, sizeof(actual), &length, format, raw);
    assert(status == POGO_FORMAT_OK);
    assert(strcmp(actual, expected) == 0);
    assert(length == strlen(expected));
}

int main(void) {
    expect("-2147483648 4294967295 89ABCDEF 0000002a -0000012 % x ok",
           "%d %u %X %08x %08d %% %c %s", INT_MIN, UINT_MAX,
           0x89abcdefu, 42u, -12, 'x', "ok");
    expect("-12 4294967295", "%ld %lu", -12L, (unsigned long)UINT32_MAX);
    expect("abc (null)", "%.3s %s", "abcdef", (const char *)NULL);
    expect("", "%.0s", "not emitted");
    expect("   12", "%5d", 12);
    expect("-128.000000 128.000000", "%Q8.24 %Q8.24", (int32_t)INT32_MIN, (int32_t)INT32_MAX);
    expect("-32768.000000 32767.999985", "%Q16.16 %Q16.16", (int32_t)INT32_MIN, (int32_t)INT32_MAX);
    expect("-1.0000 1.0000", "%Q1.15 %Q1.15", (int)INT16_MIN, (int)INT16_MAX);
    expect("-32.000 31.999", "%Q6.10 %Q6.10", (int)INT16_MIN, (int)INT16_MAX);
    expect("-0.000 -0001.500 2 -2", "%.3Q16.16 %09.3Q16.16 %.0Q16.16 %.0Q16.16",
           (int32_t)-1, (int32_t)-98304, (int32_t)98304, (int32_t)-98304);

    char guarded[8] = {'L', 'x', 'x', 'x', 'x', 'x', 'x', 'R'};
    size_t written;
    assert(pogo_snprintf(guarded + 1, 6u, &written, "abcdef") == POGO_FORMAT_NO_SPACE);
    assert(written == 5u && strcmp(guarded + 1, "abcde") == 0);
    assert(guarded[0] == 'L' && guarded[7] == 'R');
    assert(pogo_snprintf(guarded + 1, 1u, &written, "x") == POGO_FORMAT_NO_SPACE);
    assert(written == 0u && guarded[1] == '\0');
    assert(pogo_snprintf(NULL, 0u, &written, "x") == POGO_FORMAT_INVALID_ARGUMENT);
    assert(pogo_snprintf(guarded, sizeof(guarded), &written, NULL) == POGO_FORMAT_INVALID_ARGUMENT);
    const char *unsupported[] = {"%f", "%llx", "%p", "%n", "%*d", "%", "%Q", "%Q2.14",
        "%.7Q16.16", "%33d", "%999999999999999999999999d", "%.256s", "%+d", "%ls", "%05s"};
    for (size_t i = 0; i < sizeof(unsupported) / sizeof(unsupported[0]); ++i)
        assert(pogo_snprintf(guarded, sizeof(guarded), &written, unsupported[i]) == POGO_FORMAT_UNSUPPORTED);
#if LONG_MAX > INT32_MAX
    assert(pogo_snprintf(guarded, sizeof(guarded), NULL, "%ld", LONG_MAX) == POGO_FORMAT_UNSUPPORTED);
    assert(pogo_snprintf(guarded, sizeof(guarded), NULL, "%lu", ULONG_MAX) == POGO_FORMAT_UNSUPPORTED);
#endif
    /* Exhaust all 16-bit values at each supported precision. */
    for (int32_t raw = INT16_MIN; raw <= INT16_MAX; ++raw)
        for (unsigned precision = 0; precision <= 6u; ++precision) {
            compare_fixed(raw, 10u, precision);
            compare_fixed(raw, 15u, precision);
        }
    const int32_t boundaries[] = {INT32_MIN, INT32_MIN + 1, -16777216, -65536,
        -1, 0, 1, 65535, 65536, 16777215, INT32_MAX - 1, INT32_MAX};
    for (size_t i = 0; i < sizeof(boundaries) / sizeof(boundaries[0]); ++i)
        for (unsigned precision = 0; precision <= 6u; ++precision) {
            compare_fixed(boundaries[i], 16u, precision);
            compare_fixed(boundaries[i], 24u, precision);
        }
    uint32_t random = 7u;
    for (unsigned i = 0; i < 10000u; ++i) {
        random = random * UINT32_C(1664525) + UINT32_C(1013904223);
        int32_t raw;
        memcpy(&raw, &random, sizeof(raw));
        compare_fixed(raw, 16u, i % 7u);
        compare_fixed(raw, 24u, i % 7u);
    }
    printf_fixp("q=%Q8.24 n=%08x\n", (int32_t)INT32_MAX, 42u);
    assert(terminal_length == strlen("q=128.000000 n=0000002a\n"));
    assert(memcmp(terminal, "q=128.000000 n=0000002a\n", terminal_length) == 0);
    puts("print format tests passed");
    return 0;
}
