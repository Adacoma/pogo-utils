/** @file print_format.c
 * Bounded text emission and decimal conversion using only 32-bit integers.
 * The largest Q fractional remainder is 2^24-1: multiplying it by ten fits
 * uint32_t. Digit-by-digit conversion therefore needs no 64-bit helpers.
 */
#include "print_format.h"

#include <limits.h>
#include <stdint.h>
#include <string.h>

enum { SPAN_SIZE = 32, MAX_PRECISION = 6 };

/** Convert magnitude to forward digits in caller scratch (maximum 10). */
static size_t unsigned_digits(char *out, uint32_t value, unsigned base,
                              bool upper) {
    const char *alphabet = upper ? "0123456789ABCDEF" : "0123456789abcdef";
    size_t n = 0;
    do {
        /* Keep decimal division constant so compilers can strength-reduce it;
         * hexadecimal needs only masking/shifting, not a generic divider. */
        if (base == 16u) {
            out[n++] = alphabet[value & 15u];
            value >>= 4;
        } else {
            out[n++] = alphabet[value % 10u];
            value /= 10u;
        }
    } while (value != 0u);
    for (size_t i = 0; i < n / 2u; ++i) {
        char c = out[i];
        out[i] = out[n - 1u - i];
        out[n - 1u - i] = c;
    }
    return n;
}

/** Extract decimal fractional digits, then round with explicit integer carry.
 * Unsigned negation handles INT32_MIN without signed-overflow UB. */
static size_t fixed_digits(char *out, int32_t value, unsigned bits,
                           unsigned precision) {
    bool negative = value < 0;
    uint32_t magnitude = negative ? 0u - (uint32_t)value : (uint32_t)value;
    uint32_t mask = (UINT32_C(1) << bits) - 1u;
    uint32_t integer = magnitude >> bits;
    uint32_t remainder = magnitude & mask;
    char fraction[MAX_PRECISION];
    for (unsigned i = 0; i < precision; ++i) {
        remainder *= 10u;
        fraction[i] = (char)('0' + (remainder >> bits));
        remainder &= mask;
    }
    if ((remainder * 10u >> bits) >= 5u) {
        unsigned i = precision;
        while (i > 0u && fraction[i - 1u] == '9') fraction[--i] = '0';
        if (i == 0u) ++integer;
        else ++fraction[i - 1u];
    }
    size_t n = 0;
    if (negative) out[n++] = '-';
    n += unsigned_digits(out + n, integer, 10u, false);
    if (precision != 0u) {
        out[n++] = '.';
        memcpy(out + n, fraction, precision);
        n += precision;
    }
    return n;
}

/** Bounded width parser rejects excessive or overflowing decimal fields. */
static bool decimal_field(const char **cursor, unsigned limit, unsigned *value) {
    const char *p = *cursor;
    *value = 0u;
    if (*p < '0' || *p > '9') return false;
    do {
        unsigned digit = (unsigned)(*p++ - '0');
        if (*value > limit / 10u ||
            (*value == limit / 10u && digit > limit % 10u)) return false;
        *value = *value * 10u + digit;
    } while (*p >= '0' && *p <= '9');
    *cursor = p;
    return true;
}

/** Pad numeric output without padding before the sign when zero-fill is used. */
static bool number_span(pogo_format_write_fn write, void *context,
                        const char *text, size_t n, unsigned width, bool zero) {
    size_t padding = width > n ? width - n : 0u;
    if (zero && padding != 0u && text[0] == '-') {
        if (!write(context, text, 1u)) return false;
        ++text;
        --n;
    }
    if (padding != 0u) {
        char fill[SPAN_SIZE];
        memset(fill, zero ? '0' : ' ', padding);
        if (!write(context, fill, padding)) return false;
    }
    return write(context, text, n);
}

pogo_format_status_t pogo_vformat(pogo_format_write_fn write, void *context,
                                 const char *format, va_list input) {
    if (write == NULL || format == NULL) return POGO_FORMAT_INVALID_ARGUMENT;
    /* Work on our own cursor, independent of platform va_list representation. */
    va_list args;
    va_copy(args, input);
    pogo_format_status_t status = POGO_FORMAT_OK;
    const char *p = format;
    while (*p != '\0') {
        if (*p != '%') {
            const char *start = p;
            while (*p != '\0' && *p != '%' && (size_t)(p - start) < SPAN_SIZE) ++p;
            if (!write(context, start, (size_t)(p - start))) goto no_space;
            continue;
        }
        ++p;
        if (*p == '%') {
            if (!write(context, p++, 1u)) goto no_space;
            continue;
        }
        bool zero = *p == '0';
        unsigned width = 0u, precision = 0u;
        bool has_precision = false, is_long = false;
        if (*p >= '0' && *p <= '9' && !decimal_field(&p, 32u, &width))
            goto unsupported;
        if (*p == '.') {
            ++p;
            has_precision = true;
            if (!decimal_field(&p, 255u, &precision)) goto unsupported;
        }
        if (*p == 'l') { is_long = true; ++p; }
        char conversion = *p;
        if (conversion == '\0') goto unsupported;
        ++p;
        /* Max sign + 10 integer digits + dot + 6 fractional digits fits 18. */
        char text[20];
        size_t n = 0u;
        if (conversion == 'Q') {
            if (is_long || (has_precision && precision > MAX_PRECISION))
                goto unsupported;
            unsigned bits, defaults;
            int32_t value;
            if (strncmp(p, "16.16", 5u) == 0) {
                bits = 16u; defaults = 6u; p += 5;
                value = va_arg(args, int32_t);
            } else if (strncmp(p, "8.24", 4u) == 0) {
                bits = 24u; defaults = 6u; p += 4;
                value = va_arg(args, int32_t);
            } else if (strncmp(p, "1.15", 4u) == 0) {
                bits = 15u; defaults = 4u; p += 4;
                value = (int16_t)va_arg(args, int);
            } else if (strncmp(p, "6.10", 4u) == 0) {
                bits = 10u; defaults = 3u; p += 4;
                value = (int16_t)va_arg(args, int);
            } else goto unsupported;
            n = fixed_digits(text, value, bits, has_precision ? precision : defaults);
        } else if (conversion == 's') {
            if (is_long || width != 0u || zero) goto unsupported;
            const char *s = va_arg(args, const char *);
            if (s == NULL) s = "(null)";
            size_t remaining = has_precision ? precision : SIZE_MAX;
            while (remaining > 0u && *s != '\0') {
                size_t count = 0u;
                while (count < SPAN_SIZE && count < remaining && s[count] != '\0') ++count;
                if (!write(context, s, count)) goto no_space;
                s += count;
                remaining -= count;
            }
            continue;
        } else if (conversion == 'c') {
            if (is_long || has_precision || width != 0u || zero) goto unsupported;
            text[n++] = (char)va_arg(args, int);
        } else if (conversion == 'd' || conversion == 'i') {
            if (has_precision) goto unsupported;
            int32_t value;
            if (is_long) {
                long v = va_arg(args, long);
                if (v < INT32_MIN || v > INT32_MAX) goto unsupported;
                value = (int32_t)v;
            } else value = va_arg(args, int);
            if (value < 0) text[n++] = '-';
            uint32_t magnitude = value < 0 ? 0u - (uint32_t)value : (uint32_t)value;
            n += unsigned_digits(text + n, magnitude, 10u, false);
        } else if (conversion == 'u' || conversion == 'x' || conversion == 'X') {
            if (has_precision) goto unsupported;
            uint32_t value;
            if (is_long) {
                unsigned long v = va_arg(args, unsigned long);
                if (v > UINT32_MAX) goto unsupported;
                value = (uint32_t)v;
            } else value = va_arg(args, unsigned int);
            n = unsigned_digits(text, value, conversion == 'u' ? 10u : 16u,
                                conversion == 'X');
        } else goto unsupported;
        if (!number_span(write, context, text, n, width, zero)) goto no_space;
    }
    goto done;
unsupported:
    status = POGO_FORMAT_UNSUPPORTED;
    goto done;
no_space:
    status = POGO_FORMAT_NO_SPACE;
done:
    va_end(args);
    return status;
}

/** Buffer state is stack-only metadata; payload storage belongs to caller. */
typedef struct { char *output; size_t capacity; size_t used; } buffer_sink_t;

static bool buffer_write(void *context, const char *bytes, size_t length) {
    buffer_sink_t *sink = context;
    size_t available = sink->capacity - 1u - sink->used;
    size_t count = length < available ? length : available;
    memcpy(sink->output + sink->used, bytes, count);
    sink->used += count;
    return count == length;
}

pogo_format_status_t pogo_vsnprintf(char *output, size_t capacity,
                                    size_t *written, const char *format,
                                    va_list args) {
    if (written != NULL) *written = 0u;
    if (output == NULL || capacity == 0u) return POGO_FORMAT_INVALID_ARGUMENT;
    buffer_sink_t sink = {output, capacity, 0u};
    output[0] = '\0';
    pogo_format_status_t status = pogo_vformat(buffer_write, &sink, format, args);
    output[sink.used] = '\0';
    if (written != NULL) *written = sink.used;
    return status;
}

pogo_format_status_t pogo_snprintf(char *output, size_t capacity,
                                   size_t *written, const char *format, ...) {
    va_list args;
    va_start(args, format);
    pogo_format_status_t status = pogo_vsnprintf(output, capacity, written, format, args);
    va_end(args);
    return status;
}
