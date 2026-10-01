/**
 * @file print_format.h
 * @brief Small integer/fixed-point formatter with no libc printf dependency.
 *
 * Supported: %% %c %s %d %i %u %x %X; %ld/%li/%lu/%lx/%lX for values
 * representable in 32 bits; %Q1.15 %Q6.10 %Q8.24 %Q16.16. Numeric width
 * (1..32) and zero padding are optional, e.g. %08x. %.Ns limits strings
 * to N bytes (0..255). %.NQ16.16 selects 0..6 fractional decimal digits;
 * defaults are 4/3/6/6 respectively. Rounding is half away from zero.
 * Negative values rounding to zero retain their minus sign.
 *
 * No float/double, 64-bit values, '*', locale, other flags/length modifiers,
 * or libc fallback. Arguments must match their specifiers: 16-bit Q values
 * undergo promotion to int; 32-bit Q values must have type int32_t.
 * Formatting does not require init_fixp() or any arithmetic lookup tables.
 * Do not apply a standard printf format attribute to these custom APIs.
 */
#ifndef POGO_UTILS_PRINT_FORMAT_H
#define POGO_UTILS_PRINT_FORMAT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    POGO_FORMAT_OK = 0,
    POGO_FORMAT_INVALID_ARGUMENT,
    POGO_FORMAT_UNSUPPORTED,
    POGO_FORMAT_NO_SPACE
} pogo_format_status_t;

/** Synchronously consume a span, or return false to stop formatting. Spans
 * are borrowed only during the call; callbacks must copy rather than retain.
 * The core emits at most 32 bytes per callback and uses no heap allocation. */
typedef bool (*pogo_format_write_fn)(void *context, const char *bytes,
                                     size_t length);

/** Streaming output can contain a prefix on failure. Unsupported syntax
 * stops immediately; it is never delegated to printf. */
pogo_format_status_t pogo_vformat(pogo_format_write_fn write, void *context,
                                 const char *format, va_list args);

/** Bounded buffer formatting. capacity includes the trailing NUL; written
 * reports the actual prefix length, NOT snprintf's hypothetical required
 * length. A valid nonempty destination is always NUL-terminated, even on
 * failure. Stop on overflow rather than scanning an arbitrarily long %s.
 * Source strings/format must not overlap destination. */
pogo_format_status_t pogo_vsnprintf(char *output, size_t capacity,
                                    size_t *written, const char *format,
                                    va_list args);
pogo_format_status_t pogo_snprintf(char *output, size_t capacity,
                                   size_t *written, const char *format, ...);

#ifdef __cplusplus
}
#endif
#endif
