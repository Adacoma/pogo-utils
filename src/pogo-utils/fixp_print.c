/** @file fixp_print.c
 * Terminal adapter kept separate from LUT setup and the flash writer. Linking
 * printf_fixp retains only the small formatter and putchar, not libc printf.
 * This void legacy API cannot report transport/format failures: unsupported
 * syntax stops at the prefix. Use pogo_snprintf/pogo_log_printf for statuses.
 */
#include "fixp_base.h"
#include "print_format.h"

#include <stdio.h>

static bool terminal_write(void *context, const char *bytes, size_t length) {
    (void)context;
    for (size_t i = 0u; i < length; ++i) {
        if (putchar((unsigned char)bytes[i]) == EOF) return false;
    }
    return true;
}

void printf_fixp(const char *format, ...) {
    va_list args;
    va_start(args, format);
    (void)pogo_vformat(terminal_write, NULL, format, args);
    va_end(args);
}
