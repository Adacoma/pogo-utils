/**
 * @file print_log.h
 * @brief Flash-first printf-style logging with caller-bounded message storage.
 *
 * Attach to an already opened/initialized flash_log and supply a message
 * buffer. No heap or hidden flash writes: printf formats/appends to RAM only;
 * user-called service programs at most one page. Separate objects,
 * buffers, and flash_log handles provide independent prints/CSV streams.
 * All calls are serialized by the caller, not ISR/thread/reentrancy-safe.
 *
 * Messages fitting the flash RAM cache release the staging buffer immediately
 * when mirroring is disabled. One spill/mirror message can remain pending.
 * Each message is accepted atomically in RAM. BUSY rejects the whole
 * new message; too-long/unsupported messages never reach flash or terminal.
 * Accepted data is NOT durable until committed; messages may cross pages and
 * power loss can leave a committed prefix. The underlying append log format
 * and CRC are unchanged. See print_format.h for the intentionally small syntax.
 */
#ifndef POGO_UTILS_PRINT_LOG_H
#define POGO_UTILS_PRINT_LOG_H

#include "flash_log.h"
#include "print_format.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    POGO_PRINT_OK = 0,
    POGO_PRINT_INVALID_ARGUMENT,
    POGO_PRINT_BUSY,
    POGO_PRINT_MESSAGE_TOO_LONG,
    POGO_PRINT_UNSUPPORTED_FORMAT,
    POGO_PRINT_FLASH_FULL,
    POGO_PRINT_FLASH_ERROR,
    POGO_PRINT_TERMINAL_ERROR
} pogo_print_status_t;

/** Accept up to length bytes and return the accepted count (0 means blocked).
 * Called ONLY by service/flush, at most once per call. Do not retain bytes or
 * recurse into this logger. A callback may block internally: byte budgeting
 * is not a wall-clock latency guarantee. NULL disables terminal mirroring.
 * Providing this callback is the only terminal feature; no UART/stdio code is
 * linked into print_log itself. */
typedef size_t (*pogo_print_terminal_fn)(void *context, const char *bytes,
                                       size_t length);

/** Caller-owned metadata plus borrowed flash handle/message storage. Fields
 * are implementation state except rejected_messages and flash_status, which
 * are read-only diagnostics. Rejections saturate the counter at UINT32_MAX.
 * Buffer capacity includes a NUL byte. Pending strings are copied immediately;
 * never keep va_list or pointers to producer stack variables for later use. */
typedef struct {
    pogo_flash_log_t *flash;
    char *buffer;
    size_t capacity;
    size_t length;
    size_t flash_offset;
    size_t terminal_offset;
    pogo_print_terminal_fn terminal;
    void *terminal_context;
    uint32_t rejected_messages;
    pogo_flash_log_status_t flash_status;
} pogo_log_printer_t;

/** RAM-only attach, default terminal disabled. Handle must be ready, buffer
 * capacity >= 2, and buffer must not overlap handle/cache/context. Reattaching
 * discards pending state; initialize/clear flash separately during startup. */
pogo_print_status_t pogo_log_attach(pogo_log_printer_t *printer,
                                    pogo_flash_log_t *flash, char *buffer,
                                    size_t capacity);

/** Change the optional terminal sink only when no message is pending. */
pogo_print_status_t pogo_log_set_terminal(pogo_log_printer_t *printer,
                                          pogo_print_terminal_fn terminal,
                                          void *context);

/** RAM-only format/enqueue. Sources must not overlap the supplied buffer.
 * Returns FLASH_FULL if the entire message cannot fit the remaining file
 * payload, including the current cache. No partial enqueue or hidden flush. */
pogo_print_status_t pogo_log_printf(pogo_log_printer_t *printer,
                                    const char *format, ...);
pogo_print_status_t pogo_log_vprintf(pogo_log_printer_t *printer,
                                     const char *format, va_list args);

/** Drain pending bytes, commit at most one full flash page, and mirror at
 * most terminal_budget bytes. A zero budget defers mirroring. OK does not mean
 * pending data is all drained/durable; inspect pending() and flash->used.
 * Do not append through the raw flash handle while this adapter is active. */
pogo_print_status_t pogo_log_service(pogo_log_printer_t *printer,
                                     size_t terminal_budget);

/** Checkpoint at most one page, including partial pages. Repeat while BUSY
 * to commit all pending/cache bytes and finish mirroring. Partial flush
 * permanently wastes unused page capacity. Never spin on this in a controller
 * tick: call it across ticks, or finish outside time-critical work. */
pogo_print_status_t pogo_log_flush(pogo_log_printer_t *printer,
                                   size_t terminal_budget);

/** True while the adapter still owns a pending message; the flash cache can
 * still hold uncommitted data after this becomes false. */
bool pogo_log_pending(const pogo_log_printer_t *printer);

#ifdef __cplusplus
}
#endif
#endif
