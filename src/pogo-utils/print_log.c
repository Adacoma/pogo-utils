/** @file print_log.c
 * A one-message bounded producer queue over the existing one-page flash
 * cache. Offsets preserve suffixes across service calls and partial terminal
 * acceptance. No standard printf, UART dependency, or formatting-time I/O.
 */
#include "print_log.h"

#include <string.h>

static bool valid(const pogo_log_printer_t *printer) {
    return printer != NULL && printer->flash != NULL &&
           printer->flash->ready != 0u && printer->buffer != NULL &&
           printer->capacity >= 2u;
}

/** Count producer rejections without wrapping during long experiments. */
static pogo_print_status_t reject(pogo_log_printer_t *printer,
                                  pogo_print_status_t status) {
    if (printer->rejected_messages != UINT32_MAX) ++printer->rejected_messages;
    return status;
}

static pogo_print_status_t flash_result(pogo_log_printer_t *printer,
                                        pogo_flash_log_status_t status) {
    printer->flash_status = status;
    if (status == POGO_FLASH_LOG_OK || status == POGO_FLASH_LOG_NEEDS_SERVICE)
        return POGO_PRINT_OK;
    return status == POGO_FLASH_LOG_FULL ? POGO_PRINT_FLASH_FULL :
                                          POGO_PRINT_FLASH_ERROR;
}

/** Append once, leaving the remainder owned by this queue on backpressure.
 * This helper only copies to the underlying RAM cache, never programs flash. */
static pogo_print_status_t append_pending(pogo_log_printer_t *printer) {
    if (printer->flash_offset == printer->length) return POGO_PRINT_OK;
    size_t accepted = 0u;
    pogo_flash_log_status_t status = pogo_flash_log_append(printer->flash,
        printer->buffer + printer->flash_offset,
        printer->length - printer->flash_offset, &accepted);
    printer->flash_offset += accepted;
    return flash_result(printer, status);
}

static void release_completed(pogo_log_printer_t *printer) {
    if (printer->flash_offset == printer->length &&
        printer->terminal_offset == printer->length)
        printer->length = printer->flash_offset = printer->terminal_offset = 0u;
}

pogo_print_status_t pogo_log_attach(pogo_log_printer_t *printer,
                                    pogo_flash_log_t *flash, char *buffer,
                                    size_t capacity) {
    if (printer == NULL || flash == NULL || flash->ready == 0u ||
        buffer == NULL || capacity < 2u) return POGO_PRINT_INVALID_ARGUMENT;
    memset(printer, 0, sizeof(*printer));
    printer->flash = flash;
    printer->buffer = buffer;
    printer->capacity = capacity;
    buffer[0] = '\0';
    return POGO_PRINT_OK;
}

pogo_print_status_t pogo_log_set_terminal(pogo_log_printer_t *printer,
                                          pogo_print_terminal_fn terminal,
                                          void *context) {
    if (!valid(printer)) return POGO_PRINT_INVALID_ARGUMENT;
    if (printer->length != 0u) return POGO_PRINT_BUSY;
    printer->terminal = terminal;
    printer->terminal_context = context;
    return POGO_PRINT_OK;
}

pogo_print_status_t pogo_log_vprintf(pogo_log_printer_t *printer,
                                     const char *format, va_list args) {
    if (!valid(printer) || format == NULL) return POGO_PRINT_INVALID_ARGUMENT;
    if (printer->length != 0u) return reject(printer, POGO_PRINT_BUSY);
    if (printer->flash->failed != 0u) {
        printer->flash_status = POGO_FLASH_LOG_VERIFY_FAILED;
        return reject(printer, POGO_PRINT_FLASH_ERROR);
    }
    size_t length = 0u;
    pogo_format_status_t status = pogo_vsnprintf(printer->buffer,
        printer->capacity, &length, format, args);
    if (status != POGO_FORMAT_OK) {
        printer->buffer[0] = '\0'; /* A formatted prefix is not an enqueue. */
        return reject(printer, status == POGO_FORMAT_NO_SPACE ?
            POGO_PRINT_MESSAGE_TOO_LONG : POGO_PRINT_UNSUPPORTED_FORMAT);
    }
    /* Extents contain at most 256 log pages: this calculation fits size_t
     * on the supported 32-bit target and avoids 64-bit runtime helpers. */
    size_t remaining = (size_t)(printer->flash->page_count -
        printer->flash->next_page) * POGO_FLASH_LOG_PAYLOAD_SIZE -
        printer->flash->used;
    if (length > remaining) {
        printer->flash_status = POGO_FLASH_LOG_FULL;
        printer->buffer[0] = '\0';
        return reject(printer, POGO_PRINT_FLASH_FULL);
    }
    printer->length = length;
    printer->flash_offset = 0u;
    printer->terminal_offset = printer->terminal == NULL ? length : 0u;
    /* Common flash-only path can accept several short records between service
     * calls. Only a cache spill or pending mirror ties up the staging buffer. */
    pogo_print_status_t result = append_pending(printer);
    release_completed(printer);
    return result;
}

pogo_print_status_t pogo_log_printf(pogo_log_printer_t *printer,
                                    const char *format, ...) {
    va_list args;
    va_start(args, format);
    pogo_print_status_t status = pogo_log_vprintf(printer, format, args);
    va_end(args);
    return status;
}

/** Exactly one commit site: even multi-page messages/checkpoints cannot
 * program more than one flash page in a call. No retry of torn pages. */
static pogo_print_status_t drain(pogo_log_printer_t *printer,
                                 size_t terminal_budget, bool force) {
    if (!valid(printer)) return POGO_PRINT_INVALID_ARGUMENT;
    pogo_print_status_t result = append_pending(printer);
    if (result != POGO_PRINT_OK) return result;
    pogo_flash_log_status_t status = force ?
        pogo_flash_log_force_flush(printer->flash) :
        pogo_flash_log_service(printer->flash);
    result = flash_result(printer, status);
    if (result != POGO_PRINT_OK) return result;
    result = append_pending(printer);
    if (result != POGO_PRINT_OK) return result;
    /* Mirror only bytes already accepted by the flash cache. The callback
     * sees one bounded span and may accept a shorter prefix or nothing. */
    if (printer->terminal != NULL && terminal_budget != 0u &&
        printer->terminal_offset < printer->flash_offset) {
        size_t count = printer->flash_offset - printer->terminal_offset;
        if (count > terminal_budget) count = terminal_budget;
        size_t accepted = printer->terminal(printer->terminal_context,
            printer->buffer + printer->terminal_offset, count);
        if (accepted > count) return POGO_PRINT_TERMINAL_ERROR;
        printer->terminal_offset += accepted;
    }
    release_completed(printer);
    return force && (printer->length != 0u || printer->flash->used != 0u) ?
        POGO_PRINT_BUSY : POGO_PRINT_OK;
}

pogo_print_status_t pogo_log_service(pogo_log_printer_t *printer,
                                     size_t terminal_budget) {
    return drain(printer, terminal_budget, false);
}

pogo_print_status_t pogo_log_flush(pogo_log_printer_t *printer,
                                   size_t terminal_budget) {
    return drain(printer, terminal_budget, true);
}

bool pogo_log_pending(const pogo_log_printer_t *printer) {
    return valid(printer) && printer->length != 0u;
}
