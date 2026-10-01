/** Flash-first printing regression with physical NOR semantics and counters:
 * prove producers do no I/O and each service programs at most one page. */
#include "src/pogo-utils/print_log.h"
#include "pogobot.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t flash[POGO_FLASH_FILE_USER_PAGES][POGO_FLASH_FILE_PAGE_SIZE];
static unsigned programs, reads, erases;
static int torn_page = -1;
static char mirrored[2048];
static size_t mirrored_length, mirror_limit = 3u, callback_calls;

void erase_write_section_flash(void) { memset(flash, 0xff, sizeof(flash)); }
int spiBeginErase4(uint32_t address) {
    assert(address >= POGOBOT_USER_FLASH_START_OFFSET);
    uint32_t offset = address - POGOBOT_USER_FLASH_START_OFFSET;
    assert(offset < sizeof(flash) && offset % 4096u == 0u);
    memset(flash[offset / 256u], 0xff, 4096u);
    ++erases;
    return 0;
}
void write_page_flash(uint16_t page, const void *data) {
    assert(page < POGO_FLASH_FILE_USER_PAGES);
    unsigned count = (int)page == torn_page ? 8u : 256u;
    for (unsigned i = 0; i < count; ++i) flash[page][i] &= ((const uint8_t *)data)[i];
    ++programs;
}
void read_page_flash(uint16_t page, char *output) {
    assert(page < POGO_FLASH_FILE_USER_PAGES);
    memcpy(output, flash[page], 256u);
    ++reads;
}
static size_t terminal_write(void *context, const char *bytes, size_t length) {
    assert(context == mirrored);
    ++callback_calls;
    size_t n = length < mirror_limit ? length : mirror_limit;
    assert(mirrored_length + n <= sizeof(mirrored));
    memcpy(mirrored + mirrored_length, bytes, n);
    mirrored_length += n;
    return n;
}
static size_t invalid_terminal(void *context, const char *bytes, size_t length) {
    (void)context; (void)bytes;
    return length + 1u;
}
static void expect_file(pogo_flash_log_t *log, const char *expected, size_t length) {
    pogo_flash_log_t reader;
    assert(pogo_flash_log_open(&reader, log->file_id) == POGO_FLASH_LOG_OK);
    size_t offset = 0u;
    for (uint16_t i = 0; i < reader.next_page; ++i) {
        uint8_t page[256], used;
        assert(pogo_flash_log_read_page(&reader, i, page, &used) == POGO_FLASH_LOG_OK);
        assert(offset + used <= length);
        assert(memcmp(page + POGO_FLASH_LOG_HEADER_SIZE, expected + offset, used) == 0);
        offset += used;
    }
    assert(offset == length);
}

int main(void) {
    erase_write_section_flash();
    assert(pogo_flash_file_format() == POGO_FLASH_FILE_OK);
    pogo_flash_log_t text_log, csv_log;
    assert(pogo_flash_log_initialize(&text_log, 2u, "prints", 16u, false, NULL) == POGO_FLASH_LOG_OK);
    assert(pogo_flash_log_initialize(&csv_log, 3u, "csv", 1u, false, NULL) == POGO_FLASH_LOG_OK);
    pogo_log_printer_t text, csv;
    char text_buffer[700], csv_buffer[64];
    assert(pogo_log_attach(NULL, &text_log, text_buffer, sizeof(text_buffer)) == POGO_PRINT_INVALID_ARGUMENT);
    assert(pogo_log_attach(&text, &text_log, text_buffer, 1u) == POGO_PRINT_INVALID_ARGUMENT);
    assert(pogo_log_attach(&text, &text_log, text_buffer, sizeof(text_buffer)) == POGO_PRINT_OK);
    assert(pogo_log_attach(&csv, &csv_log, csv_buffer, sizeof(csv_buffer)) == POGO_PRINT_OK);
    assert(pogo_log_set_terminal(&text, terminal_write, mirrored) == POGO_PRINT_OK);
    unsigned p = programs, r = reads, e = erases;
    char source[] = "hello";
    assert(pogo_log_printf(&text, "%s %.3Q16.16\n", source, (int32_t)-98304) == POGO_PRINT_OK);
    source[0] = 'X'; /* Producer strings are copied, not retained by pointer. */
    assert(pogo_log_printf(&text, "rejected") == POGO_PRINT_BUSY);
    assert(text.rejected_messages == 1u);
    assert(pogo_log_set_terminal(&text, NULL, NULL) == POGO_PRINT_BUSY);
    assert(programs == p && reads == r && erases == e && callback_calls == 0u);
    assert(pogo_log_service(&text, 0u) == POGO_PRINT_OK);
    assert(callback_calls == 0u && pogo_log_pending(&text));
    mirror_limit = 0u;
    assert(pogo_log_service(&text, 5u) == POGO_PRINT_OK);
    assert(pogo_log_pending(&text) && mirrored_length == 0u);
    mirror_limit = 3u;
    while (pogo_log_pending(&text)) {
        size_t before = mirrored_length, calls = callback_calls;
        assert(pogo_log_service(&text, 2u) == POGO_PRINT_OK);
        assert(mirrored_length - before <= 2u && callback_calls - calls <= 1u);
    }
    assert(programs == p && reads == r && erases == e);
    assert(mirrored_length == strlen("hello -1.500\n"));
    assert(memcmp(mirrored, "hello -1.500\n", mirrored_length) == 0);
    assert(pogo_log_flush(&text, 5u) == POGO_PRINT_OK);
    assert(programs == p + 1u);
    expect_file(&text_log, "hello -1.500\n", strlen("hello -1.500\n"));

    assert(pogo_log_printf(&csv, "%u,%.2Q16.16\n", 7u, (int32_t)81920) == POGO_PRINT_OK);
    assert(pogo_log_flush(&csv, 0u) == POGO_PRINT_OK);
    expect_file(&csv_log, "7,1.25\n", 7u);
    assert(pogo_log_printf(&csv, "more") == POGO_PRINT_FLASH_FULL);
    assert(pogo_log_printf(&csv, "") == POGO_PRINT_OK);

    assert(pogo_flash_log_clear(&text_log) == POGO_FLASH_LOG_OK);
    assert(pogo_log_set_terminal(&text, NULL, NULL) == POGO_PRINT_OK);
    char long_message[601];
    memset(long_message, 'A', 600u); long_message[600] = '\0';
    assert(pogo_log_printf(&text, "%s", long_message) == POGO_PRINT_OK);
    p = programs;
    assert(pogo_log_service(&text, 0u) == POGO_PRINT_OK);
    assert(programs == p + 1u && pogo_log_pending(&text));
    assert(text.flash_offset == 488u && text_log.used == 244u);
    p = programs;
    assert(pogo_log_flush(&text, 0u) == POGO_PRINT_BUSY);
    assert(programs == p + 1u && !pogo_log_pending(&text) && text_log.used == 112u);
    assert(pogo_log_flush(&text, 0u) == POGO_PRINT_OK);
    assert(programs == p + 2u);
    expect_file(&text_log, long_message, 600u);

    /* Overlong and unsupported messages never publish a formatted prefix. */
    p = programs; r = reads; e = erases;
    assert(pogo_log_printf(&text, "%s%s", long_message, long_message) == POGO_PRINT_MESSAGE_TOO_LONG);
    assert(pogo_log_printf(&text, "prefix %f") == POGO_PRINT_UNSUPPORTED_FORMAT);
    assert(!pogo_log_pending(&text));
    assert(programs == p && reads == r && erases == e);
    text.rejected_messages = UINT32_MAX;
    assert(pogo_log_printf(&text, "%f") == POGO_PRINT_UNSUPPORTED_FORMAT);
    assert(text.rejected_messages == UINT32_MAX);

    /* Entire-record capacity check accounts for partially occupied final page. */
    assert(pogo_flash_log_clear(&csv_log) == POGO_FLASH_LOG_OK);
    assert(pogo_log_attach(&csv, &csv_log, text_buffer, sizeof(text_buffer)) == POGO_PRINT_OK);
    assert(pogo_log_printf(&csv, "%s", long_message) == POGO_PRINT_FLASH_FULL);
    assert(!pogo_log_pending(&csv) && csv_log.used == 0u);
    assert(pogo_log_attach(&csv, &csv_log, csv_buffer, sizeof(csv_buffer)) == POGO_PRINT_OK);

    /* Flash-only bursts reuse the staging buffer until the RAM cache fills;
     * they do not need a service call per short print. */
    assert(pogo_flash_log_clear(&text_log) == POGO_FLASH_LOG_OK);
    p = programs; r = reads; e = erases;
    for (unsigned i = 0; i < 10u; ++i) {
        assert(pogo_log_printf(&text, "n=%u\n", i) == POGO_PRINT_OK);
        assert(!pogo_log_pending(&text));
    }
    assert(programs == p && reads == r && erases == e && text_log.used == 40u);
    assert(pogo_log_flush(&text, 0u) == POGO_PRINT_OK);
    expect_file(&text_log, "n=0\nn=1\nn=2\nn=3\nn=4\nn=5\nn=6\nn=7\nn=8\nn=9\n", 40u);

    assert(pogo_log_set_terminal(&text, invalid_terminal, NULL) == POGO_PRINT_OK);
    assert(pogo_log_printf(&text, "bad sink") == POGO_PRINT_OK);
    assert(pogo_log_service(&text, 8u) == POGO_PRINT_TERMINAL_ERROR);
    /* Explicit recovery discards the pending mirror; cache data remains. */
    assert(pogo_log_attach(&text, &text_log, text_buffer, sizeof(text_buffer)) == POGO_PRINT_OK);
    assert(pogo_flash_log_clear(&text_log) == POGO_FLASH_LOG_OK);
    assert(pogo_log_printf(&text, "torn") == POGO_PRINT_OK);
    torn_page = text_log.first_page;
    assert(pogo_log_flush(&text, 0u) == POGO_PRINT_FLASH_ERROR);
    assert(text.flash_status == POGO_FLASH_LOG_VERIFY_FAILED);
    p = programs;
    assert(pogo_log_service(&text, 0u) == POGO_PRINT_FLASH_ERROR);
    assert(programs == p); /* Never retry a torn NOR program in place. */
    puts("print log NOR tests passed");
    return 0;
}
