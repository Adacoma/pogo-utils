/** Host regression for append-only logs on one-way-programmable NOR flash. */
#include "src/pogo-utils/flash_log.h"
#include "pogobot.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint8_t flash[POGO_FLASH_FILE_USER_PAGES][POGO_FLASH_FILE_PAGE_SIZE];
static unsigned sector_erases;
static unsigned page_programs;
static int torn_page = -1;

/** Keep a deliberately malformed entry's outer CRC valid, so structural
 * validation (rather than checksum failure) must detect it. */
static void refresh_catalog_crc(uint8_t catalog) {
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0u; i < 252u; ++i) {
        crc ^= flash[catalog][i];
        for (unsigned bit = 0u; bit < 8u; ++bit) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (UINT32_C(0xedb88320) & mask);
        }
    }
    crc ^= UINT32_MAX;
    for (unsigned i = 0u; i < 4u; ++i) {
        flash[catalog][252u + i] = (uint8_t)(crc >> (8u * i));
    }
}

void erase_write_section_flash(void) { memset(flash, 0xff, sizeof(flash)); }

int spiBeginErase4(uint32_t address) {
    assert(address >= POGOBOT_USER_FLASH_START_OFFSET);
    uint32_t offset = address - POGOBOT_USER_FLASH_START_OFFSET;
    assert(offset < sizeof(flash) && offset % 4096u == 0u);
    memset(&flash[offset / POGO_FLASH_FILE_PAGE_SIZE], 0xff, 4096u);
    ++sector_erases;
    return 0;
}

void write_page_flash(uint16_t page, const void *data) {
    const uint8_t *bytes = data;
    unsigned limit = (int)page == torn_page ? 128u : POGO_FLASH_FILE_PAGE_SIZE;
    for (unsigned i = 0u; i < limit; ++i) flash[page][i] &= bytes[i];
    ++page_programs;
}

void read_page_flash(uint16_t page, char *buffer) {
    memcpy(buffer, flash[page], POGO_FLASH_FILE_PAGE_SIZE);
}

static void append_all(pogo_flash_log_t *log, const void *bytes, size_t length) {
    const uint8_t *p = bytes;
    while (length > 0u) {
        size_t accepted = 0u;
        pogo_flash_log_status_t status = pogo_flash_log_append(
            log, p, length, &accepted);
        assert(status == POGO_FLASH_LOG_OK ||
               status == POGO_FLASH_LOG_NEEDS_SERVICE);
        p += accepted;
        length -= accepted;
        if (status == POGO_FLASH_LOG_NEEDS_SERVICE) {
            assert(pogo_flash_log_service(log) == POGO_FLASH_LOG_OK);
        }
    }
}

int main(void) {
    erase_write_section_flash();
    assert(pogo_flash_file_format() == POGO_FLASH_FILE_OK);
    uint8_t calibration[POGO_FLASH_FILE_PAGE_SIZE];
    memset(calibration, 0x5au, sizeof(calibration));
    assert(pogo_flash_file_create(1u, "calibration", 1u, 1u, calibration) ==
           POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_create(4u, "reserved", 1u,
           POGO_FLASH_LOG_FORMAT_VERSION, calibration) ==
           POGO_FLASH_FILE_INVALID_ARGUMENT);

    pogo_flash_log_t print_log;
    pogo_flash_log_t csv_log;
    bool formatted = true;
    assert(pogo_flash_log_initialize(&print_log, 2u, "prints", 2u, false,
                                     &formatted) == POGO_FLASH_LOG_OK);
    assert(!formatted);
    assert(pogo_flash_log_initialize(&csv_log, 3u, "csv", 1u, false,
                                     &formatted) == POGO_FLASH_LOG_OK);
    assert(!formatted);
    pogo_flash_file_info_t info;
    assert(pogo_flash_file_find(2u, &info) == POGO_FLASH_FILE_OK);
    assert(info.format_version == POGO_FLASH_LOG_FORMAT_VERSION);
    assert(pogo_flash_file_read_page_secure(2u, 0u, calibration, NULL) ==
           POGO_FLASH_FILE_UNSUPPORTED_FORMAT);
    assert(pogo_flash_file_replace(2u, 2u, calibration) ==
           POGO_FLASH_FILE_UNSUPPORTED_FORMAT);

    unsigned erases_before = sector_erases;
    unsigned programs_before = page_programs;
    uint8_t full[POGO_FLASH_LOG_PAYLOAD_SIZE];
    memset(full, 'A', sizeof(full));
    append_all(&print_log, full, sizeof(full));
    assert(pogo_flash_log_service(&print_log) == POGO_FLASH_LOG_OK);
    assert(sector_erases == erases_before);
    assert(page_programs == programs_before + 1u);
    append_all(&print_log, "end\n", 4u);
    assert(pogo_flash_log_service(&print_log) == POGO_FLASH_LOG_OK);
    assert(page_programs == programs_before + 1u); /* Partial stays in RAM. */
    assert(pogo_flash_log_force_flush(&print_log) == POGO_FLASH_LOG_OK);
    assert(sector_erases == erases_before);
    assert(page_programs == programs_before + 2u);

    char number[13];
    size_t n = pogo_flash_log_format_scaled_i32(number, -12345, 2u);
    assert(n == 7u && memcmp(number, "-123.45", n) == 0);
    n = pogo_flash_log_format_i32(number, INT32_MIN);
    assert(n == 11u && memcmp(number, "-2147483648", n) == 0);
    n = pogo_flash_log_format_u32(number, UINT32_MAX);
    assert(n == 10u && memcmp(number, "4294967295", n) == 0);
    n = pogo_flash_log_format_scaled_i32(number, INT32_MIN, 6u);
    assert(n == 12u && memcmp(number, "-2147.483648", n) == 0);
    append_all(&csv_log, "time,value\n", 11u);
    assert(pogo_flash_log_force_flush(&csv_log) == POGO_FLASH_LOG_OK);
    size_t accepted = 99u;
    assert(pogo_flash_log_append(&csv_log, "x", 1u, &accepted) ==
           POGO_FLASH_LOG_FULL && accepted == 0u);

    pogo_flash_log_t reopened;
    assert(pogo_flash_log_open(&reopened, 2u) == POGO_FLASH_LOG_OK);
    uint8_t page[POGO_FLASH_FILE_PAGE_SIZE];
    uint8_t used = 0u;
    assert(pogo_flash_log_read_page(&reopened, 0u, page, &used) ==
           POGO_FLASH_LOG_OK && used == sizeof(full));
    assert(memcmp(page + POGO_FLASH_LOG_HEADER_SIZE, full, sizeof(full)) == 0);
    assert(pogo_flash_log_read_page(&reopened, 1u, page, &used) ==
           POGO_FLASH_LOG_OK && used == 4u);
    assert(memcmp(page + POGO_FLASH_LOG_HEADER_SIZE, "end\n", 4u) == 0);
    assert(pogo_flash_log_clear(&reopened) == POGO_FLASH_LOG_OK);
    assert(pogo_flash_log_read_page(&reopened, 0u, page, &used) ==
           POGO_FLASH_LOG_END);
    assert(pogo_flash_file_read_page_secure(1u, 0u, page, NULL) ==
           POGO_FLASH_FILE_OK);
    assert(pogo_flash_log_open(&csv_log, 3u) == POGO_FLASH_LOG_OK);
    assert(pogo_flash_log_read_page(&csv_log, 0u, page, &used) ==
           POGO_FLASH_LOG_OK);

    /* Bytes left below one page are intentionally lost on a simulated reset;
     * reopening finds the first erased page and can resume there. */
    append_all(&reopened, "not committed", 13u);
    assert(pogo_flash_log_open(&reopened, 2u) == POGO_FLASH_LOG_OK);
    assert(pogo_flash_log_read_page(&reopened, 0u, page, &used) ==
           POGO_FLASH_LOG_END);

    append_all(&reopened, full, sizeof(full));
    assert(pogo_flash_log_service(&reopened) == POGO_FLASH_LOG_OK);
    append_all(&reopened, full, sizeof(full));
    torn_page = reopened.first_page + 1u;
    assert(pogo_flash_log_service(&reopened) == POGO_FLASH_LOG_VERIFY_FAILED);
    torn_page = -1;
    assert(pogo_flash_log_open(&reopened, 2u) == POGO_FLASH_LOG_CORRUPT);
    assert(pogo_flash_log_read_page(&reopened, 0u, page, &used) ==
           POGO_FLASH_LOG_OK && used == sizeof(full));
    assert(pogo_flash_log_read_page(&reopened, 1u, page, &used) ==
           POGO_FLASH_LOG_CORRUPT);
    assert(pogo_flash_log_clear(&reopened) == POGO_FLASH_LOG_OK);

    /* Creating on a corrupt catalog deliberately reformats all user files. */
    flash[0][4] = 0u;
    assert(pogo_flash_log_initialize(&reopened, 4u, "new", 1u, false,
                                     &formatted) == POGO_FLASH_LOG_OK);
    assert(formatted);
    assert(pogo_flash_file_find(1u, &info) == POGO_FLASH_FILE_NOT_FOUND);
    erase_write_section_flash();
    assert(pogo_flash_log_initialize(&reopened, 4u, "new", 1u, false,
                                     &formatted) == POGO_FLASH_LOG_OK);
    assert(formatted); /* Virgin flash has the same explicit recovery policy. */
    flash[0][12u + 3u * 48u + 6u] = 0u; /* ID 4 with impossible page count. */
    refresh_catalog_crc(0u);
    assert(pogo_flash_log_initialize(&reopened, 5u, "later", 1u, false,
                                     &formatted) == POGO_FLASH_LOG_OK);
    assert(formatted);
    assert(pogo_flash_file_find(4u, &info) == POGO_FLASH_FILE_NOT_FOUND);

    /* No numeric ID is reserved for calibration; even ID 1 can host a log. */
    erase_write_section_flash();
    assert(pogo_flash_file_create(4u, "hole", 1u, 1u, calibration) ==
           POGO_FLASH_FILE_OK);
    assert(pogo_flash_log_initialize(&reopened, 1u, "id_one_log", 1u, false,
                                     &formatted) == POGO_FLASH_LOG_OK);
    assert(pogo_flash_file_find(1u, &info) == POGO_FLASH_FILE_OK &&
           info.format_version == POGO_FLASH_LOG_FORMAT_VERSION &&
           info.first_page == 272u);
    append_all(&reopened, "moved log\n", 10u);
    assert(pogo_flash_log_force_flush(&reopened) == POGO_FLASH_LOG_OK);
    assert(pogo_flash_file_delete(4u) == POGO_FLASH_FILE_OK);
    pogo_flash_file_defrag_t defrag;
    assert(pogo_flash_file_defrag_begin(&defrag) == POGO_FLASH_FILE_OK);
    bool done = false;
    for (unsigned steps = 0u; steps < 20u && !done; ++steps) {
        assert(pogo_flash_file_defrag_step(&defrag, &done) == POGO_FLASH_FILE_OK);
    }
    assert(done && defrag.moved_files == 1u);
    assert(pogo_flash_log_open(&reopened, 1u) == POGO_FLASH_LOG_OK);
    assert(reopened.first_page == 256u);
    assert(pogo_flash_log_read_page(&reopened, 0u, page, &used) ==
           POGO_FLASH_LOG_OK && used == 10u);
    assert(memcmp(page + POGO_FLASH_LOG_HEADER_SIZE, "moved log\n", 10u) == 0);
    puts("NOR flash log tests passed");
    return 0;
}
