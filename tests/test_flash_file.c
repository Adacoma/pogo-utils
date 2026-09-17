#include "src/pogo-utils/flash_file.h"

/* Pogosim renames firmware entry points; this file is an ordinary host test. */
#ifdef main
#undef main
#endif

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint8_t fake_flash[256u][POGO_FLASH_FILE_PAGE_SIZE];
static unsigned read_count;
static unsigned write_count;
static bool corrupt_next_write;

void erase_write_section_flash(void) {
    memset(fake_flash, 0xff, sizeof(fake_flash));
}

void write_page_flash(uint8_t page, const void *data) {
    memcpy(fake_flash[page], data, POGO_FLASH_FILE_PAGE_SIZE);
    ++write_count;
    if (corrupt_next_write) {
        fake_flash[page][20] ^= 1u;
        corrupt_next_write = false;
    }
}

void read_page_flash(uint8_t page, char *buffer) {
    memcpy(buffer, fake_flash[page], POGO_FLASH_FILE_PAGE_SIZE);
    ++read_count;
}

static void fill_page(uint8_t *page, uint8_t seed) {
    for (size_t i = 0u; i < POGO_FLASH_FILE_PAGE_SIZE; ++i) {
        page[i] = (uint8_t)(seed + (uint8_t)i);
    }
}

int main(void) {
    uint8_t output[POGO_FLASH_FILE_PAGE_SIZE];
    pogo_flash_file_info_t info;
    uint8_t one_page[POGO_FLASH_FILE_PAGE_SIZE];
    uint8_t replacement[POGO_FLASH_FILE_PAGE_SIZE];
    uint8_t three_pages[3u * POGO_FLASH_FILE_PAGE_SIZE];
    fill_page(one_page, 7u);
    fill_page(replacement, 91u);
    fill_page(three_pages, 13u);
    fill_page(three_pages + POGO_FLASH_FILE_PAGE_SIZE, 29u);
    fill_page(three_pages + 2u * POGO_FLASH_FILE_PAGE_SIZE, 47u);

    /* Pogosim leaves fresh robot flash zero-filled until its erase API is
     * called; physical erased flash reads as 0xff. Accept both virgin forms. */
    memset(fake_flash, 0, sizeof(fake_flash));
    assert(pogo_flash_file_find(1u, &info) == POGO_FLASH_FILE_UNFORMATTED);
    assert(pogo_flash_file_find_by_name("x", &info) ==
           POGO_FLASH_FILE_UNFORMATTED);
    assert(pogo_flash_file_create(1u, "x", 1u, 1u, one_page) ==
           POGO_FLASH_FILE_UNFORMATTED);
    fake_flash[0][17] = 1u;
    assert(pogo_flash_file_find(1u, &info) ==
           POGO_FLASH_FILE_CORRUPT_CATALOG);

    erase_write_section_flash();
    assert(pogo_flash_file_find(1u, &info) == POGO_FLASH_FILE_UNFORMATTED);
    assert(pogo_flash_file_find_by_name("x", &info) ==
           POGO_FLASH_FILE_UNFORMATTED);
    assert(pogo_flash_file_create(1u, "x", 1u, 1u, one_page) ==
           POGO_FLASH_FILE_UNFORMATTED);
    assert(pogo_flash_file_format() == POGO_FLASH_FILE_OK);
    assert(write_count == 2u);
    assert(pogo_flash_file_find(1u, &info) == POGO_FLASH_FILE_NOT_FOUND);

    assert(pogo_flash_file_create(
        POGO_FLASH_FILE_ID_MAGNETOMETER_CALIBRATION,
        POGO_FLASH_FILE_NAME_MAGNETOMETER_CALIBRATION,
        1u, 3u, one_page) == POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_find(1u, &info) == POGO_FLASH_FILE_OK);
    assert(info.id == 1u && info.first_page == 2u && info.page_count == 1u);
    assert(info.format_version == 3u && info.generation == 1u);
    assert(strcmp(info.name, POGO_FLASH_FILE_NAME_MAGNETOMETER_CALIBRATION) == 0);

    read_count = 0u;
    assert(pogo_flash_file_read_page_fast(1u, 0u, output, &info) ==
           POGO_FLASH_FILE_OK);
    assert(read_count == 2u);
    assert(memcmp(output, one_page, sizeof(output)) == 0);
    read_count = 0u;
    assert(pogo_flash_file_read_page_secure(1u, 0u, output, NULL) ==
           POGO_FLASH_FILE_OK);
    assert(read_count == 2u);
    assert(memcmp(output, one_page, sizeof(output)) == 0);
    assert(pogo_flash_file_find_by_name(
        POGO_FLASH_FILE_NAME_MAGNETOMETER_CALIBRATION, &info) ==
        POGO_FLASH_FILE_OK && info.id == 1u);

    /* ID 6 maps directly to the first slot of the second catalog page. */
    assert(pogo_flash_file_create(6u, "second_catalog", 1u, 1u, replacement) ==
           POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_find(6u, &info) == POGO_FLASH_FILE_OK);
    assert(info.first_page == 3u);
    assert(pogo_flash_file_create(4u, "second_catalog", 1u, 1u, one_page) ==
           POGO_FLASH_FILE_NAME_EXISTS);

    assert(pogo_flash_file_create(2u, "three_pages", 3u, 9u, three_pages) ==
           POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_find(2u, &info) == POGO_FLASH_FILE_OK);
    assert(info.first_page == 4u && info.page_count == 3u);
    read_count = 0u;
    assert(pogo_flash_file_read_page_secure(2u, 1u, output, NULL) ==
           POGO_FLASH_FILE_OK);
    assert(read_count == 5u); /* catalog + three checks + requested-page reread */
    assert(memcmp(output, three_pages + POGO_FLASH_FILE_PAGE_SIZE,
                  POGO_FLASH_FILE_PAGE_SIZE) == 0);

    /* Fast access returns damaged bytes; secure access detects them. */
    fake_flash[info.first_page][17] ^= 1u;
    assert(pogo_flash_file_read_page_fast(2u, 0u, output, NULL) ==
           POGO_FLASH_FILE_OK);
    assert(memcmp(output, three_pages, POGO_FLASH_FILE_PAGE_SIZE) != 0);
    assert(pogo_flash_file_read_page_secure(2u, 0u, output, NULL) ==
           POGO_FLASH_FILE_BAD_CHECKSUM);
    assert(pogo_flash_file_replace(2u, 2u, three_pages) ==
           POGO_FLASH_FILE_INVALID_SIZE);
    assert(pogo_flash_file_replace(2u, 3u, three_pages) == POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_find(2u, &info) == POGO_FLASH_FILE_OK &&
           info.generation == 2u);

    assert(pogo_flash_file_replace(1u, 1u, replacement) == POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_find(1u, &info) == POGO_FLASH_FILE_OK &&
           info.first_page == 2u && info.page_count == 1u && info.generation == 2u);
    assert(pogo_flash_file_read_page_secure(1u, 0u, output, NULL) ==
           POGO_FLASH_FILE_OK);
    assert(memcmp(output, replacement, sizeof(output)) == 0);

    /* The fast path intentionally ignores only CRC; structural bounds remain. */
    uint8_t saved_catalog[POGO_FLASH_FILE_PAGE_SIZE];
    memcpy(saved_catalog, fake_flash[0], sizeof(saved_catalog));
    fake_flash[0][POGO_FLASH_FILE_PAGE_SIZE - 1u] ^= 1u;
    assert(pogo_flash_file_read_page_fast(1u, 0u, output, NULL) ==
           POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_read_page_secure(1u, 0u, output, NULL) ==
           POGO_FLASH_FILE_CORRUPT_CATALOG);
    assert(pogo_flash_file_replace(1u, 1u, one_page) ==
           POGO_FLASH_FILE_CORRUPT_CATALOG);
    memcpy(fake_flash[0], saved_catalog, sizeof(saved_catalog));
    fake_flash[0][12u + 4u] = 1u; /* Structurally invalid data-page allocation. */
    assert(pogo_flash_file_read_page_fast(1u, 0u, output, NULL) ==
           POGO_FLASH_FILE_CORRUPT_CATALOG);
    memcpy(fake_flash[0], saved_catalog, sizeof(saved_catalog));

    assert(pogo_flash_file_read_page_fast(1u, 1u, output, NULL) ==
           POGO_FLASH_FILE_INVALID_ARGUMENT);
    assert(pogo_flash_file_create(1u, "duplicate", 1u, 1u, one_page) ==
           POGO_FLASH_FILE_ALREADY_EXISTS);
    assert(pogo_flash_file_create(0u, "bad", 1u, 1u, one_page) ==
           POGO_FLASH_FILE_INVALID_ARGUMENT);
    assert(pogo_flash_file_create(11u, "bad", 1u, 1u, one_page) ==
           POGO_FLASH_FILE_INVALID_ARGUMENT);
    assert(pogo_flash_file_create(3u, "bad", 0u, 1u, one_page) ==
           POGO_FLASH_FILE_INVALID_SIZE);
    assert(pogo_flash_file_create(3u, "bad", 9u, 1u, one_page) ==
           POGO_FLASH_FILE_INVALID_SIZE);

    assert(pogo_flash_file_delete(1u) == POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_find(1u, &info) == POGO_FLASH_FILE_NOT_FOUND);
    assert(pogo_flash_file_create(3u, "reuses_hole", 1u, 1u, one_page) ==
           POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_find(3u, &info) == POGO_FLASH_FILE_OK &&
           info.first_page == 2u);

    /* Every write is read back; corruption is reported immediately. */
    corrupt_next_write = true;
    assert(pogo_flash_file_replace(3u, 1u, replacement) ==
           POGO_FLASH_FILE_VERIFY_FAILED);
    assert(pogo_flash_file_read_page_secure(3u, 0u, output, NULL) ==
           POGO_FLASH_FILE_BAD_CHECKSUM);

    puts("flash file tests passed");
    return 0;
}
