#include "src/pogo-utils/flash_file.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/** Model NOR page program as bitwise AND: writing a one cannot restore an
 * already programmed zero. The production writer must erase sectors first. */
static uint8_t flash[256u][POGO_FLASH_FILE_PAGE_SIZE];
static unsigned sector_erases;

void erase_write_section_flash(void) {
    memset(flash, 0xff, sizeof(flash));
}

int spiBeginErase4(uint32_t erase_addr) {
    assert(erase_addr >= UINT32_C(0x290000));
    uint32_t offset = erase_addr - UINT32_C(0x290000);
    assert(offset < sizeof(flash) && offset % 4096u == 0u);
    memset(&flash[offset / POGO_FLASH_FILE_PAGE_SIZE], 0xff, 4096u);
    ++sector_erases;
    return 0;
}

void write_page_flash(uint8_t page, const void *data) {
    const uint8_t *bytes = data;
    for (unsigned i = 0u; i < POGO_FLASH_FILE_PAGE_SIZE; ++i) {
        flash[page][i] &= bytes[i];
    }
}

void read_page_flash(uint8_t page, char *buffer) {
    memcpy(buffer, flash[page], POGO_FLASH_FILE_PAGE_SIZE);
}

int main(void) {
    uint8_t first[POGO_FLASH_FILE_PAGE_SIZE];
    uint8_t second[POGO_FLASH_FILE_PAGE_SIZE];
    uint8_t replacement[POGO_FLASH_FILE_PAGE_SIZE];
    uint8_t output[POGO_FLASH_FILE_PAGE_SIZE];
    pogo_flash_file_info_t info;
    for (unsigned i = 0u; i < POGO_FLASH_FILE_PAGE_SIZE; ++i) {
        first[i] = (uint8_t)i;
        second[i] = (uint8_t)(i ^ 0x5au);
        replacement[i] = (uint8_t)(255u - i);
    }
    erase_write_section_flash();
    assert(pogo_flash_file_format() == POGO_FLASH_FILE_OK);
    assert(flash[0][4] == 2u);

    assert(pogo_flash_file_create(1u, "one", 1u, 1u, first) ==
           POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_find(1u, &info) == POGO_FLASH_FILE_OK &&
           info.first_page == 16u);
    assert(pogo_flash_file_create(6u, "six", 1u, 1u, second) ==
           POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_find(6u, &info) == POGO_FLASH_FILE_OK &&
           info.first_page == 32u);
    assert(sector_erases == 4u); /* Data and catalog sector for each create. */

    assert(pogo_flash_file_replace(1u, 1u, replacement) ==
           POGO_FLASH_FILE_OK);
    assert(sector_erases == 6u);
    assert(pogo_flash_file_read_page_secure(1u, 0u, output, NULL) ==
           POGO_FLASH_FILE_OK);
    assert(memcmp(output, replacement, sizeof(output)) == 0);
    assert(pogo_flash_file_read_page_secure(6u, 0u, output, NULL) ==
           POGO_FLASH_FILE_OK);
    assert(memcmp(output, second, sizeof(output)) == 0);

    assert(pogo_flash_file_delete(1u) == POGO_FLASH_FILE_OK);
    assert(sector_erases == 7u); /* Delete rewrites only catalog sector. */
    assert(pogo_flash_file_create(3u, "three", 1u, 1u, first) ==
           POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_find(3u, &info) == POGO_FLASH_FILE_OK &&
           info.first_page == 16u);
    assert(pogo_flash_file_read_page_secure(6u, 0u, output, NULL) ==
           POGO_FLASH_FILE_OK);
    /* Even recognizable but damaged catalogs are reset on a new-file write;
     * this intentionally discards the unrelated ID-6 file. */
    flash[0][4] = 0u;
    assert(pogo_flash_file_create(4u, "recovered", 1u, 1u, first) ==
           POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_find(6u, &info) == POGO_FLASH_FILE_NOT_FOUND);
    assert(flash[0][4] == 2u);

    /* A blank catalog may coexist with orphaned data after interrupted
     * catalog erasure. Create now deliberately erases that data as well. */
    erase_write_section_flash();
    flash[32u][5u] = 0u;
    assert(pogo_flash_file_create(1u, "fresh", 1u, 1u, first) ==
           POGO_FLASH_FILE_OK);
    assert(flash[32u][5u] == 0xffu);
    assert(pogo_flash_file_read_page_secure(1u, 0u, output, NULL) ==
           POGO_FLASH_FILE_OK);
    puts("NOR flash file tests passed");
    return 0;
}
