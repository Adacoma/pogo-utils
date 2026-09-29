#include "src/pogo-utils/flash_file.h"
#include "pogobot.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/** Model NOR page program as bitwise AND: writing a one cannot restore an
 * already programmed zero. The production writer must erase sectors first. */
static uint8_t flash[POGO_FLASH_FILE_USER_PAGES][POGO_FLASH_FILE_PAGE_SIZE];
static unsigned sector_erases;

void erase_write_section_flash(void) {
    memset(flash, 0xff, sizeof(flash));
}

int spiBeginErase4(uint32_t erase_addr) {
    assert(erase_addr >= POGOBOT_USER_FLASH_START_OFFSET);
    uint32_t offset = erase_addr - POGOBOT_USER_FLASH_START_OFFSET;
    assert(offset < sizeof(flash) && offset % 4096u == 0u);
    memset(&flash[offset / POGO_FLASH_FILE_PAGE_SIZE], 0xff, 4096u);
    ++sector_erases;
    return 0;
}

void write_page_flash(uint16_t page, const void *data) {
    const uint8_t *bytes = data;
    for (unsigned i = 0u; i < POGO_FLASH_FILE_PAGE_SIZE; ++i) {
        flash[page][i] &= bytes[i];
    }
}

void read_page_flash(uint16_t page, char *buffer) {
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
    assert(flash[0][4] == 3u);
    assert(sector_erases == POGO_FLASH_FILE_CATALOG_PAGES);

    assert(pogo_flash_file_create(1u, "one", 1u, 1u, first) ==
           POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_find(1u, &info) == POGO_FLASH_FILE_OK &&
           info.first_page == 256u);
    assert(pogo_flash_file_create(6u, "six", 1u, 1u, second) ==
           POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_find(6u, &info) == POGO_FLASH_FILE_OK &&
           info.first_page == 272u);
    assert(sector_erases == 20u); /* Format plus data/catalog for each create. */

    assert(pogo_flash_file_replace(1u, 1u, replacement) ==
           POGO_FLASH_FILE_OK);
    assert(sector_erases == 22u);
    assert(pogo_flash_file_read_page_secure(1u, 0u, output, NULL) ==
           POGO_FLASH_FILE_OK);
    assert(memcmp(output, replacement, sizeof(output)) == 0);
    assert(pogo_flash_file_read_page_secure(6u, 0u, output, NULL) ==
           POGO_FLASH_FILE_OK);
    assert(memcmp(output, second, sizeof(output)) == 0);

    assert(pogo_flash_file_delete(1u) == POGO_FLASH_FILE_OK);
    assert(sector_erases == 23u); /* Delete rewrites only catalog sector. */
    assert(pogo_flash_file_create(3u, "three", 1u, 1u, first) ==
           POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_find(3u, &info) == POGO_FLASH_FILE_OK &&
           info.first_page == 256u);
    assert(pogo_flash_file_read_page_secure(6u, 0u, output, NULL) ==
           POGO_FLASH_FILE_OK);
    /* Even recognizable but damaged catalogs are reset on a new-file write;
     * this intentionally discards the unrelated ID-6 file. */
    flash[0][4] = 0u;
    assert(pogo_flash_file_create(4u, "recovered", 1u, 1u, first) ==
           POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_find(6u, &info) == POGO_FLASH_FILE_NOT_FOUND);
    assert(flash[0][4] == 3u);

    /* A blank catalog may coexist with orphaned data after interrupted
     * catalog erasure. Create now deliberately erases that data as well. */
    erase_write_section_flash();
    flash[256u][5u] = 0u;
    assert(pogo_flash_file_create(1u, "fresh", 1u, 1u, first) ==
           POGO_FLASH_FILE_OK);
    assert(flash[256u][5u] == first[5u]);
    assert(pogo_flash_file_read_page_secure(1u, 0u, output, NULL) ==
           POGO_FLASH_FILE_OK);
    /* Blank creation relies on erase (not programming all-one bytes), and a
     * rename must erase/rebuild the catalog without changing file data. */
    assert(pogo_flash_file_create_blank(2u, "blank", 2u, 1u) ==
           POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_rename(2u, "blank_renamed") ==
           POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_find_by_name("blank_renamed", &info) ==
           POGO_FLASH_FILE_OK && info.page_count == 2u);
    assert(pogo_flash_file_read_page_secure(2u, 1u, output, NULL) ==
           POGO_FLASH_FILE_OK);
    for (unsigned i = 0u; i < sizeof(output); ++i) assert(output[i] == 0xffu);

    /* A one-sector hole can be removed even when the next file spans three
     * sectors: forward copying safely overlaps its old NOR extent. */
    erase_write_section_flash();
    assert(pogo_flash_file_format() == POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_create(1u, "hole", 1u, 1u, first) ==
           POGO_FLASH_FILE_OK);
    pogo_flash_file_writer_t writer;
    assert(pogo_flash_file_write_begin_create(&writer, 2u, "large", 40u, 1u,
           NULL) == POGO_FLASH_FILE_OK);
    for (uint16_t page = 0u; page < 40u; ++page) {
        memset(output, (int)(page + 17u), sizeof(output));
        assert(pogo_flash_file_write_page(&writer, output) == POGO_FLASH_FILE_OK);
    }
    assert(pogo_flash_file_write_finish(&writer) == POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_create(3u, "tail", 1u, 1u, second) ==
           POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_find(2u, &info) == POGO_FLASH_FILE_OK &&
           info.first_page == 272u);
    assert(pogo_flash_file_find(3u, &info) == POGO_FLASH_FILE_OK &&
           info.first_page == 320u);
    assert(pogo_flash_file_delete(1u) == POGO_FLASH_FILE_OK);
    pogo_flash_file_defrag_t defrag;
    assert(pogo_flash_file_defrag_begin(&defrag) == POGO_FLASH_FILE_OK);
    bool done = false;
    for (unsigned steps = 0u; steps < 200u && !done; ++steps) {
        assert(pogo_flash_file_defrag_step(&defrag, &done) == POGO_FLASH_FILE_OK);
    }
    assert(done && defrag.moved_files == 2u);
    assert(pogo_flash_file_find(2u, &info) == POGO_FLASH_FILE_OK &&
           info.first_page == 256u && info.generation == 1u);
    assert(pogo_flash_file_read_page_secure(2u, 39u, output, NULL) ==
           POGO_FLASH_FILE_OK);
    for (unsigned i = 0u; i < sizeof(output); ++i) assert(output[i] == 56u);
    assert(pogo_flash_file_find(3u, &info) == POGO_FLASH_FILE_OK &&
           info.first_page == 304u);
    assert(pogo_flash_file_read_page_secure(3u, 0u, output, NULL) ==
           POGO_FLASH_FILE_OK);
    assert(memcmp(output, second, sizeof(output)) == 0);
    assert(pogo_flash_file_check() == POGO_FLASH_FILE_OK);
    unsigned erases_after_compaction = sector_erases;
    assert(pogo_flash_file_defrag_begin(&defrag) == POGO_FLASH_FILE_OK);
    done = false;
    for (unsigned steps = 0u; steps < 10u && !done; ++steps) {
        assert(pogo_flash_file_defrag_step(&defrag, &done) == POGO_FLASH_FILE_OK);
    }
    assert(done && defrag.moved_files == 0u &&
           sector_erases == erases_after_compaction);

    /* A damaged ordinary file is rejected during pagewise preflight, before
     * its first destination sector is erased. */
    assert(pogo_flash_file_delete(2u) == POGO_FLASH_FILE_OK);
    flash[304u][0u] ^= 1u; /* Damage the remaining file's source page. */
    assert(pogo_flash_file_defrag_begin(&defrag) == POGO_FLASH_FILE_OK);
    unsigned erases_before_preflight = sector_erases;
    assert(pogo_flash_file_defrag_step(&defrag, &done) == POGO_FLASH_FILE_OK);
    assert(pogo_flash_file_defrag_step(&defrag, &done) ==
           POGO_FLASH_FILE_BAD_CHECKSUM);
    assert(sector_erases == erases_before_preflight);
    assert(pogo_flash_file_find(3u, &info) == POGO_FLASH_FILE_OK &&
           info.first_page == 304u);
    puts("NOR flash file tests passed");
    return 0;
}
