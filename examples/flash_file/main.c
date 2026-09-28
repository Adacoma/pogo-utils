/**
 * @file main.c
 * @brief Read-only inventory of the bounded PFFS user-flash filesystem.
 *
 * Stable file IDs 1..10 select the two fixed catalog pages. The example checks
 * both catalog CRCs even when every slot is empty, lists occupied slots, then
 * checks each complete data CRC with the secure reader. No writer or
 * calibration routine is called.
 *
 * A valid entry can still report a validation failure: the fast lookup checks
 * structure, while the secure read detects damaged catalog or payload bytes.
 */
#include "pogobase.h"
#include "pogo-utils/flash_file.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/** One reusable data-page buffer per robot. Keeping it in USERDATA avoids a
 * 256-byte local array on a small embedded task stack. */
typedef struct {
    uint8_t page[POGO_FLASH_FILE_PAGE_SIZE];
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

/** Check a raw catalog, including the checksum of an entirely empty page.
 *
 * File lookup intentionally skips CRC work and returns NOT_FOUND for an empty
 * slot. This inventory mirrors the documented PFFS v2 header and CRC layout
 * locally so it works with the already installed simulator library and adds no
 * code to the mission firmware reader. `scratch` is reusable USERDATA storage.
 */
static pogo_flash_file_status_t check_catalog(
    uint8_t catalog_index,
    uint8_t scratch[POGO_FLASH_FILE_PAGE_SIZE]) {
    static const uint8_t magic[4] = {'P', 'F', 'F', 'S'};
    const unsigned crc_offset = POGO_FLASH_FILE_PAGE_SIZE - sizeof(uint32_t);
    bool all_zero = true;
    bool all_erased = true;
    read_page_flash(catalog_index, (char *)scratch);
    for (unsigned i = 0u; i < POGO_FLASH_FILE_PAGE_SIZE; ++i) {
        all_zero = all_zero && scratch[i] == 0u;
        all_erased = all_erased && scratch[i] == 0xffu;
    }
    if (memcmp(scratch, magic, sizeof(magic)) != 0 || scratch[4] != 2u ||
        scratch[5] != catalog_index ||
        scratch[6] != POGO_FLASH_FILE_MAX_FILES / POGO_FLASH_FILE_CATALOG_PAGES ||
        scratch[7] != 0u) {
        return all_zero || all_erased ? POGO_FLASH_FILE_UNFORMATTED :
            POGO_FLASH_FILE_CORRUPT_CATALOG;
    }
    /* CRC-32/ISO-HDLC covers bytes 0..251; the stored checksum is little
     * endian at bytes 252..255. This bitwise form needs no lookup table. */
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0u; i < crc_offset; ++i) {
        crc ^= scratch[i];
        for (unsigned bit = 0u; bit < 8u; ++bit) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (UINT32_C(0xedb88320) & mask);
        }
    }
    crc ^= UINT32_MAX;
    uint32_t stored = (uint32_t)scratch[crc_offset] |
        ((uint32_t)scratch[crc_offset + 1u] << 8) |
        ((uint32_t)scratch[crc_offset + 2u] << 16) |
        ((uint32_t)scratch[crc_offset + 3u] << 24);
    if (crc != stored) {
        /* An empty slot can coexist with a bad page checksum. Emit only the
         * small fixed header/first-slot diagnostic needed to distinguish an
         * interrupted update from an invalidated catalog; do not dump payloads
         * or modify flash. Offsets follow the documented PFFS v2 layout. */
        uint32_t generation = (uint32_t)scratch[8] |
            ((uint32_t)scratch[9] << 8) |
            ((uint32_t)scratch[10] << 16) |
            ((uint32_t)scratch[11] << 24);
        printf("# FLASH_CATALOG_CRC,page=%u,stored=%08lx,calculated=%08lx,"
               "generation=%lu,first_id=%u,first_flags=%u\n",
               (unsigned)catalog_index, (unsigned long)stored,
               (unsigned long)crc, (unsigned long)generation,
               (unsigned)scratch[12], (unsigned)scratch[19]);
        return POGO_FLASH_FILE_CORRUPT_CATALOG;
    }
    return POGO_FLASH_FILE_OK;
}

/** Inspect one stable slot and update the summary counters.
 *
 * `find` supplies metadata even when the subsequent CRC scan fails. The secure
 * reader uses the same USERDATA page as scratch and does not retain its address.
 */
static void list_file(uint8_t id, unsigned *found, unsigned *empty,
                      unsigned *errors) {
    pogo_flash_file_info_t info;
    pogo_flash_file_status_t status = pogo_flash_file_find(id, &info);
    if (status == POGO_FLASH_FILE_NOT_FOUND) {
        ++*empty;
        return;
    }
    if (status != POGO_FLASH_FILE_OK) {
        ++*errors;
        printf("# FLASH_FILE_ERROR,id=%u,reason=%s\n", (unsigned)id,
               pogo_flash_file_status_string(status));
        return;
    }

    ++*found;
    /* A secure read verifies the selected catalog page and every data page.
     * file_page 0 is sufficient because the check covers the whole file. */
    pogo_flash_file_status_t validation = pogo_flash_file_read_page_secure(
        id, 0u, mydata->page, NULL);
    if (validation != POGO_FLASH_FILE_OK) ++*errors;

    const char *name = info.name_length > 0u ? info.name : "(unnamed)";
    printf("# FLASH_FILE,id=%u,name=%s,format_version=%u,first_page=%u,"
           "pages=%u,bytes=%u,generation=%lu,data_crc32=%08lx,validation=%s\n",
           (unsigned)info.id, name, (unsigned)info.format_version,
           (unsigned)info.first_page, (unsigned)info.page_count,
           (unsigned)info.page_count * (unsigned)POGO_FLASH_FILE_PAGE_SIZE,
           (unsigned long)info.generation, (unsigned long)info.data_crc32,
           pogo_flash_file_status_string(validation));
}

void user_init(void) {
    /* The inventory runs once at startup. No radio callbacks or periodic
     * application work are needed after the flash reads finish. */
    main_loop_hz = 1;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;

    unsigned found = 0u;
    unsigned empty = 0u;
    unsigned errors = 0u;
    unsigned unreadable = 0u; /* IDs skipped because their catalog failed. */
    printf("# FLASH_FILE_LIST_START,robot=%u,max_files=%u\n",
           (unsigned)pogobot_helper_getid(), (unsigned)POGO_FLASH_FILE_MAX_FILES);

    for (uint8_t catalog = 0u; catalog < POGO_FLASH_FILE_CATALOG_PAGES;
         ++catalog) {
        /* The per-ID fast lookup cannot see a bad CRC on an empty slot. Check
         * each catalog first and never report its IDs as healthy empties when
         * its header or checksum is invalid. */
        pogo_flash_file_status_t status = check_catalog(
            catalog, mydata->page);
        if (status != POGO_FLASH_FILE_OK) {
            ++errors;
            unreadable += POGO_FLASH_FILE_MAX_FILES /
                POGO_FLASH_FILE_CATALOG_PAGES;
            printf("# FLASH_CATALOG_ERROR,page=%u,reason=%s\n",
                   (unsigned)catalog, pogo_flash_file_status_string(status));
            continue;
        }
        for (uint8_t offset = 0u;
             offset < POGO_FLASH_FILE_MAX_FILES /
                 POGO_FLASH_FILE_CATALOG_PAGES; ++offset) {
            uint8_t id = (uint8_t)(catalog *
                (POGO_FLASH_FILE_MAX_FILES / POGO_FLASH_FILE_CATALOG_PAGES) +
                offset + 1u);
            list_file(id, &found, &empty, &errors);
        }
    }
    printf("# FLASH_FILE_LIST_END,robot=%u,found=%u,empty=%u,unreadable=%u,"
           "errors=%u\n", (unsigned)pogobot_helper_getid(), found, empty,
           unreadable, errors);
    /* Green means both catalogs and every discovered file passed CRC checks.
     * Violet marks a bad catalog or damaged data; no flash is changed. */
    pogobot_led_setColor(errors == 0u ? 0u : 25u,
                         errors == 0u ? 25u : 0u,
                         errors == 0u ? 0u : 25u);
}

void user_step(void) {
    /* The listing is a one-shot diagnostic; the platform still requires a
     * periodic callback after user_init. */
}

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    /* Flash-state import identifies every category, including arena walls.
     * Registering their default callbacks preserves the calibration archive's
     * robot/category identities when the inventory runs in Pogosim. */
    pogobot_start(default_walls_user_init, default_walls_user_step, "walls");
    return 0;
}
