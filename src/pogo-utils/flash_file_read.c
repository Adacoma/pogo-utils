/**
 * @file flash_file_read.c
 * @brief Minimal non-CRC reader for the bounded Pogobot flash-file catalog.
 *
 * This translation unit contains every helper required by the direct-ID fast
 * path. Keeping CRC, name scanning, and mutation in other objects lets a static
 * linker omit those features from small read-only mission firmware.
 */
#include "flash_file_internal.h"

#include "pogobase.h"

#include <string.h>

/** Four-byte discriminator at the beginning of each catalog page. */
static const uint8_t catalog_magic[4] = {'P', 'F', 'F', 'S'};

/* Compile-time checks tie the arithmetic below to the serialized layout. A
 * layout change must update the constants and cannot silently leave unused or
 * overlapping bytes in a catalog page. */
_Static_assert(POGO_FLASH_FILE_CATALOG_HEADER_SIZE +
    POGO_FLASH_FILE_ENTRIES_PER_CATALOG * POGO_FLASH_FILE_ENTRY_SIZE +
    sizeof(uint32_t) == POGO_FLASH_FILE_PAGE_SIZE,
    "catalog layout must fill exactly one flash page");
_Static_assert(POGO_FLASH_FILE_MAX_FILES ==
    POGO_FLASH_FILE_CATALOG_PAGES * POGO_FLASH_FILE_ENTRIES_PER_CATALOG,
    "stable IDs must map exactly onto catalog slots");

uint16_t pogo_flash_file_internal_get_u16(const uint8_t *p) {
    /* Byte assembly is valid for unaligned buffers and every host endianness. */
    return (uint16_t)((uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8));
}

uint32_t pogo_flash_file_internal_get_u32(const uint8_t *p) {
    /* Keep every shift unsigned so promotion cannot invoke signed overflow. */
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
        ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

bool pogo_flash_file_internal_catalog_header_valid(
    const uint8_t page[POGO_FLASH_FILE_PAGE_SIZE],
    uint8_t expected_catalog_index) {
    /* Header byte 7 is reserved. Requiring zero makes future incompatible
     * layouts fail explicitly instead of being misread as version 1. */
    return memcmp(page, catalog_magic, sizeof(catalog_magic)) == 0 &&
        page[4] == POGO_FLASH_FILE_CATALOG_VERSION &&
        page[5] == expected_catalog_index &&
        page[6] == POGO_FLASH_FILE_ENTRIES_PER_CATALOG &&
        page[7] == 0u;
}

bool pogo_flash_file_internal_page_is_blank(
    const uint8_t page[POGO_FLASH_FILE_PAGE_SIZE]) {
    /* Real erased NOR flash reads as 0xff. Some simulator allocations begin as
     * zero. No other uniform value is a valid first-use representation. */
    bool all_zero = true;
    bool all_erased = true;
    for (size_t i = 0u; i < POGO_FLASH_FILE_PAGE_SIZE; ++i) {
        all_zero = all_zero && page[i] == 0u;
        all_erased = all_erased && page[i] == 0xffu;
    }
    return all_zero || all_erased;
}

bool pogo_flash_file_internal_decode_entry(
    const uint8_t *entry,
    uint8_t expected_id,
    pogo_flash_file_info_t *info) {
    /* Copy frequently used one-byte fields before validation. Reading bytes
     * directly avoids alignment and packed-structure assumptions. */
    uint8_t page_count = entry[5];
    uint8_t first_page = entry[4];
    uint8_t name_length = entry[6];
    uint8_t flags = entry[7];
    /* Use unsigned arithmetic wider than uint8_t so first+count cannot wrap
     * before comparison with the 256-page physical address space. */
    unsigned end_page = (unsigned)first_page + (unsigned)page_count;
    if (flags != POGO_FLASH_FILE_ENTRY_IN_USE || entry[0] != expected_id ||
        entry[1] != 0u || page_count == 0u ||
        page_count > POGO_FLASH_FILE_MAX_PAGES ||
        first_page < POGO_FLASH_FILE_DATA_FIRST_PAGE || end_page > 256u ||
        name_length > POGO_FLASH_FILE_MAX_NAME) {
        return false;
    }
    if (info != NULL) {
        /* Zeroing guarantees deterministic padding and terminates an empty or
         * short optional name before the serialized bytes are copied. */
        memset(info, 0, sizeof(*info));
        info->id = expected_id;
        info->format_version = pogo_flash_file_internal_get_u16(entry + 2);
        info->first_page = first_page;
        info->page_count = page_count;
        info->name_length = name_length;
        info->generation = pogo_flash_file_internal_get_u32(entry + 8);
        info->data_crc32 = pogo_flash_file_internal_get_u32(entry + 12);
        memcpy(info->name, entry + 16, name_length);
        info->name[name_length] = '\0';
    }
    return true;
}

pogo_flash_file_status_t pogo_flash_file_internal_load_slot(
    uint8_t file_id,
    pogo_flash_file_info_t *info,
    uint8_t catalog_page[static POGO_FLASH_FILE_PAGE_SIZE]) {
    if (file_id == 0u || file_id > POGO_FLASH_FILE_MAX_FILES) {
        return POGO_FLASH_FILE_INVALID_ARGUMENT;
    }
    uint8_t *page = catalog_page; /* Short alias keeps offset code readable. */
    /* ID 1 maps to slot 0, ID 6 to slot 5. Division selects one of the two
     * catalog pages; modulo selects the fixed entry within that page. */
    uint8_t slot = (uint8_t)(file_id - 1u);
    uint8_t catalog_index = (uint8_t)(slot / POGO_FLASH_FILE_ENTRIES_PER_CATALOG);
    uint8_t entry_index = (uint8_t)(slot % POGO_FLASH_FILE_ENTRIES_PER_CATALOG);
    read_page_flash(catalog_index, (char *)page);
    if (!pogo_flash_file_internal_catalog_header_valid(page, catalog_index)) {
        /* Blank means the filesystem has never been formatted. Any other
         * invalid header is data/corruption and must not be auto-erased here. */
        return pogo_flash_file_internal_page_is_blank(page) ?
            POGO_FLASH_FILE_UNFORMATTED :
            POGO_FLASH_FILE_CORRUPT_CATALOG;
    }
    const uint8_t *entry = page + POGO_FLASH_FILE_CATALOG_HEADER_SIZE +
        (size_t)entry_index * POGO_FLASH_FILE_ENTRY_SIZE;
    if (entry[7] == 0u) {
        /* Zero is the canonical empty-slot representation created by format
         * and delete. Other flag values are validated as corruption below. */
        return POGO_FLASH_FILE_NOT_FOUND;
    }
    if (!pogo_flash_file_internal_decode_entry(entry, file_id, info)) {
        return POGO_FLASH_FILE_CORRUPT_CATALOG;
    }
    return POGO_FLASH_FILE_OK;
}

pogo_flash_file_status_t pogo_flash_file_find(
    uint8_t file_id,
    pogo_flash_file_info_t *info) {
    if (info == NULL) return POGO_FLASH_FILE_INVALID_ARGUMENT;
    /* The public find API does not expose raw catalog bytes, but the internal
     * loader requires a full page destination for the physical read. */
    uint8_t catalog_page[POGO_FLASH_FILE_PAGE_SIZE];
    return pogo_flash_file_internal_load_slot(file_id, info, catalog_page);
}

pogo_flash_file_status_t pogo_flash_file_read_page_fast(
    uint8_t file_id,
    uint8_t file_page,
    uint8_t output[POGO_FLASH_FILE_PAGE_SIZE],
    pogo_flash_file_info_t *info) {
    if (output == NULL) return POGO_FLASH_FILE_INVALID_ARGUMENT;
    pogo_flash_file_info_t decoded;
    /* Reuse the caller's output buffer for the catalog read. On success it is
     * immediately overwritten by the requested data page. This saves a second
     * 256-byte stack allocation on constrained firmware. */
    pogo_flash_file_status_t status = pogo_flash_file_internal_load_slot(
        file_id, &decoded, output);
    if (status != POGO_FLASH_FILE_OK) return status;
    /* file_page is relative to the file. Only after this bound check is it safe
     * to add it to first_page and narrow the sum to the platform uint8_t API. */
    if (file_page >= decoded.page_count) return POGO_FLASH_FILE_INVALID_ARGUMENT;
    read_page_flash((uint8_t)(decoded.first_page + file_page), (char *)output);
    if (info != NULL) *info = decoded;
    return POGO_FLASH_FILE_OK;
}
