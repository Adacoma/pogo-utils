/**
 * @file flash_file_write.c
 * @brief Optional create/replace/delete/rename support for the flash catalog.
 *
 * This file is separate so read-only mission firmware does not pull allocation
 * and verification code into its image. Each catalog page owns an independent
 * erase sector; data files own one or more contiguous erase sectors.
 *
 * Mutation ordering:
 *
 *   create:  validate/format -> allocate extent -> write -> publish entry
 *   replace: validate -> erase/write extent -> publish new CRC
 *   delete:  validate -> clear slot -> rewrite one catalog sector
 *
 * Creation never exposes unwritten data. Replacement and deletion are not
 * transactional: a reset during sector erase or rewrite can destroy the old
 * payload/catalog. Every erase and page write is read back before continuing.
 *
 * The writer uses one target catalog page, one scan page, a 368-byte sector
 * bitmap, and one verification page. No heap or file-sized buffer is required.
 */
#include "flash_file_internal.h"
#include "flash_log.h"

#include "pogobase.h"

#ifdef REAL_ROBOT
/* The SDK exposes the physical start of the v3 user-flash region. The SPI
 * primitive erases one aligned 4 KiB sector at an absolute chip offset. */
#include "spi.h"
#endif

#include <limits.h>
#include <string.h>

/** Serialized catalog discriminator; kept as bytes rather than a host integer. */
static const uint8_t catalog_magic[4] = {'P', 'F', 'F', 'S'};

enum {
    POGO_FLASH_FILE_TOTAL_SECTORS =
        POGO_FLASH_FILE_USER_PAGES / POGO_FLASH_FILE_ERASE_SECTOR_PAGES
};

_Static_assert(POGO_FLASH_FILE_PAGE_SIZE == POGOBOT_USER_FLASH_PAGE_SIZE &&
    POGO_FLASH_FILE_USER_PAGES == POGOBOT_USER_FLASH_PAGE_COUNT,
    "PFFS geometry must match the SDK user-flash API");

/** Store an unaligned 16-bit value in canonical little-endian order. */
static void put_u16(uint8_t *p, uint16_t value) {
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

/** Store an unaligned 32-bit value in canonical little-endian order. */
static void put_u32(uint8_t *p, uint32_t value) {
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
    p[2] = (uint8_t)(value >> 16);
    p[3] = (uint8_t)(value >> 24);
}

/** Continue a CRC-32/ISO-HDLC calculation without a lookup table. */
static uint32_t crc32_update(uint32_t crc, const uint8_t *bytes, size_t length) {
    for (size_t i = 0u; i < length; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0u; bit < 8u; ++bit) {
            /* Branchless conditional XOR of the reflected polynomial. */
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (UINT32_C(0xedb88320) & mask);
        }
    }
    return crc;
}

/** Apply the standard all-one final XOR to an incremental CRC state. */
static uint32_t crc32_finish(uint32_t crc) {
    return crc ^ UINT32_MAX;
}

/** Verify the checksum stored in bytes 252..255 of one catalog page. */
static bool catalog_crc_valid(const uint8_t page[POGO_FLASH_FILE_PAGE_SIZE]) {
    uint32_t crc = crc32_finish(crc32_update(
        UINT32_MAX, page, POGO_FLASH_FILE_CATALOG_CRC_OFFSET));
    return crc == pogo_flash_file_internal_get_u32(
        page + POGO_FLASH_FILE_CATALOG_CRC_OFFSET);
}

/** Recalculate and overwrite a catalog page's trailing checksum. */
static void catalog_update_crc(uint8_t page[POGO_FLASH_FILE_PAGE_SIZE]) {
    uint32_t crc = crc32_finish(crc32_update(
        UINT32_MAX, page, POGO_FLASH_FILE_CATALOG_CRC_OFFSET));
    put_u32(page + POGO_FLASH_FILE_CATALOG_CRC_OFFSET, crc);
}

static void catalog_page_init(
    uint8_t page[POGO_FLASH_FILE_PAGE_SIZE], uint8_t catalog_index) {
    /* Zero is the canonical representation for reserved fields and empty
     * entries. The catalog generation at offset 8 also starts at zero. */
    memset(page, 0, POGO_FLASH_FILE_PAGE_SIZE);
    memcpy(page, catalog_magic, sizeof(catalog_magic));
    page[4] = POGO_FLASH_FILE_CATALOG_VERSION;
    page[5] = catalog_index;
    page[6] = POGO_FLASH_FILE_ENTRIES_PER_CATALOG;
    catalog_update_crc(page);
}

static bool write_page_verified(
    uint16_t page_number,
    const uint8_t expected[POGO_FLASH_FILE_PAGE_SIZE]) {
    /* Verification needs a distinct buffer because platform write functions
     * return void and therefore cannot report a controller/programming error. */
    uint8_t actual[POGO_FLASH_FILE_PAGE_SIZE];
    write_page_flash(page_number, expected);
    read_page_flash(page_number, (char *)actual);
    return memcmp(actual, expected, POGO_FLASH_FILE_PAGE_SIZE) == 0;
}

/** Erase one dedicated 4 KiB sector and verify all sixteen physical pages.
 *
 * Pogosim has no sector-erase entry point; its page writes replace bytes in
 * simulated memory, so writing all-ones emulates the same post-erase state.
 * Real firmware uses the SPI erase command before any page programming.
 */
static bool erase_sector_verified(uint16_t first_page) {
    if (first_page % POGO_FLASH_FILE_ERASE_SECTOR_PAGES != 0u ||
        first_page > POGO_FLASH_FILE_USER_PAGES -
            POGO_FLASH_FILE_ERASE_SECTOR_PAGES) return false;
    uint8_t page[POGO_FLASH_FILE_PAGE_SIZE];
#ifdef REAL_ROBOT
    if (spiBeginErase4(POGOBOT_USER_FLASH_START_OFFSET +
            (uint32_t)first_page * POGO_FLASH_FILE_PAGE_SIZE) != 0) return false;
#else
    memset(page, 0xff, sizeof(page));
    for (unsigned offset = 0u;
         offset < POGO_FLASH_FILE_ERASE_SECTOR_PAGES; ++offset) {
        write_page_flash((uint16_t)(first_page + offset), page);
    }
#endif
    for (unsigned offset = 0u;
         offset < POGO_FLASH_FILE_ERASE_SECTOR_PAGES; ++offset) {
        read_page_flash((uint16_t)(first_page + offset), (char *)page);
        for (unsigned byte = 0u; byte < POGO_FLASH_FILE_PAGE_SIZE; ++byte) {
            if (page[byte] != 0xffu) return false;
        }
    }
    return true;
}

/** Bounded alternative to strlen; MAX_NAME+1 signals an invalid long name. */
static size_t bounded_name_length(const char *name) {
    if (name == NULL) return 0u;
    size_t length = 0u;
    while (length <= POGO_FLASH_FILE_MAX_NAME && name[length] != '\0') ++length;
    return length;
}

/** Return the mutable serialized entry selected by a stable ID.
 *
 * Callers validate the ID before reaching this helper. The arithmetic is the
 * inverse of internal_load_slot's ID-to-catalog mapping.
 */
static uint8_t *catalog_entry(
    uint8_t catalog[POGO_FLASH_FILE_PAGE_SIZE], uint8_t file_id) {
    uint8_t slot = (uint8_t)(file_id - 1u);
    uint8_t entry_index = (uint8_t)(slot % POGO_FLASH_FILE_ENTRIES_PER_CATALOG);
    return catalog + POGO_FLASH_FILE_CATALOG_HEADER_SIZE +
        (size_t)entry_index * POGO_FLASH_FILE_ENTRY_SIZE;
}

static pogo_flash_file_status_t load_catalogs(
    uint8_t target_index,
    uint8_t target_page[POGO_FLASH_FILE_PAGE_SIZE],
    bool used_sectors[POGO_FLASH_FILE_TOTAL_SECTORS]) {
    /* One byte per sector avoids a 4 KiB catalog workspace. The first sixteen
     * sectors contain one catalog page each and can never hold file data. */
    memset(used_sectors, 0,
           POGO_FLASH_FILE_TOTAL_SECTORS * sizeof(used_sectors[0]));
    for (unsigned i = 0u; i < POGO_FLASH_FILE_CATALOG_PAGES; ++i) {
        used_sectors[i] = true;
    }
    uint8_t page[POGO_FLASH_FILE_PAGE_SIZE];
    for (uint8_t catalog_index = 0u;
         catalog_index < POGO_FLASH_FILE_CATALOG_PAGES; ++catalog_index) {
        read_page_flash(pogo_flash_file_internal_catalog_page(catalog_index),
                        (char *)page);
        /* Writers require the stronger catalog CRC check because publishing a
         * mutation based on damaged metadata could overwrite a live extent. */
        if (!pogo_flash_file_internal_catalog_header_valid(
                page, catalog_index) || !catalog_crc_valid(page)) {
            if (pogo_flash_file_internal_page_is_blank(
                    page) && catalog_index == 0u) {
                /* Only a blank first catalog denotes first use. A later blank
                 * page after a valid page zero is an incomplete format. */
                return POGO_FLASH_FILE_UNFORMATTED;
            }
            return POGO_FLASH_FILE_CORRUPT_CATALOG;
        }
        if (target_page != NULL && catalog_index == target_index) {
            memcpy(target_page, page, sizeof(page));
        }
        for (uint8_t entry_index = 0u;
             entry_index < POGO_FLASH_FILE_ENTRIES_PER_CATALOG; ++entry_index) {
            uint8_t file_id = (uint8_t)(catalog_index *
                POGO_FLASH_FILE_ENTRIES_PER_CATALOG + entry_index + 1u);
            const uint8_t *entry = page +
                POGO_FLASH_FILE_CATALOG_HEADER_SIZE +
                (size_t)entry_index * POGO_FLASH_FILE_ENTRY_SIZE;
            if (entry[1] == 0u) continue;
            pogo_flash_file_info_t info;
            if (!pogo_flash_file_internal_decode_entry(entry, file_id, &info)) {
                return POGO_FLASH_FILE_CORRUPT_CATALOG;
            }
            unsigned first = info.first_page /
                POGO_FLASH_FILE_ERASE_SECTOR_PAGES;
            unsigned end = ((unsigned)info.first_page + info.page_count +
                POGO_FLASH_FILE_ERASE_SECTOR_PAGES - 1u) /
                POGO_FLASH_FILE_ERASE_SECTOR_PAGES;
            for (unsigned sector = first; sector < end; ++sector) {
                /* Allocation includes any unused tail of the final sector. */
                if (used_sectors[sector]) return POGO_FLASH_FILE_CORRUPT_CATALOG;
                used_sectors[sector] = true;
            }
        }
    }
    return POGO_FLASH_FILE_OK;
}

pogo_flash_file_status_t pogo_flash_file_check(void) {
    bool used_sectors[POGO_FLASH_FILE_TOTAL_SECTORS];
    return load_catalogs(0u, NULL, used_sectors);
}

static uint16_t find_free_extent(
    const bool used_sectors[POGO_FLASH_FILE_TOTAL_SECTORS],
    uint16_t page_count) {
    /* First fit keeps each file contiguous. Deletion may leave holes too short
     * for a later large file; the allocator never silently relocates files. */
    unsigned needed = ((unsigned)page_count +
        POGO_FLASH_FILE_ERASE_SECTOR_PAGES - 1u) /
        POGO_FLASH_FILE_ERASE_SECTOR_PAGES;
    for (unsigned start = POGO_FLASH_FILE_CATALOG_PAGES;
         start + needed <= POGO_FLASH_FILE_TOTAL_SECTORS; ++start) {
        unsigned run = 0u;
        while (run < needed && !used_sectors[start + run]) ++run;
        if (run == needed) {
            return (uint16_t)(start * POGO_FLASH_FILE_ERASE_SECTOR_PAGES);
        }
    }
    return 0u; /* Catalog page zero cannot be allocated; safe failure sentinel. */
}

/** Write, read back, compare, and checksum a complete contiguous extent.
 *
 * `data` is a flat array of page_count*256 bytes. CRC is calculated from the
 * readback rather than the input so the catalog commits exactly what was
 * observed in flash after successful byte-for-byte verification.
 */
static pogo_flash_file_status_t write_data_pages(
    uint16_t first_page,
    uint16_t page_count,
    const uint8_t *data,
    bool blank_file,
    uint32_t *data_crc) {
    /* Erase every owned sector, including unused pages in the final sector.
     * This is required before NOR programming or before publishing a blank file. */
    for (unsigned offset = 0u; offset < page_count;
         offset += POGO_FLASH_FILE_ERASE_SECTOR_PAGES) {
        if (!erase_sector_verified((uint16_t)(first_page + offset))) {
            return POGO_FLASH_FILE_VERIFY_FAILED;
        }
    }
    /* A log begins with erased pages and programs each one exactly once later.
     * Its catalog CRC field is deliberately unused, so no large all-FF input
     * buffer or pointless initial page programs are necessary. */
    if (data == NULL && !blank_file) {
        *data_crc = 0u;
        return POGO_FLASH_FILE_OK;
    }
    uint8_t actual[POGO_FLASH_FILE_PAGE_SIZE]; /* Reused for every readback. */
    uint32_t crc = UINT32_MAX;                 /* ISO-HDLC initial state. */
    if (blank_file) {
        /* The verified erase above established every byte as 0xff. Compute
         * the ordinary-file CRC without programming or buffering eight pages. */
        memset(actual, 0xff, sizeof(actual));
        for (uint16_t page = 0u; page < page_count; ++page) {
            crc = crc32_update(crc, actual, sizeof(actual));
        }
        *data_crc = crc32_finish(crc);
        return POGO_FLASH_FILE_OK;
    }
    for (uint16_t page = 0u; page < page_count; ++page) {
        const uint8_t *expected = data + (size_t)page * POGO_FLASH_FILE_PAGE_SIZE;
        uint16_t physical = (uint16_t)(first_page + page);
        write_page_flash(physical, expected);
        read_page_flash(physical, (char *)actual);
        if (memcmp(actual, expected, POGO_FLASH_FILE_PAGE_SIZE) != 0) {
            return POGO_FLASH_FILE_VERIFY_FAILED;
        }
        crc = crc32_update(crc, actual, POGO_FLASH_FILE_PAGE_SIZE);
    }
    *data_crc = crc32_finish(crc);
    return POGO_FLASH_FILE_OK;
}

/** Commit an already-modified catalog page.
 *
 * The catalog generation records every catalog mutation, independently of the
 * selected file's generation. Exhaustion fails closed instead of wrapping and
 * making an old catalog appear newer to diagnostics.
 */
static pogo_flash_file_status_t write_changed_catalog(
    uint8_t catalog_index,
    uint8_t page[POGO_FLASH_FILE_PAGE_SIZE]) {
    uint32_t generation = pogo_flash_file_internal_get_u32(page + 8);
    if (generation == UINT32_MAX) return POGO_FLASH_FILE_GENERATION_EXHAUSTED;
    put_u32(page + 8, generation + 1u);
    catalog_update_crc(page);
    /* Each catalog has a private sector. Erase/rewrite only this one page;
     * unrelated catalog pages remain valid throughout the mutation. */
    uint16_t physical = pogo_flash_file_internal_catalog_page(catalog_index);
    if (!erase_sector_verified(physical) || !write_page_verified(physical, page)) {
        return POGO_FLASH_FILE_VERIFY_FAILED;
    }
    return POGO_FLASH_FILE_OK;
}

pogo_flash_file_status_t pogo_flash_file_format(void) {
    uint8_t page[POGO_FLASH_FILE_PAGE_SIZE];
    /* Clear only metadata sectors. Data becomes unreachable immediately, and
     * each data sector is erased and verified when it is next allocated. This
     * avoids the SDK's slow erase of all 1472 KiB user-flash pages. */
    for (uint8_t catalog_index = 0u;
         catalog_index < POGO_FLASH_FILE_CATALOG_PAGES; ++catalog_index) {
        uint16_t physical = pogo_flash_file_internal_catalog_page(catalog_index);
        if (!erase_sector_verified(physical)) {
            return POGO_FLASH_FILE_VERIFY_FAILED;
        }
        catalog_page_init(page, catalog_index);
        /* Each page is independently verified; failure leaves a partially
         * formatted section that readers correctly classify as corrupt. */
        if (!write_page_verified(physical, page)) {
            return POGO_FLASH_FILE_VERIFY_FAILED;
        }
    }
    return POGO_FLASH_FILE_OK;
}

static pogo_flash_file_status_t create_file(
    uint8_t file_id,
    const char *name,
    uint16_t page_count,
    uint16_t format_version,
    const uint8_t *data,
    bool log_format,
    bool blank_file,
    bool *formatted) {
    if (formatted != NULL) *formatted = false;
    /* Measure before reading catalogs so invalid caller input has no I/O side
     * effects and an unterminated string is never scanned without a bound. */
    size_t name_length = bounded_name_length(name);
    if (file_id == 0u || file_id > POGO_FLASH_FILE_MAX_FILES ||
        (!log_format && ((!blank_file && data == NULL) ||
            format_version == POGO_FLASH_LOG_FORMAT_VERSION)) ||
        (blank_file && (log_format || data != NULL)) ||
        name_length > POGO_FLASH_FILE_MAX_NAME) {
        return POGO_FLASH_FILE_INVALID_ARGUMENT;
    }
    if (page_count == 0u || page_count > POGO_FLASH_FILE_MAX_PAGES ||
        (log_format && page_count > POGO_FLASH_LOG_MAX_PAGES)) {
        return POGO_FLASH_FILE_INVALID_SIZE;
    }
    uint8_t catalog_index = (uint8_t)((file_id - 1u) /
        POGO_FLASH_FILE_ENTRIES_PER_CATALOG);
    uint8_t catalog[POGO_FLASH_FILE_PAGE_SIZE];
    bool used_sectors[POGO_FLASH_FILE_TOTAL_SECTORS];
    pogo_flash_file_status_t status = load_catalogs(
        catalog_index, catalog, used_sectors);
    if (status == POGO_FLASH_FILE_UNFORMATTED ||
        status == POGO_FLASH_FILE_CORRUPT_CATALOG) {
        /* Creation opts into destructive recovery: there is no trustworthy
         * allocation map, so clear all catalog sectors before creating.
         * Old payloads remain physically present but become unreachable. */
        status = pogo_flash_file_format();
        if (status != POGO_FLASH_FILE_OK) return status;
        if (formatted != NULL) *formatted = true;
        status = load_catalogs(catalog_index, catalog, used_sectors);
    }
    if (status != POGO_FLASH_FILE_OK) return status;
    uint8_t *entry = catalog_entry(catalog, file_id);
    /* Slot occupancy is authoritative. A different requested name cannot
     * replace an existing ID through create(). */
    if (entry[1] != 0u) return POGO_FLASH_FILE_ALREADY_EXISTS;
    if (name_length > 0u) {
        pogo_flash_file_info_t named;
        status = pogo_flash_file_find_by_name(name, &named);
        if (status == POGO_FLASH_FILE_OK) return POGO_FLASH_FILE_NAME_EXISTS;
        if (status != POGO_FLASH_FILE_NOT_FOUND) return status;
    }
    if (pogo_flash_file_internal_get_u32(catalog + 8) == UINT32_MAX) {
        return POGO_FLASH_FILE_GENERATION_EXHAUSTED;
    }
    uint16_t first_page = find_free_extent(used_sectors, page_count);
    if (first_page == 0u) return POGO_FLASH_FILE_NO_SPACE;
    uint32_t data_crc;
    /* Write data before filling/publishing the entry. If this fails, the pages
     * contain garbage/orphan data but no catalog file points at them. */
    status = write_data_pages(first_page, page_count, data, blank_file, &data_crc);
    if (status != POGO_FLASH_FILE_OK) return status;

    /* Construct the complete serialized entry in RAM. Reserved bytes and unused
     * name tail bytes remain zero because the slot is cleared first. */
    memset(entry, 0, POGO_FLASH_FILE_ENTRY_SIZE);
    entry[0] = file_id;
    entry[1] = (uint8_t)(POGO_FLASH_FILE_ENTRY_IN_USE | (name_length << 1));
    put_u16(entry + 2, format_version);
    put_u16(entry + 4, first_page);
    put_u16(entry + 6, page_count);
    put_u32(entry + 8, 1u);
    put_u32(entry + 12, data_crc);
    if (name_length > 0u) memcpy(entry + 16, name, name_length);
    return write_changed_catalog(catalog_index, catalog);
}

pogo_flash_file_status_t pogo_flash_file_create(
    uint8_t file_id, const char *name, uint16_t page_count,
    uint16_t format_version, const uint8_t *data) {
    return create_file(file_id, name, page_count, format_version, data,
                       false, false, NULL);
}

pogo_flash_file_status_t pogo_flash_file_create_blank(
    uint8_t file_id, const char *name, uint16_t page_count,
    uint16_t format_version) {
    return create_file(file_id, name, page_count, format_version, NULL,
                       false, true, NULL);
}

pogo_flash_file_status_t pogo_flash_file_internal_create_log(
    uint8_t file_id, const char *name, uint8_t page_count, bool *formatted) {
    return create_file(file_id, name, page_count,
                       POGO_FLASH_LOG_FORMAT_VERSION, NULL, true, false,
                       formatted);
}

pogo_flash_file_status_t pogo_flash_file_internal_clear_log(uint8_t file_id) {
    if (file_id == 0u || file_id > POGO_FLASH_FILE_MAX_FILES)
        return POGO_FLASH_FILE_INVALID_ARGUMENT;
    uint8_t catalog_index = (uint8_t)((file_id - 1u) /
        POGO_FLASH_FILE_ENTRIES_PER_CATALOG);
    uint8_t catalog[POGO_FLASH_FILE_PAGE_SIZE];
    bool used_sectors[POGO_FLASH_FILE_TOTAL_SECTORS];
    pogo_flash_file_status_t status = load_catalogs(
        catalog_index, catalog, used_sectors);
    if (status != POGO_FLASH_FILE_OK) return status;
    const uint8_t *entry = catalog_entry(catalog, file_id);
    if (entry[1] == 0u) return POGO_FLASH_FILE_NOT_FOUND;
    pogo_flash_file_info_t info;
    if (!pogo_flash_file_internal_decode_entry(entry, file_id, &info)) {
        return POGO_FLASH_FILE_CORRUPT_CATALOG;
    }
    if (info.format_version != POGO_FLASH_LOG_FORMAT_VERSION) {
        return POGO_FLASH_FILE_UNSUPPORTED_FORMAT;
    }
    /* Logs are limited to eight pages, so one dedicated sector is sufficient.
     * Its catalog extent/generation stay fixed; append needs no rewrite. */
    return erase_sector_verified(info.first_page) ? POGO_FLASH_FILE_OK :
        POGO_FLASH_FILE_VERIFY_FAILED;
}

pogo_flash_file_status_t pogo_flash_file_replace(
    uint8_t file_id,
    uint16_t page_count,
    const uint8_t *data) {
    if (file_id == 0u || file_id > POGO_FLASH_FILE_MAX_FILES || data == NULL) {
        return POGO_FLASH_FILE_INVALID_ARGUMENT;
    }
    uint8_t catalog_index = (uint8_t)((file_id - 1u) /
        POGO_FLASH_FILE_ENTRIES_PER_CATALOG);
    uint8_t catalog[POGO_FLASH_FILE_PAGE_SIZE];
    bool used_sectors[POGO_FLASH_FILE_TOTAL_SECTORS];
    pogo_flash_file_info_t info;
    pogo_flash_file_status_t status = load_catalogs(
        catalog_index, catalog, used_sectors);
    if (status != POGO_FLASH_FILE_OK) return status;
    uint8_t *entry = catalog_entry(catalog, file_id);
    if (entry[1] == 0u) return POGO_FLASH_FILE_NOT_FOUND;
    if (!pogo_flash_file_internal_decode_entry(entry, file_id, &info)) {
        return POGO_FLASH_FILE_CORRUPT_CATALOG;
    }
    if (info.format_version == POGO_FLASH_LOG_FORMAT_VERSION) {
        return POGO_FLASH_FILE_UNSUPPORTED_FORMAT;
    }
    /* File size is an allocation invariant. Resizing would require finding a
     * new extent and introducing a relocation/transaction policy. */
    if (page_count != info.page_count) return POGO_FLASH_FILE_INVALID_SIZE;
    if (info.generation == UINT32_MAX ||
        pogo_flash_file_internal_get_u32(catalog + 8) == UINT32_MAX) {
        return POGO_FLASH_FILE_GENERATION_EXHAUSTED;
    }
    uint32_t data_crc;
    /* Non-transactional point: erasing this dedicated sector removes the old
     * payload before a new payload or catalog checksum can be committed. */
    status = write_data_pages(info.first_page, info.page_count, data, false,
                              &data_crc);
    if (status != POGO_FLASH_FILE_OK) return status;
    /* File generation tracks successful replacements. write_changed_catalog()
     * separately increments the generation for the containing catalog page. */
    put_u32(entry + 8, info.generation + 1u);
    put_u32(entry + 12, data_crc);
    return write_changed_catalog(catalog_index, catalog);
}

pogo_flash_file_status_t pogo_flash_file_delete(uint8_t file_id) {
    if (file_id == 0u || file_id > POGO_FLASH_FILE_MAX_FILES) {
        return POGO_FLASH_FILE_INVALID_ARGUMENT;
    }
    uint8_t catalog_index = (uint8_t)((file_id - 1u) /
        POGO_FLASH_FILE_ENTRIES_PER_CATALOG);
    uint8_t catalog[POGO_FLASH_FILE_PAGE_SIZE];
    bool used_sectors[POGO_FLASH_FILE_TOTAL_SECTORS];
    pogo_flash_file_status_t status = load_catalogs(
        catalog_index, catalog, used_sectors);
    if (status != POGO_FLASH_FILE_OK) return status;
    uint8_t *entry = catalog_entry(catalog, file_id);
    if (entry[1] == 0u) return POGO_FLASH_FILE_NOT_FOUND;
    /* Clearing the entry releases the extent logically. Payload bytes remain
     * physically present until a later creation reuses and overwrites them. */
    memset(entry, 0, POGO_FLASH_FILE_ENTRY_SIZE);
    return write_changed_catalog(catalog_index, catalog);
}

pogo_flash_file_status_t pogo_flash_file_rename(
    uint8_t file_id, const char *new_name) {
    size_t name_length = bounded_name_length(new_name);
    if (file_id == 0u || file_id > POGO_FLASH_FILE_MAX_FILES ||
        name_length > POGO_FLASH_FILE_MAX_NAME) {
        return POGO_FLASH_FILE_INVALID_ARGUMENT;
    }
    uint8_t catalog_index = (uint8_t)((file_id - 1u) /
        POGO_FLASH_FILE_ENTRIES_PER_CATALOG);
    uint8_t catalog[POGO_FLASH_FILE_PAGE_SIZE];
    bool used_sectors[POGO_FLASH_FILE_TOTAL_SECTORS];
    pogo_flash_file_status_t status = load_catalogs(
        catalog_index, catalog, used_sectors);
    if (status != POGO_FLASH_FILE_OK) return status;
    uint8_t *entry = catalog_entry(catalog, file_id);
    if (entry[1] == 0u) return POGO_FLASH_FILE_NOT_FOUND;
    if ((entry[1] >> 1) == name_length &&
        (name_length == 0u || memcmp(entry + 16, new_name, name_length) == 0)) {
        return POGO_FLASH_FILE_OK; /* A no-op need not wear the catalog sector. */
    }
    if (name_length > 0u) {
        pogo_flash_file_info_t named;
        status = pogo_flash_file_find_by_name(new_name, &named);
        if (status == POGO_FLASH_FILE_OK) return POGO_FLASH_FILE_NAME_EXISTS;
        if (status != POGO_FLASH_FILE_NOT_FOUND) return status;
    }
    /* The fixed ID, extent, payload checksum, and file generation remain
     * unchanged. Only the catalog generation records the metadata mutation. */
    memset(entry + 16, 0, POGO_FLASH_FILE_MAX_NAME);
    entry[1] = (uint8_t)(POGO_FLASH_FILE_ENTRY_IN_USE | (name_length << 1));
    if (name_length > 0u) memcpy(entry + 16, new_name, name_length);
    return write_changed_catalog(catalog_index, catalog);
}

pogo_flash_file_status_t pogo_flash_file_write_begin_create(
    pogo_flash_file_writer_t *writer, uint8_t file_id, const char *name,
    uint16_t page_count, uint16_t format_version, bool *formatted) {
    if (formatted != NULL) *formatted = false;
    if (writer == NULL) return POGO_FLASH_FILE_INVALID_ARGUMENT;
    memset(writer, 0, sizeof(*writer));
    size_t name_length = bounded_name_length(name);
    if (file_id == 0u || file_id > POGO_FLASH_FILE_MAX_FILES ||
        name_length > POGO_FLASH_FILE_MAX_NAME ||
        format_version == POGO_FLASH_LOG_FORMAT_VERSION) {
        return POGO_FLASH_FILE_INVALID_ARGUMENT;
    }
    if (page_count == 0u || page_count > POGO_FLASH_FILE_MAX_PAGES) {
        return POGO_FLASH_FILE_INVALID_SIZE;
    }
    uint8_t catalog_index = (uint8_t)((file_id - 1u) /
        POGO_FLASH_FILE_ENTRIES_PER_CATALOG);
    uint8_t catalog[POGO_FLASH_FILE_PAGE_SIZE];
    bool used_sectors[POGO_FLASH_FILE_TOTAL_SECTORS];
    pogo_flash_file_status_t status = load_catalogs(
        catalog_index, catalog, used_sectors);
    if (status == POGO_FLASH_FILE_UNFORMATTED ||
        status == POGO_FLASH_FILE_CORRUPT_CATALOG) {
        status = pogo_flash_file_format();
        if (status != POGO_FLASH_FILE_OK) return status;
        if (formatted != NULL) *formatted = true;
        status = load_catalogs(catalog_index, catalog, used_sectors);
    }
    if (status != POGO_FLASH_FILE_OK) return status;
    if (catalog_entry(catalog, file_id)[1] != 0u) {
        return POGO_FLASH_FILE_ALREADY_EXISTS;
    }
    if (name_length > 0u) {
        pogo_flash_file_info_t named;
        status = pogo_flash_file_find_by_name(name, &named);
        if (status == POGO_FLASH_FILE_OK) return POGO_FLASH_FILE_NAME_EXISTS;
        if (status != POGO_FLASH_FILE_NOT_FOUND) return status;
    }
    uint32_t catalog_generation = pogo_flash_file_internal_get_u32(catalog + 8);
    if (catalog_generation == UINT32_MAX) {
        return POGO_FLASH_FILE_GENERATION_EXHAUSTED;
    }
    uint16_t first_page = find_free_extent(used_sectors, page_count);
    if (first_page == 0u) return POGO_FLASH_FILE_NO_SPACE;
    writer->first_page = first_page;
    writer->page_count = page_count;
    writer->format_version = format_version;
    writer->file_id = file_id;
    writer->name_length = (uint8_t)name_length;
    writer->catalog_generation = catalog_generation;
    writer->crc = UINT32_MAX;
    writer->mode = 1u; /* Unpublished creation. */
    if (name_length > 0u) memcpy(writer->name, name, name_length);
    return POGO_FLASH_FILE_OK;
}

pogo_flash_file_status_t pogo_flash_file_write_begin_replace(
    pogo_flash_file_writer_t *writer, uint8_t file_id) {
    if (writer == NULL) return POGO_FLASH_FILE_INVALID_ARGUMENT;
    memset(writer, 0, sizeof(*writer));
    if (file_id == 0u || file_id > POGO_FLASH_FILE_MAX_FILES) {
        return POGO_FLASH_FILE_INVALID_ARGUMENT;
    }
    uint8_t catalog_index = (uint8_t)((file_id - 1u) /
        POGO_FLASH_FILE_ENTRIES_PER_CATALOG);
    uint8_t catalog[POGO_FLASH_FILE_PAGE_SIZE];
    bool used_sectors[POGO_FLASH_FILE_TOTAL_SECTORS];
    pogo_flash_file_status_t status = load_catalogs(
        catalog_index, catalog, used_sectors);
    if (status != POGO_FLASH_FILE_OK) return status;
    const uint8_t *entry = catalog_entry(catalog, file_id);
    if (entry[1] == 0u) return POGO_FLASH_FILE_NOT_FOUND;
    pogo_flash_file_info_t info;
    if (!pogo_flash_file_internal_decode_entry(entry, file_id, &info)) {
        return POGO_FLASH_FILE_CORRUPT_CATALOG;
    }
    if (info.format_version == POGO_FLASH_LOG_FORMAT_VERSION) {
        return POGO_FLASH_FILE_UNSUPPORTED_FORMAT;
    }
    uint32_t catalog_generation = pogo_flash_file_internal_get_u32(catalog + 8);
    if (catalog_generation == UINT32_MAX || info.generation == UINT32_MAX) {
        return POGO_FLASH_FILE_GENERATION_EXHAUSTED;
    }
    writer->first_page = info.first_page;
    writer->page_count = info.page_count;
    writer->format_version = info.format_version;
    writer->file_id = file_id;
    writer->file_generation = info.generation;
    writer->catalog_generation = catalog_generation;
    writer->crc = UINT32_MAX;
    writer->mode = 2u; /* Fixed-size replacement. */
    return POGO_FLASH_FILE_OK;
}

pogo_flash_file_status_t pogo_flash_file_write_page(
    pogo_flash_file_writer_t *writer,
    const uint8_t data[POGO_FLASH_FILE_PAGE_SIZE]) {
    if (writer == NULL || data == NULL ||
        (writer->mode != 1u && writer->mode != 2u) ||
        writer->file_id == 0u || writer->file_id > POGO_FLASH_FILE_MAX_FILES ||
        writer->page_count == 0u ||
        writer->first_page < POGO_FLASH_FILE_DATA_FIRST_PAGE ||
        writer->first_page % POGO_FLASH_FILE_ERASE_SECTOR_PAGES != 0u ||
        (unsigned)writer->first_page + writer->page_count >
            POGO_FLASH_FILE_USER_PAGES ||
        writer->next_page >= writer->page_count) {
        return POGO_FLASH_FILE_INVALID_ARGUMENT;
    }
    uint16_t physical = (uint16_t)(writer->first_page + writer->next_page);
    /* The first page of each owned sector erases that sector lazily. Thus an
     * application can write a large file over many loop iterations. */
    if (writer->next_page % POGO_FLASH_FILE_ERASE_SECTOR_PAGES == 0u &&
        !erase_sector_verified(physical)) {
        writer->mode = 0u;
        return POGO_FLASH_FILE_VERIFY_FAILED;
    }
    if (!write_page_verified(physical, data)) {
        writer->mode = 0u; /* Never retry programming a possibly torn NOR page. */
        return POGO_FLASH_FILE_VERIFY_FAILED;
    }
    writer->crc = crc32_update(writer->crc, data, POGO_FLASH_FILE_PAGE_SIZE);
    ++writer->next_page;
    return POGO_FLASH_FILE_OK;
}

pogo_flash_file_status_t pogo_flash_file_write_finish(
    pogo_flash_file_writer_t *writer) {
    if (writer == NULL || (writer->mode != 1u && writer->mode != 2u) ||
        writer->file_id == 0u || writer->file_id > POGO_FLASH_FILE_MAX_FILES ||
        writer->page_count == 0u ||
        writer->first_page < POGO_FLASH_FILE_DATA_FIRST_PAGE ||
        writer->first_page % POGO_FLASH_FILE_ERASE_SECTOR_PAGES != 0u ||
        (unsigned)writer->first_page + writer->page_count >
            POGO_FLASH_FILE_USER_PAGES ||
        writer->next_page != writer->page_count) {
        return POGO_FLASH_FILE_INVALID_ARGUMENT;
    }
    uint8_t catalog_index = (uint8_t)((writer->file_id - 1u) /
        POGO_FLASH_FILE_ENTRIES_PER_CATALOG);
    uint8_t catalog[POGO_FLASH_FILE_PAGE_SIZE];
    bool used_sectors[POGO_FLASH_FILE_TOTAL_SECTORS];
    pogo_flash_file_status_t status = load_catalogs(
        catalog_index, catalog, used_sectors);
    if (status != POGO_FLASH_FILE_OK) {
        writer->mode = 0u;
        return status;
    }
    if (pogo_flash_file_internal_get_u32(catalog + 8) !=
        writer->catalog_generation) {
        writer->mode = 0u;
        return POGO_FLASH_FILE_CORRUPT_CATALOG;
    }
    uint8_t *entry = catalog_entry(catalog, writer->file_id);
    if (writer->mode == 1u) {
        if (entry[1] != 0u) {
            writer->mode = 0u;
            return POGO_FLASH_FILE_ALREADY_EXISTS;
        }
        unsigned first_sector = writer->first_page /
            POGO_FLASH_FILE_ERASE_SECTOR_PAGES;
        unsigned end_sector = ((unsigned)writer->first_page +
            writer->page_count + POGO_FLASH_FILE_ERASE_SECTOR_PAGES - 1u) /
            POGO_FLASH_FILE_ERASE_SECTOR_PAGES;
        for (unsigned sector = first_sector; sector < end_sector; ++sector) {
            if (used_sectors[sector]) {
                writer->mode = 0u;
                return POGO_FLASH_FILE_NO_SPACE;
            }
        }
        /* Recheck names because a separate catalog could have changed while
         * pages were streamed. Parallel flash writers remain unsupported. */
        if (writer->name_length > 0u) {
            pogo_flash_file_info_t named;
            status = pogo_flash_file_find_by_name(writer->name, &named);
            if (status == POGO_FLASH_FILE_OK) status = POGO_FLASH_FILE_NAME_EXISTS;
            else if (status == POGO_FLASH_FILE_NOT_FOUND) status = POGO_FLASH_FILE_OK;
            if (status != POGO_FLASH_FILE_OK) {
                writer->mode = 0u;
                return status;
            }
        }
        memset(entry, 0, POGO_FLASH_FILE_ENTRY_SIZE);
        entry[0] = writer->file_id;
        entry[1] = (uint8_t)(POGO_FLASH_FILE_ENTRY_IN_USE |
            (writer->name_length << 1));
        put_u16(entry + 2, writer->format_version);
        put_u16(entry + 4, writer->first_page);
        put_u16(entry + 6, writer->page_count);
        put_u32(entry + 8, 1u);
        if (writer->name_length > 0u) {
            memcpy(entry + 16, writer->name, writer->name_length);
        }
    } else {
        pogo_flash_file_info_t info;
        if (entry[1] == 0u || !pogo_flash_file_internal_decode_entry(
                entry, writer->file_id, &info) ||
            info.first_page != writer->first_page ||
            info.page_count != writer->page_count ||
            info.generation != writer->file_generation) {
            writer->mode = 0u;
            return POGO_FLASH_FILE_CORRUPT_CATALOG;
        }
        put_u32(entry + 8, info.generation + 1u);
    }
    put_u32(entry + 12, crc32_finish(writer->crc));
    writer->mode = 0u;
    return write_changed_catalog(catalog_index, catalog);
}

void pogo_flash_file_write_abort(pogo_flash_file_writer_t *writer) {
    if (writer != NULL) writer->mode = 0u;
}
