/**
 * @file flash_file_write.c
 * @brief Optional create/replace/delete support for the bounded flash catalog.
 *
 * This file is separate so read-only mission firmware does not pull allocation
 * and verification code into its image. Replacement deliberately writes an
 * existing extent in place and assumes the platform supports that operation.
 *
 * Mutation ordering:
 *
 *   create:  validate catalogs -> allocate -> write/verify data -> publish slot
 *   replace: validate slot/CRC -> write/verify data -> update slot/catalog CRC
 *   delete:  validate slot/CRC -> clear slot -> update catalog CRC
 *
 * Creation never exposes unwritten data. Replacement and deletion are not
 * transactional because the hardware API erases only the whole 64 KiB section;
 * there is no per-page erase or journal area. Every page write is read back
 * immediately so a detectable failure is returned before continuing.
 *
 * The writer intentionally uses bounded stack workspaces: two catalog pages, a
 * 256-byte page-usage bitmap, and one verification page. No heap is required.
 */
#include "flash_file_internal.h"

#include "pogobase.h"

#include <limits.h>
#include <string.h>

/** Serialized catalog discriminator; kept as bytes rather than a host integer. */
static const uint8_t catalog_magic[4] = {'P', 'F', 'F', 'S'};

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
    uint8_t page_number,
    const uint8_t expected[POGO_FLASH_FILE_PAGE_SIZE]) {
    /* Verification needs a distinct buffer because platform write functions
     * return void and therefore cannot report a controller/programming error. */
    uint8_t actual[POGO_FLASH_FILE_PAGE_SIZE];
    write_page_flash(page_number, expected);
    read_page_flash(page_number, (char *)actual);
    return memcmp(actual, expected, POGO_FLASH_FILE_PAGE_SIZE) == 0;
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
    uint8_t catalogs[POGO_FLASH_FILE_CATALOG_PAGES][POGO_FLASH_FILE_PAGE_SIZE],
    uint8_t file_id) {
    uint8_t slot = (uint8_t)(file_id - 1u);
    uint8_t catalog_index = (uint8_t)(slot / POGO_FLASH_FILE_ENTRIES_PER_CATALOG);
    uint8_t entry_index = (uint8_t)(slot % POGO_FLASH_FILE_ENTRIES_PER_CATALOG);
    return catalogs[catalog_index] + POGO_FLASH_FILE_CATALOG_HEADER_SIZE +
        (size_t)entry_index * POGO_FLASH_FILE_ENTRY_SIZE;
}

static pogo_flash_file_status_t load_catalogs(
    uint8_t catalogs[POGO_FLASH_FILE_CATALOG_PAGES][POGO_FLASH_FILE_PAGE_SIZE],
    bool used_pages[256]) {
    /* One byte per physical page costs 256 bytes but keeps overlap detection
     * straightforward and avoids bit-shift edge cases on small targets. */
    memset(used_pages, 0, 256u * sizeof(used_pages[0]));
    /* Catalog pages are permanently reserved and can never belong to a file. */
    used_pages[0] = true;
    used_pages[1] = true;
    for (uint8_t catalog_index = 0u;
         catalog_index < POGO_FLASH_FILE_CATALOG_PAGES; ++catalog_index) {
        read_page_flash(catalog_index, (char *)catalogs[catalog_index]);
        /* Writers require the stronger catalog CRC check because publishing a
         * mutation based on damaged metadata could overwrite a live extent. */
        if (!pogo_flash_file_internal_catalog_header_valid(
                catalogs[catalog_index], catalog_index) ||
            !catalog_crc_valid(catalogs[catalog_index])) {
            if (pogo_flash_file_internal_page_is_blank(
                    catalogs[catalog_index]) && catalog_index == 0u) {
                /* Only a blank first catalog denotes first use. Missing page 1
                 * after a valid page 0 is an incomplete/corrupt format. */
                return POGO_FLASH_FILE_UNFORMATTED;
            }
            return POGO_FLASH_FILE_CORRUPT_CATALOG;
        }
        for (uint8_t entry_index = 0u;
             entry_index < POGO_FLASH_FILE_ENTRIES_PER_CATALOG; ++entry_index) {
            uint8_t file_id = (uint8_t)(catalog_index *
                POGO_FLASH_FILE_ENTRIES_PER_CATALOG + entry_index + 1u);
            const uint8_t *entry = catalogs[catalog_index] +
                POGO_FLASH_FILE_CATALOG_HEADER_SIZE +
                (size_t)entry_index * POGO_FLASH_FILE_ENTRY_SIZE;
            if (entry[7] == 0u) continue;
            pogo_flash_file_info_t info;
            if (!pogo_flash_file_internal_decode_entry(entry, file_id, &info)) {
                return POGO_FLASH_FILE_CORRUPT_CATALOG;
            }
            for (uint8_t page = 0u; page < info.page_count; ++page) {
                uint8_t physical = (uint8_t)(info.first_page + page);
                /* Seeing a page twice means two files claim the same storage;
                 * no writer operation is safe until the catalog is repaired. */
                if (used_pages[physical]) return POGO_FLASH_FILE_CORRUPT_CATALOG;
                used_pages[physical] = true;
            }
        }
    }
    return POGO_FLASH_FILE_OK;
}

static bool catalog_name_exists(
    uint8_t catalogs[POGO_FLASH_FILE_CATALOG_PAGES][POGO_FLASH_FILE_PAGE_SIZE],
    const char *name,
    size_t name_length) {
    /* Empty names are intentionally reusable; they mean "no human label". */
    if (name_length == 0u) return false;
    for (uint8_t file_id = 1u; file_id <= POGO_FLASH_FILE_MAX_FILES; ++file_id) {
        const uint8_t *entry = catalog_entry(catalogs, file_id);
        if (entry[7] == POGO_FLASH_FILE_ENTRY_IN_USE &&
            entry[6] == name_length && memcmp(entry + 16, name, name_length) == 0) {
            return true;
        }
    }
    return false;
}

static uint8_t find_contiguous_pages(
    const bool used_pages[256], uint8_t page_count) {
    /* First fit makes allocation deterministic and naturally reuses the lowest
     * deletion hole. Fragmentation is accepted; no relocation is attempted. */
    unsigned last_start = 256u - (unsigned)page_count;
    for (unsigned start = POGO_FLASH_FILE_DATA_FIRST_PAGE;
         start <= last_start; ++start) {
        bool free = true;
        for (unsigned offset = 0u; offset < page_count; ++offset) {
            free = free && !used_pages[start + offset];
        }
        if (free) return (uint8_t)start;
    }
    return 0u; /* Physical page zero is reserved, so it is a safe sentinel. */
}

/** Write, read back, compare, and checksum a complete contiguous extent.
 *
 * `data` is a flat array of page_count*256 bytes. CRC is calculated from the
 * readback rather than the input so the catalog commits exactly what was
 * observed in flash after successful byte-for-byte verification.
 */
static pogo_flash_file_status_t write_data_pages(
    uint8_t first_page,
    uint8_t page_count,
    const uint8_t *data,
    uint32_t *data_crc) {
    uint8_t actual[POGO_FLASH_FILE_PAGE_SIZE]; /* Reused for every readback. */
    uint32_t crc = UINT32_MAX;                 /* ISO-HDLC initial state. */
    for (uint8_t page = 0u; page < page_count; ++page) {
        const uint8_t *expected = data + (size_t)page * POGO_FLASH_FILE_PAGE_SIZE;
        uint8_t physical = (uint8_t)(first_page + page);
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
    return write_page_verified(catalog_index, page) ? POGO_FLASH_FILE_OK :
        POGO_FLASH_FILE_VERIFY_FAILED;
}

pogo_flash_file_status_t pogo_flash_file_format(void) {
    uint8_t page[POGO_FLASH_FILE_PAGE_SIZE];
    /* The platform exposes only a whole-section erase. Once this call returns,
     * previous files are irrecoverable even if catalog initialization fails. */
    erase_write_section_flash();
    for (uint8_t catalog_index = 0u;
         catalog_index < POGO_FLASH_FILE_CATALOG_PAGES; ++catalog_index) {
        catalog_page_init(page, catalog_index);
        /* Each page is independently verified; failure leaves a partially
         * formatted section that readers correctly classify as corrupt. */
        if (!write_page_verified(catalog_index, page)) {
            return POGO_FLASH_FILE_VERIFY_FAILED;
        }
    }
    return POGO_FLASH_FILE_OK;
}

pogo_flash_file_status_t pogo_flash_file_create(
    uint8_t file_id,
    const char *name,
    uint8_t page_count,
    uint16_t format_version,
    const uint8_t *data) {
    /* Measure before reading catalogs so invalid caller input has no I/O side
     * effects and an unterminated string is never scanned without a bound. */
    size_t name_length = bounded_name_length(name);
    if (file_id == 0u || file_id > POGO_FLASH_FILE_MAX_FILES || data == NULL ||
        name_length > POGO_FLASH_FILE_MAX_NAME) {
        return POGO_FLASH_FILE_INVALID_ARGUMENT;
    }
    if (page_count == 0u || page_count > POGO_FLASH_FILE_MAX_PAGES) {
        return POGO_FLASH_FILE_INVALID_SIZE;
    }
    uint8_t catalogs[POGO_FLASH_FILE_CATALOG_PAGES][POGO_FLASH_FILE_PAGE_SIZE];
    bool used_pages[256];
    pogo_flash_file_status_t status = load_catalogs(catalogs, used_pages);
    if (status != POGO_FLASH_FILE_OK) return status;
    uint8_t *entry = catalog_entry(catalogs, file_id);
    /* Slot occupancy is authoritative. A different requested name cannot
     * replace an existing ID through create(). */
    if (entry[7] != 0u) return POGO_FLASH_FILE_ALREADY_EXISTS;
    if (catalog_name_exists(catalogs, name, name_length)) {
        return POGO_FLASH_FILE_NAME_EXISTS;
    }
    uint8_t slot = (uint8_t)(file_id - 1u);
    uint8_t catalog_index = (uint8_t)(slot / POGO_FLASH_FILE_ENTRIES_PER_CATALOG);
    if (pogo_flash_file_internal_get_u32(catalogs[catalog_index] + 8) == UINT32_MAX) {
        return POGO_FLASH_FILE_GENERATION_EXHAUSTED;
    }
    uint8_t first_page = find_contiguous_pages(used_pages, page_count);
    if (first_page == 0u) return POGO_FLASH_FILE_NO_SPACE;
    uint32_t data_crc;
    /* Write data before filling/publishing the entry. If this fails, the pages
     * contain garbage/orphan data but no catalog file points at them. */
    status = write_data_pages(first_page, page_count, data, &data_crc);
    if (status != POGO_FLASH_FILE_OK) return status;

    /* Construct the complete serialized entry in RAM. Reserved bytes and unused
     * name tail bytes remain zero because the slot is cleared first. */
    memset(entry, 0, POGO_FLASH_FILE_ENTRY_SIZE);
    entry[0] = file_id;
    put_u16(entry + 2, format_version);
    entry[4] = first_page;
    entry[5] = page_count;
    entry[6] = (uint8_t)name_length;
    entry[7] = POGO_FLASH_FILE_ENTRY_IN_USE;
    put_u32(entry + 8, 1u);
    put_u32(entry + 12, data_crc);
    if (name_length > 0u) memcpy(entry + 16, name, name_length);
    return write_changed_catalog(catalog_index, catalogs[catalog_index]);
}

pogo_flash_file_status_t pogo_flash_file_replace(
    uint8_t file_id,
    uint8_t page_count,
    const uint8_t *data) {
    if (data == NULL) return POGO_FLASH_FILE_INVALID_ARGUMENT;
    uint8_t catalog[POGO_FLASH_FILE_PAGE_SIZE];
    pogo_flash_file_info_t info;
    pogo_flash_file_status_t status = pogo_flash_file_internal_load_slot(
        file_id, &info, catalog);
    if (status != POGO_FLASH_FILE_OK) return status;
    /* The fast slot loader intentionally skips CRC; mutation must add the check
     * before trusting the physical extent or rewriting catalog metadata. */
    if (!catalog_crc_valid(catalog)) return POGO_FLASH_FILE_CORRUPT_CATALOG;
    /* File size is an allocation invariant. Resizing would require finding a
     * new extent and introducing a relocation/transaction policy. */
    if (page_count != info.page_count) return POGO_FLASH_FILE_INVALID_SIZE;
    if (info.generation == UINT32_MAX ||
        pogo_flash_file_internal_get_u32(catalog + 8) == UINT32_MAX) {
        return POGO_FLASH_FILE_GENERATION_EXHAUSTED;
    }
    uint32_t data_crc;
    /* Non-transactional point: after this begins, the old payload may already
     * be gone even if a later page or catalog verification fails. */
    status = write_data_pages(info.first_page, info.page_count, data, &data_crc);
    if (status != POGO_FLASH_FILE_OK) return status;
    uint8_t slot = (uint8_t)(file_id - 1u);
    uint8_t catalog_index = (uint8_t)(slot / POGO_FLASH_FILE_ENTRIES_PER_CATALOG);
    uint8_t entry_index = (uint8_t)(slot % POGO_FLASH_FILE_ENTRIES_PER_CATALOG);
    uint8_t *entry = catalog + POGO_FLASH_FILE_CATALOG_HEADER_SIZE +
        (size_t)entry_index * POGO_FLASH_FILE_ENTRY_SIZE;
    /* File generation tracks successful replacements. write_changed_catalog()
     * separately increments the generation for the containing catalog page. */
    put_u32(entry + 8, info.generation + 1u);
    put_u32(entry + 12, data_crc);
    return write_changed_catalog(catalog_index, catalog);
}

pogo_flash_file_status_t pogo_flash_file_delete(uint8_t file_id) {
    uint8_t catalog[POGO_FLASH_FILE_PAGE_SIZE];
    pogo_flash_file_status_t status = pogo_flash_file_internal_load_slot(
        file_id, NULL, catalog);
    if (status != POGO_FLASH_FILE_OK) return status;
    /* As with replacement, never mutate a catalog whose checksum is suspect. */
    if (!catalog_crc_valid(catalog)) return POGO_FLASH_FILE_CORRUPT_CATALOG;
    uint8_t slot = (uint8_t)(file_id - 1u);
    uint8_t catalog_index = (uint8_t)(slot / POGO_FLASH_FILE_ENTRIES_PER_CATALOG);
    uint8_t entry_index = (uint8_t)(slot % POGO_FLASH_FILE_ENTRIES_PER_CATALOG);
    uint8_t *entry = catalog + POGO_FLASH_FILE_CATALOG_HEADER_SIZE +
        (size_t)entry_index * POGO_FLASH_FILE_ENTRY_SIZE;
    /* Clearing the entry releases the extent logically. Payload bytes remain
     * physically present until a later creation reuses and overwrites them. */
    memset(entry, 0, POGO_FLASH_FILE_ENTRY_SIZE);
    return write_changed_catalog(catalog_index, catalog);
}
