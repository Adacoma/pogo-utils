#ifndef POGO_UTILS_FLASH_FILE_INTERNAL_H
#define POGO_UTILS_FLASH_FILE_INTERNAL_H

/**
 * @file flash_file_internal.h
 * @brief Serialized catalog layout and helpers shared by flash-file modules.
 *
 * This header is private to pogo-utils. Applications should include
 * flash_file.h and must not depend on byte offsets below.
 *
 * Each 256-byte catalog page is encoded explicitly in little-endian order:
 *
 *   0..3     ASCII magic "PFFS"
 *   4        catalog format version
 *   5        catalog index (0 or 1)
 *   6        number of fixed entries (5)
 *   7        reserved, must be zero
 *   8..11    catalog generation counter
 *   12..251  five 48-byte entries
 *   252..255 CRC-32 of bytes 0..251
 *
 * Entry layout relative to its 48-byte slot:
 *
 *   0        stable file ID
 *   1        reserved, must be zero
 *   2..3     caller-owned payload format version
 *   4        first physical data page
 *   5        contiguous page count
 *   6        optional name length
 *   7        flags (exactly 1 means in use; 0 means empty)
 *   8..11    file replacement generation
 *   12..15   CRC-32 over every byte in the data extent
 *   16..47   optional name bytes, not NUL-terminated on flash
 */
#include "flash_file.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum {
    POGO_FLASH_FILE_CATALOG_VERSION = 1,     /**< Serialized schema version. */
    POGO_FLASH_FILE_CATALOG_HEADER_SIZE = 12, /**< Bytes before entry zero. */
    POGO_FLASH_FILE_ENTRY_SIZE = 48,         /**< Serialized bytes per slot. */
    POGO_FLASH_FILE_ENTRIES_PER_CATALOG = 5, /**< Fixed slots in one page. */
    POGO_FLASH_FILE_CATALOG_CRC_OFFSET = 252, /**< Stored CRC byte offset. */
    POGO_FLASH_FILE_ENTRY_IN_USE = 1         /**< Only accepted active flags. */
};

/** Load and structurally decode the fixed slot selected by file_id.
 * catalog_page is required and receives the raw page. CRC is intentionally not
 * checked here so fast readers do not pull CRC code into their binaries. The
 * caller may pass NULL for info when it only needs validation/raw catalog data.
 */
pogo_flash_file_status_t pogo_flash_file_internal_load_slot(
    uint8_t file_id,
    pogo_flash_file_info_t *info,
    uint8_t catalog_page[static POGO_FLASH_FILE_PAGE_SIZE]);

/** Check magic, version, page identity, entry count, and reserved header byte.
 * This does not check the catalog CRC or inspect any entry.
 */
bool pogo_flash_file_internal_catalog_header_valid(
    const uint8_t page[POGO_FLASH_FILE_PAGE_SIZE],
    uint8_t expected_catalog_index);

/** Return true for a virgin simulator page (all zeroes) or an erased hardware
 * flash page (all ones). Mixed contents are not considered blank.
 */
bool pogo_flash_file_internal_page_is_blank(
    const uint8_t page[POGO_FLASH_FILE_PAGE_SIZE]);

/** Validate one occupied serialized entry and optionally decode it.
 *
 * Validation covers ID/flags/reserved fields, name length, page-count bounds,
 * and physical extent bounds. Cross-entry overlap is a writer-side check and
 * is not performed here because the fast reader deliberately touches one slot.
 */
bool pogo_flash_file_internal_decode_entry(
    const uint8_t *entry,
    uint8_t expected_id,
    pogo_flash_file_info_t *info);

/** Decode unaligned little-endian integers without casting serialized bytes. */
uint16_t pogo_flash_file_internal_get_u16(const uint8_t *p);
uint32_t pogo_flash_file_internal_get_u32(const uint8_t *p);

#endif /* POGO_UTILS_FLASH_FILE_INTERNAL_H */
