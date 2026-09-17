#ifndef POGO_UTILS_FLASH_FILE_H
#define POGO_UTILS_FLASH_FILE_H

/**
 * @file flash_file.h
 * @brief Small, bounded named-file catalog over Pogobot user flash.
 *
 * Catalog pages 0 and 1 contain five fixed slots each. Stable file IDs 1..10
 * map directly to those slots, so ID lookup reads exactly one catalog page.
 * Files occupy 1..8 contiguous, complete 256-byte pages starting at page 2.
 * Human-readable names are optional metadata; IDs are the persistent identity.
 *
 * The fast reader checks structural bounds but deliberately skips CRC checks.
 * The secure reader checks the catalog CRC and the CRC of every file page.
 * Replacement writes an existing extent in place and is not transactional: an
 * interrupted write is detected by the secure reader but cannot restore the
 * previous contents. The platform must support rewriting the selected pages.
 *
 * On-flash organization (all page numbers are relative to the 64 KiB writable
 * user section):
 *
 *   page 0     catalog for IDs 1..5
 *   page 1     catalog for IDs 6..10
 *   pages 2..255 data extents allocated by first fit
 *
 * There is no heap allocation, directory tree, variable-length byte stream,
 * compaction, or open-file state. A "file page" is always exactly one physical
 * 256-byte page. That restriction is deliberate: callers can keep one page
 * buffer and the fast reader can resolve an ID with two physical reads.
 *
 * Public functions are synchronous. They do not lock the flash peripheral and
 * must not be interleaved by concurrent callers for the same robot. Returned
 * metadata is decoded into ordinary host values; callers never receive pointers
 * into a temporary catalog buffer.
 */

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    POGO_FLASH_FILE_PAGE_SIZE = 256,    /**< Physical and logical page size. */
    POGO_FLASH_FILE_CATALOG_PAGES = 2, /**< Reserved metadata pages 0 and 1. */
    POGO_FLASH_FILE_DATA_FIRST_PAGE = 2, /**< First allocatable physical page. */
    POGO_FLASH_FILE_MAX_FILES = 10,    /**< Number of stable ID slots. */
    POGO_FLASH_FILE_MAX_PAGES = 8,     /**< Maximum contiguous pages per file. */
    POGO_FLASH_FILE_MAX_NAME = 32,     /**< Stored name bytes, excluding NUL. */

    /** Reserved stable ID used by pogo-utils' magnetometer calibration. */
    POGO_FLASH_FILE_ID_MAGNETOMETER_CALIBRATION = 1
};

#define POGO_FLASH_FILE_NAME_MAGNETOMETER_CALIBRATION \
    "magnetometer_calibration"

/** Decoded catalog entry returned to callers.
 *
 * The structure is an in-memory API object, not the serialized representation.
 * Its padding and native byte order therefore have no effect on flash format.
 * `name` is always NUL-terminated by the decoder, including for a 32-byte name.
 */
typedef struct {
    uint8_t id;             /**< Stable ID whose numeric value selects the slot. */
    uint8_t first_page;     /**< First physical page in the contiguous extent. */
    uint8_t page_count;     /**< Immutable number of pages in the extent. */
    uint8_t name_length;    /**< Stored label length; zero means unnamed. */
    uint16_t format_version; /**< Payload schema version owned by the caller. */
    uint32_t generation;    /**< Per-file replacement counter, starting at one. */
    uint32_t data_crc32;    /**< CRC over all allocated pages, in page order. */
    char name[POGO_FLASH_FILE_MAX_NAME + 1]; /**< Optional NUL-terminated label. */
} pogo_flash_file_info_t;

/** Result codes shared by reader and writer entry points.
 *
 * `UNFORMATTED` is distinct from `CORRUPT_CATALOG`: a caller may deliberately
 * format the former, whereas automatically formatting the latter could destroy
 * recoverable files. Read functions never modify flash for either result.
 */
typedef enum {
    POGO_FLASH_FILE_OK = 0,             /**< Operation completed successfully. */
    POGO_FLASH_FILE_INVALID_ARGUMENT,   /**< NULL pointer, invalid ID, or name. */
    POGO_FLASH_FILE_UNFORMATTED,        /**< Catalog page is uniformly blank. */
    POGO_FLASH_FILE_CORRUPT_CATALOG,    /**< Header, entry, extent, or CRC fails. */
    POGO_FLASH_FILE_NOT_FOUND,          /**< Selected slot/name is not in use. */
    POGO_FLASH_FILE_ALREADY_EXISTS,     /**< Creation selected an occupied ID. */
    POGO_FLASH_FILE_NAME_EXISTS,        /**< Creation duplicated a nonempty name. */
    POGO_FLASH_FILE_INVALID_SIZE,       /**< Page count is zero, too large, or changed. */
    POGO_FLASH_FILE_NO_SPACE,           /**< No sufficiently long free extent exists. */
    POGO_FLASH_FILE_BAD_CHECKSUM,       /**< Secure read found damaged file data. */
    POGO_FLASH_FILE_VERIFY_FAILED,      /**< Immediate write/readback differed. */
    POGO_FLASH_FILE_GENERATION_EXHAUSTED /**< A monotonic counter reached UINT32_MAX. */
} pogo_flash_file_status_t;

/** Find an ID using structural checks only; no CRC is calculated.
 *
 * @param file_id Stable ID in the inclusive range 1..10.
 * @param info Required destination for decoded metadata.
 * @return OK, NOT_FOUND, UNFORMATTED, CORRUPT_CATALOG, or INVALID_ARGUMENT.
 */
pogo_flash_file_status_t pogo_flash_file_find(
    uint8_t file_id,
    pogo_flash_file_info_t *info);

/** Scan both catalog pages for an optional human-readable name.
 *
 * Names are exact, case-sensitive byte strings. Empty names are not searchable
 * and numeric IDs remain authoritative even when a name is present.
 */
pogo_flash_file_status_t pogo_flash_file_find_by_name(
    const char *name,
    pogo_flash_file_info_t *info);

/** Read one file page with no catalog or data CRC validation.
 * Exactly one catalog page and one data page are read on success. output is
 * also used as catalog scratch space, so no persistent cache is required.
 * `file_page` is zero-based within the file, not a physical flash page number.
 * `info` is optional and is written only on success.
 */
pogo_flash_file_status_t pogo_flash_file_read_page_fast(
    uint8_t file_id,
    uint8_t file_page,
    uint8_t output[POGO_FLASH_FILE_PAGE_SIZE],
    pogo_flash_file_info_t *info);

/** Read one page after validating the catalog and the complete file CRC.
 * Validation reads every page in the file and may reread the requested page.
 * The complete-file scan means latency grows linearly with `page_count`.
 */
pogo_flash_file_status_t pogo_flash_file_read_page_secure(
    uint8_t file_id,
    uint8_t file_page,
    uint8_t output[POGO_FLASH_FILE_PAGE_SIZE],
    pogo_flash_file_info_t *info);

/** Erase the complete 64 KiB user section and initialize both catalogs.
 *
 * This is the only API that erases the whole user section. It is destructive
 * even when a catalog already exists; applications should call it only as an
 * explicit first-use or reset policy.
 */
pogo_flash_file_status_t pogo_flash_file_format(void);

/** Create a file in the slot selected by file_id.
 * data must contain exactly page_count consecutive 256-byte pages.
 * `name` may be NULL or empty for an unnamed file. Data pages are written and
 * verified before the catalog entry is published, so a data-write failure does
 * not make the new slot visible.
 */
pogo_flash_file_status_t pogo_flash_file_create(
    uint8_t file_id,
    const char *name,
    uint8_t page_count,
    uint16_t format_version,
    const uint8_t *data);

/** Replace an existing file in place. page_count must equal its stored size.
 *
 * Replacement is deliberately non-transactional: data pages are overwritten
 * before the catalog CRC/generation is updated. A reset or power loss may leave
 * the old catalog referring to partially new data.
 */
pogo_flash_file_status_t pogo_flash_file_replace(
    uint8_t file_id,
    uint8_t page_count,
    const uint8_t *data);

/** Remove a catalog entry. Data pages are left untouched and may be reused.
 *
 * Deletion therefore does not securely erase payload bytes. The next creation
 * may allocate the released extent and overwrite them.
 */
pogo_flash_file_status_t pogo_flash_file_delete(uint8_t file_id);

/** Return a static diagnostic string. No allocation or formatting is done. */
const char *pogo_flash_file_status_string(pogo_flash_file_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* POGO_UTILS_FLASH_FILE_H */
