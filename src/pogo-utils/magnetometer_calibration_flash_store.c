/**
 * @file magnetometer_calibration_flash_store.c
 * @brief Optional named-file writer for magnetometer calibration firmware.
 *
 * This object depends on PFFS mutation routines and should be linked only by a
 * calibration program. Ordinary missions need magnetometer_calibration_flash.c
 * and flash_file_read.c, but not this file or the generic allocator.
 *
 * Policy is intentionally narrow: reserved ID 1 must be absent or already be a
 * one-page record of the current PMAG format. Its allocation is never resized,
 * renamed, or moved. Formatting is performed only on first use (plus the
 * documented Pogosim uninitialized-memory workaround below).
 */
#include "magnetometer_calibration_flash_internal.h"
#include "flash_file.h"
#include "pogobase.h"

#include <string.h>

magnetometer_calibration_flash_status_t
magnetometer_calibration_flash_store(
    const magnetometer_heading_detection_t *hd,
    magnetometer_calibration_metadata_t *metadata) {
    /* Serialize and validate everything before any flash operation. This keeps
     * invalid models/metadata from erasing or partially replacing stored data. */
    uint8_t page[MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE];
    uint32_t crc; /* Inner PMAG checksum and stable calibration identity. */
    magnetometer_calibration_flash_status_t prepared =
        magnetometer_calibration_flash_internal_prepare_page(
            hd, metadata, page, &crc);
    if (prepared != MAGNETOMETER_CALIBRATION_FLASH_OK) return prepared;

    pogo_flash_file_info_t info; /* Existing ID-1 allocation, when present. */
    pogo_flash_file_status_t file_status = pogo_flash_file_find(
        POGO_FLASH_FILE_ID_MAGNETOMETER_CALIBRATION, &info);
#ifndef REAL_ROBOT
    /* This branch is compiled out of physical firmware. It compensates for a
     * Pogosim v0.10.10 construction bug, not for an old flash-file format. */
    if (file_status == POGO_FLASH_FILE_CORRUPT_CATALOG) {
        uint8_t catalog_page[MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE];
        static const uint8_t catalog_magic[4] = {'P', 'F', 'F', 'S'};
        read_page_flash(0u, (char *)catalog_page);
        /* Pogosim v0.10.10 does not initialize a new robot's flash array, so
         * it can contain allocator residue rather than 0x00 or erased 0xff.
         * Simulator calibration may establish a catalog over such a page, but
         * a recognizable damaged catalog still fails closed. */
        if (memcmp(catalog_page, catalog_magic, sizeof(catalog_magic)) != 0) {
            /* No PFFS signature means this simulator allocation has never held
             * a recognizable catalog. The normal formatting path may claim it. */
            file_status = POGO_FLASH_FILE_UNFORMATTED;
        }
    }
#endif
    if (file_status == POGO_FLASH_FILE_UNFORMATTED) {
        /* Formatting erases the complete user section. It is never attempted
         * for an existing but malformed PFFS catalog. */
        file_status = pogo_flash_file_format();
        if (file_status != POGO_FLASH_FILE_OK)
            return MAGNETOMETER_CALIBRATION_FLASH_VERIFY_FAILED;
        /* A successful format creates two empty catalogs, so ID 1 is now known
         * to be absent without performing another lookup. */
        file_status = POGO_FLASH_FILE_NOT_FOUND;
    }
    if (file_status == POGO_FLASH_FILE_NOT_FOUND) {
        /* Creation allocates one physical page and publishes the fixed name,
         * payload version, CRC, and generation through catalog slot ID 1. */
        file_status = pogo_flash_file_create(
            POGO_FLASH_FILE_ID_MAGNETOMETER_CALIBRATION,
            POGO_FLASH_FILE_NAME_MAGNETOMETER_CALIBRATION,
            1u, MAGNETOMETER_CALIBRATION_FLASH_FORMAT_VERSION, page);
    } else if (file_status == POGO_FLASH_FILE_OK) {
        /* Refuse to reinterpret an ID assigned with a different allocation or
         * payload schema. Replacement may change contents, never file shape. */
        if (info.page_count != 1u ||
            info.format_version != MAGNETOMETER_CALIBRATION_FLASH_FORMAT_VERSION) {
            return MAGNETOMETER_CALIBRATION_FLASH_STORAGE_ERROR;
        }
        file_status = pogo_flash_file_replace(
            POGO_FLASH_FILE_ID_MAGNETOMETER_CALIBRATION, 1u, page);
    }
    if (file_status != POGO_FLASH_FILE_OK) {
        /* Preserve the one public distinction calibration code can act on:
         * physical write/readback failure versus catalog/policy failure. */
        return file_status == POGO_FLASH_FILE_VERIFY_FAILED ?
            MAGNETOMETER_CALIBRATION_FLASH_VERIFY_FAILED :
            MAGNETOMETER_CALIBRATION_FLASH_STORAGE_ERROR;
    }
    /* Match the loader's nonzero-reference convention. The caller sees the ID
     * only after PFFS creation/replacement has completed successfully. */
    metadata->calibration_id = crc != 0u ? crc : 1u;
    return MAGNETOMETER_CALIBRATION_FLASH_OK;
}
