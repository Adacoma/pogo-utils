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
 * renamed, or moved. Creating ID 1 also initializes an absent/corrupt PFFS
 * catalog, which erases the full user section as requested by write policy.
 */
#include "magnetometer_calibration_flash_internal.h"
#include "flash_file.h"
#include "pogobase.h"

#include <stdio.h>

/** Log the exact PFFS operation and result before the public calibration API
 * maps several low-level errors to one storage/verification status. The log is
 * emitted only by the writer linked into calibration firmware. */
static void report_flash_error(const char *stage,
                               pogo_flash_file_status_t status) {
    printf("# MAG_CAL_FLASH_ERROR,robot=%u,stage=%s,status=%u,reason=%s\n",
           (unsigned)pogobot_helper_getid(), stage, (unsigned)status,
           pogo_flash_file_status_string(status));
}

/** Make the destructive recovery decision visible in calibration's serial log.
 * The generic create API also handles corruption in the other catalog page,
 * which a direct-ID fast lookup may not inspect. */
static void report_reformat(pogo_flash_file_status_t reason) {
    printf("# MAG_CAL_FLASH_REFORMAT,robot=%u,reason=%s,"
           "erases_all_user_files=1\n", (unsigned)pogobot_helper_getid(),
           pogo_flash_file_status_string(reason));
}

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
    const char *stage = "find"; /* Report the operation that first failed. */
    pogo_flash_file_status_t file_status = pogo_flash_file_find(
        POGO_FLASH_FILE_ID_MAGNETOMETER_CALIBRATION, &info);
    if (file_status == POGO_FLASH_FILE_UNFORMATTED ||
        file_status == POGO_FLASH_FILE_CORRUPT_CATALOG) {
        /* create() owns the format-and-retry policy. It will erase the entire
         * user section, even if other records could have been recovered. */
        report_reformat(file_status);
    }
    if (file_status == POGO_FLASH_FILE_NOT_FOUND ||
        file_status == POGO_FLASH_FILE_UNFORMATTED ||
        file_status == POGO_FLASH_FILE_CORRUPT_CATALOG) {
        /* Creation initializes PFFS if needed, then publishes the fixed name,
         * payload version, CRC, and generation through catalog slot ID 1. */
        stage = "create";
        file_status = pogo_flash_file_create(
            POGO_FLASH_FILE_ID_MAGNETOMETER_CALIBRATION,
            POGO_FLASH_FILE_NAME_MAGNETOMETER_CALIBRATION,
            1u, MAGNETOMETER_CALIBRATION_FLASH_FORMAT_VERSION, page);
    } else if (file_status == POGO_FLASH_FILE_OK) {
        /* Refuse to reinterpret an ID assigned with a different allocation or
         * payload schema. Replacement may change contents, never file shape. */
        if (info.page_count != 1u ||
            info.format_version != MAGNETOMETER_CALIBRATION_FLASH_FORMAT_VERSION) {
            printf("# MAG_CAL_FLASH_ERROR,robot=%u,stage=existing-schema,"
                   "pages=%u,version=%u\n",
                   (unsigned)pogobot_helper_getid(), (unsigned)info.page_count,
                   (unsigned)info.format_version);
            return MAGNETOMETER_CALIBRATION_FLASH_STORAGE_ERROR;
        }
        stage = "replace";
        file_status = pogo_flash_file_replace(
            POGO_FLASH_FILE_ID_MAGNETOMETER_CALIBRATION, 1u, page);
        if (file_status == POGO_FLASH_FILE_UNFORMATTED ||
            file_status == POGO_FLASH_FILE_CORRUPT_CATALOG) {
            /* find() reads only catalog 0. A damaged catalog 1 is discovered
             * by the stronger replacement path; retry as destructive create. */
            report_reformat(file_status);
            stage = "create";
            file_status = pogo_flash_file_create(
                POGO_FLASH_FILE_ID_MAGNETOMETER_CALIBRATION,
                POGO_FLASH_FILE_NAME_MAGNETOMETER_CALIBRATION,
                1u, MAGNETOMETER_CALIBRATION_FLASH_FORMAT_VERSION, page);
        }
    }
    if (file_status != POGO_FLASH_FILE_OK) {
        /* Preserve the one public distinction calibration code can act on:
         * physical write/readback failure versus catalog/policy failure. */
        report_flash_error(stage, file_status);
        return file_status == POGO_FLASH_FILE_VERIFY_FAILED ?
            MAGNETOMETER_CALIBRATION_FLASH_VERIFY_FAILED :
            MAGNETOMETER_CALIBRATION_FLASH_STORAGE_ERROR;
    }
    /* Match the loader's nonzero-reference convention. The caller sees the ID
     * only after PFFS creation/replacement has completed successfully. */
    metadata->calibration_id = crc != 0u ? crc : 1u;
    return MAGNETOMETER_CALIBRATION_FLASH_OK;
}
