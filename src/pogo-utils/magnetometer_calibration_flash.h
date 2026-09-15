#ifndef POGO_UTILS_MAGNETOMETER_CALIBRATION_FLASH_H
#define POGO_UTILS_MAGNETOMETER_CALIBRATION_FLASH_H

/**
 * @file magnetometer_calibration_flash.h
 * @brief Versioned persistence for fitted magnetometer heading models.
 *
 * The record intentionally contains no robot identity, motor fingerprint, or
 * application heading convention. It is portable, but callers must ensure the
 * sensor mounting and motor convention are compatible with the source robot.
 */
#include "magnetometer_heading_detection.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    MAGNETOMETER_CALIBRATION_FLASH_PAGE = 0,
    MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE = 256,
    MAGNETOMETER_CALIBRATION_FLASH_FORMAT_VERSION = 1
};

typedef struct {
    /** +1/-1 for the detector's current chirality, zero when invalid. Flash
     * stores a canonical CW value and adapts it at the API boundary. */
    int8_t heading_ccw_sign;
    bool heading_ccw_sign_valid;
    uint16_t heading_ccw_sign_consistency_permille;
    uint16_t sample_count;
    uint16_t attempt_count;
    uint8_t bins_used;
    uint32_t calibration_id; /**< CRC-derived identity, filled by store/load. */
} magnetometer_calibration_metadata_t;

typedef enum {
    MAGNETOMETER_CALIBRATION_FLASH_OK = 0,
    MAGNETOMETER_CALIBRATION_FLASH_INVALID_ARGUMENT,
    MAGNETOMETER_CALIBRATION_FLASH_EMPTY,
    MAGNETOMETER_CALIBRATION_FLASH_BAD_MAGIC,
    MAGNETOMETER_CALIBRATION_FLASH_UNSUPPORTED_VERSION,
    MAGNETOMETER_CALIBRATION_FLASH_BAD_LENGTH,
    MAGNETOMETER_CALIBRATION_FLASH_BAD_CHECKSUM,
    MAGNETOMETER_CALIBRATION_FLASH_INVALID_MODEL,
    MAGNETOMETER_CALIBRATION_FLASH_INVALID_METADATA,
    MAGNETOMETER_CALIBRATION_FLASH_VERIFY_FAILED
} magnetometer_calibration_flash_status_t;

/** Erase the complete 64 KiB user section, write page zero, and read it back.
 * This operation destroys every other user-flash page. Inputs are fully
 * validated before erase. The input steering sign is interpreted using hd's
 * current chirality; the record itself is chirality-independent. metadata may
 * not be NULL.
 */
magnetometer_calibration_flash_status_t
magnetometer_calibration_flash_erase_store(
    const magnetometer_heading_detection_t *hd,
    magnetometer_calibration_metadata_t *metadata);

/** Read and validate page zero, then install the model transactionally.
 * Runtime detector settings are retained and its live filter is reset. On
 * success the returned steering sign is adapted to hd's retained chirality.
 * On failure both hd and metadata are left unchanged. metadata may be NULL.
 */
magnetometer_calibration_flash_status_t magnetometer_calibration_flash_load(
    magnetometer_heading_detection_t *hd,
    magnetometer_calibration_metadata_t *metadata);

const char *magnetometer_calibration_flash_status_string(
    magnetometer_calibration_flash_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* POGO_UTILS_MAGNETOMETER_CALIBRATION_FLASH_H */
