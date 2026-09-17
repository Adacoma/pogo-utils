#ifndef POGO_UTILS_MAGNETOMETER_CALIBRATION_FLASH_H
#define POGO_UTILS_MAGNETOMETER_CALIBRATION_FLASH_H

/**
 * @file magnetometer_calibration_flash.h
 * @brief Versioned persistence for fitted magnetometer heading models.
 *
 * The record intentionally contains no robot identity, motor fingerprint, or
 * application heading convention. It is portable, but callers must ensure the
 * sensor mounting and motor convention are compatible with the source robot.
 *
 * Storage is exclusively through flash_file ID 1. The catalog entry declares a
 * one-page payload and repeats this record's format version. The payload has an
 * independent magic, length, CRC, and semantic validation, so mission firmware
 * can use the flash filesystem's small fast reader without giving up validation
 * of the actual calibration model.
 *
 * Store/load are synchronous and use caller-owned detector/metadata objects.
 * They allocate no heap memory and retain no pointers after returning. Loading
 * is transactional at the detector API boundary: a temporary model is decoded
 * and validated before it replaces the detector's existing model.
 */
#include "magnetometer_heading_detection.h"

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    /** Complete flash-file payload size; one physical flash page. */
    MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE = 256,
    /** Schema of the serialized PMAG record, not the PFFS catalog schema. */
    MAGNETOMETER_CALIBRATION_FLASH_FORMAT_VERSION = 1
};

/** Collection diagnostics persisted beside the fitted heading model.
 *
 * These values describe how the model was obtained; they do not participate in
 * the live affine heading calculation. `calibration_id` is derived from the
 * record CRC and is useful for invalidating controller state when robots change
 * calibration records.
 */
typedef struct {
    /** +1/-1 for the detector's current chirality, zero when invalid. Flash
     * stores a canonical CW value and adapts it at the API boundary. */
    int8_t heading_ccw_sign;
    bool heading_ccw_sign_valid; /**< Whether the stored sign may be trusted. */
    uint16_t heading_ccw_sign_consistency_permille; /**< Agreement in [0,1000]. */
    uint16_t sample_count; /**< Accepted calibration vectors used by the fit. */
    uint16_t attempt_count; /**< Total acquisition attempts, accepted or rejected. */
    uint8_t bins_used;     /**< Occupied angular sectors in the fitted model. */
    uint32_t calibration_id; /**< CRC-derived identity, filled by store/load. */
} magnetometer_calibration_metadata_t;

/** Persistence-layer result codes.
 *
 * Parsing errors distinguish wire corruption from a numerically invalid model.
 * STORAGE_ERROR denotes a PFFS catalog/allocation problem rather than a PMAG
 * payload problem. Callers normally treat every non-OK status as a startup
 * failure because mission code is intentionally unable to recalibrate itself.
 */
typedef enum {
    MAGNETOMETER_CALIBRATION_FLASH_OK = 0, /**< Record stored/loaded successfully. */
    MAGNETOMETER_CALIBRATION_FLASH_INVALID_ARGUMENT, /**< Bad pointer/chirality. */
    MAGNETOMETER_CALIBRATION_FLASH_EMPTY, /**< No ID-1 file/catalog yet. */
    MAGNETOMETER_CALIBRATION_FLASH_BAD_MAGIC, /**< Payload is not a PMAG record. */
    MAGNETOMETER_CALIBRATION_FLASH_UNSUPPORTED_VERSION, /**< Schema mismatch. */
    MAGNETOMETER_CALIBRATION_FLASH_BAD_LENGTH, /**< Header/field layout malformed. */
    MAGNETOMETER_CALIBRATION_FLASH_BAD_CHECKSUM, /**< Inner record CRC mismatch. */
    MAGNETOMETER_CALIBRATION_FLASH_INVALID_MODEL, /**< Decoded numeric model fails. */
    MAGNETOMETER_CALIBRATION_FLASH_INVALID_METADATA, /**< Diagnostics inconsistent. */
    MAGNETOMETER_CALIBRATION_FLASH_VERIFY_FAILED, /**< Flash write/readback failed. */
    MAGNETOMETER_CALIBRATION_FLASH_STORAGE_ERROR /**< PFFS lookup/catalog error. */
} magnetometer_calibration_flash_status_t;

/** Create or replace the named magnetometer-calibration flash file.
 * A previously unformatted user section is formatted once; later stores keep
 * other catalog files and cannot change this file's one-page allocation.
 * Inputs are fully validated and serialized before flash is modified. On
 * success `metadata->calibration_id` is updated; otherwise caller data is not
 * changed. This writer belongs in calibration firmware, not mission firmware.
 */
magnetometer_calibration_flash_status_t magnetometer_calibration_flash_store(
    const magnetometer_heading_detection_t *hd,
    magnetometer_calibration_metadata_t *metadata);

/** Read and validate the named file, then install the model transactionally.
 * Runtime detector settings are retained and its live filter is reset. On
 * success the returned steering sign is adapted to hd's retained chirality.
 * On failure both hd and metadata are left unchanged. metadata may be NULL.
 * The loader never formats, creates, replaces, or deletes flash files.
 */
magnetometer_calibration_flash_status_t magnetometer_calibration_flash_load(
    magnetometer_heading_detection_t *hd,
    magnetometer_calibration_metadata_t *metadata);

/** Return a static, allocation-free diagnostic string for logs/UART output. */
const char *magnetometer_calibration_flash_status_string(
    magnetometer_calibration_flash_status_t status);

#ifdef __cplusplus
}
#endif

#endif /* POGO_UTILS_MAGNETOMETER_CALIBRATION_FLASH_H */
