#ifndef POGO_UTILS_MAGNETOMETER_CALIBRATION_FLASH_INTERNAL_H
#define POGO_UTILS_MAGNETOMETER_CALIBRATION_FLASH_INTERNAL_H

/**
 * @file magnetometer_calibration_flash_internal.h
 * @brief Private bridge between record serialization and optional flash writes.
 *
 * The encoder lives with the loader so there is one definition of the PMAG wire
 * format. The store operation lives in a separate object so mission binaries
 * that only load calibration do not retain the PFFS writer/allocation code.
 */
#include "magnetometer_calibration_flash.h"

/** Validate a fitted detector and metadata, canonicalize chirality, and encode
 * one complete 256-byte PMAG page without touching flash.
 *
 * @param hd Detector containing the fitted model to serialize.
 * @param metadata Collection diagnostics in hd's current chirality convention.
 * @param page Required destination; unused tail bytes are filled with 0xff.
 * @param record_crc Required output used as the stable calibration identity.
 */
magnetometer_calibration_flash_status_t
magnetometer_calibration_flash_internal_prepare_page(
    const magnetometer_heading_detection_t *hd,
    const magnetometer_calibration_metadata_t *metadata,
    uint8_t page[MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE],
    uint32_t *record_crc);

#endif /* POGO_UTILS_MAGNETOMETER_CALIBRATION_FLASH_INTERNAL_H */
