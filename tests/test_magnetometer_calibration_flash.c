#include "src/pogo-utils/magnetometer_calibration_flash.h"
#include "src/pogo-utils/flash_file.h"

/* Pogosim renames firmware entry points; this file is an ordinary host test. */
#ifdef main
#undef main
#endif

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint8_t fake_flash[256u][MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE];
static bool corrupt_next_write;

void erase_write_section_flash(void) {
    memset(fake_flash, 0xff, sizeof(fake_flash));
}

void write_page_flash(uint8_t page, const void *data) {
    /* The named-file writer is specified under the platform assumption that
     * write_page_flash can replace an allocated page without changing size. */
    memcpy(fake_flash[page], data, MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE);
    if (corrupt_next_write) {
        fake_flash[page][20] ^= 1u;
        corrupt_next_write = false;
    }
}

void read_page_flash(uint8_t page, char *buffer) {
    memcpy(buffer, fake_flash[page], MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE);
}

uint32_t current_time_milliseconds(void) {
    return 0u;
}

int magn_read_XYZ(int16_t *x, int16_t *y, int16_t *z, uint16_t timeout_ms) {
    (void)x;
    (void)y;
    (void)z;
    (void)timeout_ms;
    return 1;
}

static magnetometer_heading_model_t valid_model(void) {
    magnetometer_heading_model_t model;
    memset(&model, 0, sizeof(model));
    model.e1[0] = 1.0f;
    model.e2[1] = 1.0f;
    model.w2[0][0] = 1.0f;
    model.w2[1][1] = 1.0f;
    model.s_norm = 1.0f;
    model.affine[0][0] = 1.0f;
    model.affine[1][1] = 1.0f;
    model.affine_q15[0][0] = 32767;
    model.affine_q15[1][1] = 32767;
    model.n_bins_used = 36;
    model.fit_ok = true;
    model.fixed_ready = true;
    return model;
}

static magnetometer_calibration_metadata_t valid_metadata(void) {
    magnetometer_calibration_metadata_t metadata;
    memset(&metadata, 0, sizeof(metadata));
    metadata.heading_ccw_sign = -1;
    metadata.heading_ccw_sign_valid = true;
    metadata.heading_ccw_sign_consistency_permille = 900u;
    metadata.sample_count = 120u;
    metadata.attempt_count = 140u;
    metadata.bins_used = 36u;
    return metadata;
}

int main(void) {
    /* Pogosim v0.10.10 can expose allocator residue in a fresh flash array.
     * Simulator calibration establishes a catalog unless PFFS identifies a
     * genuinely malformed catalog, which must still fail closed. */
    memset(fake_flash, 0xa5, sizeof(fake_flash));
    magnetometer_heading_detection_t detector;
    magnetometer_heading_detection_init(&detector);
    detector.model = valid_model();
    magnetometer_calibration_metadata_t metadata = valid_metadata();
    assert(magnetometer_calibration_flash_store(&detector, &metadata) ==
           MAGNETOMETER_CALIBRATION_FLASH_OK);
    assert(memcmp(fake_flash[0], "PFFS", 4u) == 0);
    memset(fake_flash, 0xa5, sizeof(fake_flash));
    memcpy(fake_flash[0], "PFFS", 4u);
    assert(magnetometer_calibration_flash_store(&detector, &metadata) ==
           MAGNETOMETER_CALIBRATION_FLASH_STORAGE_ERROR);
    assert(memcmp(fake_flash[0], "PFFS", 4u) == 0 && fake_flash[0][4] == 0xa5u);

    /* Some fresh allocations are instead uniformly zero-filled. */
    memset(fake_flash, 0, sizeof(fake_flash));
    magnetometer_heading_detection_init(&detector);
    magnetometer_heading_detection_t unchanged = detector;
    assert(magnetometer_calibration_flash_load(&detector, NULL) ==
           MAGNETOMETER_CALIBRATION_FLASH_EMPTY);
    assert(memcmp(&detector, &unchanged, sizeof(detector)) == 0);
    detector.model = valid_model();
    metadata = valid_metadata();
    assert(magnetometer_calibration_flash_store(&detector, &metadata) ==
           MAGNETOMETER_CALIBRATION_FLASH_OK);
    assert(memcmp(fake_flash[0], "PFFS", 4u) == 0);

    erase_write_section_flash();
    magnetometer_heading_detection_init(&detector);
    unchanged = detector;
    assert(magnetometer_calibration_flash_load(&detector, NULL) ==
           MAGNETOMETER_CALIBRATION_FLASH_EMPTY);
    assert(memcmp(&detector, &unchanged, sizeof(detector)) == 0);

    detector.model = valid_model();
    assert(magnetometer_heading_detection_set_offset(&detector, 0.25f));
    assert(magnetometer_heading_detection_set_filter_gain(&detector, 0.5f));
    metadata = valid_metadata();
    assert(magnetometer_calibration_flash_store(&detector, &metadata) ==
           MAGNETOMETER_CALIBRATION_FLASH_OK);
    assert(metadata.calibration_id != 0u);
    uint32_t canonical_record_id = metadata.calibration_id;
    pogo_flash_file_info_t calibration_info;
    assert(pogo_flash_file_find(
        POGO_FLASH_FILE_ID_MAGNETOMETER_CALIBRATION, &calibration_info) ==
        POGO_FLASH_FILE_OK);
    uint8_t calibration_page = calibration_info.first_page;

    /* Storing from CCW with the equivalent adapted sign produces the same
     * canonical record and does not rewrite the caller's sign convention. */
    assert(magnetometer_heading_detection_set_chirality(
        &detector, MAGNETOMETER_HEADING_CCW));
    metadata = valid_metadata();
    metadata.heading_ccw_sign = 1;
    assert(magnetometer_calibration_flash_store(&detector, &metadata) ==
           MAGNETOMETER_CALIBRATION_FLASH_OK);
    assert(metadata.heading_ccw_sign == 1);
    assert(metadata.calibration_id == canonical_record_id);
    assert(magnetometer_heading_detection_set_chirality(
        &detector, MAGNETOMETER_HEADING_CW));

    magnetometer_heading_detection_t loaded;
    magnetometer_heading_detection_init(&loaded);
    assert(magnetometer_heading_detection_set_offset(&loaded, 0.25f));
    assert(magnetometer_heading_detection_set_filter_gain(&loaded, 0.5f));
    loaded.heading_valid = true;
    magnetometer_calibration_metadata_t loaded_metadata;
    assert(magnetometer_calibration_flash_load(&loaded, &loaded_metadata) ==
           MAGNETOMETER_CALIBRATION_FLASH_OK);
    assert(loaded.offset_rad == 0.25f);
    assert(loaded.filter_gain == 0.5f);
    assert(!loaded.heading_valid && loaded.window_count == 0u);
    assert(memcmp(&loaded.model, &detector.model, sizeof(loaded.model)) == 0);
    assert(loaded_metadata.heading_ccw_sign == -1);
    assert(loaded_metadata.heading_ccw_sign_valid);
    assert(loaded_metadata.calibration_id == canonical_record_id);
    float original_heading = magnetometer_heading_detection_estimate_from_samples(
        &detector, 100, 50, 0);
    float loaded_heading = magnetometer_heading_detection_estimate_from_samples(
        &loaded, 100, 50, 0);
    assert(isfinite(original_heading));
    assert(original_heading == loaded_heading);

    /* Records use canonical CW handedness but callers receive the sign for
     * their configured runtime angle convention. */
    magnetometer_heading_detection_t loaded_ccw;
    magnetometer_heading_detection_init(&loaded_ccw);
    assert(magnetometer_heading_detection_set_chirality(
        &loaded_ccw, MAGNETOMETER_HEADING_CCW));
    magnetometer_calibration_metadata_t loaded_ccw_metadata;
    assert(magnetometer_calibration_flash_load(&loaded_ccw, &loaded_ccw_metadata) ==
           MAGNETOMETER_CALIBRATION_FLASH_OK);
    assert(loaded_ccw_metadata.heading_ccw_sign == 1);
    assert(loaded_ccw_metadata.calibration_id == canonical_record_id);

    uint8_t saved_page[MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE];
    memcpy(saved_page, fake_flash[calibration_page], sizeof(saved_page));
    fake_flash[calibration_page][20] ^= 1u;
    unchanged = loaded;
    assert(magnetometer_calibration_flash_load(&loaded, NULL) ==
           MAGNETOMETER_CALIBRATION_FLASH_BAD_CHECKSUM);
    assert(memcmp(&loaded, &unchanged, sizeof(loaded)) == 0);
    memcpy(fake_flash[calibration_page], saved_page, sizeof(saved_page));

    fake_flash[calibration_page][0] = 'X';
    assert(magnetometer_calibration_flash_load(&loaded, NULL) ==
           MAGNETOMETER_CALIBRATION_FLASH_BAD_MAGIC);
    memcpy(fake_flash[calibration_page], saved_page, sizeof(saved_page));

    fake_flash[calibration_page][4] = 2u;
    assert(magnetometer_calibration_flash_load(&loaded, NULL) ==
           MAGNETOMETER_CALIBRATION_FLASH_UNSUPPORTED_VERSION);
    memcpy(fake_flash[calibration_page], saved_page, sizeof(saved_page));

    fake_flash[calibration_page][6] = 0u;
    assert(magnetometer_calibration_flash_load(&loaded, NULL) ==
           MAGNETOMETER_CALIBRATION_FLASH_BAD_LENGTH);
    memcpy(fake_flash[calibration_page], saved_page, sizeof(saved_page));

    magnetometer_heading_model_t invalid_model = detector.model;
    invalid_model.mean[0] = NAN;
    detector.model = invalid_model;
    metadata = valid_metadata();
    assert(magnetometer_calibration_flash_store(&detector, &metadata) ==
           MAGNETOMETER_CALIBRATION_FLASH_INVALID_MODEL);
    assert(memcmp(fake_flash[calibration_page], saved_page,
                  sizeof(saved_page)) == 0);
    detector.model = valid_model();
    assert(magnetometer_heading_detection_set_offset(&detector, 0.25f));
    assert(magnetometer_heading_detection_set_filter_gain(&detector, 0.5f));

    invalid_model = detector.model;
    invalid_model.reference[0] = INT32_MAX;
    detector.model = invalid_model;
    metadata = valid_metadata();
    assert(magnetometer_calibration_flash_store(&detector, &metadata) ==
           MAGNETOMETER_CALIBRATION_FLASH_INVALID_MODEL);
    assert(memcmp(fake_flash[calibration_page], saved_page,
                  sizeof(saved_page)) == 0);
    detector.model = valid_model();

    invalid_model = detector.model;
    invalid_model.affine_q15[0][0] = 32768;
    detector.model = invalid_model;
    metadata = valid_metadata();
    assert(magnetometer_calibration_flash_store(&detector, &metadata) ==
           MAGNETOMETER_CALIBRATION_FLASH_INVALID_MODEL);
    assert(memcmp(fake_flash[calibration_page], saved_page,
                  sizeof(saved_page)) == 0);
    detector.model = valid_model();

    metadata = valid_metadata();
    metadata.heading_ccw_sign = 0;
    assert(magnetometer_calibration_flash_store(&detector, &metadata) ==
           MAGNETOMETER_CALIBRATION_FLASH_INVALID_METADATA);
    assert(memcmp(fake_flash[calibration_page], saved_page,
                  sizeof(saved_page)) == 0);

    metadata = valid_metadata();
    corrupt_next_write = true;
    assert(magnetometer_calibration_flash_store(&detector, &metadata) ==
           MAGNETOMETER_CALIBRATION_FLASH_VERIFY_FAILED);

    /* A later valid store repairs the fixed allocation without reformatting. */
    detector.model = valid_model();
    metadata = valid_metadata();
    assert(magnetometer_calibration_flash_store(&detector, &metadata) ==
           MAGNETOMETER_CALIBRATION_FLASH_OK);
    assert(memcmp(fake_flash[0], "PFFS", 4u) == 0);
    uint32_t first_named_id = metadata.calibration_id;
    uint8_t unrelated[POGO_FLASH_FILE_PAGE_SIZE];
    memset(unrelated, 0x5a, sizeof(unrelated));
    assert(pogo_flash_file_create(2u, "unrelated", 1u, 1u, unrelated) ==
           POGO_FLASH_FILE_OK);
    metadata = valid_metadata();
    metadata.attempt_count++;
    assert(magnetometer_calibration_flash_store(&detector, &metadata) ==
           MAGNETOMETER_CALIBRATION_FLASH_OK);
    assert(metadata.calibration_id != first_named_id);
    assert(pogo_flash_file_read_page_secure(2u, 0u, saved_page, NULL) ==
           POGO_FLASH_FILE_OK);
    assert(memcmp(saved_page, unrelated, sizeof(saved_page)) == 0);
    magnetometer_heading_detection_init(&loaded);
    assert(magnetometer_calibration_flash_load(&loaded, &loaded_metadata) ==
           MAGNETOMETER_CALIBRATION_FLASH_OK);
    assert(loaded_metadata.attempt_count == valid_metadata().attempt_count + 1u);

    /* A valid record outside the catalog is no longer a supported layout. */
    memcpy(saved_page, fake_flash[calibration_page], sizeof(saved_page));
    erase_write_section_flash();
    memcpy(fake_flash[0], saved_page, sizeof(saved_page));
    magnetometer_heading_detection_init(&loaded);
    unchanged = loaded;
    assert(magnetometer_calibration_flash_load(&loaded, &loaded_metadata) ==
           MAGNETOMETER_CALIBRATION_FLASH_STORAGE_ERROR);
    assert(memcmp(&loaded, &unchanged, sizeof(loaded)) == 0);

    puts("magnetometer calibration flash tests passed");
    return 0;
}
