/**
 * @file magnetometer_calibration_flash.c
 * @brief Checked, ABI-independent magnetometer calibration persistence.
 */
#include "magnetometer_calibration_flash.h"

#include <float.h>
#include <limits.h>
#include <stddef.h>
#include <string.h>

#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128
#error "Magnetometer flash records require IEEE binary32 float"
#endif
_Static_assert(sizeof(float) == sizeof(uint32_t), "float must be 32 bits");

enum {
    RECORD_CRC_OFFSET = 164,
    RECORD_SIZE = 168,
    RECORD_MODEL_FIT_OK = 1u,
    RECORD_MODEL_FIXED_READY = 2u,
    RECORD_METADATA_SIGN_VALID = 1u
};

static const uint8_t record_magic[4] = {'P', 'M', 'A', 'G'};

typedef struct {
    uint8_t *bytes;
    size_t pos;
    size_t limit;
} page_writer_t;

typedef struct {
    const uint8_t *bytes;
    size_t pos;
    size_t limit;
} page_reader_t;

static bool put_u8(page_writer_t *writer, uint8_t value) {
    if (writer->pos >= writer->limit) {
        return false;
    }
    writer->bytes[writer->pos++] = value;
    return true;
}

static bool put_u16(page_writer_t *writer, uint16_t value) {
    return put_u8(writer, (uint8_t)value) && put_u8(writer, (uint8_t)(value >> 8));
}

static bool put_u32(page_writer_t *writer, uint32_t value) {
    return put_u16(writer, (uint16_t)value) && put_u16(writer, (uint16_t)(value >> 16));
}

static bool put_i32(page_writer_t *writer, int32_t value) {
    return put_u32(writer, (uint32_t)value);
}

static bool put_float(page_writer_t *writer, float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return put_u32(writer, bits);
}

static bool get_u8(page_reader_t *reader, uint8_t *value) {
    if (reader->pos >= reader->limit) {
        return false;
    }
    *value = reader->bytes[reader->pos++];
    return true;
}

static bool get_u16(page_reader_t *reader, uint16_t *value) {
    uint8_t lo;
    uint8_t hi;
    if (!get_u8(reader, &lo) || !get_u8(reader, &hi)) {
        return false;
    }
    *value = (uint16_t)((uint16_t)lo | (uint16_t)((uint16_t)hi << 8));
    return true;
}

static bool get_u32(page_reader_t *reader, uint32_t *value) {
    uint16_t lo;
    uint16_t hi;
    if (!get_u16(reader, &lo) || !get_u16(reader, &hi)) {
        return false;
    }
    *value = (uint32_t)lo | ((uint32_t)hi << 16);
    return true;
}

static bool get_i32(page_reader_t *reader, int32_t *value) {
    uint32_t bits;
    if (!get_u32(reader, &bits)) {
        return false;
    }
    /* Decode two's-complement wire bits without relying on the host's signed
     * integer object representation or an out-of-range unsigned conversion. */
    *value = bits <= (uint32_t)INT32_MAX ? (int32_t)bits :
        -1 - (int32_t)(UINT32_MAX - bits);
    return true;
}

static bool get_float(page_reader_t *reader, float *value) {
    uint32_t bits;
    if (!get_u32(reader, &bits)) {
        return false;
    }
    memcpy(value, &bits, sizeof(bits));
    return true;
}

/* CRC-32/ISO-HDLC: reflected polynomial, all-one initial/final XOR. */
static uint32_t record_crc32(const uint8_t *bytes, size_t length) {
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8u; ++bit) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (UINT32_C(0xedb88320) & mask);
        }
    }
    return crc ^ UINT32_MAX;
}

static bool metadata_is_valid(const magnetometer_calibration_metadata_t *metadata) {
    if (metadata == NULL ||
        metadata->sample_count < MAGNETOMETER_HEADING_CAL_MIN_POINTS ||
        metadata->sample_count > MAGNETOMETER_HEADING_CAL_CAPACITY ||
        metadata->attempt_count < metadata->sample_count ||
        metadata->bins_used > MAGNETOMETER_HEADING_CAL_BINS ||
        metadata->heading_ccw_sign_consistency_permille > 1000u) {
        return false;
    }
    if (metadata->heading_ccw_sign_valid) {
        return metadata->heading_ccw_sign == -1 || metadata->heading_ccw_sign == 1;
    }
    return metadata->heading_ccw_sign == 0 &&
        metadata->heading_ccw_sign_consistency_permille == 0u;
}

static bool detector_chirality_is_valid(
    const magnetometer_heading_detection_t *hd) {
    return hd != NULL &&
        (hd->chirality == MAGNETOMETER_HEADING_CW ||
         hd->chirality == MAGNETOMETER_HEADING_CCW);
}

static bool encode_page(uint8_t page[MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE],
                        const magnetometer_heading_model_t *model,
                        const magnetometer_calibration_metadata_t *metadata,
                        uint32_t *record_crc) {
    memset(page, 0xff, MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE);
    memcpy(page, record_magic, sizeof(record_magic));
    page_writer_t writer = {page, 4u, RECORD_CRC_OFFSET};
    uint8_t model_flags = RECORD_MODEL_FIT_OK |
        (model->fixed_ready ? RECORD_MODEL_FIXED_READY : 0u);
    uint8_t metadata_flags = metadata->heading_ccw_sign_valid ?
        RECORD_METADATA_SIGN_VALID : 0u;
    if (!put_u16(&writer, MAGNETOMETER_CALIBRATION_FLASH_FORMAT_VERSION) ||
        !put_u16(&writer, RECORD_SIZE) || !put_u16(&writer, 0u) ||
        !put_u16(&writer, 0u)) {
        return false;
    }
#define PUT_FLOAT_ARRAY(array, count) \
    do { for (size_t i = 0; i < (count); ++i) { \
        if (!put_float(&writer, (array)[i])) return false; \
    } } while (0)
    PUT_FLOAT_ARRAY(model->mean, 3u);
    PUT_FLOAT_ARRAY(model->e1, 3u);
    PUT_FLOAT_ARRAY(model->e2, 3u);
    if (!put_float(&writer, model->u0) || !put_float(&writer, model->v0)) return false;
    PUT_FLOAT_ARRAY(model->w2[0], 2u);
    PUT_FLOAT_ARRAY(model->w2[1], 2u);
    if (!put_float(&writer, model->s_norm)) return false;
    PUT_FLOAT_ARRAY(model->affine[0], 3u);
    PUT_FLOAT_ARRAY(model->affine[1], 3u);
    PUT_FLOAT_ARRAY(model->bias, 2u);
#undef PUT_FLOAT_ARRAY
    for (size_t i = 0; i < 3u; ++i) if (!put_i32(&writer, model->reference[i])) return false;
    for (size_t row = 0; row < 2u; ++row)
        for (size_t col = 0; col < 3u; ++col)
            if (!put_i32(&writer, model->affine_q15[row][col])) return false;
    for (size_t i = 0; i < 2u; ++i) if (!put_i32(&writer, model->bias_q2[i])) return false;
    uint8_t sign_code = metadata->heading_ccw_sign > 0 ? 1u :
        (metadata->heading_ccw_sign < 0 ? 2u : 0u);
    if (!put_u8(&writer, (uint8_t)model->n_bins_used) ||
        !put_u8(&writer, model_flags) ||
        !put_u8(&writer, sign_code) ||
        !put_u8(&writer, metadata_flags) ||
        !put_u16(&writer, metadata->heading_ccw_sign_consistency_permille) ||
        !put_u16(&writer, metadata->sample_count) ||
        !put_u16(&writer, metadata->attempt_count) ||
        !put_u8(&writer, metadata->bins_used) || !put_u8(&writer, 0u) ||
        writer.pos != RECORD_CRC_OFFSET) {
        return false;
    }
    uint32_t crc = record_crc32(page, RECORD_CRC_OFFSET);
    writer.limit = RECORD_SIZE;
    if (!put_u32(&writer, crc) || writer.pos != RECORD_SIZE) {
        return false;
    }
    *record_crc = crc;
    return true;
}

static magnetometer_calibration_flash_status_t decode_page(
    const uint8_t page[MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE],
    magnetometer_heading_model_t *model,
    magnetometer_calibration_metadata_t *metadata) {
    bool erased = true;
    for (size_t i = 0; i < sizeof(record_magic); ++i) erased = erased && page[i] == 0xffu;
    if (erased) return MAGNETOMETER_CALIBRATION_FLASH_EMPTY;
    if (memcmp(page, record_magic, sizeof(record_magic)) != 0)
        return MAGNETOMETER_CALIBRATION_FLASH_BAD_MAGIC;

    page_reader_t reader = {page, 4u, RECORD_SIZE};
    uint16_t version;
    uint16_t length;
    uint16_t header_flags;
    uint16_t reserved;
    if (!get_u16(&reader, &version)) return MAGNETOMETER_CALIBRATION_FLASH_BAD_LENGTH;
    if (version != MAGNETOMETER_CALIBRATION_FLASH_FORMAT_VERSION)
        return MAGNETOMETER_CALIBRATION_FLASH_UNSUPPORTED_VERSION;
    if (!get_u16(&reader, &length) || length != RECORD_SIZE ||
        !get_u16(&reader, &header_flags) || !get_u16(&reader, &reserved) ||
        header_flags != 0u || reserved != 0u)
        return MAGNETOMETER_CALIBRATION_FLASH_BAD_LENGTH;
    uint32_t stored_crc;
    page_reader_t crc_reader = {page, RECORD_CRC_OFFSET, RECORD_SIZE};
    if (!get_u32(&crc_reader, &stored_crc) ||
        stored_crc != record_crc32(page, RECORD_CRC_OFFSET))
        return MAGNETOMETER_CALIBRATION_FLASH_BAD_CHECKSUM;

    memset(model, 0, sizeof(*model));
    memset(metadata, 0, sizeof(*metadata));
#define GET_FLOAT_ARRAY(array, count) \
    do { for (size_t i = 0; i < (count); ++i) { \
        if (!get_float(&reader, &(array)[i])) \
            return MAGNETOMETER_CALIBRATION_FLASH_BAD_LENGTH; \
    } } while (0)
    GET_FLOAT_ARRAY(model->mean, 3u);
    GET_FLOAT_ARRAY(model->e1, 3u);
    GET_FLOAT_ARRAY(model->e2, 3u);
    if (!get_float(&reader, &model->u0) || !get_float(&reader, &model->v0))
        return MAGNETOMETER_CALIBRATION_FLASH_BAD_LENGTH;
    GET_FLOAT_ARRAY(model->w2[0], 2u);
    GET_FLOAT_ARRAY(model->w2[1], 2u);
    if (!get_float(&reader, &model->s_norm))
        return MAGNETOMETER_CALIBRATION_FLASH_BAD_LENGTH;
    GET_FLOAT_ARRAY(model->affine[0], 3u);
    GET_FLOAT_ARRAY(model->affine[1], 3u);
    GET_FLOAT_ARRAY(model->bias, 2u);
#undef GET_FLOAT_ARRAY
    for (size_t i = 0; i < 3u; ++i)
        if (!get_i32(&reader, &model->reference[i])) return MAGNETOMETER_CALIBRATION_FLASH_BAD_LENGTH;
    for (size_t row = 0; row < 2u; ++row)
        for (size_t col = 0; col < 3u; ++col)
            if (!get_i32(&reader, &model->affine_q15[row][col]))
                return MAGNETOMETER_CALIBRATION_FLASH_BAD_LENGTH;
    for (size_t i = 0; i < 2u; ++i)
        if (!get_i32(&reader, &model->bias_q2[i])) return MAGNETOMETER_CALIBRATION_FLASH_BAD_LENGTH;
    uint8_t bins;
    uint8_t model_flags;
    uint8_t sign;
    uint8_t metadata_flags;
    uint8_t metadata_bins;
    uint8_t payload_reserved;
    if (!get_u8(&reader, &bins) || !get_u8(&reader, &model_flags) ||
        !get_u8(&reader, &sign) || !get_u8(&reader, &metadata_flags) ||
        !get_u16(&reader, &metadata->heading_ccw_sign_consistency_permille) ||
        !get_u16(&reader, &metadata->sample_count) ||
        !get_u16(&reader, &metadata->attempt_count) ||
        !get_u8(&reader, &metadata_bins) || !get_u8(&reader, &payload_reserved) ||
        reader.pos != RECORD_CRC_OFFSET || payload_reserved != 0u ||
        (model_flags & ~(RECORD_MODEL_FIT_OK | RECORD_MODEL_FIXED_READY)) != 0u ||
        (metadata_flags & ~RECORD_METADATA_SIGN_VALID) != 0u ||
        (model_flags & RECORD_MODEL_FIT_OK) == 0u)
        return MAGNETOMETER_CALIBRATION_FLASH_BAD_LENGTH;
    model->n_bins_used = bins;
    model->fit_ok = true;
    model->fixed_ready = (model_flags & RECORD_MODEL_FIXED_READY) != 0u;
    if (sign > 2u) return MAGNETOMETER_CALIBRATION_FLASH_INVALID_METADATA;
    metadata->heading_ccw_sign = sign == 1u ? 1 : (sign == 2u ? -1 : 0);
    metadata->heading_ccw_sign_valid = (metadata_flags & RECORD_METADATA_SIGN_VALID) != 0u;
    metadata->bins_used = metadata_bins;
    metadata->calibration_id = stored_crc != 0u ? stored_crc : 1u;
    if (!magnetometer_heading_model_is_valid(model))
        return MAGNETOMETER_CALIBRATION_FLASH_INVALID_MODEL;
    if (!metadata_is_valid(metadata) || metadata->bins_used != bins)
        return MAGNETOMETER_CALIBRATION_FLASH_INVALID_METADATA;
    return MAGNETOMETER_CALIBRATION_FLASH_OK;
}

magnetometer_calibration_flash_status_t
magnetometer_calibration_flash_erase_store(
    const magnetometer_heading_detection_t *hd,
    magnetometer_calibration_metadata_t *metadata) {
    if (!detector_chirality_is_valid(hd) || metadata == NULL)
        return MAGNETOMETER_CALIBRATION_FLASH_INVALID_ARGUMENT;
    if (!magnetometer_heading_model_is_valid(&hd->model))
        return MAGNETOMETER_CALIBRATION_FLASH_INVALID_MODEL;
    if (!metadata_is_valid(metadata) || metadata->bins_used != hd->model.n_bins_used)
        return MAGNETOMETER_CALIBRATION_FLASH_INVALID_METADATA;
    uint8_t expected[MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE];
    uint8_t actual[MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE];
    uint32_t crc;
    /* Steering handedness is measured in the detector's configured angle
     * convention. Persist it in the model's canonical CW convention so the
     * same record remains usable by a CCW mission. */
    magnetometer_calibration_metadata_t canonical_metadata = *metadata;
    if (canonical_metadata.heading_ccw_sign_valid) {
        canonical_metadata.heading_ccw_sign = (int8_t)(
            canonical_metadata.heading_ccw_sign * (int)hd->chirality);
    }
    if (!encode_page(expected, &hd->model, &canonical_metadata, &crc))
        return MAGNETOMETER_CALIBRATION_FLASH_INVALID_ARGUMENT;
    erase_write_section_flash();
    write_page_flash(MAGNETOMETER_CALIBRATION_FLASH_PAGE, expected);
    read_page_flash(MAGNETOMETER_CALIBRATION_FLASH_PAGE, (char *)actual);
    if (memcmp(expected, actual, sizeof(expected)) != 0)
        return MAGNETOMETER_CALIBRATION_FLASH_VERIFY_FAILED;
    magnetometer_heading_model_t verified_model;
    magnetometer_calibration_metadata_t verified_metadata;
    magnetometer_calibration_flash_status_t status =
        decode_page(actual, &verified_model, &verified_metadata);
    if (status != MAGNETOMETER_CALIBRATION_FLASH_OK)
        return MAGNETOMETER_CALIBRATION_FLASH_VERIFY_FAILED;
    metadata->calibration_id = crc != 0u ? crc : 1u;
    return MAGNETOMETER_CALIBRATION_FLASH_OK;
}

magnetometer_calibration_flash_status_t magnetometer_calibration_flash_load(
    magnetometer_heading_detection_t *hd,
    magnetometer_calibration_metadata_t *metadata) {
    if (!detector_chirality_is_valid(hd))
        return MAGNETOMETER_CALIBRATION_FLASH_INVALID_ARGUMENT;
    uint8_t page[MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE];
    magnetometer_heading_model_t model;
    magnetometer_calibration_metadata_t decoded;
    read_page_flash(MAGNETOMETER_CALIBRATION_FLASH_PAGE, (char *)page);
    magnetometer_calibration_flash_status_t status = decode_page(page, &model, &decoded);
    if (status != MAGNETOMETER_CALIBRATION_FLASH_OK) return status;
    if (!magnetometer_heading_detection_install_model(hd, &model))
        return MAGNETOMETER_CALIBRATION_FLASH_INVALID_MODEL;
    if (decoded.heading_ccw_sign_valid) {
        decoded.heading_ccw_sign = (int8_t)(
            decoded.heading_ccw_sign * (int)hd->chirality);
    }
    if (metadata != NULL) *metadata = decoded;
    return MAGNETOMETER_CALIBRATION_FLASH_OK;
}

const char *magnetometer_calibration_flash_status_string(
    magnetometer_calibration_flash_status_t status) {
    switch (status) {
    case MAGNETOMETER_CALIBRATION_FLASH_OK: return "ok";
    case MAGNETOMETER_CALIBRATION_FLASH_INVALID_ARGUMENT: return "invalid argument";
    case MAGNETOMETER_CALIBRATION_FLASH_EMPTY: return "magnetometer calibration flash is empty";
    case MAGNETOMETER_CALIBRATION_FLASH_BAD_MAGIC: return "bad magnetometer calibration magic";
    case MAGNETOMETER_CALIBRATION_FLASH_UNSUPPORTED_VERSION: return "unsupported magnetometer calibration version";
    case MAGNETOMETER_CALIBRATION_FLASH_BAD_LENGTH: return "malformed magnetometer calibration record";
    case MAGNETOMETER_CALIBRATION_FLASH_BAD_CHECKSUM: return "magnetometer calibration checksum mismatch";
    case MAGNETOMETER_CALIBRATION_FLASH_INVALID_MODEL: return "invalid stored magnetometer model";
    case MAGNETOMETER_CALIBRATION_FLASH_INVALID_METADATA: return "invalid stored magnetometer metadata";
    case MAGNETOMETER_CALIBRATION_FLASH_VERIFY_FAILED: return "magnetometer flash write verification failed";
    default: return "unknown magnetometer flash status";
    }
}
