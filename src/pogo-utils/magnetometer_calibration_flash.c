/**
 * @file magnetometer_calibration_flash.c
 * @brief Checked, ABI-independent magnetometer calibration persistence.
 *
 * This file owns the PMAG payload schema and the read-only load path. It does
 * not write flash; the optional store path is isolated in
 * magnetometer_calibration_flash_store.c to keep mission binaries small.
 *
 * PMAG version-1 byte layout inside its 256-byte PFFS data page:
 *
 *   0..3      magic "PMAG"
 *   4..5      record version
 *   6..7      meaningful record length (168)
 *   8..9      header flags (currently zero)
 *   10..11    reserved (zero)
 *   12..107   float model construction and affine-map fields
 *   108..151  fixed-point reference/map/bias fields
 *   152       fitted occupied-bin count
 *   153       model flags
 *   154       encoded steering sign (0 unknown, 1 positive, 2 negative)
 *   155       metadata flags
 *   156..163  consistency/sample/attempt/bin metadata and reserved byte
 *   164..167  CRC-32 over bytes 0..163
 *   168..255  deterministic 0xff padding
 *
 * All multibyte values are little-endian. Floats are serialized as their IEEE
 * binary32 bit patterns. No native C structure is copied to or from flash, so
 * compiler padding and host alignment cannot alter the record.
 */
#include "magnetometer_calibration_flash.h"
#include "magnetometer_calibration_flash_internal.h"
#include "flash_file.h"

#include <float.h>
#include <limits.h>
#include <stddef.h>
#include <string.h>

#if FLT_RADIX != 2 || FLT_MANT_DIG != 24 || FLT_MAX_EXP != 128
#error "Magnetometer flash records require IEEE binary32 float"
#endif
/* memcpy between float and uint32_t below also requires equal object sizes. */
_Static_assert(sizeof(float) == sizeof(uint32_t), "float must be 32 bits");

enum {
    RECORD_CRC_OFFSET = 164,          /**< First byte excluded from record CRC. */
    RECORD_SIZE = 168,                /**< Header+payload+CRC meaningful bytes. */
    RECORD_MODEL_FIT_OK = 1u,         /**< Serialized model passed float fit. */
    RECORD_MODEL_FIXED_READY = 2u,    /**< Checked Q15 map is available. */
    RECORD_METADATA_SIGN_VALID = 1u   /**< Steering-sign metadata is usable. */
};

/** Inner payload discriminator, independent of the outer PFFS catalog magic. */
static const uint8_t record_magic[4] = {'P', 'M', 'A', 'G'};

/** Bounds-checked cursor used to construct a serialized page sequentially. */
typedef struct {
    uint8_t *bytes; /**< Destination page. */
    size_t pos;     /**< Offset of the next byte to write. */
    size_t limit;   /**< Exclusive upper bound for the current region. */
} page_writer_t;

/** Read-only counterpart of page_writer_t used during decoding. */
typedef struct {
    const uint8_t *bytes; /**< Source page. */
    size_t pos;           /**< Offset of the next byte to consume. */
    size_t limit;         /**< Exclusive upper bound for reads. */
} page_reader_t;

/** Append one byte if the writer still has room in its current region. */
static bool put_u8(page_writer_t *writer, uint8_t value) {
    if (writer->pos >= writer->limit) {
        return false;
    }
    writer->bytes[writer->pos++] = value;
    return true;
}

/** Append an unaligned little-endian 16-bit unsigned value. */
static bool put_u16(page_writer_t *writer, uint16_t value) {
    return put_u8(writer, (uint8_t)value) && put_u8(writer, (uint8_t)(value >> 8));
}

/** Append an unaligned little-endian 32-bit unsigned value. */
static bool put_u32(page_writer_t *writer, uint32_t value) {
    return put_u16(writer, (uint16_t)value) && put_u16(writer, (uint16_t)(value >> 16));
}

/** Preserve the two's-complement bit pattern through the unsigned encoder. */
static bool put_i32(page_writer_t *writer, int32_t value) {
    return put_u32(writer, (uint32_t)value);
}

/** Serialize an IEEE binary32 value without aliasing through an integer pointer. */
static bool put_float(page_writer_t *writer, float value) {
    uint32_t bits;
    memcpy(&bits, &value, sizeof(bits));
    return put_u32(writer, bits);
}

/** Consume one byte if the reader has not reached its region limit. */
static bool get_u8(page_reader_t *reader, uint8_t *value) {
    if (reader->pos >= reader->limit) {
        return false;
    }
    *value = reader->bytes[reader->pos++];
    return true;
}

/** Consume one unaligned little-endian 16-bit unsigned value. */
static bool get_u16(page_reader_t *reader, uint16_t *value) {
    uint8_t lo;
    uint8_t hi;
    if (!get_u8(reader, &lo) || !get_u8(reader, &hi)) {
        return false;
    }
    *value = (uint16_t)((uint16_t)lo | (uint16_t)((uint16_t)hi << 8));
    return true;
}

/** Consume one unaligned little-endian 32-bit unsigned value. */
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

/** Reconstruct binary32 bits without violating C's strict-aliasing rules. */
static bool get_float(page_reader_t *reader, float *value) {
    uint32_t bits;
    if (!get_u32(reader, &bits)) {
        return false;
    }
    memcpy(value, &bits, sizeof(bits));
    return true;
}

/** Calculate CRC-32/ISO-HDLC with reflected polynomial 0xedb88320.
 *
 * A bitwise implementation trades speed for small code and no lookup table.
 * Initial and final XORs are all ones, matching the outer PFFS checksums.
 */
static uint32_t record_crc32(const uint8_t *bytes, size_t length) {
    uint32_t crc = UINT32_MAX;
    for (size_t i = 0; i < length; ++i) {
        crc ^= bytes[i];
        for (unsigned bit = 0; bit < 8u; ++bit) {
            /* Convert the low bit into an all-zero/all-one selection mask. */
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (UINT32_C(0xedb88320) & mask);
        }
    }
    return crc ^ UINT32_MAX;
}

/** Check metadata relationships that are not expressible through field widths. */
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
        /* A usable sign is always exactly one of the two orientations. */
        return metadata->heading_ccw_sign == -1 || metadata->heading_ccw_sign == 1;
    }
    /* Invalid/unknown sign metadata must not carry stale confidence or sign. */
    return metadata->heading_ccw_sign == 0 &&
        metadata->heading_ccw_sign_consistency_permille == 0u;
}

/** Validate the caller's runtime angle convention before sign adaptation. */
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
    /* Initialize the entire page so CRC-independent tail bytes are deterministic
     * and resemble ordinary erased flash rather than stack contents. */
    memset(page, 0xff, MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE);
    memcpy(page, record_magic, sizeof(record_magic));
    /* Bytes 4..163 are the bounded header/payload region. The CRC is appended
     * only after the cursor proves the schema consumed exactly that region. */
    page_writer_t writer = {page, 4u, RECORD_CRC_OFFSET};
    uint8_t model_flags = RECORD_MODEL_FIT_OK |
        (model->fixed_ready ? RECORD_MODEL_FIXED_READY : 0u);
    uint8_t metadata_flags = metadata->heading_ccw_sign_valid ?
        RECORD_METADATA_SIGN_VALID : 0u;
    /* Fixed header: version, record length, feature flags, reserved word. */
    if (!put_u16(&writer, MAGNETOMETER_CALIBRATION_FLASH_FORMAT_VERSION) ||
        !put_u16(&writer, RECORD_SIZE) || !put_u16(&writer, 0u) ||
        !put_u16(&writer, 0u)) {
        return false;
    }
#define PUT_FLOAT_ARRAY(array, count) \
    do { for (size_t i = 0; i < (count); ++i) { \
        if (!put_float(&writer, (array)[i])) return false; \
    } } while (0)
    /* Store both explanatory fit components and the precomposed runtime map.
     * The former support diagnostics; the latter avoids recomputation at boot. */
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
    /* Fixed-point fields let constrained missions run without rebuilding Q15
     * coefficients from floats. */
    for (size_t i = 0; i < 3u; ++i) if (!put_i32(&writer, model->reference[i])) return false;
    for (size_t row = 0; row < 2u; ++row)
        for (size_t col = 0; col < 3u; ++col)
            if (!put_i32(&writer, model->affine_q15[row][col])) return false;
    for (size_t i = 0; i < 2u; ++i) if (!put_i32(&writer, model->bias_q2[i])) return false;
    /* Zero is reserved for unknown because directly casting -1 to uint8_t
     * would couple the wire representation to two's-complement conventions. */
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
    /* CRC covers every meaningful byte except the CRC field itself. */
    uint32_t crc = record_crc32(page, RECORD_CRC_OFFSET);
    writer.limit = RECORD_SIZE;
    if (!put_u32(&writer, crc) || writer.pos != RECORD_SIZE) {
        return false;
    }
    *record_crc = crc;
    return true;
}

/** Shared validation/encoding step used before the writer touches flash. */
magnetometer_calibration_flash_status_t
magnetometer_calibration_flash_internal_prepare_page(
    const magnetometer_heading_detection_t *hd,
    const magnetometer_calibration_metadata_t *metadata,
    uint8_t page[MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE],
    uint32_t *record_crc) {
    if (!detector_chirality_is_valid(hd) || metadata == NULL || record_crc == NULL)
        return MAGNETOMETER_CALIBRATION_FLASH_INVALID_ARGUMENT;
    if (!magnetometer_heading_model_is_valid(&hd->model))
        return MAGNETOMETER_CALIBRATION_FLASH_INVALID_MODEL;
    if (!metadata_is_valid(metadata) || metadata->bins_used != hd->model.n_bins_used)
        return MAGNETOMETER_CALIBRATION_FLASH_INVALID_METADATA;
    /* Persist steering handedness in the model's canonical CW convention.
     * Multiplication by chirality is its own inverse because chirality is ±1. */
    magnetometer_calibration_metadata_t canonical_metadata = *metadata;
    if (canonical_metadata.heading_ccw_sign_valid) {
        canonical_metadata.heading_ccw_sign = (int8_t)(
            canonical_metadata.heading_ccw_sign * (int)hd->chirality);
    }
    if (!encode_page(page, &hd->model, &canonical_metadata, record_crc))
        return MAGNETOMETER_CALIBRATION_FLASH_INVALID_ARGUMENT;
    return MAGNETOMETER_CALIBRATION_FLASH_OK;
}

static magnetometer_calibration_flash_status_t decode_page(
    const uint8_t page[MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE],
    magnetometer_heading_model_t *model,
    magnetometer_calibration_metadata_t *metadata) {
    /* An allocated file should not normally contain an erased payload, but this
     * distinction gives a precise error after an interrupted/partial write. */
    bool erased = true;
    for (size_t i = 0; i < sizeof(record_magic); ++i) erased = erased && page[i] == 0xffu;
    if (erased) return MAGNETOMETER_CALIBRATION_FLASH_EMPTY;
    if (memcmp(page, record_magic, sizeof(record_magic)) != 0)
        return MAGNETOMETER_CALIBRATION_FLASH_BAD_MAGIC;

    /* The payload cursor is bounded by the declared compile-time record size,
     * never by untrusted bytes from the page itself. */
    page_reader_t reader = {page, 4u, RECORD_SIZE};
    uint16_t version;      /* PMAG schema version. */
    uint16_t length;       /* Meaningful bytes including trailing CRC. */
    uint16_t header_flags; /* Reserved feature word; currently must be zero. */
    uint16_t reserved;     /* Detects incompatible writers/layout damage. */
    if (!get_u16(&reader, &version)) return MAGNETOMETER_CALIBRATION_FLASH_BAD_LENGTH;
    if (version != MAGNETOMETER_CALIBRATION_FLASH_FORMAT_VERSION)
        return MAGNETOMETER_CALIBRATION_FLASH_UNSUPPORTED_VERSION;
    if (!get_u16(&reader, &length) || length != RECORD_SIZE ||
        !get_u16(&reader, &header_flags) || !get_u16(&reader, &reserved) ||
        header_flags != 0u || reserved != 0u)
        return MAGNETOMETER_CALIBRATION_FLASH_BAD_LENGTH;
    /* Validate bytes before interpreting floats or installing any model. */
    uint32_t stored_crc;
    page_reader_t crc_reader = {page, RECORD_CRC_OFFSET, RECORD_SIZE};
    if (!get_u32(&crc_reader, &stored_crc) ||
        stored_crc != record_crc32(page, RECORD_CRC_OFFSET))
        return MAGNETOMETER_CALIBRATION_FLASH_BAD_CHECKSUM;

    /* Callers pass temporary outputs. Clear them so bools, padding, and fields
     * not directly represented on flash have deterministic initial values. */
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
    uint8_t bins;             /* Model's fitted occupied-sector count. */
    uint8_t model_flags;      /* fit_ok/fixed_ready bits. */
    uint8_t sign;             /* Portable 0/1/2 sign encoding. */
    uint8_t metadata_flags;   /* Currently only sign-valid. */
    uint8_t metadata_bins;    /* Redundant metadata/model consistency check. */
    uint8_t payload_reserved; /* Must stay zero for version 1. */
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
    /* fit_ok is required by the wire format check above; fixed_ready is an
     * optional capability and may legitimately be false. */
    model->n_bins_used = bins;
    model->fit_ok = true;
    model->fixed_ready = (model_flags & RECORD_MODEL_FIXED_READY) != 0u;
    if (sign > 2u) return MAGNETOMETER_CALIBRATION_FLASH_INVALID_METADATA;
    metadata->heading_ccw_sign = sign == 1u ? 1 : (sign == 2u ? -1 : 0);
    metadata->heading_ccw_sign_valid = (metadata_flags & RECORD_METADATA_SIGN_VALID) != 0u;
    metadata->bins_used = metadata_bins;
    /* Zero is reserved by controller code as "no reference". Preserve a stable
     * nonzero identity even for the rare valid record whose CRC equals zero. */
    metadata->calibration_id = stored_crc != 0u ? stored_crc : 1u;
    if (!magnetometer_heading_model_is_valid(model))
        return MAGNETOMETER_CALIBRATION_FLASH_INVALID_MODEL;
    if (!metadata_is_valid(metadata) || metadata->bins_used != bins)
        return MAGNETOMETER_CALIBRATION_FLASH_INVALID_METADATA;
    return MAGNETOMETER_CALIBRATION_FLASH_OK;
}

magnetometer_calibration_flash_status_t magnetometer_calibration_flash_load(
    magnetometer_heading_detection_t *hd,
    magnetometer_calibration_metadata_t *metadata) {
    if (!detector_chirality_is_valid(hd))
        return MAGNETOMETER_CALIBRATION_FLASH_INVALID_ARGUMENT;
    uint8_t page[MAGNETOMETER_CALIBRATION_FLASH_PAGE_SIZE];
    /* Decode into temporaries so every failure leaves the live detector and
     * optional caller metadata unchanged. */
    magnetometer_heading_model_t model;
    magnetometer_calibration_metadata_t decoded;
    pogo_flash_file_info_t file_info;
    /* Fast outer access is sufficient because PMAG has its own CRC and full
     * semantic validation. This keeps mission firmware and boot latency small. */
    pogo_flash_file_status_t file_status = pogo_flash_file_read_page_fast(
        POGO_FLASH_FILE_ID_MAGNETOMETER_CALIBRATION, 0u, page, &file_info);
    if (file_status == POGO_FLASH_FILE_OK) {
        /* The catalog protects allocation interpretation. Payload decoding then
         * independently verifies its magic, length, version, and checksum. */
        if (file_info.page_count != 1u ||
            file_info.format_version != MAGNETOMETER_CALIBRATION_FLASH_FORMAT_VERSION) {
            return MAGNETOMETER_CALIBRATION_FLASH_UNSUPPORTED_VERSION;
        }
    } else if (file_status == POGO_FLASH_FILE_NOT_FOUND) {
        return MAGNETOMETER_CALIBRATION_FLASH_EMPTY;
    } else if (file_status == POGO_FLASH_FILE_UNFORMATTED) {
        /* Both erased hardware flash and zero-filled simulator flash are empty. */
        return MAGNETOMETER_CALIBRATION_FLASH_EMPTY;
    } else {
        return MAGNETOMETER_CALIBRATION_FLASH_STORAGE_ERROR;
    }
    magnetometer_calibration_flash_status_t status = decode_page(page, &model, &decoded);
    if (status != MAGNETOMETER_CALIBRATION_FLASH_OK) return status;
    /* install_model retains runtime chirality/offset/filter settings and resets
     * only the live sample/filter cache. */
    if (!magnetometer_heading_detection_install_model(hd, &model))
        return MAGNETOMETER_CALIBRATION_FLASH_INVALID_MODEL;
    if (decoded.heading_ccw_sign_valid) {
        /* Convert canonical CW metadata back to the caller's chosen convention. */
        decoded.heading_ccw_sign = (int8_t)(
            decoded.heading_ccw_sign * (int)hd->chirality);
    }
    if (metadata != NULL) *metadata = decoded;
    return MAGNETOMETER_CALIBRATION_FLASH_OK;
}

const char *magnetometer_calibration_flash_status_string(
    magnetometer_calibration_flash_status_t status) {
    /* Kept with the read-side object so mission firmware can report a precise
     * startup failure without pulling the writer into its image. */
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
    case MAGNETOMETER_CALIBRATION_FLASH_STORAGE_ERROR: return "flash-file catalog error";
    default: return "unknown magnetometer flash status";
    }
}
