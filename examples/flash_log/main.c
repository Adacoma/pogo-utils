/**
 * @file main.c
 * @brief Two independent text/CSV flash logs with bounded per-step work.
 *
 * Set FLASH_LOG_DUMP_ONLY to 1 and rebuild after a logging run to read both
 * files without modifying flash. CLEAR_ON_BOOT is an explicit destructive
 * per-file reset; leave it zero to resume existing logs across launches.
 */
#include "pogobase.h"
#include "pogo-utils/flash_log.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef FLASH_LOG_DUMP_ONLY
#define FLASH_LOG_DUMP_ONLY 0
#endif
#ifndef FLASH_LOG_CLEAR_ON_BOOT
#define FLASH_LOG_CLEAR_ON_BOOT 0
#endif
#ifndef FLASH_LOG_FORCE_EVERY_N_TICKS
#define FLASH_LOG_FORCE_EVERY_N_TICKS 0
#endif

enum { LOG_COUNT = 2, LINE_CAPACITY = 48, DUMP_CHUNK = 32 };
static const uint8_t log_ids[LOG_COUNT] = {2u, 3u};
#if !FLASH_LOG_DUMP_ONLY
static const char *const log_names[LOG_COUNT] = {"prints", "csv"};
#endif

typedef struct {
    pogo_flash_log_t log[LOG_COUNT]; /* One 256-byte cache per independent file. */
#if FLASH_LOG_DUMP_ONLY
    /* This read-only mode reuses each handle's cache as readback scratch. */
#else
    char pending[LOG_COUNT][LINE_CAPACITY]; /* Unaccepted suffixes survive ticks. */
    uint8_t length[LOG_COUNT];
    uint8_t offset[LOG_COUNT];
#endif
    uint8_t active[LOG_COUNT];
    uint8_t warned[LOG_COUNT];
#if FLASH_LOG_DUMP_ONLY
    uint8_t dump_file;
    uint8_t dump_page;
    uint8_t dump_offset;
    uint8_t dump_used;
#else
    uint32_t tick;
#endif
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

/** Keep the warning outside the library, once per stream, so normal users
 * do not retain diagnostic strings or printf code solely to log bytes. */
static void warn_once(uint8_t stream, pogo_flash_log_status_t status) {
    if (mydata->warned[stream] != 0u) return;
    mydata->warned[stream] = 1u;
    printf("# FLASH_LOG_WARN,id=%u,status=%u\n", (unsigned)log_ids[stream],
           (unsigned)status);
}

/** Small bounded row construction; fixed-point text avoids float formatting.
 * The example's two file formats are independent byte streams. */
#if !FLASH_LOG_DUMP_ONLY
static void prepare_line(uint8_t stream) {
    char *out = mydata->pending[stream];
    size_t n = 0u;
    if (stream == 0u) {
        memcpy(out, "tick=", 5u);
        n = 5u;
    }
    n += pogo_flash_log_format_u32(out + n, mydata->tick);
    if (stream == 1u) {
        out[n++] = ',';
        n += pogo_flash_log_format_scaled_i32(out + n,
            (int32_t)(mydata->tick % 10000u), 2u);
    }
    out[n++] = '\n';
    mydata->length[stream] = (uint8_t)n;
    mydata->offset[stream] = 0u;
}
#endif

/** Read at most 32 text bytes per tick in diagnostic mode. Flash remains
 * untouched, and the bounded print avoids dumping a whole page in one step. */
#if FLASH_LOG_DUMP_ONLY
static void dump_step(void) {
    if (mydata->dump_file >= LOG_COUNT) return;
    uint8_t stream = mydata->dump_file;
    if (mydata->active[stream] == 0u) {
        ++mydata->dump_file;
        mydata->dump_page = mydata->dump_offset = mydata->dump_used = 0u;
        return;
    }
    if (mydata->dump_used == 0u) {
        pogo_flash_log_status_t status = pogo_flash_log_read_page(
            &mydata->log[stream], mydata->dump_page,
            mydata->log[stream].page,
            &mydata->dump_used);
        if (status == POGO_FLASH_LOG_END ||
            mydata->dump_page >= mydata->log[stream].page_count) {
            ++mydata->dump_file;
            mydata->dump_page = mydata->dump_offset = mydata->dump_used = 0u;
            return;
        }
        if (status != POGO_FLASH_LOG_OK) {
            warn_once(stream, status);
            ++mydata->dump_file;
            return;
        }
    }
    unsigned left = (unsigned)mydata->dump_used - mydata->dump_offset;
    unsigned count = left < DUMP_CHUNK ? left : DUMP_CHUNK;
    char text[DUMP_CHUNK + 1u];
    memcpy(text, mydata->log[stream].page + POGO_FLASH_LOG_HEADER_SIZE +
           mydata->dump_offset, count);
    text[count] = '\0';
    printf("# FLASH_LOG_DUMP,id=%u,page=%u,text=%s\n",
           (unsigned)log_ids[stream], (unsigned)mydata->dump_page, text);
    mydata->dump_offset = (uint8_t)(mydata->dump_offset + count);
    if (mydata->dump_offset == mydata->dump_used) {
        ++mydata->dump_page;
        mydata->dump_offset = mydata->dump_used = 0u;
    }
}
#endif

void user_init(void) {
    main_loop_hz = 5;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    for (uint8_t i = 0u; i < LOG_COUNT; ++i) {
        bool formatted = false;
#if FLASH_LOG_DUMP_ONLY
        pogo_flash_log_status_t status = pogo_flash_log_open(
            &mydata->log[i], log_ids[i]);
#else
        pogo_flash_log_status_t status = pogo_flash_log_initialize(
            &mydata->log[i], log_ids[i], log_names[i],
            POGO_FLASH_FILE_MAX_PAGES, FLASH_LOG_CLEAR_ON_BOOT != 0,
            &formatted);
#endif
        if (formatted) {
            printf("# FLASH_LOG_FORMATTED,id=%u,all_user_files_erased=1\n",
                   (unsigned)log_ids[i]);
        }
        if (status == POGO_FLASH_LOG_OK) {
            mydata->active[i] = 1u;
        } else {
            warn_once(i, status);
        }
    }
}

void user_step(void) {
#if FLASH_LOG_DUMP_ONLY
    dump_step();
#else
    /* At most one synchronous page program in a step. A full cache in the
     * other stream simply retains its pending suffix until its next turn. */
    uint8_t selected = (uint8_t)(mydata->tick % LOG_COUNT);
    if (mydata->active[selected] != 0u) {
#if FLASH_LOG_FORCE_EVERY_N_TICKS > 0
        /* An optional checkpoint writes a partial page instead of the normal
         * full-page service, never a second page in the same step. */
        pogo_flash_log_status_t status =
            mydata->tick != 0u &&
            mydata->tick % FLASH_LOG_FORCE_EVERY_N_TICKS == 0u ?
            pogo_flash_log_force_flush(&mydata->log[selected]) :
            pogo_flash_log_service(&mydata->log[selected]);
#else
        pogo_flash_log_status_t status = pogo_flash_log_service(
            &mydata->log[selected]);
#endif
        if (status != POGO_FLASH_LOG_OK) {
            warn_once(selected, status);
            mydata->active[selected] = 0u;
        }
    }
    for (uint8_t i = 0u; i < LOG_COUNT; ++i) {
        if (mydata->active[i] == 0u) continue;
        if (mydata->length[i] == 0u) prepare_line(i);
        size_t accepted = 0u;
        pogo_flash_log_status_t status = pogo_flash_log_append(
            &mydata->log[i], mydata->pending[i] + mydata->offset[i],
            (size_t)mydata->length[i] - mydata->offset[i], &accepted);
        mydata->offset[i] = (uint8_t)(mydata->offset[i] + accepted);
        if (mydata->offset[i] == mydata->length[i]) {
            mydata->length[i] = mydata->offset[i] = 0u;
        }
        if (status != POGO_FLASH_LOG_OK &&
            status != POGO_FLASH_LOG_NEEDS_SERVICE) {
            warn_once(i, status);
            mydata->active[i] = 0u; /* Robot itself continues normally. */
        }
    }
    ++mydata->tick;
#endif
}

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    pogobot_start(default_walls_user_init, default_walls_user_step, "walls");
    return 0;
}
