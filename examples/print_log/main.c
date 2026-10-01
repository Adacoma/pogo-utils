/** @file main.c
 * Flash-first printf-style text and CSV, with fixed-point decimal values.
 * No robot motion/calibration required. Normally stops producing after 100
 * ticks, then checkpoints across ticks. Terminal mirroring is opt-in; normal
 * logging does not call printf or use a float/double variadic argument.
 */
#include "pogobase.h"
#include "pogo-utils/print_log.h"
#include "pogo-utils/fixp_base.h"

#include <string.h>

#ifndef PRINT_LOG_TERMINAL
#define PRINT_LOG_TERMINAL 0
#endif
#ifndef PRINT_LOG_CLEAR_ON_BOOT
#define PRINT_LOG_CLEAR_ON_BOOT 0
#endif
#ifndef PRINT_LOG_TICKS
#define PRINT_LOG_TICKS 100
#endif
#if PRINT_LOG_TERMINAL
#include <stdio.h>
#endif

enum { STREAMS = 2, MESSAGE_BYTES = 96, TERMINAL_BUDGET = 16 };
static const uint8_t ids[STREAMS] = {2u, 3u};
static const char *const names[STREAMS] = {"prints", "csv"};

typedef struct {
    pogo_flash_log_t flash[STREAMS]; /* 256-byte page cache per file. */
    pogo_log_printer_t printer[STREAMS]; /* Small borrowed-storage metadata. */
    char message[STREAMS][MESSAGE_BYTES]; /* One pending message per file. */
    uint32_t tick;
    uint8_t active[STREAMS];
    uint8_t finished[STREAMS];
    uint8_t error;
} USERDATA;
DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

#if PRINT_LOG_TERMINAL
/** Explicit transport adapter; putchar may block, so the byte budget alone
 * is not a timing guarantee. No firmware printf formatter is needed here. */
static size_t terminal_write(void *context, const char *bytes, size_t length) {
    (void)context;
    size_t n = 0u;
    while (n < length && putchar((unsigned char)bytes[n]) != EOF) ++n;
    return n;
}
#endif

void user_init(void) {
    main_loop_hz = 5;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    for (unsigned i = 0; i < STREAMS; ++i) {
        /* Retain an existing matching log's size/data. Missing logs use the
         * current 64 KiB default. Creation autoformats absent/corrupt PFFS
         * catalogs: use only disposable flash for this example's first run. */
        pogo_flash_log_status_t status = pogo_flash_log_open(&mydata->flash[i], ids[i]);
        if (status == POGO_FLASH_LOG_OK) {
            pogo_flash_file_info_t info;
            if (pogo_flash_file_find(ids[i], &info) != POGO_FLASH_FILE_OK ||
                strcmp(info.name, names[i]) != 0) status = POGO_FLASH_LOG_WRONG_FORMAT;
            else if (PRINT_LOG_CLEAR_ON_BOOT)
                status = pogo_flash_log_clear(&mydata->flash[i]);
        } else {
            status = pogo_flash_log_initialize(&mydata->flash[i], ids[i], names[i],
                POGO_FLASH_LOG_DEFAULT_PAGES, PRINT_LOG_CLEAR_ON_BOOT != 0, NULL);
        }
        if (status != POGO_FLASH_LOG_OK ||
            pogo_log_attach(&mydata->printer[i], &mydata->flash[i],
                mydata->message[i], MESSAGE_BYTES) != POGO_PRINT_OK) {
            mydata->error = 1u;
            continue;
        }
#if PRINT_LOG_TERMINAL
        /* Mirror human-readable prints only; CSV still has its own file. */
        if (i == 0u) (void)pogo_log_set_terminal(&mydata->printer[i], terminal_write, NULL);
#endif
        mydata->active[i] = 1u;
    }
    pogobot_led_setColor(mydata->error ? 255 : 0, mydata->error ? 0 : 255, 0);
}

void user_step(void) {
    unsigned selected = mydata->tick % STREAMS;
#if PRINT_LOG_TICKS > 0
    bool finishing = mydata->tick >= PRINT_LOG_TICKS;
#else
    bool finishing = false; /* Compile-time endless mode, no unsigned >= 0. */
#endif
    if (mydata->active[selected] && !mydata->finished[selected]) {
        /* Only one stream services flash in this tick: at most one page
         * program total. A final checkpoint is retried across later ticks. */
        pogo_print_status_t status = finishing ?
            pogo_log_flush(&mydata->printer[selected], TERMINAL_BUDGET) :
            pogo_log_service(&mydata->printer[selected], TERMINAL_BUDGET);
        if (finishing && status == POGO_PRINT_OK) mydata->finished[selected] = 1u;
        else if (status != POGO_PRINT_OK && status != POGO_PRINT_BUSY) {
            mydata->active[selected] = 0u;
            mydata->error = 1u; /* Never recursively log a log-storage error. */
        }
    }
    if (!finishing) {
        /* Exactly represented quarter-step diagnostic values, no float
         * conversion or table setup: raw Q16.16 stores value * 65536. */
        q16_16_t value = (q16_16_t)((int32_t)(mydata->tick % 400u) * 16384 - 3276800);
        for (unsigned i = 0; i < STREAMS; ++i) {
            if (!mydata->active[i] || pogo_log_pending(&mydata->printer[i])) continue;
            pogo_print_status_t status = i == 0u ?
                pogo_log_printf(&mydata->printer[i], "tick=%lu value=%.3Q16.16\n",
                                (unsigned long)mydata->tick, value) :
                pogo_log_printf(&mydata->printer[i], "%lu,%.3Q16.16\n",
                                (unsigned long)mydata->tick, value);
            if (status != POGO_PRINT_OK) {
                mydata->active[i] = 0u;
                mydata->error = 1u;
            }
        }
    }
    if (mydata->error) pogobot_led_setColor(255, 0, 0);
    else if (mydata->finished[0] && mydata->finished[1]) pogobot_led_setColor(0, 0, 255);
    ++mydata->tick;
}

/** Standard Pogobot entry point; Pogosim maps it to each robot's main. */
int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    pogobot_start(default_walls_user_init, default_walls_user_step, "walls");
    return 0;
}
