/**
 * @file main.c
 * @brief Remove PFFS v3 files by resetting its catalog sectors.
 *
 * This firmware is intentionally separate from calibration: booting it is an
 * explicit recovery decision that removes every PFFS file. It runs once,
 * verifies the newly written catalog pages through the writer, and stops.
 */
#include "pogobase.h"
#include "pogo-utils/flash_file.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/** State must be per robot in Pogosim; a process-global flag would make only
 * the first simulated robot format its flash. */
typedef struct {
    bool attempted;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

void user_init(void) {
    main_loop_hz = 1;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
    mydata->attempted = false;
    /* The reset utility never commands motion, even if a previous image left
     * motor PWM registers active across a warm start. */
    pogobot_motor_set(motorL, motorStop);
    pogobot_motor_set(motorR, motorStop);
    pogobot_motor_set(motorB, motorStop);
    pogobot_led_setColor(25u, 8u, 0u);
    printf("# FLASH_FORMAT_READY,robot=%u,clears_all_pffs_files=1\n",
           (unsigned)pogobot_helper_getid());
}

void user_step(void) {
    if (mydata->attempted) return;
    mydata->attempted = true;
    /* Only catalog sectors are erased; payload sectors remain physically
     * untouched until a later file allocation reuses them. */
    pogo_flash_file_status_t status = pogo_flash_file_format();
    if (status == POGO_FLASH_FILE_OK) {
        /* Check every v3 catalog identity before reporting format success. */
        static const uint8_t magic[4] = {'P', 'F', 'F', 'S'};
        uint8_t page[POGO_FLASH_FILE_PAGE_SIZE];
        for (uint8_t index = 0u; index < POGO_FLASH_FILE_CATALOG_PAGES;
             ++index) {
            read_page_flash((uint16_t)index *
                            POGO_FLASH_FILE_ERASE_SECTOR_PAGES, (char *)page);
            if (memcmp(page, magic, sizeof(magic)) != 0 ||
                page[4] != 3u || page[5] != index) {
                status = POGO_FLASH_FILE_VERIFY_FAILED;
                break;
            }
        }
    }
    if (status == POGO_FLASH_FILE_OK) {
        pogobot_led_setColor(0u, 25u, 0u);
        printf("# FLASH_FORMAT_DONE,robot=%u,version=3\n",
               (unsigned)pogobot_helper_getid());
    } else {
        pogobot_led_setColor(25u, 0u, 25u);
        printf("# FLASH_FORMAT_FAILED,robot=%u,reason=%s\n",
               (unsigned)pogobot_helper_getid(),
               pogo_flash_file_status_string(status));
    }
}

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    return 0;
}
