/**
 * @file main.c
 * @brief Explicitly erase the 64 KiB user section and create empty PFFS v2 catalogs.
 *
 * This firmware is intentionally separate from calibration: booting it is an
 * explicit recovery decision that destroys every user-flash file. It runs once,
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
    printf("# FLASH_FORMAT_READY,robot=%u,erases_all_user_files=1\n",
           (unsigned)pogobot_helper_getid());
}

void user_step(void) {
    if (mydata->attempted) return;
    mydata->attempted = true;
    /* The format primitive erases the whole user section and verifies both
     * new catalog pages before reporting success. create() can invoke it too. */
    pogo_flash_file_status_t status = pogo_flash_file_format();
    if (status == POGO_FLASH_FILE_OK) {
        /* Simulator examples can link an older installed pogo-utils library.
         * Do not report v2 success if that library actually wrote v1 catalogs. */
        static const uint8_t magic[4] = {'P', 'F', 'F', 'S'};
        uint8_t page[POGO_FLASH_FILE_PAGE_SIZE];
        for (uint8_t index = 0u; index < POGO_FLASH_FILE_CATALOG_PAGES;
             ++index) {
            read_page_flash(index, (char *)page);
            if (memcmp(page, magic, sizeof(magic)) != 0 ||
                page[4] != 2u || page[5] != index) {
                status = POGO_FLASH_FILE_VERIFY_FAILED;
                break;
            }
        }
    }
    if (status == POGO_FLASH_FILE_OK) {
        pogobot_led_setColor(0u, 25u, 0u);
        printf("# FLASH_FORMAT_DONE,robot=%u,version=2\n",
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
