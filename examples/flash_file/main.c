/**
 * @file main.c
 * @brief Read-only inventory of the bounded PFFS user-flash filesystem.
 *
 * Stable file IDs 1..10 select the two fixed catalog pages. The example lists
 * every occupied slot, then checks its catalog CRC and complete data CRC with
 * the secure reader. No writer or calibration routine is called.
 *
 * A valid entry can still report a validation failure: the fast lookup checks
 * structure, while the secure read detects damaged catalog or payload bytes.
 */
#include "pogobase.h"
#include "pogo-utils/flash_file.h"

#include <stdint.h>
#include <stdio.h>

/** One reusable data-page buffer per robot. Keeping it in USERDATA avoids a
 * 256-byte local array on a small embedded task stack. */
typedef struct {
    uint8_t page[POGO_FLASH_FILE_PAGE_SIZE];
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

/** Inspect one stable slot and update the summary counters.
 *
 * `find` supplies metadata even when the subsequent CRC scan fails. The secure
 * reader uses the same USERDATA page as scratch and does not retain its address.
 */
static void list_file(uint8_t id, unsigned *found, unsigned *empty,
                      unsigned *errors) {
    pogo_flash_file_info_t info;
    pogo_flash_file_status_t status = pogo_flash_file_find(id, &info);
    if (status == POGO_FLASH_FILE_NOT_FOUND) {
        ++*empty;
        return;
    }
    if (status != POGO_FLASH_FILE_OK) {
        ++*errors;
        printf("# FLASH_FILE_ERROR,id=%u,reason=%s\n", (unsigned)id,
               pogo_flash_file_status_string(status));
        return;
    }

    ++*found;
    /* A secure read verifies the selected catalog page and every data page.
     * file_page 0 is sufficient because the check covers the whole file. */
    pogo_flash_file_status_t validation = pogo_flash_file_read_page_secure(
        id, 0u, mydata->page, NULL);
    if (validation != POGO_FLASH_FILE_OK) ++*errors;

    const char *name = info.name_length > 0u ? info.name : "(unnamed)";
    printf("# FLASH_FILE,id=%u,name=%s,format_version=%u,first_page=%u,"
           "pages=%u,bytes=%u,generation=%lu,data_crc32=%08lx,validation=%s\n",
           (unsigned)info.id, name, (unsigned)info.format_version,
           (unsigned)info.first_page, (unsigned)info.page_count,
           (unsigned)info.page_count * (unsigned)POGO_FLASH_FILE_PAGE_SIZE,
           (unsigned long)info.generation, (unsigned long)info.data_crc32,
           pogo_flash_file_status_string(validation));
}

void user_init(void) {
    /* The inventory runs once at startup. No radio callbacks or periodic
     * application work are needed after the flash reads finish. */
    main_loop_hz = 1;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;

    unsigned found = 0u;
    unsigned empty = 0u;
    unsigned errors = 0u;
    printf("# FLASH_FILE_LIST_START,robot=%u,max_files=%u\n",
           (unsigned)pogobot_helper_getid(), (unsigned)POGO_FLASH_FILE_MAX_FILES);

    for (uint8_t id = 1u; id <= POGO_FLASH_FILE_MAX_FILES; ++id) {
        list_file(id, &found, &empty, &errors);
    }
    printf("# FLASH_FILE_LIST_END,robot=%u,found=%u,empty=%u,errors=%u\n",
           (unsigned)pogobot_helper_getid(), found, empty, errors);
    /* Green means every discovered file passed CRC validation. Violet marks
     * missing/corrupt catalog structure or damaged data; no flash is changed. */
    pogobot_led_setColor(errors == 0u ? 0u : 25u,
                         errors == 0u ? 25u : 0u,
                         errors == 0u ? 0u : 25u);
}

void user_step(void) {
    /* The listing is a one-shot diagnostic; the platform still requires a
     * periodic callback after user_init. */
}

int main(void) {
    pogobot_init();
    pogobot_start(user_init, user_step);
    /* Flash-state import identifies every category, including arena walls.
     * Registering their default callbacks preserves the calibration archive's
     * robot/category identities when the inventory runs in Pogosim. */
    pogobot_start(default_walls_user_init, default_walls_user_step, "walls");
    return 0;
}
