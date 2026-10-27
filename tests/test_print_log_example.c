/** Exercise the actual example startup against strict NOR storage. Names must
 * resume existing logs, choose unused IDs and never replace unrelated files. */
#define main print_log_example_main
#include "../examples/print_log/main.c"
#undef main

#include <assert.h>
#include <stdio.h>
#include <string.h>

unsigned main_loop_hz, max_nb_processed_msg_per_tick;
void *msg_rx_fn, *msg_tx_fn;
static uint8_t flash[POGO_FLASH_FILE_USER_PAGES][POGO_FLASH_FILE_PAGE_SIZE];
static unsigned programs, erases;

/* No framework callbacks are dispatched: tests call user_init explicitly. */
void pogobot_init(void) {}
void pogobot_start(void (*init)(void), void (*step)(void), ...) { (void)init; (void)step; }
void pogobot_led_setColor(uint8_t r, uint8_t g, uint8_t b) { (void)r; (void)g; (void)b; }
void default_walls_user_init(void) {}
void default_walls_user_step(void) {}

void erase_write_section_flash(void) { memset(flash, 0xff, sizeof(flash)); }
int spiBeginErase4(uint32_t address) {
    assert(address >= POGOBOT_USER_FLASH_START_OFFSET);
    uint32_t offset = address - POGOBOT_USER_FLASH_START_OFFSET;
    assert(offset < sizeof(flash) && offset % 4096u == 0u);
    memset(flash[offset / 256u], 0xff, 4096u);
    ++erases;
    return 0;
}
void write_page_flash(uint16_t page, const void *data) {
    assert(page < POGO_FLASH_FILE_USER_PAGES);
    for (unsigned i = 0u; i < 256u; ++i)
        flash[page][i] &= ((const uint8_t *)data)[i];
    ++programs;
}
void read_page_flash(uint16_t page, char *output) {
    assert(page < POGO_FLASH_FILE_USER_PAGES);
    memcpy(output, flash[page], 256u);
}

static void boot(void) {
    memset(mydata, 0, sizeof(*mydata));
    user_init();
}

static void ordinary_file(uint8_t id, const char *name) {
    const uint8_t data = 0x5au;
    assert(pogo_flash_file_create_bytes(id, name, 1u, 1u, &data) == POGO_FLASH_FILE_OK);
}

static void expect_ordinary(uint8_t id) {
    uint8_t page[256];
    pogo_flash_file_info_t info;
    assert(pogo_flash_file_read_page_secure(id, 0u, page, &info) == POGO_FLASH_FILE_OK);
    assert(info.size_bytes == 1u && info.format_version == 1u && page[0] == 0x5au);
}

int main(void) {
    /* Virgin flash is recovered before opening either handle. */
    erase_write_section_flash();
    boot();
    assert(!mydata->error && mydata->active[0] && mydata->active[1]);
    assert(mydata->flash[0].file_id == 1u && mydata->flash[1].file_id == 2u);
    assert(mydata->flash[0].page_count == POGO_FLASH_LOG_DEFAULT_PAGES);
    assert(mydata->flash[1].page_count == POGO_FLASH_LOG_DEFAULT_PAGES);

    /* Former reserved IDs may all be occupied by ordinary files. */
    assert(pogo_flash_file_format() == POGO_FLASH_FILE_OK);
    ordinary_file(1u, "one"); ordinary_file(2u, "two"); ordinary_file(3u, "three");
    boot();
    assert(!mydata->error && mydata->flash[0].file_id == 4u && mydata->flash[1].file_id == 5u);
    for (uint8_t id = 1u; id <= 3u; ++id) expect_ordinary(id);

    /* Existing names win over low free IDs and keep nondefault allocations.
     * CLEAR_ON_BOOT follows the same selection, without changing file size. */
    assert(pogo_flash_file_format() == POGO_FLASH_FILE_OK);
    pogo_flash_log_t log;
    assert(pogo_flash_log_initialize(&log, 17u, "prints", 1u, false, NULL) == POGO_FLASH_LOG_OK);
    size_t accepted;
    assert(pogo_flash_log_append(&log, "saved", 5u, &accepted) == POGO_FLASH_LOG_OK && accepted == 5u);
    assert(pogo_flash_log_force_flush(&log) == POGO_FLASH_LOG_OK);
    assert(pogo_flash_log_initialize(&log, 79u, "csv", 3u, false, NULL) == POGO_FLASH_LOG_OK);
    unsigned before_programs = programs, before_erases = erases;
    boot();
    assert(!mydata->error && mydata->flash[0].file_id == 17u && mydata->flash[1].file_id == 79u);
    assert(mydata->flash[0].page_count == 1u && mydata->flash[1].page_count == 3u);
    assert(pogo_flash_log_committed_bytes(&mydata->flash[0]) == (PRINT_LOG_CLEAR_ON_BOOT ? 0u : 5u));
    if (!PRINT_LOG_CLEAR_ON_BOOT) assert(programs == before_programs && erases == before_erases);

    /* A colliding ordinary name is an error even when clearing was requested. */
    assert(pogo_flash_file_format() == POGO_FLASH_FILE_OK);
    ordinary_file(17u, "prints");
    assert(pogo_flash_log_initialize(&log, 79u, "csv", 1u, false, NULL) == POGO_FLASH_LOG_OK);
    boot();
    assert(mydata->error && !mydata->active[0] && mydata->active[1]);
    expect_ordinary(17u);

    /* A full ID table fails without erasing or replacing any occupied slot. */
    assert(pogo_flash_file_format() == POGO_FLASH_FILE_OK);
    for (uint8_t id = 1u; id <= POGO_FLASH_FILE_MAX_FILES; ++id)
        assert(pogo_flash_file_create_blank_bytes(id, NULL, 0u, 1u) == POGO_FLASH_FILE_OK);
    before_programs = programs; before_erases = erases;
    boot();
    assert(mydata->error && !mydata->active[0] && !mydata->active[1]);
    assert(programs == before_programs && erases == before_erases);

    /* Current corrupt metadata retains the prior destructive recovery policy. */
    flash[0][255] ^= 1u;
    boot();
    assert(!mydata->error && mydata->active[0] && mydata->active[1]);
    assert(pogo_flash_file_check() == POGO_FLASH_FILE_OK);

    /* Incompatible catalogs are still refused by this example, without writes. */
    flash[0][4] = 3u;
    before_programs = programs; before_erases = erases;
    boot();
    assert(mydata->error && !mydata->active[0] && !mydata->active[1]);
    assert(programs == before_programs && erases == before_erases && flash[0][4] == 3u);
    puts("print_log example name allocation tests passed");
    return 0;
}
