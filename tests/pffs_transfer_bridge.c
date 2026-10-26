/** Host-only line bridge for integration tests. This is a strict NOR model,
 * not a Pogosim run. ! commands are test controls, absent from the real shell. */
#include "examples/flash_file/transfer.h"
#include "pogobot.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static uint8_t flash[POGO_FLASH_FILE_USER_PAGES][256];
static unsigned programs, erases;
static bool torn;
static uint32_t now;

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
    for (unsigned i = 0; i < (torn ? 8u : 256u); ++i)
        flash[page][i] &= ((const uint8_t *)data)[i];
    ++programs;
}
void read_page_flash(uint16_t page, char *output) {
    assert(page < POGO_FLASH_FILE_USER_PAGES);
    memcpy(output, flash[page], 256u);
}

int main(void) {
    erase_write_section_flash();
    assert(pogo_flash_file_format() == POGO_FLASH_FILE_OK);
    pffs_transfer_t transfer = {0};
    pogo_flash_log_t scratch = {0};
    char line[160];
    while (fgets(line, sizeof(line), stdin) != NULL) {
        line[strcspn(line, "\r\n")] = '\0';
        char *words[8]; unsigned count = 0u;
        char *word = strtok(line, " \t");
        while (word != NULL && count < 8u) {
            words[count++] = word;
            word = strtok(NULL, " \t");
        }
        if (count == 0u) continue;
        if (strcmp(words[0], "!stats") == 0) {
            printf("TEST %u %u %u\n", programs, erases, pffs_transfer_busy(&transfer));
        } else if (strcmp(words[0], "!blank") == 0) {
            erase_write_section_flash(); puts("TEST OK");
        } else if (strcmp(words[0], "!format") == 0) {
            assert(!pffs_transfer_busy(&transfer));
            assert(pogo_flash_file_format() == POGO_FLASH_FILE_OK); puts("TEST OK");
        } else if (strcmp(words[0], "!reboot") == 0) {
            memset(&transfer, 0, sizeof(transfer)); memset(&scratch, 0, sizeof(scratch));
            puts("TEST OK");
        } else if (strcmp(words[0], "!poll") == 0 && count == 2u) {
            now = (uint32_t)strtoul(words[1], NULL, 10);
            pffs_transfer_poll(&transfer, now); puts("TEST OK");
        } else if (strcmp(words[0], "!torn") == 0) {
            torn = true; puts("TEST OK");
        } else if (strcmp(words[0], "!corruptcat") == 0) {
            flash[0][0] ^= 1u; puts("TEST OK");
        } else if (strcmp(words[0], "!corrupt") == 0 && count == 2u) {
            pogo_flash_file_info_t info;
            assert(pogo_flash_file_find((uint8_t)atoi(words[1]), &info) == POGO_FLASH_FILE_OK);
            flash[info.first_page][0] ^= 1u; puts("TEST OK");
        } else if (strcmp(words[0], "!native") == 0) {
            uint8_t data[512];
            for (unsigned i = 0; i < sizeof(data); ++i) data[i] = (uint8_t)(i * 13u);
            assert(pogo_flash_file_create(71u, "native", 2u, 1u, data) == POGO_FLASH_FILE_OK);
            puts("TEST OK");
        } else if (strcmp(words[0], "!log") == 0) {
            pogo_flash_log_t log;
            size_t accepted;
            const uint8_t tail[] = {0u, 255u, 'x', '\n'};
            assert(pogo_flash_log_initialize(&log, 70u, "events", 4u, false, NULL) == POGO_FLASH_LOG_OK);
            assert(pogo_flash_log_append(&log, "alpha\n", 6u, &accepted) == POGO_FLASH_LOG_OK);
            assert(pogo_flash_log_force_flush(&log) == POGO_FLASH_LOG_OK);
            assert(pogo_flash_log_append(&log, tail, sizeof(tail), &accepted) == POGO_FLASH_LOG_OK);
            assert(pogo_flash_log_force_flush(&log) == POGO_FLASH_LOG_OK);
            puts("TEST OK");
        } else if (!pffs_transfer_command(&transfer, &scratch, words, count, now)) {
            puts("TEST UNKNOWN");
        }
        fflush(stdout);
    }
    return 0;
}
