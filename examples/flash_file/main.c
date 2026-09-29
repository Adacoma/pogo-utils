/**
 * @file main.c
 * @brief Serial shell for the bounded PFFS user-flash filesystem.
 *
 * Stable file IDs 1..80 select the fixed catalog pages. The example checks
 * every catalog CRC even when its slots are empty, lists occupied slots, then
 * checks ordinary files with their whole-file CRC or logs page by page.
 * The shell receives newline-terminated commands from Pogobot UART or,
 * in Pogosim, from the simulator process's standard input. No heap is used.
 *
 * A valid entry can still report a validation failure: the fast lookup checks
 * structure, while the secure read detects damaged catalog or payload bytes.
 */
#include "pogobase.h"
#include "pogo-utils/flash_file.h"
#include "pogo-utils/flash_log.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifdef REAL_ROBOT
#include "uart.h"
#else
/* These POSIX headers are simulator-only; firmware never depends on them. */
#include <errno.h>
#include <sys/select.h>
#include <unistd.h>
#endif

/** Reuse the log handle's page cache as ordinary-file inventory scratch too.
 * This one-shot reader never appends after aliasing the cache for readback. */
typedef struct {
    pogo_flash_log_t log;
} USERDATA;

DECLARE_USERDATA(USERDATA);
REGISTER_USERDATA(USERDATA);

enum { SHELL_LINE_SIZE = 160, SHELL_MAX_WORDS = 5 };

/** Input state is process-wide in Pogosim, so exactly one selected robot owns
 * stdin at a time. On hardware there is only one robot and one UART. */
static struct {
    char line[SHELL_LINE_SIZE];
    unsigned length;
    bool overflow;
    bool after_cr;
} shell_input;

#ifndef REAL_ROBOT
enum { SHELL_MAX_SIM_ROBOTS = 32 };
static uint16_t robot_ids[SHELL_MAX_SIM_ROBOTS];
static unsigned robot_count;
static uint16_t selected_robot = UINT16_MAX;
static bool stdin_closed;
#endif

/** Check a raw catalog, including the checksum of an entirely empty page.
 *
 * File lookup intentionally skips CRC work and returns NOT_FOUND for an empty
 * slot. This inventory mirrors the documented PFFS v3 header and CRC layout
 * locally without adding diagnostic code to the mission firmware reader.
 * `scratch` is reusable USERDATA storage.
 */
static pogo_flash_file_status_t check_catalog(
    uint8_t catalog_index,
    uint8_t scratch[POGO_FLASH_FILE_PAGE_SIZE]) {
    static const uint8_t magic[4] = {'P', 'F', 'F', 'S'};
    const unsigned crc_offset = POGO_FLASH_FILE_PAGE_SIZE - sizeof(uint32_t);
    bool all_zero = true;
    bool all_erased = true;
    read_page_flash((uint16_t)catalog_index *
                    POGO_FLASH_FILE_ERASE_SECTOR_PAGES, (char *)scratch);
    for (unsigned i = 0u; i < POGO_FLASH_FILE_PAGE_SIZE; ++i) {
        all_zero = all_zero && scratch[i] == 0u;
        all_erased = all_erased && scratch[i] == 0xffu;
    }
    if (memcmp(scratch, magic, sizeof(magic)) != 0 || scratch[4] != 3u ||
        scratch[5] != catalog_index ||
        scratch[6] != POGO_FLASH_FILE_MAX_FILES / POGO_FLASH_FILE_CATALOG_PAGES ||
        scratch[7] != 0u) {
        return all_zero || all_erased ? POGO_FLASH_FILE_UNFORMATTED :
            POGO_FLASH_FILE_CORRUPT_CATALOG;
    }
    /* CRC-32/ISO-HDLC covers bytes 0..251; the stored checksum is little
     * endian at bytes 252..255. This bitwise form needs no lookup table. */
    uint32_t crc = UINT32_MAX;
    for (unsigned i = 0u; i < crc_offset; ++i) {
        crc ^= scratch[i];
        for (unsigned bit = 0u; bit < 8u; ++bit) {
            uint32_t mask = (uint32_t)-(int32_t)(crc & 1u);
            crc = (crc >> 1) ^ (UINT32_C(0xedb88320) & mask);
        }
    }
    crc ^= UINT32_MAX;
    uint32_t stored = (uint32_t)scratch[crc_offset] |
        ((uint32_t)scratch[crc_offset + 1u] << 8) |
        ((uint32_t)scratch[crc_offset + 2u] << 16) |
        ((uint32_t)scratch[crc_offset + 3u] << 24);
    if (crc != stored) {
        /* An empty slot can coexist with a bad page checksum. Emit only the
         * small fixed header/first-slot diagnostic needed to distinguish an
         * interrupted update from an invalidated catalog; do not dump payloads
         * or modify flash. Offsets follow the documented PFFS v3 layout. */
        uint32_t generation = (uint32_t)scratch[8] |
            ((uint32_t)scratch[9] << 8) |
            ((uint32_t)scratch[10] << 16) |
            ((uint32_t)scratch[11] << 24);
        printf("# FLASH_CATALOG_CRC,page=%u,stored=%08lx,calculated=%08lx,"
               "generation=%lu,first_id=%u,first_flags=%u\n",
               (unsigned)catalog_index, (unsigned long)stored,
               (unsigned long)crc, (unsigned long)generation,
               (unsigned)scratch[12], (unsigned)scratch[13]);
        return POGO_FLASH_FILE_CORRUPT_CATALOG;
    }
    return POGO_FLASH_FILE_OK;
}

/** Inspect one stable slot and update the summary counters.
 *
 * `find` supplies metadata even when the subsequent CRC scan fails. Ordinary
 * secure reads and log reads reuse the same USERDATA page as scratch.
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
    if (info.format_version == POGO_FLASH_LOG_FORMAT_VERSION) {
        /* A log's catalog CRC covers metadata, not its changing data extent.
         * Its dedicated reader checks each committed page instead. */
        pogo_flash_log_status_t result = pogo_flash_log_open(&mydata->log, id);
        unsigned bytes = 0u;
        unsigned pages = 0u;
        if (result == POGO_FLASH_LOG_OK) {
            for (uint8_t i = 0u; i < info.page_count; ++i) {
                uint8_t used = 0u;
                result = pogo_flash_log_read_page(&mydata->log, i,
                                                  mydata->log.page,
                                                  &used);
                if (result == POGO_FLASH_LOG_END) {
                    result = POGO_FLASH_LOG_OK;
                    break;
                }
                if (result != POGO_FLASH_LOG_OK) break;
                bytes += used;
                ++pages;
            }
        }
        if (result != POGO_FLASH_LOG_OK) ++*errors;
        printf("# FLASH_LOG_FILE,id=%u,name=%s,first_page=%u,pages=%u,"
               "committed_pages=%u,committed_bytes=%u,validation=%s\n",
               (unsigned)id, info.name_length ? info.name : "(unnamed)",
               (unsigned)info.first_page, (unsigned)info.page_count,
               pages, bytes, result == POGO_FLASH_LOG_OK ? "ok" : "damaged");
        return;
    }
    /* A secure read verifies the selected catalog page and every data page.
     * file_page 0 is sufficient because the check covers the whole file. */
    pogo_flash_file_status_t validation = pogo_flash_file_read_page_secure(
        id, 0u, mydata->log.page, NULL);
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

/** List every slot, checking catalog CRCs before treating holes as empty. */
static void shell_list(void) {
    unsigned found = 0u;
    unsigned empty = 0u;
    unsigned errors = 0u;
    unsigned unreadable = 0u; /* IDs skipped because their catalog failed. */
    printf("# FLASH_FILE_LIST_START,robot=%u,max_files=%u\n",
           (unsigned)pogobot_helper_getid(), (unsigned)POGO_FLASH_FILE_MAX_FILES);

    for (uint8_t catalog = 0u; catalog < POGO_FLASH_FILE_CATALOG_PAGES;
         ++catalog) {
        /* The per-ID fast lookup cannot see a bad CRC on an empty slot. Check
         * each catalog first and never report its IDs as healthy empties when
         * its header or checksum is invalid. */
        pogo_flash_file_status_t status = check_catalog(
            catalog, mydata->log.page);
        if (status != POGO_FLASH_FILE_OK) {
            ++errors;
            unreadable += POGO_FLASH_FILE_MAX_FILES /
                POGO_FLASH_FILE_CATALOG_PAGES;
            printf("# FLASH_CATALOG_ERROR,page=%u,reason=%s\n",
                   (unsigned)catalog, pogo_flash_file_status_string(status));
            continue;
        }
        for (uint8_t offset = 0u;
             offset < POGO_FLASH_FILE_MAX_FILES /
                 POGO_FLASH_FILE_CATALOG_PAGES; ++offset) {
            uint8_t id = (uint8_t)(catalog *
                (POGO_FLASH_FILE_MAX_FILES / POGO_FLASH_FILE_CATALOG_PAGES) +
                offset + 1u);
            list_file(id, &found, &empty, &errors);
        }
    }
    printf("# FLASH_FILE_LIST_END,robot=%u,found=%u,empty=%u,unreadable=%u,"
           "errors=%u\n", (unsigned)pogobot_helper_getid(), found, empty,
           unreadable, errors);
    /* Green means all catalogs and every discovered file passed CRC checks.
     * Violet marks a bad catalog or damaged data; no flash is changed. */
    pogobot_led_setColor(errors == 0u ? 0u : 25u,
                         errors == 0u ? 25u : 0u,
                         errors == 0u ? 0u : 25u);
}

/** Strict decimal parser: no signs, whitespace, or wraparound are accepted. */
static bool parse_u32(const char *text, uint32_t *value) {
    if (text == NULL || *text == '\0') return false;
    uint32_t result = 0u;
    for (const char *p = text; *p != '\0'; ++p) {
        if (*p < '0' || *p > '9') return false;
        uint32_t digit = (uint32_t)(*p - '0');
        if (result > (UINT32_MAX - digit) / 10u) return false;
        result = result * 10u + digit;
    }
    *value = result;
    return true;
}

/** Resolve a stable numeric ID or an exact case-sensitive catalog name. */
static pogo_flash_file_status_t resolve_file(
    const char *word, pogo_flash_file_info_t *info) {
    uint32_t id;
    if (parse_u32(word, &id)) {
        if (id == 0u || id > POGO_FLASH_FILE_MAX_FILES) {
            return POGO_FLASH_FILE_INVALID_ARGUMENT;
        }
        return pogo_flash_file_find((uint8_t)id, info);
    }
    return pogo_flash_file_find_by_name(word, info);
}

/** Validate all catalogs and their extents before create_blank: the library's
 * create API otherwise autoformats absent/corrupt metadata without asking. */
static bool catalogs_ready(void) {
    pogo_flash_file_status_t status = pogo_flash_file_check();
    if (status == POGO_FLASH_FILE_OK) return true;
    printf("error: %s; use 'format YES' only if erasure is intended\n",
           pogo_flash_file_status_string(status));
    return false;
}

/** Report sector allocation separately from reserved payload pages. A file
 * owns every sector intersecting its contiguous, fixed-size extent. */
static void shell_df(void) {
    if (!catalogs_ready()) return;

    unsigned files_used = 0u;
    unsigned payload_pages = 0u;
    unsigned sectors_used = 0u;
    for (uint8_t id = 1u; id <= POGO_FLASH_FILE_MAX_FILES; ++id) {
        pogo_flash_file_info_t info;
        pogo_flash_file_status_t status = pogo_flash_file_find(id, &info);
        if (status == POGO_FLASH_FILE_NOT_FOUND) continue;
        if (status != POGO_FLASH_FILE_OK) {
            printf("error: ID %u: %s\n", (unsigned)id,
                   pogo_flash_file_status_string(status));
            return;
        }
        ++files_used;
        payload_pages += info.page_count;
        sectors_used += ((unsigned)info.page_count +
            POGO_FLASH_FILE_ERASE_SECTOR_PAGES - 1u) /
            POGO_FLASH_FILE_ERASE_SECTOR_PAGES;
    }

    /* SDK v3 exposes all 5888 pages. Catalogs each own a whole sector; data
     * extents may span multiple sectors, so file count is not space usage. */
    const unsigned physical_pages = POGO_FLASH_FILE_USER_PAGES;
    const unsigned sector_kib = POGO_FLASH_FILE_ERASE_SECTOR_PAGES *
        POGO_FLASH_FILE_PAGE_SIZE / 1024u;
    const unsigned physical_kib = physical_pages * POGO_FLASH_FILE_PAGE_SIZE / 1024u;
    const unsigned catalog_kib = POGO_FLASH_FILE_DATA_FIRST_PAGE *
        POGO_FLASH_FILE_PAGE_SIZE / 1024u;
    const unsigned data_sectors = (physical_pages - POGO_FLASH_FILE_DATA_FIRST_PAGE) /
        POGO_FLASH_FILE_ERASE_SECTOR_PAGES;
    const unsigned size_kib = data_sectors * sector_kib;
    const unsigned used_kib = sectors_used * sector_kib;
    const unsigned avail_kib = size_kib - used_kib;
    const unsigned use_percent = data_sectors == 0u ? 0u :
        sectors_used * 100u / data_sectors;

    puts("Filesystem  Sector cap  Used  Avail  Use%  Files  Payload pages");
    printf("pffs        %2uK         %2uK   %2uK    %3u%%  %2u/%2u  %uB\n",
           size_kib, used_kib, avail_kib, use_percent,
           files_used, (unsigned)POGO_FLASH_FILE_MAX_FILES,
           payload_pages * (unsigned)POGO_FLASH_FILE_PAGE_SIZE);
    printf("Physical: %uK user flash; %uK catalog; %uK data extents.\n",
           physical_kib, catalog_kib, size_kib);
    puts("Payload pages count reservations, not bytes written; free space may be fragmented.");
}

/** Print one file's stable identity and allocation; no payload is changed. */
static void shell_stat(const pogo_flash_file_info_t *info) {
    printf("id=%u name=%s format=%u page=%u pages=%u generation=%lu crc=%08lx\n",
           (unsigned)info->id, info->name_length ? info->name : "(unnamed)",
           (unsigned)info->format_version, (unsigned)info->first_page,
           (unsigned)info->page_count, (unsigned long)info->generation,
           (unsigned long)info->data_crc32);
}

/** Preserve readable text while escaping binary log bytes that could alter
 * terminal state or truncate downstream text capture. */
static void print_log_byte(uint8_t byte) {
    if ((byte >= 32u && byte <= 126u) || byte == '\n' || byte == '\t') {
        putchar((int)byte);
    } else {
        printf("\\x%02x", (unsigned)byte);
    }
}

/** Hexadecimal dump keeps binary files and erased (0xff) padding unambiguous.
 * For an ordinary file, one secure read checks all pages before fast reads. */
static void shell_cat(const pogo_flash_file_info_t *info) {
    if (info->format_version == POGO_FLASH_LOG_FORMAT_VERSION) {
        pogo_flash_log_status_t status = pogo_flash_log_open(&mydata->log, info->id);
        if (status != POGO_FLASH_LOG_OK) {
            printf("error: log open status %u\n", (unsigned)status);
            return;
        }
        for (uint8_t page = 0u; page < info->page_count; ++page) {
            uint8_t used = 0u;
            status = pogo_flash_log_read_page(&mydata->log, page,
                                              mydata->log.page, &used);
            if (status == POGO_FLASH_LOG_END) break;
            if (status != POGO_FLASH_LOG_OK) {
                printf("\nerror: log page %u status %u\n",
                       (unsigned)page, (unsigned)status);
                return;
            }
            for (unsigned i = 0u; i < used; ++i) {
                print_log_byte(mydata->log.page[POGO_FLASH_LOG_HEADER_SIZE + i]);
            }
        }
        putchar('\n');
        return;
    }
    pogo_flash_file_status_t status = pogo_flash_file_read_page_secure(
        info->id, 0u, mydata->log.page, NULL);
    if (status != POGO_FLASH_FILE_OK) {
        printf("error: %s\n", pogo_flash_file_status_string(status));
        return;
    }
    for (uint16_t page = 0u; page < info->page_count; ++page) {
        if (page != 0u) {
            status = pogo_flash_file_read_page_fast(info->id, page,
                                                    mydata->log.page, NULL);
            if (status != POGO_FLASH_FILE_OK) {
                printf("error: page %u: %s\n", (unsigned)page,
                       pogo_flash_file_status_string(status));
                return;
            }
        }
        for (unsigned offset = 0u; offset < POGO_FLASH_FILE_PAGE_SIZE;
             offset += 16u) {
            printf("%04x:", (unsigned)page * POGO_FLASH_FILE_PAGE_SIZE + offset);
            for (unsigned i = 0u; i < 16u; ++i) {
                printf(" %02x", (unsigned)mydata->log.page[offset + i]);
            }
            putchar('\n');
        }
    }
}

/** Turn one hex digit into its value without relying on locale or libc. */
static int hex_digit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return -1;
}

/** One-page-only edit uses the existing 256-byte USERDATA scratch buffer.
 * Multi-page replacement would require retaining up to 2 KiB through erase. */
static void shell_write(const pogo_flash_file_info_t *info,
                        const char *offset_word, const char *hex) {
    uint32_t offset;
    if (!parse_u32(offset_word, &offset) || offset >= POGO_FLASH_FILE_PAGE_SIZE ||
        info->page_count != 1u ||
        info->format_version == POGO_FLASH_LOG_FORMAT_VERSION) {
        puts("error: write needs a one-page ordinary file and offset 0..255");
        return;
    }
    size_t digits = strlen(hex);
    if (digits == 0u || (digits & 1u) != 0u ||
        digits / 2u > POGO_FLASH_FILE_PAGE_SIZE - offset) {
        puts("error: hex bytes are empty, odd-length, or exceed the page");
        return;
    }
    /* Check the entire input before reading/writing flash. */
    for (size_t i = 0u; i < digits; ++i) {
        if (hex_digit(hex[i]) < 0) {
            puts("error: write expects plain hexadecimal byte pairs");
            return;
        }
    }
    pogo_flash_file_status_t status = pogo_flash_file_read_page_secure(
        info->id, 0u, mydata->log.page, NULL);
    if (status != POGO_FLASH_FILE_OK) {
        printf("error: %s\n", pogo_flash_file_status_string(status));
        return;
    }
    for (size_t i = 0u; i < digits; i += 2u) {
        mydata->log.page[offset + i / 2u] = (uint8_t)(
            (hex_digit(hex[i]) << 4) | hex_digit(hex[i + 1u]));
    }
    status = pogo_flash_file_replace(info->id, 1u, mydata->log.page);
    printf("write: %s\n", pogo_flash_file_status_string(status));
}

/** Fixed-size, whitespace-delimited command dispatcher. Labels cannot
 * contain spaces, matching the stored exact-name lookup and small line cap. */
static void shell_execute(char *line) {
    char *words[SHELL_MAX_WORDS];
    unsigned count = 0u;
    char *p = line;
    while (*p != '\0') {
        while (*p == ' ' || *p == '\t') ++p;
        if (*p == '\0') break;
        if (count == SHELL_MAX_WORDS) {
            puts("error: too many arguments");
            return;
        }
        words[count++] = p;
        while (*p != '\0' && *p != ' ' && *p != '\t') ++p;
        if (*p != '\0') *p++ = '\0';
    }
    if (count == 0u) return;
    const char *command = words[0];
    if (strcmp(command, "help") == 0 && count == 1u) {
        puts("help | ls | df | stat <id|name> | cat <id|name> | rm <id|name>");
        puts("touch <id> <name|-> [pages] | mv <id|name> <new_name|->");
        puts("write <id|name> <offset> <hexbytes> | format YES");
#ifndef REAL_ROBOT
        puts("robots | use <robot_id>  (simulator only)");
#endif
        puts("Ordinary cat is hex; log bytes are text with binary escaped.");
        puts("write supports one-page ordinary files only.");
        return;
    }
#ifndef REAL_ROBOT
    if (strcmp(command, "robots") == 0 && count == 1u) {
        for (unsigned i = 0u; i < robot_count; ++i) {
            printf("robot=%u%s\n", (unsigned)robot_ids[i],
                   robot_ids[i] == selected_robot ? " (selected)" : "");
        }
        return;
    }
    if (strcmp(command, "use") == 0 && count == 2u) {
        uint32_t id;
        if (parse_u32(words[1], &id) && id <= UINT16_MAX) {
            for (unsigned i = 0u; i < robot_count; ++i) {
                if (robot_ids[i] == id) {
                    selected_robot = (uint16_t)id;
                    printf("selected robot %u\n", (unsigned)selected_robot);
                    return;
                }
            }
        }
        puts("error: unknown robot ID; use 'robots'");
        return;
    }
#endif
    if (strcmp(command, "ls") == 0 && count == 1u) {
        shell_list();
        return;
    }
    if (strcmp(command, "df") == 0 && count == 1u) {
        shell_df();
        return;
    }
    if (strcmp(command, "format") == 0 && count == 2u &&
        strcmp(words[1], "YES") == 0) {
        /* Formatting drops all PFFS entries, including calibration, but does
         * not physically wipe data sectors until they are reused. */
        pogo_flash_file_status_t status = pogo_flash_file_format();
        printf("format: %s (all PFFS files removed; old data not securely wiped)\n",
               pogo_flash_file_status_string(status));
        return;
    }
    if (strcmp(command, "touch") == 0 && (count == 3u || count == 4u)) {
        uint32_t id, pages = 1u;
        if (!parse_u32(words[1], &id) || id == 0u ||
            id > POGO_FLASH_FILE_MAX_FILES ||
            (count == 4u && !parse_u32(words[3], &pages)) ||
            pages == 0u || pages > POGO_FLASH_FILE_MAX_PAGES) {
            puts("error: touch needs ID 1..80 and pages 1..5632");
            return;
        }
        if (!catalogs_ready()) return;
        const char *name = strcmp(words[2], "-") == 0 ? "" : words[2];
        pogo_flash_file_status_t status = pogo_flash_file_create_blank(
            (uint8_t)id, name, (uint16_t)pages, 1u);
        printf("touch: %s\n", pogo_flash_file_status_string(status));
        return;
    }
    if ((strcmp(command, "stat") == 0 || strcmp(command, "cat") == 0 ||
         strcmp(command, "rm") == 0) && count == 2u) {
        pogo_flash_file_info_t info;
        pogo_flash_file_status_t status = resolve_file(words[1], &info);
        if (status != POGO_FLASH_FILE_OK) {
            printf("error: %s\n", pogo_flash_file_status_string(status));
            return;
        }
        if (strcmp(command, "stat") == 0) shell_stat(&info);
        else if (strcmp(command, "cat") == 0) shell_cat(&info);
        else {
            status = pogo_flash_file_delete(info.id);
            printf("rm: %s\n", pogo_flash_file_status_string(status));
        }
        return;
    }
    if (strcmp(command, "mv") == 0 && count == 3u) {
        pogo_flash_file_info_t info;
        pogo_flash_file_status_t status = resolve_file(words[1], &info);
        if (status == POGO_FLASH_FILE_OK) {
            const char *name = strcmp(words[2], "-") == 0 ? "" : words[2];
            status = pogo_flash_file_rename(info.id, name);
        }
        printf("mv: %s\n", pogo_flash_file_status_string(status));
        return;
    }
    if (strcmp(command, "write") == 0 && count == 4u) {
        pogo_flash_file_info_t info;
        pogo_flash_file_status_t status = resolve_file(words[1], &info);
        if (status == POGO_FLASH_FILE_OK) shell_write(&info, words[2], words[3]);
        else printf("error: %s\n", pogo_flash_file_status_string(status));
        return;
    }
    puts("error: unknown command or arguments; use 'help'");
}

/** Hardware terminals typically run without local echo; simulator diagnostics
 * use newline-terminated markers so Pogosim's per-robot log shows them. */
static void shell_prompt(void) {
#ifdef REAL_ROBOT
    printf("flash[%u]> ", (unsigned)pogobot_helper_getid());
#else
    printf("# FLASH_SHELL_PROMPT,robot=%u\n", (unsigned)selected_robot);
#endif
}

/** Consume a byte at a time. Overflow discards the complete command, rather
 * than executing a truncated destructive command. CRLF is one newline. */
static void shell_feed(char byte) {
    if (byte == '\n' && shell_input.after_cr) {
        shell_input.after_cr = false;
        return;
    }
    shell_input.after_cr = byte == '\r';
    if (byte == '\r' || byte == '\n') {
#ifdef REAL_ROBOT
        putchar('\n');
#endif
        if (shell_input.overflow) puts("error: command line too long");
        else {
            shell_input.line[shell_input.length] = '\0';
            shell_execute(shell_input.line);
        }
        shell_input.length = 0u;
        shell_input.overflow = false;
        shell_prompt();
        return;
    }
    if (byte == '\b' || byte == 127) {
        if (shell_input.length > 0u) {
            --shell_input.length;
#ifdef REAL_ROBOT
            fputs("\b \b", stdout);
#endif
        }
        return;
    }
    if ((unsigned char)byte < 32u && byte != '\t') {
        /* A NUL must not truncate the parser's view of a command. */
        shell_input.overflow = true;
        return;
    }
    if (shell_input.overflow) return;
    if (shell_input.length + 1u >= sizeof(shell_input.line)) {
        shell_input.overflow = true;
        return;
    }
    shell_input.line[shell_input.length++] = byte;
#ifdef REAL_ROBOT
    putchar(byte);
#endif
}

/** Disable radio work; this diagnostic shell does not drive the robot. */
void user_init(void) {
    main_loop_hz = 20;
    max_nb_processed_msg_per_tick = 0;
    msg_rx_fn = NULL;
    msg_tx_fn = NULL;
#ifndef REAL_ROBOT
    uint16_t id = pogobot_helper_getid();
    if (robot_count < SHELL_MAX_SIM_ROBOTS) {
        robot_ids[robot_count++] = id;
        if (selected_robot == UINT16_MAX) selected_robot = id;
    }
    printf("# FLASH_SHELL_READY,robot=%u,selected=%u\n",
           (unsigned)id, (unsigned)selected_robot);
#else
    printf("# FLASH_SHELL_READY,robot=%u\n", (unsigned)pogobot_helper_getid());
#endif
    puts("Type 'help' for commands. 'format YES' erases ALL user files.");
}

void user_step(void) {
#ifndef REAL_ROBOT
    /* Stdin belongs to the process, not a simulated robot. Only the selected
     * robot consumes it, so commands run against that robot's flash state. */
    if (stdin_closed || pogobot_helper_getid() != selected_robot) return;
    for (unsigned i = 0u; i < 64u; ++i) {
        fd_set input;
        FD_ZERO(&input);
        FD_SET(STDIN_FILENO, &input);
        struct timeval timeout = {0, 0};
        if (select(STDIN_FILENO + 1, &input, NULL, NULL, &timeout) <= 0) break;
        char byte;
        ssize_t read_count = read(STDIN_FILENO, &byte, 1u);
        if (read_count < 0 && (errno == EINTR || errno == EAGAIN)) break;
        if (read_count != 1) {
            stdin_closed = true;
            break;
        }
        shell_feed(byte);
        if (pogobot_helper_getid() != selected_robot) break;
    }
#endif
}

int main(void) {
    pogobot_init();
#ifdef REAL_ROBOT
    /* Bypass the timed control-loop wrapper: sector erase/format and serial
     * dumps can exceed its per-step deadline even though the shell is healthy. */
    user_init();
    shell_prompt();
    for (;;) {
        if (uart_read_nonblock()) shell_feed((char)uart_read());
    }
#else
    pogobot_start(user_init, user_step);
    /* Keep the wall category registered so imported flash archives preserve
     * the same robot/category identity layout as the calibration scenario. */
    pogobot_start(default_walls_user_init, default_walls_user_step, "walls");
#endif
    return 0;
}
