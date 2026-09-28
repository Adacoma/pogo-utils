#ifndef POGO_UTILS_TEST_POGOBOT_H
#define POGO_UTILS_TEST_POGOBOT_H

/* Minimal hardware API surface needed to compile the flash writer's
 * REAL_ROBOT sector-erase branch in a host-side NOR-flash test. */
#include <stdbool.h>
#include <stdint.h>

typedef struct { uint8_t unused; } message_t;
typedef struct { uint32_t unused; } time_reference_t;

void erase_write_section_flash(void);
void write_page_flash(uint8_t page, const void *data);
void read_page_flash(uint8_t page, char *buffer);

#endif
