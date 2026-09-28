#ifndef POGO_UTILS_TEST_SPI_H
#define POGO_UTILS_TEST_SPI_H

#include <stdint.h>

/* Physical builds use the SDK declaration; this stub lets the host test model
 * the same 4 KiB SPI erase primitive without an SDK checkout. */
int spiBeginErase4(uint32_t erase_addr);

#endif
