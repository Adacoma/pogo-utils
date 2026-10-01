# Tutorial: separate print and CSV logs

Goal: build bytes cheaply in RAM and commit at most one full page per scheduled
service call, using separate fixed-size files. Start from
[examples/flash_log](../../examples/flash_log/README.md) and read
[the log system guide](../systems/flash_logs.md).

## 1. Reserve IDs, names, and storage

Choose two free IDs (the example uses 2 and 3) with names `prints` and `csv`.
A default **new** file is 256 pages / 64 KiB. Usable payload is 62,464 bytes,
not 65,536: each page has a 12-byte header. Existing logs retain their old size.

Put two `pogo_flash_log_t` handles plus any pending producer buffers in per-robot
USERDATA. Initial create/clear is synchronous and can erase 16 sectors per log;
do it at startup with motors stopped, not in a deadline-sensitive motion tick.
Creation on absent/corrupt PFFS catalogs formats metadata and loses other files.

`initialize` checks an existing name/type/size. To resume an old smaller log,
open it first and verify identity rather than requesting a new larger size.
Do not repurpose a calibration ID.

## 2. Keep the unaccepted suffix

This self-contained module shows a bounded producer for **one** stream.
Create a separate instance for prints and CSV. It can hold one 32-byte record;
a producer must wait if the previous record is pending.

```c
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "pogo-utils/flash_log.h"

typedef struct {
    pogo_flash_log_t log;
    char pending[32];
    size_t length, sent;
} stream_t;

pogo_flash_log_status_t stream_init(stream_t *s, uint8_t id, const char *name,
                                    bool *formatted) {
    memset(s, 0, sizeof(*s));
    return pogo_flash_log_initialize(&s->log, id, name,
        POGO_FLASH_LOG_DEFAULT_PAGES, false, formatted);
}

/* Enqueue RAM bytes only. False means producer backpressure, not data accepted. */
bool stream_queue(stream_t *s, const void *bytes, size_t length) {
    if (length > sizeof(s->pending) || s->sent < s->length ||
        (length != 0 && bytes == NULL)) return false;
    if (length != 0) memcpy(s->pending, bytes, length);
    s->length = length;
    s->sent = 0;
    return true;
}

/* At most one full-page flash commit; retain suffix until the next call. */
pogo_flash_log_status_t stream_service(stream_t *s) {
    pogo_flash_log_status_t st = pogo_flash_log_service(&s->log);
    if (st != POGO_FLASH_LOG_OK) return st;
    if (s->sent < s->length) {
        size_t accepted = 0;
        st = pogo_flash_log_append(&s->log, s->pending + s->sent,
                                  s->length - s->sent, &accepted);
        s->sent += accepted;
    }
    return st; /* NEEDS_SERVICE is normal; FULL/error needs caller policy. */
}

/* Integer CSV, no general printf or float conversion required. */
bool stream_queue_csv(stream_t *s, uint32_t time_ms, int32_t millivalue) {
    char row[32];
    size_t n = pogo_flash_log_format_u32(row, time_ms);
    row[n++] = ',';
    n += pogo_flash_log_format_scaled_i32(row + n, millivalue, 3);
    row[n++] = '\n';
    return stream_queue(s, row, n);
}
```

Call init for both streams, checking each status and `formatted` flag.
Queue short text including its newline into the print stream.
Alternate which stream's service is called on each tick to cap flash work.
NEEDS_SERVICE tells the producer to keep pending data; FULL needs a deliberate
stop/drop/clear policy. Do not clear automatically merely because the file filled.

This queue's refusal is visible to the caller. If records arrive faster than
service, add a bounded queue, decimate, or count drops. Do not silently discard
the suffix or repeatedly reappend the prefix. For longer records, feed chunks
using the accepted-length contract.

## 3. Understand persistence checkpoints

Ordinary service commits only full pages. Partial cached bytes are fast but
lost on reset. At an explicit checkpoint:

1. Stop adding records.
2. Continue servicing until producer pending bytes are accepted.
3. Call `pogo_flash_log_force_flush` on each stream and check success.

Force flush consumes the current page's unused payload capacity. Forcing every
tiny CSV row to flash drastically reduces log length and increases wear.
Neither flushing nor page CRC makes PFFS metadata transactions power-safe.

## 4. Read the concatenated stream

Open a read-only handle and loop with `pogo_flash_log_read_page` until END.
For each successful page consume only the `used` bytes beginning at
`POGO_FLASH_LOG_HEADER_SIZE`. Records/newlines may cross page boundaries.
Use a separate 256-byte output if the same handle will write again.

The [shell](../../examples/flash_file/README.md) recognizes logs in `cat`
and validates them with their own page reader.
The example's `FLASH_LOG_DUMP_ONLY=1` mode emits bounded text chunks.
Arbitrary binary data is not a NUL-terminated C string: do not use `%s`.
For binary records define length, byte order, schema/version and units.

## 5. Run and validate the example

```sh
make -C examples/flash_log sim
./examples/flash_log/flash_log -c conf/magnetometer.yaml
```

This changes the imported/exported archive. A `FLASH_LOG_WARN,...status=7`
indicates FULL; old committed data is retained and the program continues.
`FLASH_LOG_CLEAR_ON_BOOT=1` explicitly erases both logs.
`FLASH_LOG_FORCE_EVERY_N_TICKS` demonstrates partial checkpoints.
Rebuild after changing source switches and do not confuse YAML with compiler
definitions.

Check long-run accepted/drop counts, reset persistence, torn-page detection,
and two-stream fairness. Measure worst-case physical program/readback latency
and wear; a host simulation cannot certify a 50 ms control deadline.
