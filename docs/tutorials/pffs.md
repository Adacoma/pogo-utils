# Tutorial: PFFS files

Goal: store application records with a stable ID or optional name, read one page
quickly or securely, and replace data without changing allocation.
See [flash_file.h](../../src/pogo-utils/flash_file.h) for exact signatures and
[the PFFS guide](../flash_files.md) for layout and power-failure limits.

## 1. Inspect without changing anything

```sh
make -C examples/flash_file sim
./examples/flash_file/flash_file -c conf/flash_file.yaml
```

The shell imports and exports `magnetometer.pgflash`; back it up first.
Use `robots` / `use <robot_id>` in simulation, then `ls`, `df`, and
`stat magnetometer_calibration`. Each robot has a separate filesystem.
On hardware build/upload the same example's firmware and use UART.
Read [shell commands](../../examples/flash_file/README.md).

`df` reports **allocated sectors**, not useful data bytes. A one-page file
owns a whole 4 KiB erase sector. There are 80 stable IDs and 1,408 KiB allocatable
data, but large files need contiguous free space.

## 2. Create and replace a one-page record

The following self-contained module uses ID 42 **only as an example**.
Check that your chosen ID/name is free; no numeric ID is reserved by the library.

```c
#include <stdint.h>
#include <string.h>
#include "pogo-utils/flash_file.h"

/* Wire format: four magic bytes, little-endian counter, zero-filled padding. */
pogo_flash_file_status_t save_counter(uint32_t counter, bool create) {
    uint8_t page[POGO_FLASH_FILE_PAGE_SIZE] = {0};
    memcpy(page, "CNT1", 4);
    for (unsigned i = 0; i < 4; ++i)
        page[4 + i] = (uint8_t)(counter >> (8u * i));
    if (create) {
        /* WARNING: create autoformats absent/corrupt catalogs. */
        return pogo_flash_file_create(42, "counter", 1, 1, page);
    }
    return pogo_flash_file_replace(42, 1, page); /* Same page count/extent. */
}

pogo_flash_file_status_t load_counter(uint32_t *counter) {
    if (counter == NULL) return POGO_FLASH_FILE_INVALID_ARGUMENT;
    uint8_t page[POGO_FLASH_FILE_PAGE_SIZE];
    pogo_flash_file_info_t info;
    pogo_flash_file_status_t s =
        pogo_flash_file_read_page_secure(42, 0, page, &info);
    if (s != POGO_FLASH_FILE_OK) return s;
    if (info.format_version != 1 || info.page_count != 1 ||
        memcmp(page, "CNT1", 4) != 0)
        return POGO_FLASH_FILE_UNSUPPORTED_FORMAT;
    uint32_t value = 0;
    for (unsigned i = 0; i < 4; ++i)
        value |= (uint32_t)page[4 + i] << (8u * i);
    *counter = value; /* Publish only after validation. */
    return POGO_FLASH_FILE_OK;
}
```

This serializes fields explicitly; never store native structs with implicit
padding/endianness. The format-version field describes **your** payload.
Do not use reserved log format 0x8001 for an ordinary file.

For fast access replace the secure call with
`pogo_flash_file_read_page_fast`: exactly one catalog read plus one data read,
structural bounds checked, CRCs skipped. A payload's own CRC/semantic validation
can be useful, as with magnetometer calibration. Secure ordinary reads validate
catalog and **all allocated pages**, not just the requested page.

For name resolution use `find_by_name("counter",&info)` then its ID.
Name lookup can scan 16 catalogs; cache the stable ID only while your application's
filesystem-lifecycle assumptions remain valid.

## 3. Large files without a large RAM buffer

Use the streaming writer: begin, write every page in order, then finish.
This module illustrates deterministic page generation:

```c
#include <stdint.h>
#include "pogo-utils/flash_file.h"

pogo_flash_file_status_t create_pattern(uint8_t id, uint16_t pages) {
    pogo_flash_file_writer_t writer;
    bool formatted = false;
    pogo_flash_file_status_t s = pogo_flash_file_write_begin_create(
        &writer, id, "pattern", pages, 1, &formatted);
    if (s != POGO_FLASH_FILE_OK) return s;
    /* Application should warn if formatted: other file metadata was lost. */
    uint8_t page[POGO_FLASH_FILE_PAGE_SIZE];
    for (uint16_t p = 0; p < pages; ++p) {
        for (unsigned b = 0; b < sizeof(page); ++b)
            page[b] = (uint8_t)(p + b);
        s = pogo_flash_file_write_page(&writer, page);
        if (s != POGO_FLASH_FILE_OK) {
            pogo_flash_file_write_abort(&writer); /* RAM only; no rollback. */
            return s;
        }
    }
    return pogo_flash_file_write_finish(&writer);
}
```

Metadata is published after all pages verify. For replacement begin with
`write_begin_replace`; keep the same number of pages. Old data can become
invalid on the first erase, before finish. Each write handles a page, but
sector erases/catalog updates are synchronous and may violate a control deadline.
Do not interleave another writer, rename, delete, or defrag while streaming.

## 4. Manage allocation

In the shell on disposable data, after explicitly accepting any format loss:

```text
touch 42 notes 1
write notes 0 4869
cat notes
mv notes renamed
stat renamed
rm renamed
df
```

`mv` changes the optional label, not the ID. File size is immutable:
resize means delete/recreate (and possible relocation), not replace.
`write` in the shell edits one-page ordinary files only.
To recover contiguity after deletion use `defrag YES`, backed up and under
stable power. It is incremental but **not transactional**.
`format YES` logically deletes all PFFS files, including calibration; it does
not securely wipe payload bytes.

Readers never format. Library create autoformats missing/corrupt catalogs by
design, while shell touch refuses that implicit destruction and requires explicit
format. Choose recovery policy consciously in your application.

## 5. Verify failure semantics

Check status strings rather than assuming NOT_FOUND means a corrupt catalog.
Test duplicate ID/name, illegal size, full ID table, fragmented free space,
secure read after payload corruption, and interrupted replacement.
Host PFFS/NOR tests cover many format invariants; they do not establish physical
erase latency, endurance, or power-fail recovery. Logs need
[their own reader/tutorial](flash_logs.md).
