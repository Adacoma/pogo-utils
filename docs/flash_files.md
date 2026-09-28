# Bounded flash-file catalog

`flash_file` provides small named files over the Pogobot 64 KiB user-flash
section. Catalog format v2 is intentionally bounded rather than a general filesystem: this
keeps ID-based one-page reads fast and keeps writer/allocation code out of
read-only mission binaries.

For a read-only inventory of all ten stable IDs, see
`examples/flash_file/README.md`. Its example program prints catalog metadata
and checks both catalog page CRCs, including empty catalogs, before performing
a secure data CRC check for each occupied file.

## Layout

- Physical pages 0 and 1 are catalogs, sharing the reserved 4 KiB erase sector
  at pages 0 through 15.
- Each catalog contains five fixed 48-byte entries.
- Stable IDs 1 through 10 map directly to those ten slots.
- Data sectors start at physical page 16, then 32, 48, and so on through 240.
- A file owns one whole 4 KiB erase sector but uses 1 through 8 contiguous,
  complete 256-byte pages at its start. Ten files fit in the fifteen available
  data sectors.
- Names of up to 32 bytes are optional labels. The numeric ID is authoritative.

An entry stores its ID, optional name, first page, immutable page count, payload
format version, replacement generation, and CRC-32 over every byte of every
allocated data page. Catalog pages have their own CRC-32. All multibyte fields
use explicit little-endian encoding; raw C structures are never persisted.

ID 1 is reserved as `magnetometer_calibration`. IDs 2 through 10 are currently
available for other stable project-wide assignments.

## Read paths

`pogo_flash_file_read_page_fast` uses the caller's output page as temporary
catalog storage. Because the ID selects its slot directly, a successful access
performs exactly one catalog read and one data-page read. It checks magic,
format, identity, allocation bounds, and the requested page index, but skips
both CRC calculations.

`pogo_flash_file_read_page_secure` also validates the selected catalog page and
the CRC of the complete file. It reads every file page and may reread the
requested page, trading latency for detection of interrupted or corrupted
writes. Neither path uses a heap or a persistent RAM cache.

Optional name lookup and diagnostic strings live in separate source files so
ID-only callers do not need to link them.

## Write policy

The writer formats, creates, replaces, and deletes files. It is isolated in
`flash_file_write.c`; applications that only read files need not link it.

Creation selects the first free data sector. Replacement keeps the same sector
and page count; deletion clears the catalog entry and the sector may later be
reused. The writer erases the selected 4 KiB data sector before programming its
pages. It also erases the catalog sector before restoring both catalog pages
from 512 bytes of validated RAM copies. On physical robots, sector erase uses
the SDK's `spiBeginErase4` at the user-section base `0x290000`; Pogosim emulates
sector erase with page writes.

This is not copy-on-write. Loss of power during a data or catalog-sector rewrite
can destroy the prior contents, including both catalogs if interruption occurs
after their shared sector is erased. Each erase and programmed page is read back
immediately. Readers detect a missing or corrupt catalog or a payload whose
CRC no longer matches.

`pogo_flash_file_create` validates both catalogs before allocating. If either
is absent or corrupt, it automatically formats the entire 64 KiB user section,
then creates the requested file. This is deliberately destructive: files in a
partially damaged catalog, unrelated data, and orphaned data after an interrupted
catalog rewrite are all erased, even if some bytes might have been recovered.
Invalid arguments are rejected before formatting. Readers, replacement, and
deletion do not autoformat; a valid catalog retains all unrelated files.

An unformatted catalog is recognized when erased hardware flash is uniformly
`0xff`, or when a simulator provides a uniformly zeroed page. Only an entirely
`0x00` or entirely `0xff` page is accepted as blank; mixed data or a malformed
catalog remains a corruption error.

## Magnetometer use

New calibration firmware stores its existing versioned 256-byte record under
reserved ID 1. If PFFS is absent or corrupt, its first store creates a new
filesystem and calibration file automatically. Later stores replace that
one-page file and retain unrelated catalog files when the catalog is valid.
Mission firmware
uses the fast outer read because the inner magnetometer record already has its
own CRC and semantic validation.

Old PFFS v1 catalogs are not compatible with sector-isolated v2. Creating a
calibration file over one will now erase all v1 user files automatically. The
separate `examples/flash_file_format` remains available for an explicit reset.

Pogosim v0.10.10 can expose uninitialized allocator contents in a new robot's
flash array. The same create-time recovery policy now formats such contents
in both simulator and physical builds; read-only firmware never formats it.

Only the catalog layout is supported. A calibration record written directly to
physical page zero is rejected and must be recreated with the calibration
example.
