# Bounded flash-file catalog

`flash_file` provides small named files over the Pogobot 64 KiB user-flash
section. It is intentionally bounded rather than a general filesystem: this
keeps ID-based one-page reads fast and keeps writer/allocation code out of
read-only mission binaries.

## Layout

- Physical pages 0 and 1 are catalogs.
- Each catalog contains five fixed 48-byte entries.
- Stable IDs 1 through 10 map directly to those ten slots.
- Data starts at physical page 2.
- A file owns 1 through 8 contiguous, complete 256-byte pages.
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

Creation uses first-fit contiguous allocation. Replacement must supply exactly
the existing page count and writes the same physical extent. Deletion clears
only the catalog entry; the abandoned pages may later be reused. Allocation is
therefore allowed to fragment and no compactor is provided.

This policy assumes `write_page_flash` can replace an allocated catalog or data
page without changing the file size. Replacement is not copy-on-write: loss of
power during a write can destroy the prior contents. Each written page is read
back immediately, and the secure reader detects a payload that no longer
matches the committed CRC.

Formatting remains explicitly destructive because the platform erase routine
erases the complete user section.

An unformatted catalog is recognized when erased hardware flash is uniformly
`0xff`, or when a simulator provides a uniformly zeroed page. Only an entirely
`0x00` or entirely `0xff` page is accepted as blank; mixed data or a malformed
catalog remains a corruption error.

## Magnetometer use

New calibration firmware stores its existing versioned 256-byte record under
reserved ID 1. The first store formats an unformatted section; later stores
replace that one-page file and retain unrelated catalog files. Mission firmware
uses the fast outer read because the inner magnetometer record already has its
own CRC and semantic validation.

Pogosim v0.10.10 can expose uninitialized allocator contents in a new robot's
flash array. The calibration writer therefore treats an unrecognized non-`PFFS`
page as first-use storage in simulator builds and formats it. A damaged page
whose magic is already `PFFS` remains an error. This workaround is excluded
from physical-robot builds, where unknown data is not erased automatically.

Only the catalog layout is supported. A calibration record written directly to
physical page zero is rejected and must be recreated with the calibration
example.
