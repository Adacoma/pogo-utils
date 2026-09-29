# Bounded flash-file catalog

`flash_file` provides small named files over the Pogobot 64 KiB user-flash
section. Catalog format v2 is intentionally bounded rather than a general filesystem: this
keeps ID-based one-page reads fast and keeps writer/allocation code out of
read-only mission binaries.

For an interactive serial shell, see `examples/flash_file/README.md`. Its `ls`
command inventories all ten stable IDs, checks both catalog-page CRCs
(including empty catalogs), and validates each occupied file's data.

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
format version, replacement generation, and (for ordinary files) CRC-32 over
every byte of every allocated data page. Catalog pages have their own CRC-32.
All multibyte fields use explicit little-endian encoding; raw C structures are
never persisted.

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
writes. Append-only logs are the exception: this reader returns
`UNSUPPORTED_FORMAT` for them, because their per-page CRCs are checked by
`flash_log`. Neither ordinary-file read path uses a heap or a persistent RAM
cache.

Optional name lookup and diagnostic strings live in separate source files so
ID-only callers do not need to link them.

## Write policy

The writer formats, creates, replaces, renames, and deletes files. It is isolated in
`flash_file_write.c`; applications that only read files need not link it.

`pogo_flash_file_create_blank` creates an all-`0xff` ordinary file without a
multi-page caller buffer. `pogo_flash_file_rename` changes only its optional
label; ID, allocation, and payload stay fixed. `pogo_flash_file_check` validates
both catalogs and all extents without formatting or checking payload bytes.
The shell calls it before `touch` so damaged metadata cannot trigger implicit
destructive recovery; `format YES` is the shell's explicit reset command.

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

## Append-only logs

`flash_log` reserves payload format version `0x8001` while keeping the PFFS v2
catalog layout and 1-to-8-page file limit. The catalog's `data_crc32` field is
unused for logs, and its generation does not advance on append or clear; a
12-byte header in each committed page stores a page index,
used-byte length, and CRC-32. A log can hold at most 8 × 244 = 1,952 data
bytes, or less if partial pages are force-flushed. IDs 2 and 3 are used by the
example for print and CSV streams; callers may select other available IDs,
except reserved calibration ID 1.

Each log handle owns a 256-byte RAM cache. Appending copies bytes and reports
how many were accepted. `service()` commits only a full page, at most one per
call, with write/readback verification and no sector erase. `force_flush()`
commits a partial page and discards its remaining space. A full file or write
failure returns a status; it never stops the application. Clearing erases only
that log's dedicated sector. Text line endings, CSV separators, and binary
record framing are caller-owned; the library stores only byte streams.
Opening an existing log scans committed pages and
resumes if they form a clean prefix. A torn or damaged page blocks new writes
until explicit clear, while earlier valid pages remain available for readback.

Initialization creates a missing log and, as for ordinary PFFS creation,
autoformats all user flash if the catalog is absent or corrupt. The caller can
receive a `formatted` report and should warn users: calibration and all other
files are erased in that case. Opening for readback never formats or writes.
See [the two-stream example](../examples/flash_log/README.md).

Physical page writes are synchronous. The example limits itself to one page
program per step, but a sub-50-ms hardware timing guarantee has not yet been
measured. Uncommitted cache bytes do not survive reset.
