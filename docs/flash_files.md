# PFFS v4 flash files

PFFS is a bounded catalog over the Pogobot SDK's 5,888 user-flash pages
(1,472 KiB). It favors direct ID reads, small firmware code, and one-page
caller buffers. It is not a general-purpose, transactional filesystem.
The [serial shell](../examples/flash_file/README.md) exposes `ls`, `df`,
`stat`, `cat`, `touch`, `write`, `mv`, `rm`, and explicit `format YES`.
Its [UART transfer tool](uart_file_transfer.md) uploads binary files and
downloads ordinary files or committed log payloads without whole-file RAM buffers.

## Layout and capacity

- Sixteen catalog pages are at relative pages 0, 16, ..., 240. Each owns its
  own 4 KiB erase sector and holds five 48-byte entries.
- Stable IDs 1–80 select slots directly. No numeric ID is reserved;
  `magnetometer_calibration` is a conventional file name. Names of up to
  32 bytes are optional labels for other files.
- File data occupy relative pages 256–5887: 352 allocatable 4 KiB sectors,
  or 1,408 KiB. Each file is one contiguous extent, aligned to a sector and
  rounded up to a whole number of sectors. Ordinary files record an exact
  length of 0–1,441,792 bytes. Their page count is `max(1,ceil(bytes/256))`,
  at most 5632 pages, and cannot change during replacement. Even an empty
  file reserves one page and owns a full sector. The final page is padded
  with `0xff` after the recorded EOF.
- The first-fit allocator can report `NO_SPACE` even when `df` shows free
  sectors if no *contiguous* run is large enough. Deletion does not compact;
  the shell's explicit `defrag YES` command can compact extents afterward.

The physical SDK user-flash base is `0x90000`. Catalog and ordinary-file
checksums use CRC-32/ISO-HDLC. Multibyte catalog fields are little-endian;
no C structure is persisted directly. The entry records ID, active/name
flags, payload format, packed extent/byte length, generation, CRC, and name.
The catalog has its own generation and CRC. Each entry stays 48 bytes:

| Entry bytes | Content |
| --- | --- |
| 0 | Stable ID |
| 1 | Active flag and name length |
| 2–3 | Payload format version |
| 4–7 | Bits 0–20: exact byte length; bits 21–29: first sector; bits 30–31: zero |
| 8–11 | File replacement generation |
| 12–15 | CRC over all file pages, including final `0xff` padding |
| 16–47 | Up to 32 name bytes |

Sector alignment frees enough address bits for the exact length without larger
catalogs, fewer IDs, shorter names, or additional reads. Page count is derived
from length; the first page is `first_sector*16`.

### Upgrading from an older catalog

Version 3 and earlier layouts are unsupported. A recognizable incompatible
catalog returns `UNSUPPORTED_FORMAT`; create and log initialization do **not**
erase it automatically. Export important files with the old shell firmware
before upgrading, then explicitly `format YES` with the new shell and upload
the saved records. Alternatively run magnetometer calibration, which performs
the reset described below. Other workflows need an explicit reset or replacement
of an old simulator flash archive. There is no automatic
migration or old-layout decoder. Exported log payloads are archives; ordinary
uploads do not reconstruct append-log page headers.

The dedicated magnetometer-calibration writer opts into destructive recovery:
on `UNSUPPORTED_FORMAT` it explicitly formats all catalogs to the current
version and creates the calibration file. No separate reset utility is needed
for that workflow. This discards every old file entry and is not a migration;
export files first if they must be retained. A format/write verification failure
is reported rather than treated as a successful calibration save.

## Reads and writes

`pogo_flash_file_read_page_fast` reads one catalog page and one requested data
page. It checks structure and page bounds but skips both CRCs. The
magnetometer loader instead looks up its name once (up to 16 catalog reads)
and directly reads its one-page payload, which has an inner CRC and semantic
validation. `pogo_flash_file_read_page_secure` checks catalog CRC and the
whole ordinary-file CRC, reading every page; its latency grows with file size.
Neither reader allocates a heap buffer. Optional name lookup scans catalogs.

`pogo_flash_file_create_bytes` and `pogo_flash_file_replace_bytes` accept an
exact byte count and a caller-owned input array of that length. They pad the
last page without reading beyond the input. `create_blank_bytes` needs no
input array and creates `0xff` contents, including a zero-length file.
For large files, `write_begin_create_bytes` or `write_begin_replace_bytes`,
repeated `write_page`, and `write_finish` use one 256-byte caller page buffer.
The streaming caller must pad bytes after EOF with `0xff`, including the one
page submitted for an empty file; invalid padding is rejected before that
page's erase/program. Creation publishes metadata after all pages are verified.
Replacement may change byte length within the same page-count range. It retains
the extent and erases each owned sector before rewriting its pages.

The original page-count `create`, `create_blank`, `replace`, and
`write_begin_create` APIs remain source-compatible wrappers that record
`page_count*256` bytes. `write_begin_replace` retains the existing exact length.
Page readers still return full pages; use `info.size_bytes` to enforce EOF.
The writer uses a sector occupancy map (368 bytes), two 256-byte catalog buffers,
and a 256-byte verification buffer rather than a file-sized workspace.

Catalog edits erase only the selected catalog's 4 KiB sector. `format` erases
all 16 catalog sectors, **logically** deleting all files; it does not wipe the
data region. Data sectors are erased and verified when allocated again. The
whole-region SDK erase is not used by PFFS. A corrupt or missing catalog is
automatically formatted by create, including calibration and log creation;
readers, replacement, rename, and deletion never autoformat. The shell checks
metadata first and requires explicit `format YES` for recovery.

Mutations are synchronous and not power-fail atomic. An interrupted catalog
rewrite can lose files in that catalog; an interrupted replacement can leave
data inconsistent with the old CRC. Each erase/program is read back. A damaged
catalog or payload is reported, not silently repaired. Actual erase/program
latency and stack use still need measurement on physical robots.

Defragmentation scans catalog entries by physical location and copies each
file left into the compacted prefix using a single page buffer. It can work
even when the new extent overlaps the old one: source pages are copied in
ascending order, and each destination sector is erased just before use.
Ordinary files receive a pagewise CRC precheck; the shell preflights logs with
their per-page reader. Metadata is changed only after all copied pages verify.
One page or catalog action is serviced at a time, and the shell blocks other
commands during the pass. Because overlap may destroy old pages before the
catalog update, **power loss can still corrupt the current file**. Back up
first; this is not a transactional repair or a secure erase.

## Append-only logs

`flash_log` uses reserved payload format `0x8001` and any free ID. Its
compact page header has a one-byte page index, limiting logs to 256 pages
(64 KiB allocated, at most 62,464 payload bytes). Callers choose a fixed size;
the two-stream example uses 256 pages for each newly created log. Logs use a
separate CRC on each committed page; their catalog whole-file CRC field is
unused. One 256-byte RAM cache supports append,
periodic full-page `service`, optional partial `force_flush`, and readback.
For logs, catalog `size_bytes` remains the reserved container length, including
page headers. `pogo_flash_log_committed_bytes` returns the verified committed
payload length cached by `open` and updated after successful flushes; it excludes
headers, padding, and uncommitted RAM bytes. It adds no reads or catalog writes
to a flush. The shell reports reserved and committed lengths separately, and
normal UART downloads return only committed payload.
Clearing erases every sector owned by that log; uncommitted cache bytes are lost
on reset. Creating or clearing a 64 KiB log erases and verifies 16 data sectors
synchronously (creation also updates a catalog sector), so physical timing
must be checked before doing so in a deadline-limited control step.
See the [two-stream example](../examples/flash_log/README.md).

## Application integration

The public entry point is [flash_file.h](../src/pogo-utils/flash_file.h).
Keep IDs, names and immutable page counts in application policy; the library
does not reserve calibration ID 1 or provide a POSIX file descriptor interface.
For small records, serialize only their meaningful bytes and use a byte-count
writer; calibration now records 168 bytes in one physical page. For large
files, begin a writer, provide every page in order, and finish to publish the
catalog entry. Abort clears RAM state only, not previously erased data.

Use `pogo_flash_file_check` for read-only catalog/extent validation and status
strings for diagnostics. `NOT_FOUND`, `UNFORMATTED`, `CORRUPT_CATALOG`,
`BAD_CHECKSUM`, `NO_SPACE`, and `VERIFY_FAILED` are different failure classes;
do not collapse corruption into an empty directory. CRC detects accidental
damage, not authenticity or atomicity.

Follow the [PFFS tutorial](tutorials/pffs.md) for checked record/streaming examples,
[log guide](systems/flash_logs.md) for append streams, and
[extension guide](extending.md) for new payload schemas. In a robot loop,
schedule synchronous erases and catalog edits deliberately, and serialize access
across all clients using the same flash peripheral.
