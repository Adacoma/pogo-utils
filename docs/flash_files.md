# PFFS v3 flash files

PFFS is a bounded catalog over the Pogobot SDK's 5,888 user-flash pages
(1,472 KiB). It favors direct ID reads, small firmware code, and one-page
caller buffers. It is not a general-purpose, transactional filesystem.
The [serial shell](../examples/flash_file/README.md) exposes `ls`, `df`,
`stat`, `cat`, `touch`, `write`, `mv`, `rm`, and explicit `format YES`.

## Layout and capacity

- Sixteen catalog pages are at relative pages 0, 16, ..., 240. Each owns its
  own 4 KiB erase sector and holds five 48-byte entries.
- Stable IDs 1–80 select slots directly. No numeric ID is reserved;
  `magnetometer_calibration` is a conventional file name. Names of up to
  32 bytes are optional labels for other files.
- File data occupy relative pages 256–5887: 352 allocatable 4 KiB sectors,
  or 1,408 KiB. Each file is one contiguous extent, aligned to a sector and
  rounded up to a whole number of sectors. A file can contain 1–5632 pages
  (256 bytes each), with an immutable page count.
- The first-fit allocator can report `NO_SPACE` even when `df` shows free
  sectors if no *contiguous* run is large enough. Deletion does not compact;
  the shell's explicit `defrag YES` command can compact extents afterward.

The physical SDK user-flash base is `0x90000`. Catalog and ordinary-file
checksums use CRC-32/ISO-HDLC. Multibyte catalog fields are little-endian;
no C structure is persisted directly. The entry records ID, active/name
flags, payload format, 16-bit first page and page count, generation, CRC, and
name. The catalog has its own generation and CRC. Version 2 and earlier PFFS
catalogs are unsupported; a create operation encountering one reformats the
catalogs and loses access to old files. Recalibrate robots after migration.

## Reads and writes

`pogo_flash_file_read_page_fast` reads one catalog page and one requested data
page. It checks structure and page bounds but skips both CRCs. The
magnetometer loader instead looks up its name once (up to 16 catalog reads)
and directly reads its one-page payload, which has an inner CRC and semantic
validation. `pogo_flash_file_read_page_secure` checks catalog CRC and the
whole ordinary-file CRC, reading every page; its latency grows with file size.
Neither reader allocates a heap buffer. Optional name lookup scans catalogs.

`pogo_flash_file_create` and `replace` accept a complete caller-owned data
array. For large files, `write_begin_create` or `write_begin_replace`, repeated
`write_page`, and `write_finish` use only one 256-byte caller page buffer.
Creation publishes metadata after all pages are verified. Replacement retains
the extent and size, and erases each owned sector before rewriting its pages.
`create_blank` creates erased (`0xff`) pages without an input array. The
writer uses a sector occupancy map (368 bytes), two 256-byte catalog buffers,
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
Clearing erases every sector owned by that log; uncommitted cache bytes are lost
on reset. Creating or clearing a 64 KiB log erases and verifies 16 data sectors
synchronously (creation also updates a catalog sector), so physical timing
must be checked before doing so in a deadline-limited control step.
See the [two-stream example](../examples/flash_log/README.md).
