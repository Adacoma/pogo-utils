# Buffered flash logs

[flash_log.h](../../src/pogo-utils/flash_log.h) implements small append-only
byte streams inside PFFS extents. One `pogo_flash_log_t` owns one 256-byte
RAM cache and one fixed-size file. It uses no heap, printf interception, scheduler,
or automatic background flushing. Keep separate objects/files for prints and CSV.

## Format and capacity

Each 256-byte page contains a 12-byte header and up to 244 payload bytes:
magic, version, page index, used length, and CRC. The index is one byte, so
the maximum/default allocation is 256 pages = 64 KiB, with at most
62,464 useful bytes. Passing `POGO_FLASH_LOG_DEFAULT_PAGES` explicitly requests
this size for a new log. Existing smaller logs do not expand when defaults change.

PFFS reserves payload version 0x8001 for logs. Catalog structure/CRC and extent
bounds still matter, but the catalog whole-file data CRC is unused.
Use the log reader, not ordinary-file secure reading, for payload validation.
A page is programmed once, so partial flush wastes its remaining capacity
instead of rewriting it.

## Lifecycle and API

`initialize(log,id,name,pages,clear_existing,&formatted)` creates a missing
file or opens an exactly matching existing one, optionally clearing it.
It refuses an occupied ID with different name/type/size. Create-time autoformat
of missing/corrupt PFFS catalogs loses access to other files; report
`formatted` and make this policy explicit to users.

`open(log,id)` never writes; it scans at most 256 pages for a valid committed
prefix followed by erased pages. Opening a live handle discards its uncommitted
cache. It can resume a healthy old log at its original size.
`clear(log)` erases all sectors owned by that log and resets its state;
other files on a healthy catalog are retained.

`append(log,bytes,length,&accepted)` only copies into RAM. Keep the unaccepted
suffix if it returns NEEDS_SERVICE. `service(log)` writes/readback-verifies
at most one full cached page; it is a no-op for a partial page.
`force_flush(log)` explicitly commits a partial page. Serialize calls across
all handles sharing a robot's flash peripheral.

`read_page(log,index,out,&used)` CRC-checks one committed page, returning END
for an unused page. Payload starts at `POGO_FLASH_LOG_HEADER_SIZE`; concatenate
only `used` bytes, not headers or padding. Use output separate from the cache
if future appends are possible. Read-only code may reuse the cache but must
reopen before returning to write mode.

## Backpressure and failures

FULL (status 7) means the fixed allocation is exhausted, not a sensor problem.
NEEDS_SERVICE means RAM cache backpressure, not necessarily a full file.
Other statuses distinguish invalid arguments, wrong format/name, missing file,
corruption, allocation failure, readback failure, and flash error.

After a torn/invalid page, do not retry programming that NOR page in place.
The write state fails closed until explicit clear; already valid pages can
still be read through the page reader. A normal reset loses only uncommitted
RAM data. Page CRC is accidental-corruption detection, not authentication or
power-failure transaction recovery.

If data cannot be dropped, the producer needs a bounded pending queue or must
stop producing until service catches up. The library has only one page cache.
Never send logging errors back into the same full log recursively.

## Serialization and cost

Print text, CSV, and binary records are application-owned bytes. Supply newlines,
separators, headers, binary lengths/versions, and explicit byte order yourself.
Records can span pages; readers reconstruct the stream rather than expecting
one record per physical page.

Decimal helpers `format_u32`, `format_i32`, and `format_scaled_i32` write
bounded character sequences and return lengths; **they do not NUL-terminate**.
They can avoid linking general float printf formatting. Use `snprintf` only
if its firmware/code-size cost is acceptable; it is not necessary for logging.

Append is bounded RAM work. Service is synchronous flash program/readback:
“one page per call” is not a guaranteed millisecond latency. Creating/clearing
a 64 KiB log erases 16 data sectors synchronously and may be inappropriate
inside a 50 ms loop. Forced flushes lower payload density and increase wear.
Hardware latency, stack use, endurance, and producer-rate headroom require
measurement.

Follow the [two-stream tutorial](../tutorials/flash_logs.md),
[example](../../examples/flash_log/README.md), and [PFFS guide](../flash_files.md).
