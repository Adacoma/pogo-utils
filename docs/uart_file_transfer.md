# UART upload and download

The [flash_file shell](../examples/flash_file/README.md) can exchange binary
files with a computer through UART. Run the shell firmware on the robot first,
then close the interactive serial terminal before starting the
[Python tool](../tools/pffs_transfer.py). The tool does not boot or flash the
robot. Use an idle shell prompt with no partially typed command.

## Host commands

Python 3.8 or later is required. In your Python environment, install the
host-only serial dependency once:

```sh
python3 -m pip install pyserial
```

From the repository root, with IDs 10/11 free and the filesystem already valid:

```sh
python3 tools/pffs_transfer.py --port /dev/ttyUSB0 upload record.bin --id 10 --name record
python3 tools/pffs_transfer.py --port /dev/ttyUSB0 download record received.bin
python3 tools/pffs_transfer.py --port /dev/ttyUSB0 upload config.csv --id 11 --name config --envelope
python3 tools/pffs_transfer.py --port /dev/ttyUSB0 download config received.csv
python3 tools/pffs_transfer.py --port /dev/ttyUSB0 download prints robot-prints.txt
```

An ID can be used in place of the download name. Use `--port COM3` on Windows.
The default baud rate is 115200; set `--baud` to match your UART console.
`--timeout` defaults to 10 seconds per reply and must be below the firmware's
30-second inactivity timeout; `--retries` defaults to three retries. Large-file
CRC preflight can take longer than a small transfer, so adjust the reply
timeout if needed. Opening a serial port can reset some adapters/boards;
make sure the shell is running before starting a transfer.

Uploads are **raw by default**: the original bytes start at byte zero, without
a PFT1 header. PFFS v4 saves their exact byte length in the catalog, so normal
downloads return the original bytes without padding or a `--length` argument.
A 256-byte record occupies one page; an empty file downloads as zero bytes
while retaining one allocated page. `--envelope` optionally adds a PFT1 prefix
for readers using that schema; downloads remove it automatically. `upload --raw` remains an
explicit spelling of the default; it cannot be combined with `--envelope`.
Downloading a log returns its committed payload bytes, concatenated across
pages without log headers or padding. Cached, uncommitted log bytes cannot be
downloaded after switching firmware.

## Raw records and replacements

The default raw mode preserves mission records at byte zero and downloads stop
at catalog EOF. `--length N` is an optional prefix export:

```sh
python3 tools/pffs_transfer.py --port /dev/ttyUSB0 upload record.bin --id 10 --name record
python3 tools/pffs_transfer.py --port /dev/ttyUSB0 download record record-copy.bin --length 180
```

Raw upload stores the input at byte zero and pads the final 256-byte page with
0xff. Its PFFS payload format is 1, and its catalog stores the input length. A
raw empty file therefore allocates one erased page. `download --length N`
retains only N bytes but still validates the complete remote stream. For a
trimmed download, the reported byte count and CRC describe the saved prefix.
For a physical file image, `download --raw` exports every allocated page, including
upload/log headers and erased padding.
Raw mode still validates files; it does not bypass checks to salvage corrupt data.

For readers that require the optional 16-byte PFT1 envelope:

```sh
python3 tools/pffs_transfer.py --port /dev/ttyUSB0 upload config.csv --id 11 --name config --envelope
```

The envelope needs `ceil((16 + bytes)/256)` pages, so a 256-byte input needs
two pages. It is intended for readers that understand PFT1, including the host
downloader. Existing mission readers expecting a native record use raw uploads.

Overwriting requires explicit flags on each side:

```sh
python3 tools/pffs_transfer.py --port /dev/ttyUSB0 upload record.bin --id 10 --name record --replace
python3 tools/pffs_transfer.py --port /dev/ttyUSB0 download config received.csv --overwrite
```

Remote replacement requires the existing ID/name to match, a compatible payload
type, and the same page count: `ceil((16 + input_bytes)/256)` for enveloped
uploads or `max(1,ceil(input_bytes/256))` for raw uploads. A different byte
length is allowed only within that same page-count range. Raw replacement
retains the existing ordinary payload type; log replacement is refused.
The tool never deletes/recreates a file to resize it or renames another file.
To replace a previous PFT1 upload, retain `--envelope --replace`. Replacement
cannot switch between raw and enveloped layouts or shrink an old allocation.

A new upload is published only after all data and checksums are verified.
Replacement erases the existing file in place and is not power-fail atomic.
Disconnecting, aborting or losing power can damage that file; abort does not
restore its old contents. Catalog updates also retain PFFS's non-transactional
power-failure limitations. Back up important files first, especially calibration.

Downloads use a temporary local file. The final destination appears only after
content verification and transfer completion. Existing local files are refused
unless `--overwrite` is supplied; failures remove the temporary output. The
no-overwrite publication uses a hard link on the destination filesystem and
fails safely if that filesystem does not support it. `--overwrite` uses atomic
replacement instead.

## Storage and firmware scope

The shell requires the PFFS v4 catalog; see the [upgrade procedure](flash_files.md#upgrading-from-an-older-catalog)
before replacing firmware on a robot with older files. The shell reserves ordinary payload type
0x8002 for an optional `--envelope` upload with a 16-byte prefix:

| Byte offset | Content |
| --- | --- |
| 0..3 | ASCII PFT1 |
| 4..7 | Original byte length, unsigned little endian |
| 8..11 | CRC-32/ISO-HDLC of original content, little endian |
| 12..15 | Zero, reserved |
| 16 onward | Original bytes, then 0xff padding to the allocated page boundary |

PFFS's ordinary-file CRC still covers the entire allocation, including this
prefix/padding. Applications using normal fast reads see the prefix and must
decode it or use default raw uploads for their existing schema. Ordinary shell
`cat` dumps bytes through catalog EOF, including an optional stored prefix.
The host downloader unwraps only
type 0x8002 with a valid header/content CRC. The maximum payload is 1,441,776
bytes with the header or 1,441,792 raw bytes, subject to free contiguous sectors
and an available file ID. For an enveloped file the catalog length includes
the 16-byte header; the header's length describes the original content alone.

Transfer state lives only in this example, with no mandatory mission-library
dependency. It borrows the shell's existing 256-byte cache, keeps a 48-byte last
chunk for retries, and uses small metadata plus bounded stack scratch. There
is no whole-file firmware buffer or heap allocation. Transfer/secure-read
operations are synchronous and are intended for this diagnostic shell, not a
time-critical motion loop. Firmware size and physical UART throughput have not
been measured.

## Wire protocol and recovery

The tool uses ASCII commands below the shell's 159-byte line limit. Every
session has a fresh nonzero 32-bit hexadecimal nonce. Except for byte strings,
CRC32s/nonces are hexadecimal and lengths/offsets/IDs are decimal.

```text
put TOKEN ID NAME BYTES CRC MODE       MODE: new, replace, raw, raw-replace
putdata TOKEN OFFSET HEXBYTES CRC      Up to 48 decoded bytes
putend TOKEN
get TOKEN ID_OR_NAME MODE              MODE: file or raw
getdata TOKEN OFFSET
getend TOKEN CRC
abort TOKEN
```

`NAME=-` means unnamed. Replies begin with `PFT1 TOKEN` in both upload modes;
this wire-protocol marker is separate from the optional stored header. Begin
replies are `PUT BYTES CRC 48 SIGNATURE` or `GET BYTES CRC 48 SIGNATURE`. SIGNATURE is the
CRC32 of all begin-command words, each followed by a NUL, including the command
and nonce. The host checks it before accepting the destination/source metadata.
Chunk CRCs cover little-endian uint32 TOKEN and OFFSET followed by decoded data.
Payload chunks contain arbitrary binary bytes encoded as hexadecimal.

Uploads receive `ACK NEXT_OFFSET`. Downloads receive `DATA OFFSET HEXBYTES CRC`.
`DONE PUT BYTES CRC` and `DONE GET BYTES CRC` finish sessions. Repeated begin
requests before data, the last data request and completion requests are safe
to retry after a lost response. Changed begin arguments are refused while busy.
Bad checksums never program a page; other errors are `ERROR REASON`. This is
accidental-corruption detection, not authentication or encryption.

The shell suppresses UART echo/prompts during transfer and blocks other shell
commands, including robot switching/defrag. A new upload's data is not listed
until completion. Inactivity for 30 seconds aborts RAM state and unlocks the
shell; the tool also sends a best-effort abort on failure or interruption.
Resume across disconnect/restart is not implemented. If reconnecting finds an
active transfer, wait for timeout or issue `abort TOKEN` for its known session.

Host tests run the C engine against a strict NOR model and exercise the Python
client over pipes. They do not launch Pogosim or access a serial device. See
[host-test instructions](getting_started.md#host-tests).
