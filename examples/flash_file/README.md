# Flash-file shell

This example is an interactive serial shell for the bounded PFFS v3 filesystem.
On a Pogobot, type commands in the Pogobios-style UART console. In Pogosim,
type them in the terminal that launched the simulator. `robots` shows the
available simulated robot IDs, and `use <robot_id>` switches which robot's
flash receives subsequent commands. Press Enter after each command.

```console
make -C examples/flash_file sim
./examples/flash_file/flash_file -c conf/flash_file.yaml
```

Run from the repository root. The configuration imports **and exports**
`magnetometer.pgflash`, with the same robot and wall categories as the
calibration example. Quit using ESC in the GUI so Pogosim saves the modified
archive. Back up this archive before testing destructive commands.

Commands (names are exact, case-sensitive, and contain no spaces):

| Command | Action |
| --- | --- |
| `help` | Show the command summary. |
| `ls` | List IDs 1–80 and validate catalog/file CRCs. |
| `df` | Show allocatable sector capacity, file-ID usage, and allocated payload pages. |
| `stat <id\|name>` | Show one file's metadata. |
| `cat <id\|name>` | Hex-dump all allocated bytes of an ordinary file, including erased padding; print a log's committed bytes with non-text bytes escaped. |
| `touch <id> <name\|-> [pages]` | Create a blank ordinary file (1–5632 pages, default 1); `-` means unnamed. |
| `write <id\|name> <offset> <hexbytes>` | Replace bytes within an existing **one-page ordinary file**. Example: `write notes 0 4869` stores `Hi`. |
| `mv <id\|name> <new_name\|->` | Rename the optional label; the stable numeric ID never changes. |
| `rm <id\|name>` | Remove the catalog entry; payload may remain until its sector is reused. |
| `format YES` | Recreate all 16 catalog sectors, logically deleting every PFFS file. Old payload bytes are not wiped. |

`touch` refuses a missing or damaged filesystem instead of silently formatting
it. Use `format YES` explicitly if erasure is intended. This command destroys
*all* user files, including magnetometer calibration. Likewise, renaming or
removing or renaming the file named `magnetometer_calibration` can make
calibration unavailable to mission programs, regardless of its numeric ID.
`write` verifies the current whole-file CRC before replacing the page and
rewrites its dedicated erase sector; it is not atomic across power loss.
Multi-page editing is deliberately omitted so the shell needs only a 256-byte
data buffer; the library's streaming writer supports large-file updates.
`touch` does support multi-page files.
Command lines are limited to 159 bytes; several `write` commands can fill a
page, but each wears its sector, so use `flash_log` for frequent appends.

The `ls` output includes each file's ID, name, payload format, page count,
generation, CRC, and validation result. Ordinary files use a whole-file CRC;
append-only logs validate each committed page. A malformed catalog is reported
as `FLASH_CATALOG_ERROR` rather than treating its slots as empty.

`df` first validates the catalogs. `Sector cap`, `Used`, and `Avail` count whole
4 KiB data sectors: each occupied file consumes as many contiguous sectors as
its page count requires. The 1,472 KiB physical user-flash section contains
64 KiB of catalog sectors and 1,408 KiB available for file extents. `Payload
pages` counts reserved pages, **not** bytes actually written or committed in
a log. Deleted files are considered free by the catalog even though their old
payload bytes remain until those sectors are reused. Fragmentation may prevent
a large allocation despite free sectors shown by `df`.
