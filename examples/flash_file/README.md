# Flash-file inventory

This example lists all ten stable flash-file IDs without changing user flash.
It expects PFFS v2 catalogs; a v1 catalog is reported as incompatible/corrupt.
For each occupied ID it prints the optional name, payload format version, first
physical page, page count and byte size, replacement generation, recorded data
CRC-32, and the result of a secure read. The secure read validates the selected
catalog page and every page of that file. Empty slots are counted in the final
summary; malformed slots and checksum failures are reported as errors. Both
catalog page CRCs are checked before listing any slots, so an empty catalog
with a bad checksum is reported as `FLASH_CATALOG_ERROR` and its five slots are
counted as `unreadable`, not `empty`. `FLASH_CATALOG_CRC` prints the stored and
calculated checksums plus the catalog generation and first slot's ID/flags when
a page header is recognizable but its checksum differs.

Run it after the four-robot magnetometer calibration example has exported
`magnetometer.pgflash`:

```console
make -C examples/flash_file sim
./examples/flash_file/flash_file -c conf/flash_file.yaml
```

The configuration imports the archive and uses the same robot IDs and wall
category as `conf/magnetometer_calibration.yaml`. Run from the repository root
so the relative archive path resolves. A successful inventory includes file ID
1 named `magnetometer_calibration`; other IDs may be present if the robots have
stored additional files. The example does not export an archive, so the input
file is left untouched.

On a physical robot, calibration succeeds only when its LED turns green and it
prints `# MAG_CAL_STORED`. A violet LED means it stopped in a fatal state; the
`# MAG_CAL_FATAL` line from that run identifies the failure. An empty inventory
after a violet calibration is therefore not evidence of a stored model.

`validation=ok` means the outer filesystem CRCs match. It does not interpret a
file's application-specific payload. The magnetometer loader separately checks
the inner PMAG record and its model values.
