# Explicit flash-file reset

This firmware erases all 16 PFFS catalog sectors and creates
empty PFFS v3 catalogs. It is for deliberate recovery from an incompatible or
damaged catalog. **Every PFFS file becomes inaccessible.** Data-sector bytes
remain until reuse; this is not a secure wipe. It does not erase the separate
motor-calibration security registers.

Normal new-file creation now formats absent or corrupt PFFS automatically, so
this separate utility is optional and primarily useful when you want to reset
the filesystem without immediately creating a file.

For robot 23342, run this only after deciding that the existing user-flash
contents can be discarded. Compile and upload this example as a physical
Pogobot binary, launch it once, and wait for `# FLASH_FORMAT_DONE` and a green
LED. Then upload and run the current `magnetometer_calibration` binary; its
successful result is `# MAG_CAL_STORED` and a green LED. Finally, run the
`flash_file` shell, use `ls` and `stat magnetometer_calibration`, and check
that the named calibration file is present and valid. It may use any free ID;
ID 1 is not reserved.

The reset is not part of normal calibration. Leaving this firmware installed
and launching it again clears the catalogs again.

Both physical and simulator example builds compile this checkout's PFFS v3
sources. The reset example verifies the resulting catalog version before
reporting success.

## Integration reference

Deliberately recreate empty v3 catalogs without immediately creating a payload file.

Entry point: [main.c](main.c); build rules: [Makefile](Makefile).
See the [system guide](../../docs/flash_files.md) and [all examples](../../docs/examples.md).

### Build and run

From the repository root, with the library/platform dependencies installed:

```sh
make -C examples/flash_file_format sim
./examples/flash_file_format/flash_file_format -c conf/flash_file.yaml
make -C examples/flash_file_format bin
```

The last command only builds firmware; artifact:
`examples/flash_file_format/build/bin/firmware.bin`.
The simulator command starts an experiment. For SDK paths and upload procedure,
see [getting started](../../docs/getting_started.md). Rebuild/reinstall compiled
library modules before rebuilding simulator examples; changing YAML does not
change compiler-defined dimensions or source switches.

### Configuration and expected behavior

No interactive confirmation: launching this utility performs the reset. It logically deletes files, not a secure data wipe.

FLASH_FORMAT_READY then FLASH_FORMAT_DONE/version and green indicate verified reset; calibration must be recreated afterward.

### Constraints and validation

Destructive on every launch. Does not erase separate motor-calibration security registers; normal create-time autoformat makes this optional.

Check stdout/diagnostics and the relevant header contracts rather than treating
a successful build as scientific validation. No new hardware timing or trajectory
validation is claimed by this documentation; host tests cover only selected
persistence paths. See [troubleshooting](../../docs/troubleshooting.md) before
interpreting a failure.
