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
read-only `flash_file` inventory and check that ID 1 is present and valid.

The reset is not part of normal calibration. Leaving this firmware installed
and launching it again clears the catalogs again.

Both physical and simulator example builds compile this checkout's PFFS v3
sources. The reset example verifies the resulting catalog version before
reporting success.
