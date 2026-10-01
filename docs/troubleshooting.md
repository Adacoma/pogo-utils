# Troubleshooting

Use the matching example's README and public header first. Capture robot ID,
library/version, SDK/simulator revision, config, seed, status strings and the
first failure—not only the last LED color. The [audit](code_audit.md) is a
historical investigation; inspect current code before treating a finding as open.

## Builds and dependencies

| Symptom | Safe checks / likely explanation |
| --- | --- |
| pogobase/pogo-utils header missing | Check SDK/simulator installation and example-relative include paths; `-C examples/...` changes the working directory. |
| Undefined symbol or wrong behavior after editing source | Most simulator targets link installed libpogo-utils. Build and reinstall compiled modules, then rebuild the application; check for duplicate installations. |
| New flash writer with old reader | Modern examples compile local flash modules to keep versions matched; use current Makefile and do not mix old installed storage objects. |
| SDK page index truncation / signature mismatch | Current PFFS needs updated 16-bit flash-page API and 5,888 user pages; older SDK archives/headers are incompatible. |
| Firmware tool/include not found | Check POGO_SDK, variables.mak/common.mak, cross-compiler, and SDK build outputs. |
| Numerical make target exits successfully without producing a binary | Its missing-version-header guard can skip builds. Pass `POGOUTILS_INCLUDE_DIR=../../src` and inspect compiler/link output. |
| Strict C11 compiling a standalone optimizer implementation rejects M_PI | Some legacy modules use nonstandard math constants. The current CMake build uses GNU C extensions; link that built library or configure constants/toolchain deliberately. |
| Simulator link failure for SDL/Arrow/Box2D/etc. | Install Pogosim development dependencies and verify include/link directories and pkg-config. A CMake library build alone does not supply them. |
| Tests hang or fail in Release | Setup is present inside some assert expressions; use Debug with assertions enabled. |
| Optimizer configuration changed only in YAML | Numerical examples often use source constants; a YAML key has no effect without a parameter parser. |

Do not use a clean build that erases experiment outputs without checking what
its clean target does. Custom `SIM_CFLAGS` replacements can discard includes;
preserve required flags when adding defines. See [build instructions](getting_started.md).

## Calibration and heading

`MAG_CAL_FATAL` or violet in the calibration program means no trusted saved
result. Inspect preceding `MAG_CAL_FLASH_ERROR` / status details, not just the
catalog-error summary. Green/`MAG_CAL_STORED` means verified storage.

If a mission cannot load calibration, list the **same robot's** files with the
shell and `stat magnetometer_calibration`. Check PFFS version, payload schema,
file name/size, inner CRC, and model validity. No ID is reserved.
Deleting/renaming that file makes missions unable to find it.

In Pogosim, complete/export calibration first, then import with matching
categories/IDs. Different robot counts/categories can restore different state.
The supplied ACU calibration/mission archive paths currently differ; align them.
An archive saved for wall objects as well as robots does not mean every object
contains a calibration record.

A heading is a calibrated planar reference, not geographic north. Tilt,
changed mounting, nearby magnets/metal and motor interference may invalidate it.
Check sign, chirality and offset before retuning PID.
A valid cached value still needs its original timestamp and age test.
Never “fix” freshness by stamping now on an old angle.

Photosensor startup can wait if the config's light object/start flash remains
commented out. Weak gradients can yield finite angles; readiness must include
application quality policy.

## Motion and violet LEDs

LED meaning is application-specific. The calibration app's violet is terminal;
PID demo violet can mean temporary unavailable; richer missions distinguish
startup failure from post-start reset/refill recovery.
A stationary robot is not automatically a latched software deadlock.

Inspect coordinator behavior, planner fault/reason, IR face observations, sample
age/reference, heading progress, motor calibration and steering headroom.
Continue stepping with invalid samples during read outages so watchdogs advance.
Do not repeatedly cancel the maneuver with explicit STOP unless that is intended.
Config setters are not per-tick tuning calls; they stop/reset targets and retain
latched faults. Recovery needs explicit reset plus a trustworthy new window.

At normalized forward speed 1 there is no symmetric steering headroom.
Active walls require their IR beacon program; passive physical walls are not
automatically sensed by these examples. IR visibility is not distance/contact.
A physically blocked robot may not pivot; Vicsek-U-turns includes bounded
reverse/pivot yielding but cannot guarantee measured escape.

ACU/Vicsek groups can curve/loop through heading coupling even when a lone robot
goes straight. Compare gain/noise/dt/neighbor expiry/wall-event settings and
packet rates; do not assume a group shares an identical static target.
ACU compiled defaults (beta/noise/speed 3/1/0.40) differ from the supplied
ACU scenario (9/0/0.40); hardware does not read simulator YAML.
VK/AC/VU message tags do not interoperate.
See [motion guide](systems/motion_and_avoidance.md).

## Flash and logs

Readers report absent/corrupt catalogs without changing them.
Library creation automatically formats absent/corrupt catalogs by policy:
**access to other files is lost**, including recoverable ones.
The shell instead requires explicit `format YES`. Back up before recovery.
Format/deletion is logical removal, not secure wiping; replacement/defrag are
non-transactional under power loss.

NO_SPACE can mean fragmentation or exhausted ID slots despite free bytes.
Use validated `df`/extent metadata. Defrag can restore contiguity but risks
overlap corruption if interrupted; maintain power and backups.

`FLASH_LOG_WARN,status=7` is FULL. A restarted log resumes old committed data,
so it can fill immediately. Default 64 KiB applies to newly created logs only.
Clear deliberately with the log API/example switch or remove/recreate it;
do not format the entire filesystem just to empty a log.
NEEDS_SERVICE is cache backpressure: retain the unaccepted suffix.
Partial pages remain in RAM until full service or explicit force flush.
A torn page fails closed until clear; do not overwrite a programmed NOR page.

TIME OVERFLOW during erase/fit/programming means the operation exceeded the
chosen step deadline. One-page service bounds work quantity, not hardware latency.
Move initialization/clear/fitting outside high-frequency motion ticks and measure
actual worst cases. See [PFFS](flash_files.md) and [logs](systems/flash_logs.md).

## Numerical and learning results

Wrong model outputs: check quantization, genome layout, dimensions/activation
macros, exporter metadata and recurrent reset. Distributed MNIST's exporter
basis default differs from the C default. Benchmarks with random parameters
do not report trained accuracy; exported-subset accuracy is not full MNIST accuracy.

Optimizer asks returning NULL: check initial fitness, readiness and whether a
candidate already awaits tell. Do not fabricate fitness just to unlock the state.
Facade allocation failure needs individual slot sizing, not merely a bigger
total arena. Default allocator largest slot is 192 bytes; the optim example
currently passes five entries from its six-element custom class table, leaving
4096 disabled. Documented defaults/union overrides require deliberate config.

Fixed-point saturation does not protect an earlier signed overflow.
Check finite conversion input, negative shifts, nonlinear domains, accumulator
width and table initialization. Current Q16.16 add/sub/abs hardening is not a
claim that all arithmetic paths are portable.
SSR invalid estimates require inspecting neighbors, fit points/MSE, kernel/tau
and phase motion before changing classification thresholds.
