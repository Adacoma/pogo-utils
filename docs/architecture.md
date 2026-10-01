# Architecture and scope

pogo-utils supports research controllers on Pogobot firmware and Pogosim.
It deliberately leaves scheduling, experiment policy, robot identity, message
transport, and lifecycle with the application/platform.

## Repository map

| Path | Responsibility |
| --- | --- |
| [src/pogo-utils](../src/pogo-utils/) | Installed public headers and compiled C modules; several numerical runtimes are header-only. |
| [examples](../examples/) | Complete applications, platform callbacks, Makefiles, generated model assets, selected Python tools. |
| [conf](../conf/) | Pogosim scenarios, object categories, application parameters, flash import/export paths. |
| [tests](../tests/) | Focused host tests and strict NOR/platform stubs. |
| [CMakeLists.txt](../CMakeLists.txt), [build.sh](../build.sh) | Library build, header installation, host tests, example build orchestration. |
| [docs](./) | System guides, tutorials, coverage, engineering handoff and historical audit. |
| `libs/`, dependency links | External repositories/toolchains, not library-owned code. |

The installed public interface contains 42 headers; see the exhaustive
[coverage map](examples.md#public-header-coverage). Do not include
`*_internal.h` from applications.

## Motion ownership

```text
application: acquire sensor once, timestamp, identify heading reference
    -> heading_sample_t
    -> kinematics coordinator
         -> heading PID (normal tracking)
         -> IR wall planner (turn / settle / forward commit)
         -> calibrated motors (one final motor command)
```

The magnetometer acquisition/filter object and calibration metadata belong
to the application. PID reads no sensors or clocks. The current heading-aware
wall planner reads neither sensors nor motors; it consumes snapshots and IR
observations. Kinematics arbitrates normal, pivot, reverse, stop, and escape
commands and writes motors. Applications should not also write those motors
during coordinated motion.

Older [wall avoidance](../src/pogo-utils/wall_avoidance.h) and
[light-heading avoidance](../src/pogo-utils/wall_avoidance_heading.h) modules
are separate direct-actuation paths. They are still supported by examples,
but mixing them with the coordinator creates conflicting motor ownership.

A heading's angle alone is insufficient: original acquisition timestamp,
validity, and reference ID travel together. Changing source, chirality, offset,
or calibration changes the reference. Resetting a filter after a transient
outage need not change the calibrated frame. Time differences use bounded
unsigned wraparound, not absolute comparisons over arbitrary durations.

## Calibration and persistence

Collection and fitting are separate from live heading estimation.
[magnetometer_calibration.c](../src/pogo-utils/magnetometer_calibration.c)
belongs in calibration programs; mission examples load the
`magnetometer_calibration` named PFFS file and warm the live filter.
Flash loader and store are separate compiled files so read-only missions need
not retain fitting or store code.

PFFS owns catalog/extent layout over the SDK flash API; it has separate fast
reader, secure reader, name lookup, writer, and status modules. Stable numeric
IDs resolve directly to a catalog slot; names require a scan. Append logging
builds on PFFS allocation but uses a distinct page format/checksum scheme.
No persistence module provides locks, filesystem transactions, or an OS file
descriptor.

## Numerical and collective components

Fixed/dynamic MLPs and recurrent runtimes provide inference; most do not provide
a generic learning engine. The MNIST Python exporters and ESN training helpers
are specific setup/training paths. Dense int8 accumulation still requires range
analysis and stack/RAM budgets.

Standalone optimizers borrow caller vectors/workspaces. The unified facade
owns allocations obtained from a supplied `tiny_alloc` arena. Candidate
evaluation and time budgeting remain with the application. Social learning
and HIT accept remote observations but do not implement message transport.

SSR owns diffusion, consensus, fitting, and classification state. It reports
phase/result information; the application decides motility and wall avoidance.
Vicsek, ACU, and Vicsek-U-turns are **examples**, not interchangeable public
controller modules. Their protocol tags and control laws differ.

## Platform and resource boundaries

Platform-dependent C modules include `pogobase.h` or SDK interfaces; neutral
math/control code should not gain unnecessary platform dependencies. Simulator
globals cannot safely replace per-robot `USERDATA` state. File operations are
synchronous, and initialization/erase/training may take much longer than one
regular control tick.

Static allocation avoids a general heap but does not make stack use bounded
independently of dimensions: some numerical headers use local arrays/VLAs.
Choose explicit workspaces where offered and measure final linked firmware.
The linker can discard unused sections; that does not justify claiming a
particular byte saving without measuring the actual binary.

See [extending](extending.md), [troubleshooting](troubleshooting.md), and
[current evidence](current_state.md).
