# Getting started

Run the commands here from the repository root unless stated otherwise.
Build commands compile programs; commands starting example executables actually
run Pogosim or hardware code. Flash-writing examples can alter persistent data.

## Prerequisites

For the library: CMake 3.16+, a C11/C++ compiler, and the compatible Pogosim
headers used by platform-dependent modules. For simulator examples: an installed
Pogosim development library plus its dependencies, including Box2D, SDL2,
SDL2_gfx, SDL2_ttf, yaml-cpp, spdlog, fmt, and Arrow, and pkg-config.
The example Makefiles compile C as C11 and link through a C++20 compiler.
Use the dependency installation procedure in your Pogosim checkout rather than
guessing distribution package names.

Hardware additionally needs the matching Pogobot SDK includes, libraries,
LiteX tools, and cross-compiler. This repository does not pin dependency
revisions; record what you install. Current flash code requires the SDK's
16-bit page API and 5,888-page user region.

Many motion/storage examples use these source paths (numerical Makefiles can
infer a different path; explicit overrides are safer):

```text
POGO_SDK=../../pogobot-sdk
POGOSIM_INCLUDE_DIR=../../pogosim/src
POGOUTILS_INCLUDE_DIR=../../src
```

Paths are relative to the **example directory**. If appropriate, create
`pogobot-sdk` and `pogosim` links at the repository root pointing to your
existing checkouts; do not replace an existing path. Alternatively pass paths
to make:

```sh
make -C examples/kinematics sim POGOSIM_INCLUDE_DIR=/absolute/path/pogosim/src
make -C examples/kinematics bin POGO_SDK=/absolute/path/pogobot-sdk POGOSIM_INCLUDE_DIR=/absolute/path/pogosim/src
```

The Makefiles also contain platform-specific include/link directories, normally
`/usr/local` and, on macOS, `/opt/homebrew`. See the
[representative Makefile](../examples/magnetometer_heading_detection/Makefile)
before overriding compiler flags: replacing an entire flags variable can discard
required includes, standards, and linker settings.

## Library and simulator builds

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
cmake --install build
make -C examples/MLP_int8 sim POGOUTILS_INCLUDE_DIR=../../src
./examples/MLP_int8/MLP_int8 -c conf/test.yaml
```

Configure `CMAKE_INSTALL_PREFIX` if you do not want the default installation
location. The example Makefile must then search that prefix's includes/libraries.
Some numerical Makefiles silently skip every target if their inferred
pogo-utils path lacks `version.h`; pass `POGOUTILS_INCLUDE_DIR=../../src`
when building them from this repository. A zero make exit status alone does
not prove a binary was compiled.
Installation may require elevated permissions; the explicit commands do not
automatically invoke sudo. [build.sh](../build.sh) does invoke sudo if available
and also builds all examples. `cmake --build build --target examples` runs
make clean and make sim in each example; it is not a CTest target.

Firmware compiles source from this checkout, whereas simulator examples
usually link installed `libpogo-utils`. Some modern examples compile local
flash/calibration-storage modules as well to keep PFFS versions consistent.
Rebuild **and reinstall** the library after compiled-module changes, then
rebuild the application. Do not combine old headers with new library objects.

## Hardware

```sh
make -C examples/kinematics bin
make -C examples/kinematics connect TTY=/dev/ttyUSB0
```

The artifact is `examples/kinematics/build/bin/firmware.bin`. The connect
target opens the SDK's terminal/upload workflow; follow its instructions and
your robot's bootloader procedure. Use your actual serial device. Keep motors
off the ground during first sign/control tests. Flash format, calibration
creation, and log clearing have destructive or persistent effects.

The kinematics Makefile also provides `size`, `symbols`, and `layout` targets
and emits `firmware.map`. Its default size optimization/LTO applies to locally
compiled firmware sources; `POGOBIN_LTO=0` disables LTO for comparison.
Keep the default SDK archive extraction policy unless you have audited startup
and interrupt roots. Image byte count, section totals, runtime RAM and stack
are different measurements; no simulator build can substitute for them.

## First magnetometer mission

```sh
make -C examples/magnetometer_calibration sim
make -C examples/go_straight sim
./examples/magnetometer_calibration/magnetometer_calibration -c conf/magnetometer_calibration.yaml
./examples/go_straight/go_straight -c conf/magnetometer.yaml
```

Do not run these two applications simultaneously against the same archive.
Wait for `MAG_CAL_STORED` for each robot, then quit normally so Pogosim exports
`magnetometer.pgflash`. The mission imports that archive. Same category/ID
matching matters; the extra wall objects are not calibrated robots.
On hardware run the calibration firmware first, check green/success serial
output, then upload the mission without formatting user flash.

The ACU calibration config currently exports `acu.pgflash`, while
`conf/acu.yaml` imports `magnetometer.pgflash`. Before using that pair, set
the mission's `flash_state.input_file` and `output_file` to `acu.pgflash`
in your experiment configuration (or deliberately arrange the same archive
path). Merely using both filenames as supplied does not connect the pair.
See [sensing](systems/sensing_and_calibration.md) and
[troubleshooting](troubleshooting.md).

## Host tests

Build tests with assertions enabled:

```sh
cmake -S . -B build-debug -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON
cmake --build build-debug --parallel
ctest --test-dir build-debug --output-on-failure
```

The six targets exercise PFFS, strict NOR PFFS behavior, NOR append logs,
magnetometer flash persistence, bounded integer/fixed-point formatting, and
flash-first printing/backpressure. They use host mocks, not real flash or robot
motion. Some tests perform setup calls inside assertions: Release
`-DNDEBUG` removes those calls, so it is not a valid test configuration.
These tests do not establish hardware latency, wear endurance, controller
stability, or the correctness of every numerical runtime.

## Customizing an example

Read its README and source first, copy it **outside** the existing example if
you want an independent application, and adjust its Makefile paths and binary
name. Put each robot's mutable state in `USERDATA`, register callbacks in
`user_init`, and advance modules in `user_step`. Simulator YAML parameters
are application-specific: a key has no effect unless that example reads it.
Compile-time dimensions/switches must be consistent across translation units.
See [architecture](architecture.md) and [extension guide](extending.md).
