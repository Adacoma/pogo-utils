# pogo-utils
This library contains a collection of useful C code routines for programming [Pogobot robots](https://pogobot.github.io/).

## Installation
First, install the pogosim simulator, found [Here](https://github.com/Adacoma/pogosim).

Then:
```shell
./build.sh
```

## Quickstart
To use pogo-utils in your own projects, you copy one of the examples in the "examples" directory, and update the Makefile. In particular, you must set the variables:
```shell
POGO_SDK=../../pogobot-sdk      # Path to the pogobot-sdk, found at: https://github.com/nekonaute/pogobot-sdk
POGOSIM_INCLUDE_DIR=../../pogosim/src/          # Path to the pogosim simulator, found at: https://github.com/Adacoma/pogosim
POGOUTILS_INCLUDE_DIR=../../src                 # Path to pogo-util, the library in this repository, and found at: https://github.com/Adacoma/pogo-utils
```
so that they conform to your setup.

Then, you can compile your project using:
```shell
make clean && make -j 10 sim # To compile a simulation
# OR, to compile for real robots:
make clean bin
```

