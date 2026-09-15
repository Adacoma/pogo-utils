# Build-size fix for the attached Pogobot / Pogosim example Makefile.
# Changes are restricted to build configuration; controller code is unchanged.
# Rebuild from clean after changing flags or the SDK archive link mode.
.DEFAULT_GOAL := all
.DELETE_ON_ERROR:

# Get the absolute path of the Makefile
MAKEFILE_PATH := $(abspath $(lastword $(MAKEFILE_LIST)))
MAKEFILE_DIR := $(notdir $(patsubst %/,%,$(dir $(MAKEFILE_PATH))))

##### Detect OS #####
OS := $(shell uname)
ifeq ($(OS), Darwin)  # macOS
    WHOLE_ARCHIVE_FLAG = -Wl,-force_load
	SIM_LD_FLAGS_BOX2D = -L/usr/local/lib -Wl,-force_load,/usr/local/lib/libbox2d.a
	SIM_LD_FLAGS_POGOSIM = -Wl,-force_load,/usr/local/lib/libpogosim.a -lpogo-utils
    STDCPP_LIB = -lc++
	SIM_LD_EXT = -L/usr/local/lib -L/opt/homebrew/lib
	SIM_INCLUDES_EXT = -I/usr/local/include -I/opt/homebrew/include
else  # Linux
	SIM_LD_FLAGS_BOX2D = -lbox2d
	SIM_LD_FLAGS_POGOSIM = -Wl,--whole-archive -lpogosim -Wl,--no-whole-archive -lpogo-utils
    WHOLE_ARCHIVE_FLAG = -Wl,--whole-archive
    STDCPP_LIB = -lstdc++
	SIM_LD_EXT = -L/usr/local/lib
	SIM_INCLUDES_EXT = -I/usr/local/include
endif

##### Settings for the pogobot binary #####
ROBOT_CATEGORY?=robots
BUILD_DIR?=./build
POGO_SDK?=../../pogobot-sdk
POGOSIM_INCLUDE_DIR?=../../pogosim/src/
POGOSIM_SRC_DIR?=$(POGOSIM_INCLUDE_DIR)/pogosim/
# Apply the SAME size/LTO/section flags to the app, Pogosim C shim, and
# every locally compiled pogo-utils module. Link-time -flto cannot retroactively
# optimize a library object that was compiled without LTO bytecode.
POGOBIN_LTO ?= 1
POGOBIN_LTO_FLAGS = $(if $(filter 1,$(POGOBIN_LTO)),-flto,)
POGOBIN_COMMON_FLAGS = -Os $(POGOBIN_LTO_FLAGS) \
    -fno-exceptions -fno-unwind-tables -fno-asynchronous-unwind-tables \
    -fexcess-precision=standard -fsingle-precision-constant
POGOBIN_CFLAGS = $(POGOBIN_COMMON_FLAGS) -ffunction-sections -fdata-sections
POGOBIN_LDFLAGS = $(POGOBIN_COMMON_FLAGS) -Wl,--gc-sections \
    -Wl,-Map=$(BUILD_DIR)/bin/firmware.map,--cref -Wl,--print-memory-usage
# Optional additions are applied to ALL three firmware compilation rules.
# Example: make bin POGOBIN_CPPFLAGS=-DMAGNETOMETER_HEADING_ENABLE_BENCHMARK=0
# This only removes computation timing, not calibration or the fixed-point path.
POGOBIN_CPPFLAGS ?=
POGOBIN_EXTRA_CFLAGS ?=
POGOBIN_EXTRA_LDFLAGS ?=
POGOBIN_DEFINES = -DREAL_ROBOT -DROBOT_CATEGORY=$(ROBOT_CATEGORY) $(POGOBIN_CPPFLAGS)

# Keep the original SDK archive extraction policy by default: startup, ISR,
# and registration records can otherwise silently disappear from an archive.
# --gc-sections still applies after extraction; whole != keep every section.
# "grouped" is an OPTIONAL audited optimization, not a boot-safety guarantee.
# Use it only after checking the installed linker script and required roots.
POGOBIN_SDK_LINK_MODE ?= whole
POGOBIN_EXTRA_ROOTS ?=
ifeq ($(POGOBIN_SDK_LINK_MODE),whole)
    POGOBIN_SDK_LIBS = -Wl,--whole-archive -lcompiler_rt -lc -lpogobot -Wl,--no-whole-archive
else ifeq ($(POGOBIN_SDK_LINK_MODE),grouped)
    POGOBIN_SDK_LIBS = -Wl,--start-group -lpogobot -lc -lcompiler_rt -Wl,--end-group
else
    $(error POGOBIN_SDK_LINK_MODE must be whole or grouped)
endif
POGOUTILS_INCLUDE_DIR=../../src

POGOSIM_SRC_DIR=$(POGOSIM_INCLUDE_DIR)/pogosim
POGOUTILS_SRC_DIR=$(POGOUTILS_INCLUDE_DIR)/pogo-utils

POGO_SDK_TOOLS=$(POGO_SDK)/tools
POGO_SDK_INCS=$(POGO_SDK)/includes
POGO_SDK_LIBS=$(POGO_SDK)/libs

POGO_VAR=$(POGO_SDK_TOOLS)
# Simulator-only / clean builds do not need an SDK. Firmware inspection targets
# do need its tool definitions, as do directly requested firmware objects.
REQUESTED_GOALS := $(if $(MAKECMDGOALS),$(MAKECMDGOALS),all)
SDK_GOALS := $(filter bin connect all default size symbols layout,$(REQUESTED_GOALS)) \
    $(filter $(BUILD_DIR)/bin/% build/bin/%,$(REQUESTED_GOALS))
ifneq ($(strip $(SDK_GOALS)),)
include $(POGO_VAR)/variables.mak
include $(POGO_SDK_TOOLS)/common.mak

# Some SDK common.mak versions compute GCC_PATH using $(CC) before selecting
# CC_normal, accidentally finding HOST GCC headers. Query the selected cross
# compiler directly after the SDK rules have selected it.
# IMPORTANT: do not derive this directory from -print-libgcc-file-name:
# libgcc.a may live in a multilib subdirectory (e.g. rv64imafdc/lp64d), but
# GCC's compiler headers are not necessarily stored beside that archive.
# The SDK appends /include and /include-fixed to GCC_PATH, so remove only
# the final /include from the actual compiler include-directory query.
# COMMONFLAGS in this SDK references GCC_PATH lazily. CC_normal also works
# with V=0; the SDK's quiet CC wrapper is not a plain compiler command.
ifneq ($(CLANG),1)
ifneq ($(strip $(CC_normal)),)
GCC_PATH := $(patsubst %/include,%,$(shell $(CC_normal) $(CPUFLAGS) -print-file-name=include))
endif
endif
endif

# These are ordinary commands, not the SDK's V-dependent @echo wrappers.
POGOBIN_SIZE ?= $(TARGET_PREFIX)size
POGOBIN_NM ?= $(TARGET_PREFIX)nm
POGOBIN_OBJDUMP ?= $(TARGET_PREFIX)objdump
POGOBIN_READELF ?= $(TARGET_PREFIX)readelf

TTY?=/dev/ttyUSB0

POGOUTILS_EXCLUDED_SRCS ?= magnetometer_calibration.c
SRCS := $(filter-out SDL_FontCache.c $(POGOUTILS_EXCLUDED_SRCS), $(notdir $(wildcard *.c)) $(notdir $(wildcard $(POGOSIM_SRC_DIR)/*.c)) $(notdir $(wildcard $(POGOUTILS_SRC_DIR)/*.c)))
OBJECTS=$(SRCS:.c=.o)
OBJECTS_BUILD = $(patsubst %.c,$(BUILD_DIR)/bin/%.o,$(notdir $(SRCS)))
DEP_FILES=$(patsubst %.c,$(BUILD_DIR)/bin/%.d,$(notdir $(SRCS)))

##### Compiler settings for the simulator #####
SIM_CC = cc
SIM_CXX = c++
SIM_CFLAGS = -Wall -MMD -MP -O2 -std=c11 -I$(POGOSIM_INCLUDE_DIR) $(SIM_INCLUDES_EXT)
SIM_CXXFLAGS = -Wall -MMD -MP -O2 -std=c++20 $(shell pkg-config --cflags spdlog) -pthread -I$(POGOSIM_INCLUDE_DIR) $(SIM_INCLUDES_EXT)
SIM_LDFLAGS = $(SIM_LD_FLAGS_BOX2D)  $(SIM_LD_FLAGS_POGOSIM) -lyaml-cpp \
              $(shell pkg-config --libs spdlog) \
              -lSDL2 -lSDL2_gfx -lSDL2_ttf -lbox2d $(STDCPP_LIB) -lfmt -lm -larrow $(SIM_LD_EXT)

SIM_TARGET = $(MAKEFILE_DIR)

SIM_SRCS_CXX = $(wildcard *.cpp)
SIM_SRCS_C = $(wildcard *.c)
SIM_OBJECTS_CXX = $(patsubst %.cpp,$(BUILD_DIR)/sim/%.o,$(SIM_SRCS_CXX))
SIM_OBJECTS_C = $(patsubst %.c,$(BUILD_DIR)/sim/%.o,$(SIM_SRCS_C))
SIM_OBJECTS = $(SIM_OBJECTS_CXX) $(SIM_OBJECTS_C)

all: directories sim bin

default: all

# Ensure build directories exist
$(BUILD_DIR)/bin $(BUILD_DIR)/sim:
	mkdir -p $@

##### Rules for the simulator #####

$(BUILD_DIR)/sim/%.o: %.cpp | $(BUILD_DIR)/sim
	$(SIM_CXX) $(SIM_CXXFLAGS) -c $< -o $@

$(BUILD_DIR)/sim/%.o: %.c | $(BUILD_DIR)/sim
	$(SIM_CC) $(SIM_CFLAGS) -c $< -o $@

sim: directories $(SIM_TARGET)

$(SIM_TARGET): $(SIM_OBJECTS) | $(BUILD_DIR)/sim
	$(SIM_CXX) $(SIM_CXXFLAGS) -o $@ $(SIM_OBJECTS) $(SIM_LDFLAGS)
	cp $(SIM_TARGET) $(BUILD_DIR)/sim/$(SIM_TARGET)

##### Rules for the Pogobot binary #####

# Print the section summary AND raw upload-image size after every "make bin".
# Keep the ELF/map for diagnosis; ELF debug bytes are not firmware payload.
define print_firmware_size
$(POGOBIN_SIZE) $(BUILD_DIR)/bin/firmware.elf
@printf 'firmware.bin bytes: '; wc -c < $(BUILD_DIR)/bin/firmware.bin
@printf 'Link map: %s\n' '$(BUILD_DIR)/bin/firmware.map'
endef

bin: directories $(BUILD_DIR)/bin/firmware.bin
	$(print_firmware_size)

size: $(BUILD_DIR)/bin/firmware.bin
	$(print_firmware_size)

symbols: $(BUILD_DIR)/bin/firmware.elf
	$(POGOBIN_NM) -S --size-sort --radix=d --defined-only $< > $(BUILD_DIR)/bin/firmware.symbols.txt
	tail -n 50 $(BUILD_DIR)/bin/firmware.symbols.txt

layout: $(BUILD_DIR)/bin/firmware.elf
	$(POGOBIN_OBJDUMP) -h $<
	$(POGOBIN_READELF) -h -l -W $<

INCLUDES+=-I. -I$(POGO_SDK_INCS) -I$(POGOSIM_INCLUDE_DIR) -I$(POGOUTILS_INCLUDE_DIR)

# pull in dependency info for *existing* .o files
-include $(DEP_FILES) $(SIM_OBJECTS:.o=.d)

$(BUILD_DIR)/bin/%.bin: $(BUILD_DIR)/bin/%.elf
	$(OBJCOPY) -O binary $< $@
	chmod -x $@

$(BUILD_DIR)/bin/firmware.elf: $(OBJECTS_BUILD) $(POGO_SDK_TOOLS)/linker.ld $(MAKEFILE_PATH)
	$(CC) $(LDFLAGS) $(POGOBIN_LDFLAGS) $(POGOBIN_EXTRA_LDFLAGS) \
		-T $(POGO_SDK_TOOLS)/linker.ld \
		-N -o $@ \
		$(OBJECTS_BUILD) \
		$(POGOBIN_EXTRA_ROOTS) \
		-L$(POGO_SDK_LIBS) $(POGOBIN_SDK_LIBS)
	chmod -x $@


$(BUILD_DIR)/bin/%.o: %.c | $(BUILD_DIR)/bin
	$(compile) $(POGOBIN_CFLAGS) $(POGOBIN_EXTRA_CFLAGS) $(POGOBIN_DEFINES)

$(BUILD_DIR)/bin/%.o: $(POGOSIM_SRC_DIR)/%.c | $(BUILD_DIR)/bin
	$(compile) $(POGOBIN_CFLAGS) $(POGOBIN_EXTRA_CFLAGS) $(POGOBIN_DEFINES)

$(BUILD_DIR)/bin/%.o: $(POGOUTILS_SRC_DIR)/%.c |  $(BUILD_DIR)/bin
	$(compile) $(POGOBIN_CFLAGS) $(POGOBIN_EXTRA_CFLAGS) $(POGOBIN_DEFINES)

# Rebuild local objects when this Makefile is edited. Command-line flag changes
# still require a clean rebuild (as with ordinary make flag variables).
$(OBJECTS_BUILD) $(SIM_OBJECTS): $(MAKEFILE_PATH)

clean:
	$(RM) -r $(BUILD_DIR) .*~ *~
	$(RM) -f $(SIM_OBJECTS) $(SIM_TARGET)

connect:
	$(POGO_SDK_TOOLS)/litex_term.py --serial-boot --kernel $(BUILD_DIR)/bin/firmware.bin --kernel-adr $(ROM_BASE) --safe $(TTY)

directories: $(BUILD_DIR)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

.PHONY: all default sim bin clean connect directories size symbols layout


# MODELINE "{{{1
# vim:noexpandtab:softtabstop=4:shiftwidth=4:fileencoding=utf-8
# vim:foldmethod=marker
