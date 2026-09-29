# Interactive shell. Keep calibration fitting out of this diagnostic image.
MAKEFILE_DIR := flash_file
POGOUTILS_EXCLUDED_SRCS := magnetometer_calibration.c \
    magnetometer_calibration_flash_store.c
include ../magnetometer_heading_detection/Makefile

# Prefer this checkout's API declarations over a potentially older installed
# pogo-utils header when compiling the simulator application source.
SIM_CFLAGS += -I../../src

# The installed simulator library may predate create_blank/rename. Compile this
# checkout's writer for the shell; its other PFFS helpers remain in the library.
SIM_OBJECTS += build/sim/flash_file_write_local.o
$(SIM_TARGET): build/sim/flash_file_write_local.o
build/sim/flash_file_write_local.o: ../../src/pogo-utils/flash_file_write.c | build/sim
	$(SIM_CC) $(SIM_CFLAGS) -c $< -o $@
