# Read-only flash-file inventory. Reuse the simulator/firmware rules shared by
# the magnetometer examples while excluding writers and calibration fitting
# from this example's firmware objects.
MAKEFILE_DIR := flash_file
POGOUTILS_EXCLUDED_SRCS := magnetometer_calibration.c \
    magnetometer_calibration_flash_store.c flash_file_write.c flash_file_name.c
include ../magnetometer_heading_detection/Makefile

# Prefer this checkout's API declarations over a potentially older installed
# pogo-utils header when compiling the simulator application source.
SIM_CFLAGS += -I../../src
