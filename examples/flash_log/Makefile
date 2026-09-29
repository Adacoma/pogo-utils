# Build the small two-file logger against the installed pogo-utils library.
MAKEFILE_DIR := flash_log
POGOUTILS_EXCLUDED_SRCS := magnetometer_calibration.c \
    magnetometer_calibration_flash_store.c
include ../magnetometer_heading_detection/Makefile

SIM_CFLAGS += -I../../src
