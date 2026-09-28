# Explicit, destructive user-flash reset for physical recovery. The shared
# rules compile the local flash writer but omit calibration fitting and storage.
MAKEFILE_DIR := flash_file_format
POGOUTILS_EXCLUDED_SRCS := magnetometer_calibration.c \
    magnetometer_calibration_flash_store.c
include ../magnetometer_heading_detection/Makefile

SIM_CFLAGS += -I../../src
