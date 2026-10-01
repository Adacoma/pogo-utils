# Compile the new formatting/adapter objects locally even when the installed
# library predates them. Firmware already collects this checkout's C sources.
MAKEFILE_DIR := print_log
POGOUTILS_EXCLUDED_SRCS := magnetometer_calibration.c \
    magnetometer_calibration_flash_store.c
include ../magnetometer_heading_detection/Makefile
SIM_LOCAL_PRINT_OBJECTS := $(patsubst %.c,build/sim/%_local.o,print_format.c print_log.c)
SIM_OBJECTS += $(SIM_LOCAL_PRINT_OBJECTS)
$(SIM_TARGET): $(SIM_LOCAL_PRINT_OBJECTS)
-include $(SIM_LOCAL_PRINT_OBJECTS:.o=.d)
