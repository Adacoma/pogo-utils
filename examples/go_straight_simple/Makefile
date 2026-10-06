# Reuse go_straight's SDK/size flags; keep this example's binary name.
override MAKEFILE_DIR := go_straight_simple
POGOUTILS_EXCLUDED_SRCS := magnetometer_calibration.c \
    magnetometer_calibration_flash_store.c
include ../go_straight/Makefile

# Keep the coordinator, sensor and planner from the same checkout in sim builds.
SIM_LOCAL_CONTROL_SRCS := kinematics.c heading_PID.c calibrated_motors.c \
    wall_avoidance_magnetometer.c magnetometer_heading_detection.c
SIM_LOCAL_CONTROL_OBJECTS := $(patsubst %.c,$(BUILD_DIR)/sim/%_local.o,$(SIM_LOCAL_CONTROL_SRCS))
SIM_OBJECTS += $(SIM_LOCAL_CONTROL_OBJECTS)
$(SIM_TARGET): $(SIM_LOCAL_CONTROL_OBJECTS)
-include $(SIM_LOCAL_CONTROL_OBJECTS:.o=.d)
