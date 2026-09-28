# This variant has its own controller source but shares Vicsek's build rules.
# MAKEFILE_DIR overrides the included Makefile's source-derived target name.
MAKEFILE_DIR := vicsek_u_turns
include ../vicsek/Makefile

# The simulator normally links an installed libpogo-utils archive. Link the
# current coordinator source explicitly so this example's reverse mode works
# even before the shared library is reinstalled. Firmware rules already build
# every local pogo-utils source, including kinematics.c.
SIM_CFLAGS := -I../../src $(SIM_CFLAGS)
SIM_OBJECTS += $(BUILD_DIR)/sim/kinematics.o
$(SIM_TARGET): $(BUILD_DIR)/sim/kinematics.o
$(BUILD_DIR)/sim/kinematics.o: ../../src/pogo-utils/kinematics.c | $(BUILD_DIR)/sim
	$(SIM_CC) $(SIM_CFLAGS) -c $< -o $@
