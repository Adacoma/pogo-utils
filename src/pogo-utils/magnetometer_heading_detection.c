/**
 * @file magnetometer_heading_detection.c
 * @brief Runtime-only magnetometer heading estimation.
 *
 * Calibration is compiled separately so mission binaries can load a stored
 * model without carrying sample collection or numerical fitting code.
 */
#define MHD_BUILD_RUNTIME 1
#include "magnetometer_heading_impl.inc"
