#ifndef BNO_LUT_H
#define BNO_LUT_H

#include <Adafruit_BNO055.h>

// Hardcoded BNO055 calibration data (matches the layout of adafruit_bno055_offsets_t)
const adafruit_bno055_offsets_t BNO_CALIBRATION_OFFSETS = {
    7,      // accel_offset_x
    -50,    // accel_offset_y
    -42,    // accel_offset_z
    83,     // mag_offset_x
    115,    // mag_offset_y
    -420,   // mag_offset_z
    -1,     // gyro_offset_x
    -1,     // gyro_offset_y
    0,      // gyro_offset_z
    1000,   // accel_radius
    769     // mag_radius
};

#endif // BNO_LUT_H
