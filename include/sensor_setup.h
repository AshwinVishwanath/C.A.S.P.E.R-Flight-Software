#ifndef SENSOR_SETUP_H
#define SENSOR_SETUP_H

extern float baselineAltitude;
extern float accel_offsets[3];
extern float mag_offsets[3];

// Initializes sensors and perform stationary calibration
void setupSensors();
bool waitForCalibration();

// Returns relative altitude (filtered and offset)
float getRelativeAltitude();

// Get corrected IMU data:
// yaw, pitch, roll from BNO055
// ax, ay, az corrected in NED frame after offsets and orientation transform
// mx, my, mz corrected after offsets (not strictly needed for EKF now)
void getCorrectedIMUData(float &yaw, float &pitch, float &roll, 
                         float &ax_ned, float &ay_ned, float &az_ned);

#endif // SENSOR_SETUP_H
