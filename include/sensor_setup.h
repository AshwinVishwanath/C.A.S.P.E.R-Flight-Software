#ifndef SENSOR_SETUP_H
#define SENSOR_SETUP_H

#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>

extern float baselineAltitude; // Baseline for relative altitude
extern float altitudeBias;     // Bias used to correct altitude readings

void setupSensors();
void getCorrectedIMUData(float &yaw, float &pitch, float &roll, float &ax_ned, float &ay_ned, float &az_ned, float &mx, float &my, float &mz);
float getRelativeAltitude();
float manualCalibrateBMP388(); // Added function declaration
// Function to get raw sensor data (accelerometer, gyroscope, magnetometer)
void getSensorData(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &mx, float &my, float &mz);

#endif // SENSOR_SETUP_H

