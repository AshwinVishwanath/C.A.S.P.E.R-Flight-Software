#ifndef EKF_SENSOR_FUSION_H
#define EKF_SENSOR_FUSION_H

#include <ArduinoEigen.h>

// Initialize the EKF with starting position, velocity, and altitude
void ekfInit(float x, float y, float z, float vx, float vy, float vz);

// Predict step of the EKF
void ekfPredict(float ax, float ay, float az, float dt);

// Update step of the EKF
void ekfUpdate(float ax_meas, float ay_meas, float az_meas, float alt_meas);
// Update using only barometric altitude
void ekfUpdateBaro(float alt_meas);

// Expose last Kalman gain for logging
extern Eigen::MatrixXf lastKalmanGain;

// Retrieve the current state of the EKF (position, velocity, altitude)
void ekfGetState(float &x, float &y, float &z, float &vx, float &vy, float &vz);

#endif // EKF_SENSOR_FUSION_H
