#ifndef FILTER_H
#define FILTER_H

#include <Eigen.h>

// Initialize the EKF with starting position and velocity
void ekfInit(float x, float y, float z, float vx, float vy, float vz);

// EKF Predict step: uses acceleration in NED frame
// ax, ay, az are accelerations in NED frame (m/s²)
// dt is time step
void ekfPredict(float ax, float ay, float az, float dt);

// EKF Update step with barometric altitude measurement
// alt_meas: measured altitude (relative)
void ekfUpdate(float alt_meas);

// Get state estimate
void ekfGetState(float &x, float &y, float &z, float &vx, float &vy, float &vz);

#endif // FILTER_H
