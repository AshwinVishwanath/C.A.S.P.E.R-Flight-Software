#include <Arduino.h>
#include "sensor_setup.h"
#include <ArduinoEigen.h>
#include <math.h>

// Constants for sensor fusion
static const int n_x = 6; // x, y, z, vx, vy, vz
static const int n_z = 7; // accelerometer (ax, ay, az) and barometer (alt)

static Eigen::VectorXf x_mean; 
static Eigen::MatrixXf P;

static Eigen::MatrixXf Q; // Process noise
static Eigen::MatrixXf R; // Measurement noise

static bool initialized = false;

// Initialize the EKF with starting position, velocity
void ekfInit(float x, float y, float z, float vx, float vy, float vz) {
    x_mean = Eigen::VectorXf(n_x);
    x_mean << x, y, z, vx, vy, vz;

    P = Eigen::MatrixXf::Identity(n_x, n_x) * 0.1f;

    // Process noise matrix Q
    Q = (Eigen::MatrixXf(n_x, n_x) << 
        0.01f, 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  // Row 1: x position
        0.0f,  0.01f, 0.0f,  0.0f,  0.0f,  0.0f,  // Row 2: y position
        0.0f,  0.0f,  0.01f, 0.0f,  0.0f,  0.0f,  // Row 3: z position
        0.0f,  0.0f,  0.0f,  0.1f,  0.0f,  0.0f,  // Row 4: x velocity
        0.0f,  0.0f,  0.0f,  0.0f,  0.1f,  0.0f,  // Row 5: y velocity
        0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.1f   // Row 6: z velocity
    ).finished();

    // Measurement noise matrix R
    R = (Eigen::MatrixXf(n_z, n_z) << 
        0.1f, 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  // Row 1: x acceleration
        0.0f, 0.1f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  // Row 2: y acceleration
        0.0f, 0.0f,  0.1f,  0.0f,  0.0f,  0.0f,  0.0f,  // Row 3: z acceleration
        0.0f, 0.0f,  0.0f,  0.1f,  0.0f,  0.0f,  0.0f,  // Row 4: unused (placeholder)
        0.0f, 0.0f,  0.0f,  0.0f,  0.1f,  0.0f,  0.0f,  // Row 5: unused (placeholder)
        0.0f, 0.0f,  0.0f,  0.0f,  0.0f,  0.1f,  0.0f,  // Row 6: unused (placeholder)
        0.0f, 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.1f   // Row 7: altitude
    ).finished();

    initialized = true;
}


void ekfPredict(float ax, float ay, float az, float dt) {
    if (!initialized) return;

    float x = x_mean(0);
    float y = x_mean(1);
    float z = x_mean(2);
    float vx = x_mean(3);
    float vy = x_mean(4);
    float vz = x_mean(5);

    // Update position
    x += vx * dt;
    y += vy * dt;
    z += vz * dt;

    // Update velocity using accelerometer input
    vx += ax * dt;
    vy += ay * dt;
    vz += az * dt;

    // Update state vector
    Eigen::VectorXf x_pred(n_x);
    x_pred << x, y, z, vx, vy, vz;

    // State transition matrix
    Eigen::MatrixXf F_mat = Eigen::MatrixXf::Identity(n_x, n_x);
    F_mat(0, 3) = dt;
    F_mat(1, 4) = dt;
    F_mat(2, 5) = dt;

    // Update covariance matrix
    P = F_mat * P * F_mat.transpose() + Q;

    // Update the state vector
    x_mean = x_pred;
}

void ekfUpdate(float ax_meas, float ay_meas, float az_meas, float alt_meas) {
    if (!initialized) return;

    // Measurement vector (acceleration and altitude)
    Eigen::VectorXf z(n_z);
    z << ax_meas, ay_meas, az_meas, 0.0f, 0.0f, 0.0f, alt_meas;

    // Measurement model
    Eigen::VectorXf h(n_z);
    h << x_mean(3), x_mean(4), x_mean(5), 0.0f, 0.0f, 0.0f, x_mean(2);

    // Innovation
    Eigen::VectorXf y = z - h;

    // Measurement matrix H
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(n_z, n_x);
    H(0, 3) = 1; 
    H(1, 4) = 1; 
    H(2, 5) = 1; 
    H(6, 2) = 1; 

    // Innovation covariance
    Eigen::MatrixXf S = H * P * H.transpose() + R;
    Eigen::MatrixXf K = P * H.transpose() * S.inverse();

    // Update state
    x_mean = x_mean + K * y;

    // Update covariance
    P = (Eigen::MatrixXf::Identity(n_x, n_x) - K * H) * P;
}

void ekfGetState(float &x, float &y, float &z, float &vx, float &vy, float &vz) {
    x = x_mean(0);
    y = x_mean(1);
    z = x_mean(2);
    vx = x_mean(3);
    vy = x_mean(4);
    vz = x_mean(5);
}
