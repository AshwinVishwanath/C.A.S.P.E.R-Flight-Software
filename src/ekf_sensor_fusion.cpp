#include <Arduino.h>
#include "sensor_setup.h"
#include <ArduinoEigen.h>
#include <math.h>

// Constants for sensor fusion
static const int n_x = 9; // x,y,z,vx,vy,vz, roll, pitch, yaw
static const int n_z = 9; // accelerometer (ax, ay, az), magnetometer (mx, my, mz), and barometer (alt)

static Eigen::VectorXf x_mean; 
static Eigen::MatrixXf P;

static Eigen::MatrixXf Q; // Process noise
static Eigen::MatrixXf R; // Measurement noise

static bool initialized = false;

// Initialize the EKF with starting position, velocity, and orientation
void ekfInit(float x, float y, float z, float vx, float vy, float vz, float roll, float pitch, float yaw) {
    x_mean = Eigen::VectorXf(n_x);
    x_mean << x, y, z, vx, vy, vz, roll, pitch, yaw;

    P = Eigen::MatrixXf::Identity(n_x, n_x) * 0.1f;

    Q = Eigen::MatrixXf::Zero(n_x, n_x);
    Q(0,0) = 0.01f; Q(1,1) = 0.01f; Q(2,2) = 0.01f; 
    Q(3,3) = 0.1f; Q(4,4) = 0.1f; Q(5,5) = 0.1f; 
    Q(6,6) = 0.001f; Q(7,7) = 0.001f; Q(8,8) = 0.001f; 

    R = Eigen::MatrixXf::Identity(n_z, n_z) * 0.1f;

    initialized = true;
}

void ekfPredict(float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    if (!initialized) return;

    float x = x_mean(0);
    float y = x_mean(1);
    float z = x_mean(2);
    float vx = x_mean(3);
    float vy = x_mean(4);
    float vz = x_mean(5);
    float roll = x_mean(6);
    float pitch = x_mean(7);
    float yaw = x_mean(8);

    x += vx * dt;
    y += vy * dt;
    z += vz * dt;

    vx += ax * dt;
    vy += ay * dt;
    vz += az * dt;

    roll += gx * dt;
    pitch += gy * dt;
    yaw += gz * dt;

    Eigen::VectorXf x_pred(n_x);
    x_pred << x, y, z, vx, vy, vz, roll, pitch, yaw;

    Eigen::MatrixXf F_mat = Eigen::MatrixXf::Identity(n_x, n_x);
    F_mat(0, 3) = dt;
    F_mat(1, 4) = dt;
    F_mat(2, 5) = dt;

    P = F_mat * P * F_mat.transpose() + Q;

    x_mean = x_pred;
}

void ekfUpdate(float ax_meas, float ay_meas, float az_meas, float mx_meas, float my_meas, float mz_meas, float alt_meas) {
    if (!initialized) return;

    Eigen::VectorXf z(n_z);
    z << ax_meas, ay_meas, az_meas, mx_meas, my_meas, mz_meas, 0.0f, 0.0f, alt_meas;

    Eigen::VectorXf h(n_z);
    h << x_mean(3), x_mean(4), x_mean(5), x_mean(6), x_mean(7), x_mean(8), x_mean(2), x_mean(2), x_mean(2);

    Eigen::VectorXf y = z - h;

    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(n_z, n_x);
    H(0, 3) = 1; 
    H(1, 4) = 1; 
    H(2, 5) = 1; 
    H(3, 6) = 1; 
    H(4, 7) = 1; 
    H(5, 8) = 1; 
    H(6, 2) = 1; 
    H(7, 2) = 1; 
    H(8, 2) = 1; 

    Eigen::MatrixXf S = H * P * H.transpose() + R;
    Eigen::MatrixXf K = P * H.transpose() * S.inverse();

    x_mean = x_mean + K * y;
    P = (Eigen::MatrixXf::Identity(n_x, n_x) - K * H) * P;
}

void ekfGetState(float &x, float &y, float &z, float &vx, float &vy, float &vz, float &roll, float &pitch, float &yaw) {
    x = x_mean(0);
    y = x_mean(1);
    z = x_mean(2);
    vx = x_mean(3);
    vy = x_mean(4);
    vz = x_mean(5);
    roll = x_mean(6);
    pitch = x_mean(7);
    yaw = x_mean(8);
}
