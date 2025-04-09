#include <Arduino.h>
#include "sensor_setup.h"
#include <ArduinoEigen.h>
#include <math.h>

// 6D state vector: [ x, y, z, vx, vy, vz ]
// We'll do a standard approach:
//   - Use accelerometer data in the PREDICT step
//   - Use baro altitude in the UPDATE step

static const int n_x = 6; // dimension of the state
static const int n_z = 1; // dimension of the measurement (just baro altitude)

static Eigen::VectorXf x_mean;    // The state estimate vector
static Eigen::MatrixXf P;         // The state covariance matrix
static Eigen::MatrixXf Q;         // Process (model) noise covariance
static Eigen::MatrixXf R;         // Measurement noise covariance

static bool initialized = false;

/**
 * Initialize the EKF with a starting state and covariance.
 *
 * param x  Initial x-position
 * param y  Initial y-position
 * param z  Initial z-position
 * param vx Initial velocity along x
 * param vy Initial velocity along y
 * param vz Initial velocity along z
 */
void ekfInit(float x, float y, float z, float vx, float vy, float vz) {
    // Allocate space for our vectors/matrices now that we know the dimensions
    x_mean = Eigen::VectorXf(n_x);
    P      = Eigen::MatrixXf(n_x, n_x);
    Q      = Eigen::MatrixXf(n_x, n_x);
    R      = Eigen::MatrixXf(n_z, n_z);

    // Set the initial state
    x_mean << x, y, z, vx, vy, vz;

    // Initialize the covariance to something moderate
    P = Eigen::MatrixXf::Identity(n_x, n_x) * 0.1f;

    // A simple example of process noise — tune as needed
    Q.setZero();
    Q(0,0) = 0.01f;   // x
    Q(1,1) = 0.01f;   // y
    Q(2,2) = 0.01f;   // z
    Q(3,3) = 0.35f;    // vx
    Q(4,4) = 0.31f;    // vy
    Q(5,5) = 0.3f6;    // vz

    // Measurement noise for baro altitude only (1×1)
    R(0,0) = 0.2f;    // for example, 0.1 m^2 variance

    initialized = true;
}

/**
 * EKF Predict step: use the system dynamics to project state ahead in time.
 *
 * Here, we treat (ax, ay, az) from the accelerometer as the actual
 * acceleration inputs that update velocity, and thus position.
 *
 * param ax  measured acceleration in x
 * param ay  measured acceleration in y
 * param az  measured acceleration in z
 * param dt  time step
 */
void ekfPredict(float ax, float ay, float az, float dt) {
    if (!initialized) return;

    // Extract the current state
    float x  = x_mean(0);
    float y  = x_mean(1);
    float z  = x_mean(2);
    float vx = x_mean(3);
    float vy = x_mean(4);
    float vz = x_mean(5);

    // Update position by integrating velocity
    x += vx * dt;
    y += vy * dt;
    z += vz * dt;

    // Update velocity by integrating acceleration
    vx += ax * dt;
    vy += ay * dt;
    vz += az * dt;

    // Write back into a predicted state vector
    Eigen::VectorXf x_pred(n_x);
    x_pred << x, y, z, vx, vy, vz;

    // Build the linear state transition matrix, F
    Eigen::MatrixXf F = Eigen::MatrixXf::Identity(n_x, n_x);
    // position depends on velocity over dt
    F(0,3) = dt; // x depends on vx
    F(1,4) = dt; // y depends on vy
    F(2,5) = dt; // z depends on vz

    // Covariance predict: P' = F * P * F^T + Q
    P = F * P * F.transpose() + Q;

    // Update the filter's state
    x_mean = x_pred;
}

/**
 * EKF Update step for barometric altitude.
 *
 * The barometer measures altitude ~ z-position in our state.
 *
 * param alt_meas  barometer altitude measurement
 */
void ekfUpdateBaro(float alt_meas) {
    if (!initialized) return;

    // 1D measurement vector
    Eigen::VectorXf z(n_z);
    z(0) = alt_meas;

    // Predicted measurement is just the state's z component
    Eigen::VectorXf h(n_z);
    h(0) = x_mean(2);  // z

    // Innovation (residual)
    Eigen::VectorXf y = z - h;

    // Measurement matrix H: 1×6, mapping state -> baro altitude
    Eigen::MatrixXf H = Eigen::MatrixXf::Zero(n_z, n_x);
    H(0,2) = 1.0f;  // alt ~ z

    // Innovation covariance
    Eigen::MatrixXf S = H * P * H.transpose() + R;

    // Kalman gain
    Eigen::MatrixXf K = P * H.transpose() * S.inverse();

    // State update: x_mean = x_mean + K * (z - h)
    x_mean += K * y;

    // Covariance update: P = (I - K*H) * P
    Eigen::MatrixXf I = Eigen::MatrixXf::Identity(n_x, n_x);
    P = (I - K * H) * P;
}

/**
 * Retrieve the current state estimate.
 * 
 * param x   (output) position in x
 * param y   (output) position in y
 * param z   (output) position in z
 * param vx  (output) velocity in x
 * param vy  (output) velocity in y
 * param vz  (output) velocity in z
 */
void ekfGetState(float &x, float &y, float &z, float &vx, float &vy, float &vz) {
    x = x_mean(0);
    y = x_mean(1);
    z = x_mean(2);
    vx = x_mean(3);
    vy = x_mean(4);
    vz = x_mean(5);
}
