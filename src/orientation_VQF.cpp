#include "orientation_vqf.h"
#include "BNO_LUT.h"
#include <ArduinoEigen.h>
#include <math.h>

// State Vector: [q_w, q_x, q_y, q_z]
static Eigen::VectorXf x_state(4);

// Process and measurement noise covariance matrices
static Eigen::MatrixXf Q;
static Eigen::MatrixXf R;

// Low-pass filter coefficients
static float alpha_accel = 0.98f;  // Accelerometer filter
static float alpha_gyro = 0.98f;   // Gyroscope filter
static float alpha_mag = 0.98f;    // Magnetometer filter

// Filtered sensor values
static float ax_f, ay_f, az_f;
static float gx_f, gy_f, gz_f;
static float mx_f, my_f, mz_f;

// Calibration offsets
static float yaw_offset = 0.0f;
static float pitch_offset = 0.0f;
static float roll_offset = 0.0f;
static bool calibrated = false;

void vqfInit() {
    // Initialize quaternion to [1, 0, 0, 0] (no rotation)
    x_state = Eigen::VectorXf::Zero(4);
    x_state(0) = 1.0f;

    // Process noise
    Q = Eigen::MatrixXf::Zero(4, 4);

    // Measurement noise
    R = Eigen::MatrixXf::Identity(6, 6) * 0.01f;
}

void vqfPredict(float gx, float gy, float gz, float dt) {
    // Apply low-pass filter to gyro
    gx_f = alpha_gyro * gx_f + (1.0f - alpha_gyro) * gx;
    gy_f = alpha_gyro * gy_f + (1.0f - alpha_gyro) * gy;
    gz_f = alpha_gyro * gz_f + (1.0f - alpha_gyro) * gz;

    // Extract current quaternion
    float qw = x_state(0);
    float qx = x_state(1);
    float qy = x_state(2);
    float qz = x_state(3);

    // Quaternion derivative
    float dq_w = 0.5f * (-qx * gx_f - qy * gy_f - qz * gz_f);
    float dq_x = 0.5f * (qw * gx_f + qy * gz_f - qz * gy_f);
    float dq_y = 0.5f * (qw * gy_f - qx * gz_f + qz * gx_f);
    float dq_z = 0.5f * (qw * gz_f + qx * gy_f - qy * gx_f);

    // Update quaternion
    x_state(0) += dq_w * dt;
    x_state(1) += dq_x * dt;
    x_state(2) += dq_y * dt;
    x_state(3) += dq_z * dt;

    // Normalize quaternion
    float norm = sqrt(pow(x_state(0), 2) + pow(x_state(1), 2) + pow(x_state(2), 2) + pow(x_state(3), 2));
    x_state(0) /= norm;
    x_state(1) /= norm;
    x_state(2) /= norm;
    x_state(3) /= norm;
}

void vqfUpdate(float ax, float ay, float az, float mx, float my, float mz) {
    // Apply low-pass filter to accelerometer and magnetometer
    ax_f = alpha_accel * ax_f + (1.0f - alpha_accel) * ax;
    ay_f = alpha_accel * ay_f + (1.0f - alpha_accel) * ay;
    az_f = alpha_accel * az_f + (1.0f - alpha_accel) * az;

    mx_f = alpha_mag * mx_f + (1.0f - alpha_mag) * mx;
    my_f = alpha_mag * my_f + (1.0f - alpha_mag) * my;
    mz_f = alpha_mag * mz_f + (1.0f - alpha_mag) * mz;

    // Normalize accelerometer and magnetometer
    float accelNorm = sqrt(ax_f * ax_f + ay_f * ay_f + az_f * az_f);
    ax_f /= accelNorm;
    ay_f /= accelNorm;
    az_f /= accelNorm;

    float magNorm = sqrt(mx_f * mx_f + my_f * my_f + mz_f * mz_f);
    mx_f /= magNorm;
    my_f /= magNorm;
    mz_f /= magNorm;

    // Tilt-compensated magnetometer
    float pitch_acc = atan2(-ax_f, sqrt(ay_f * ay_f + az_f * az_f));
    float roll_acc = atan2(ay_f, az_f);

    float mx_tilt = mx_f * cos(pitch_acc) + mz_f * sin(pitch_acc);
    float my_tilt = my_f * cos(roll_acc) + mz_f * sin(roll_acc);
    float yaw_mag = atan2(-my_tilt, mx_tilt);

    // Update quaternion from roll, pitch, and yaw
    float qw = cos(roll_acc / 2) * cos(pitch_acc / 2) * cos(yaw_mag / 2) +
               sin(roll_acc / 2) * sin(pitch_acc / 2) * sin(yaw_mag / 2);
    float qx = sin(roll_acc / 2) * cos(pitch_acc / 2) * cos(yaw_mag / 2) -
               cos(roll_acc / 2) * sin(pitch_acc / 2) * sin(yaw_mag / 2);
    float qy = cos(roll_acc / 2) * sin(pitch_acc / 2) * cos(yaw_mag / 2) +
               sin(roll_acc / 2) * cos(pitch_acc / 2) * sin(yaw_mag / 2);
    float qz = cos(roll_acc / 2) * cos(pitch_acc / 2) * sin(yaw_mag / 2) -
               sin(roll_acc / 2) * sin(pitch_acc / 2) * cos(yaw_mag / 2);

    x_state(0) = qw;
    x_state(1) = qx;
    x_state(2) = qy;
    x_state(3) = qz;

    // Normalize quaternion
    float norm = sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
    x_state(0) /= norm;
    x_state(1) /= norm;
    x_state(2) /= norm;
    x_state(3) /= norm;
}

void vqfGetEuler(float &roll, float &pitch, float &yaw) {
    float qw = x_state(0);
    float qx = x_state(1);
    float qy = x_state(2);
    float qz = x_state(3);

    roll = atan2(2.0f * (qw * qx + qy * qz), 1.0f - 2.0f * (qx * qx + qy * qy)) * 180.0f / M_PI;
    pitch = asin(2.0f * (qw * qy - qz * qx)) * 180.0f / M_PI;
    yaw = atan2(2.0f * (qw * qz + qx * qy), 1.0f - 2.0f * (qy * qy + qz * qz)) * 180.0f / M_PI;

    // Apply offsets
    if (calibrated) {
        roll -= roll_offset;
        pitch -= pitch_offset;
        yaw -= yaw_offset;
    }
}

void vqfCalibrateOrientation() {
    const unsigned long calibrationTime = 10000; // 10 seconds
    unsigned long startTime = millis();
    unsigned long count = 0;

    float yaw_sum = 0.0f;
    float pitch_sum = 0.0f;
    float roll_sum = 0.0f;

    Serial.println("Calibrating VQF orientation... Please keep the device still.");

    while (millis() - startTime < calibrationTime) {
        float roll_temp, pitch_temp, yaw_temp;
        vqfGetEuler(roll_temp, pitch_temp, yaw_temp);

        roll_sum += roll_temp;
        pitch_sum += pitch_temp;
        yaw_sum += yaw_temp;

        count++;
        delay(10); // 100 Hz sampling rate
    }

    roll_offset = roll_sum / count;
    pitch_offset = pitch_sum / count;
    yaw_offset = yaw_sum / count;

    calibrated = true;

    Serial.println("VQF calibration complete:");
    Serial.print("Roll Offset: "); Serial.println(roll_offset, 3);
    Serial.print("Pitch Offset: "); Serial.println(pitch_offset, 3);
    Serial.print("Yaw Offset: "); Serial.println(yaw_offset, 3);
}
