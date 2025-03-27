/*
   orientation_VQF.cpp

   - No low-pass filtering on gyro or accelerometer data.
   - Y is "up," so rotation about Y = "roll."
   - Magnetometer usage removed (yaw will drift).
   - A small complementary-fusion step is used to align with gravity from accelerometer data,
     giving the gyro a heavier weight overall.
*/

#include "orientation_VQF.h"
#include "BNO_LUT.h"
#include <ArduinoEigen.h>
#include <math.h>

// State Vector: [q_w, q_x, q_y, q_z]
static Eigen::VectorXf x_state(4);

// Complementary-fusion weight: fraction by which we move toward
// the accelerometer-based tilt each update (lower = rely on gyro more).
static float kFusionGain = 0.02f;  // 2% tilt correction each update

// Calibration offsets
static float roll_offset  = 0.0f;  // about Y
static float pitch_offset = 0.0f;  // about X
static float yaw_offset   = 0.0f;  // about Z
static bool  calibrated   = false;

void vqfInit()
{
    // Initialize quaternion to [1, 0, 0, 0]
    x_state = Eigen::VectorXf::Zero(4);
    x_state(0) = 1.0f;  // w = 1
}

void vqfPredict(float gx, float gy, float gz, float dt)
{
    // Extract current quaternion
    float qw = x_state(0);
    float qx = x_state(1);
    float qy = x_state(2);
    float qz = x_state(3);

    // Quaternion derivative from gyro
    // (Using standard small-angle quaternion kinematics)
    float dq_w = 0.5f * (-qx * gx - qy * gy - qz * gz);
    float dq_x = 0.5f * ( qw * gx + qy * gz - qz * gy);
    float dq_y = 0.5f * ( qw * gy - qx * gz + qz * gx);
    float dq_z = 0.5f * ( qw * gz + qx * gy - qy * gx);

    // Integrate
    x_state(0) += dq_w * dt;
    x_state(1) += dq_x * dt;
    x_state(2) += dq_y * dt;
    x_state(3) += dq_z * dt;

    // Normalize
    float norm = sqrt(
        x_state(0)*x_state(0) + x_state(1)*x_state(1) +
        x_state(2)*x_state(2) + x_state(3)*x_state(3)
    );
    x_state /= norm;
}

void vqfUpdate(float ax, float ay, float az)
{
    // Check that accelerometer is near 1g:
    float accelNorm = sqrt(ax*ax + ay*ay + az*az);
    if (accelNorm < 0.5f || accelNorm > 1.5f) {
        // If not near 1g, skip tilt correction
        return;
    }

    // Normalize so the magnitude is 1
    ax /= accelNorm;
    ay /= accelNorm;
    az /= accelNorm;

    // We want "Y up," so let's interpret:
    //   roll = rotation about Y,
    //   pitch = rotation about X,
    // Gravity is measured as (ax, ay, az).

    // Let's define:
    //   pitch_acc = angle about X (tilt forward/back),
    //   roll_acc  = angle about Y (tilt left/right).
    //
    // Using geometry: if Y is “up,”
    //   pitch_acc = atan2(-az, sqrt(ax^2 + ay^2))  (rotation about X)
    //   roll_acc  = atan2(ax,  ay)                 (rotation about Y)
    float pitch_acc = atan2(-az, sqrt(ax*ax + ay*ay));
    float roll_acc  = atan2( ax, ay);

    // Build a tilt quaternion from these two Euler angles
    float halfRoll  = roll_acc  * 0.5f;
    float halfPitch = pitch_acc * 0.5f;

    float cy = cos(halfRoll);
    float sy = sin(halfRoll);
    float cx = cos(halfPitch);
    float sx = sin(halfPitch);

    // For Euler angles (roll about Y, pitch about X) => q = qy(roll)*qx(pitch)
    // roll_y = [cy, 0, sy, 0]
    // pitch_x= [cx, sx, 0, 0]
    // Hamilton product => q_meas:
    float qw_meas = cy * cx;
    float qx_meas = cy * sx;
    float qy_meas = sy * cx;
    float qz_meas = -sy * sx;  // sign from Y*X multiplication

    // Current estimate
    float qw_est = x_state(0);
    float qx_est = x_state(1);
    float qy_est = x_state(2);
    float qz_est = x_state(3);

    // Complementary-fusion: blend orientation
    float alpha  = 1.0f - kFusionGain; // fraction to keep from old estimate
    float qw_new = alpha*qw_est + (1.0f - alpha)*qw_meas;
    float qx_new = alpha*qx_est + (1.0f - alpha)*qx_meas;
    float qy_new = alpha*qy_est + (1.0f - alpha)*qy_meas;
    float qz_new = alpha*qz_est + (1.0f - alpha)*qz_meas;

    // Normalize
    float nq = sqrt(qw_new*qw_new + qx_new*qx_new + qy_new*qy_new + qz_new*qz_new);
    x_state(0) = qw_new / nq;
    x_state(1) = qx_new / nq;
    x_state(2) = qy_new / nq;
    x_state(3) = qz_new / nq;
}

void vqfGetEuler(float &roll_deg, float &pitch_deg, float &yaw_deg)
{
    // Standard Tait-Bryan angles (roll about X, pitch about Y, yaw about Z):
    //   roll_x  = atan2(2*(qw*qx + qy*qz), 1 - 2*(qx^2 + qy^2))
    //   pitch_y = asin(2*(qw*qy - qz*qx))
    //   yaw_z   = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))

    float qw = x_state(0);
    float qx = x_state(1);
    float qy = x_state(2);
    float qz = x_state(3);

    float roll_x = atan2(
        2.0f*(qw*qx + qy*qz),
        1.0f - 2.0f*(qx*qx + qy*qy)
    );
    float pitch_y = asinf(
        2.0f*(qw*qy - qz*qx)
    );
    float yaw_z = atan2(
        2.0f*(qw*qz + qx*qy),
        1.0f - 2.0f*(qy*qy + qz*qz)
    );

    // We want final:
    //   roll_deg (rotation about Y)   = pitch_y
    //   pitch_deg (rotation about X)  = roll_x
    //   yaw_deg (rotation about Z)    = yaw_z
    float roll  = pitch_y;
    float pitch = roll_x;
    float yaw   = yaw_z;

    // Convert to degrees
    roll_deg  = roll  * 180.0f / M_PI;
    pitch_deg = pitch * 180.0f / M_PI;
    yaw_deg   = yaw   * 180.0f / M_PI;

    // Apply user-calibration offsets
    if (calibrated) {
        roll_deg  -= roll_offset;
        pitch_deg -= pitch_offset;
        yaw_deg   -= yaw_offset;
    }
}

void vqfCalibrateOrientation()
{
    const unsigned long calibrationTime = 10000; // 10 seconds
    unsigned long startTime = millis();
    unsigned long count = 0;

    float roll_sum  = 0.0f;
    float pitch_sum = 0.0f;
    float yaw_sum   = 0.0f;

    Serial.println("Calibrating orientation... Keep device still, Y up.");

    while (millis() - startTime < calibrationTime) {
        float roll_temp, pitch_temp, yaw_temp;
        vqfGetEuler(roll_temp, pitch_temp, yaw_temp);

        roll_sum  += roll_temp;
        pitch_sum += pitch_temp;
        yaw_sum   += yaw_temp;

        count++;
        delay(10); // ~100 Hz
    }

    roll_offset  = roll_sum / count;
    pitch_offset = pitch_sum / count;
    yaw_offset   = yaw_sum / count;
    calibrated   = true;

    Serial.println("VQF calibration complete:");
    Serial.print("Roll Offset: ");  Serial.println(roll_offset, 3);
    Serial.print("Pitch Offset: "); Serial.println(pitch_offset, 3);
    Serial.print("Yaw Offset: ");   Serial.println(yaw_offset, 3);
}
