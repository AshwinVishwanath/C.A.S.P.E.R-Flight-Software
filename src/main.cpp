#include <Arduino.h>
#include "sensor_setup.h"
#include "ekf_sensor_fusion.h"

void setup() {
    Serial.begin(115200);
    delay(2000); // Wait for Serial Monitor to connect
    Serial.println("Starting sensor setup...");

    // Setup BNO055 and BMP388 sensors
    setupSensors();
    delay(100);

    // Initialize EKF with starting position/velocity (assuming stationary at baseline)
    ekfInit(0.0f, 0.0f, baselineAltitude, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    Serial.println("EKF initialized with starting state:");
    Serial.print("Baseline Altitude: "); 
    Serial.println(baselineAltitude, 3);

    Serial.println("Setup complete. Starting main loop...");
}

void loop() {
    static unsigned long loopStartTime = micros();
    const unsigned long loopInterval = 1000; // 1ms = 1 kHz update rate

    unsigned long currentTime = micros();
    if (currentTime - loopStartTime >= loopInterval) {
        loopStartTime = currentTime;
        float dt = 0.001f; // 1 millisecond time step

        // Variables to hold IMU data
        float yaw, pitch, roll;
        float ax_ned, ay_ned, az_ned;
        float mx, my, mz;

        // Get IMU data (Yaw, Pitch, Roll, Accelerometer, Magnetometer)
        getCorrectedIMUData(yaw, pitch, roll, ax_ned, ay_ned, az_ned, mx, my, mz);

        // Get relative altitude from BMP388
        float relativeAltitude = getRelativeAltitude();

        // EKF Predict step with NED accelerations and orientation from BNO055
        ekfPredict(ax_ned, ay_ned, az_ned, yaw, pitch, roll, dt);

        // EKF update with accelerometer, magnetometer, and barometric altitude
        ekfUpdate(ax_ned, ay_ned, az_ned, mx, my, mz, relativeAltitude);

        // Get the current state from the EKF
        float x, y, z, vx, vy, vz, roll_est, pitch_est, yaw_est;
        ekfGetState(x, y, z, vx, vy, vz, roll_est, pitch_est, yaw_est);

        // Print the current state for analysis
        Serial.print(">");
        Serial.print("altitude:"); Serial.print(z, 3);
        Serial.print(",vx:"); Serial.print(vx, 3);
        Serial.print(",vy:"); Serial.print(vy, 3);
        Serial.print(",vz:"); Serial.print(vz, 3);
        Serial.print(",yaw:"); Serial.print(yaw_est * 180.0f / M_PI, 3);
        Serial.print(",pitch:"); Serial.print(pitch_est * 180.0f / M_PI, 3);
        Serial.print(",roll:"); Serial.print(roll_est * 180.0f / M_PI, 3);
        Serial.println();
    }
}
