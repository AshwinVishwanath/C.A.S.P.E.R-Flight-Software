#include <Arduino.h>
#include "sensor_setup.h"
#include "ekf_sensor_fusion.h"
#include "orientation_VQF.h" // Include VQF header for orientation tracking

void setup() {
    Serial.begin(115200);
    delay(2000); // Wait for Serial Monitor to connect
    Serial.println("Starting sensor setup...");

    // Setup BNO055 and BMP388 sensors
    setupSensors();
    delay(100);

    // Initialize EKF with starting position, velocity, and altitude
    float baselineAltitude = getRelativeAltitude(); // Ensure baseline altitude is read during setup
    ekfInit(0.0f, 0.0f, baselineAltitude, 0.0f, 0.0f, 0.0f);
    Serial.println("EKF initialized with starting state:");
    Serial.print("Baseline Altitude: ");
    Serial.println(baselineAltitude, 3);

    // Initialize VQF for orientation tracking
    vqfInit();
    vqfCalibrateOrientation(); // Perform 10-second calibration for roll, pitch, yaw offsets
    Serial.println("VQF initialized and calibrated for roll, pitch, and yaw tracking.");

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
        float ax, ay, az;
        float gx, gy, gz;
        float mx, my, mz;

        // Get raw IMU data
        getSensorData(ax, ay, az, gx, gy, gz, mx, my, mz);

        // Get relative altitude from BMP388
        float relativeAltitude = getRelativeAltitude();

        // **Update EKF for Position, Velocity, and Altitude**
        ekfPredict(ax, ay, az, dt);
        ekfUpdate(ax, ay, az, relativeAltitude);

        // **Update VQF for Orientation (Roll, Pitch, Yaw)**
        vqfPredict(gx, gy, gz, dt);
        vqfUpdate(ax, ay, az, mx, my, mz);

        // Get the current state from the EKF
        float x, y, z, vx, vy, vz;
        ekfGetState(x, y, z, vx, vy, vz);

        // Get orientation from the VQF
        float roll, pitch, yaw;
        vqfGetEuler(roll, pitch, yaw);

        // **Print the current state for analysis**
        Serial.print(">");
        Serial.print("altitude:"); Serial.print(z, 3);
        Serial.print(",vx:"); Serial.print(vx, 3);
        Serial.print(",vy:"); Serial.print(vy, 3);
        Serial.print(",vz:"); Serial.print(vz, 3);
        Serial.print(",x:"); Serial.print(x, 3);
        Serial.print(",y:"); Serial.print(y, 3);
        Serial.print(",roll:"); Serial.print(roll, 3); // Roll in degrees
        Serial.print(",pitch:"); Serial.print(pitch, 3); // Pitch in degrees
        Serial.print(",yaw:"); Serial.print(yaw, 3); // Yaw in degrees
        Serial.println();
    }
}
