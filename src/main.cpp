#include <Arduino.h>
#include "sensor_setup.h"
#include "filter.h"

void setup() {
    Serial.begin(115200);
    setupSensors();
    delay(100);

    if (!waitForCalibration()) {
        Serial.println("Calibration failed, cannot proceed.");
        while(1);
    }

    // Initialize EKF with starting position/velocity (assuming stationary at baseline)
    ekfInit(0.0f, 0.0f, baselineAltitude, 0.0f, 0.0f, 0.0f);

    Serial.println("Calibration complete. Starting main loop...");
}

void loop() {
    static unsigned long loopStartTime = micros();
    const unsigned long loopInterval = 1000; // 1ms = 1 kHz update rate

    unsigned long currentTime = micros();
    if (currentTime - loopStartTime >= loopInterval) {
        loopStartTime = currentTime;
        float dt = 0.001f;

        float yaw, pitch, roll;
        float ax_ned, ay_ned, az_ned;
        getCorrectedIMUData(yaw, pitch, roll, ax_ned, ay_ned, az_ned);

        float relativeAltitude = getRelativeAltitude();

        // EKF Predict step with acceleration in NED
        ekfPredict(ax_ned, ay_ned, az_ned, dt);

        // EKF update with barometric altitude
        ekfUpdate(relativeAltitude);

        float x,y,z,vx,vy,vz;
        ekfGetState(x,y,z,vx,vy,vz);

        // Print line in required format for the serial plotter
        Serial.print(">");
        Serial.print("altitude:");Serial.print(z,3);
        Serial.print(",vx:");Serial.print(vx,3);
        Serial.print(",vy:");Serial.print(vy,3);
        Serial.print(",vz:");Serial.print(vz,3);
        Serial.print(",yaw:");Serial.print(yaw*180.0f/M_PI,3);
        Serial.print(",pitch:");Serial.print(pitch*180.0f/M_PI,3);
        Serial.print(",roll:");Serial.print(roll*180.0f/M_PI,3);
        Serial.println(); // \r\n is added automatically by println
    }
}
