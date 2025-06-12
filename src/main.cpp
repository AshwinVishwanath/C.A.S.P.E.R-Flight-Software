#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include "sensor_setup.h"
#include "ekf_sensor_fusion.h"
#include "orientation_estimation.h" // Include VQF header for orientation tracking
#include "datalogging.h"     // Added to integrate datalogging functions

enum SystemState {
    INIT_WAIT,
    SENSOR_SETUP,
    RUNNING
  };
  
  SystemState systemState = INIT_WAIT;
  unsigned long stateStartTime = 0;
  
  void setup() {
    Serial.begin(115200);
    stateStartTime = millis();  // record the start time for waiting
  }
  
  void loop() {
    switch(systemState) {
      case INIT_WAIT:
        // Wait for 2000ms without blocking other operations
        if (millis() - stateStartTime >= 2000) {
          Serial.println("Starting sensor setup...");
          systemState = SENSOR_SETUP;
          stateStartTime = millis();  // restart timer for next state
        }
        break;
  
      case SENSOR_SETUP:
        // Do sensor initialization and wait a short period if necessary
        setupSensors();
        // You can perform a non-blocking wait of 100ms here if desired:
        if (millis() - stateStartTime >= 100) {
          float alt = getRelativeAltitude();
          ekfInit(0.0f, 0.0f, alt, 0.0f, 0.0f, 0.0f);
          Serial.println("EKF initialized with starting state:");
          Serial.print("Baseline Altitude: ");
          Serial.println(alt, 3);
          setupFiles();
          Serial.println("Setup complete. Starting main loop...");
          systemState = RUNNING;
        }
        break;
  
      case RUNNING:
        // Your normal loop code here (run at 1kHz update rate)
        static unsigned long loopStartTime = micros();
        const unsigned long loopInterval = 1000; // microseconds (1ms)
        unsigned long currentTime = micros();
        if (currentTime - loopStartTime >= loopInterval) {
          loopStartTime = currentTime;
          float dt = 0.001f; // 1ms time step
  
          float ax, ay, az, gx, gy, gz, mx, my, mz;
          getSensorData(ax, ay, az, gx, gy, gz, mx, my, mz);
          float relAlt = getRelativeAltitude();
  
          ekfPredict(ax, ay, az, dt);
          ekfUpdate(ax, ay, az, relAlt);
          float x, y, z, vx, vy, vz;
          ekfGetState(x, y, z, vx, vy, vz);
  
          updateIntegratedAngles(gx, gy, gz, dt);
          float Roll, Pitch, Yaw;
          getIntegratedAngles(Roll, Pitch, Yaw);
  
          Serial.print(">");
          Serial.print("altitude:"); Serial.print(z, 3);
          Serial.print(",vx:"); Serial.print(vx, 3);
          Serial.print(",vy:"); Serial.print(vy, 3);
          Serial.print(",vz:"); Serial.print(vz, 3);
          Serial.print(",x:"); Serial.print(x, 3);
          Serial.print(",y:"); Serial.print(y, 3);
          Serial.print(",roll:"); Serial.print(Roll, 3);
          Serial.print(",pitch:"); Serial.print(Pitch, 3);
          Serial.print(",yaw:"); Serial.print(Yaw, 3);
          Serial.println();
  
          logSensorData();
        }
        break;
    }
  }
  