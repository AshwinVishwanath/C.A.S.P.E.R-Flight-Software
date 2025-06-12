#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include "sensor_setup.h"
#include "ekf_sensor_fusion.h"
#include "orientation_estimation.h"
#include "datalogging.h"

enum SystemState {
  INIT,
  CALIBRATION,
  ASCENT,
  DESCENT,
  LANDED
};

unsigned long launchDetectTimestamp = 0;
unsigned long landingDetectTimestamp = 0;
float ascentBufferVyMax = -9999.0f;
float ascentBufferAyMax = -9999.0f;
float maxAltitudeSummary = -9999.0f;
unsigned long ascentEntryTwoSecondsBeforeLaunchIndex = 0;

struct LogEntry {
  String rawLine;
  String filteredLine;
};

const int ASCENT_BUFFER_SIZE = 5000;
LogEntry ascentRollingBuffer[ASCENT_BUFFER_SIZE];
int ascentBufferIndex = 0;
bool ascentBufferFull = false;

bool ascentAccelActive = false;
unsigned long ascentAccelStartTime = 0;

bool ascentDetected = false;
unsigned long ascentStartTime = 0;
float maxAltitude = 0.0;

bool apogeeDetected = false;
unsigned long apogeeTimestamp = 0;

float prevAltitude = 0.0;
unsigned long landingTimerStart = 0;

SystemState systemState = INIT;
unsigned long stateStartTime = 0;
const int LED_PIN = 36;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  stateStartTime = millis();
}

void loop() {
  switch(systemState) {

    case INIT: {
      const unsigned long ledInterval = 500;
      static unsigned long lastLedToggle = 0;
      static bool ledState = false;
      if (millis() - lastLedToggle >= ledInterval) {
        lastLedToggle = millis();
        ledState = !ledState;
        analogWrite(LED_PIN, ledState ? 255 : 0);
      }

      if (millis() - stateStartTime >= 2000) {
        Summarylog("Starting sensor setup (transition INIT->CALIBRATION)...");
        systemState = CALIBRATION;
        stateStartTime = millis();
      }
      break;
    }

    case CALIBRATION: {
      const unsigned long ledInterval = 125;
      static unsigned long lastLedToggle = 0;
      static bool ledState = false;
      if (millis() - lastLedToggle >= ledInterval) {
        lastLedToggle = millis();
        ledState = !ledState;
        analogWrite(LED_PIN, ledState ? 255 : 0);
      }

      setupSensors();
      if (millis() - stateStartTime >= 100) {
        float alt = getRelativeAltitude();
        ekfInit(0.0f, 0.0f, alt, 0.0f, 0.0f, 0.0f);
        Summarylog(String("Calibration complete, Baseline Altitude: ") + alt);
        setupFiles();
        Summarylog("Calibration done. Transitioning to ASCENT state.");
        ascentBufferIndex = 0;
        ascentBufferFull = false;
        ascentAccelActive = false;
        ascentStartTime = 0;
        maxAltitude = alt;
        apogeeDetected = false;
        systemState = ASCENT;
        stateStartTime = millis();
      }
      break;
    }

  case ASCENT: {
      const unsigned long ledInterval = 250;
      static unsigned long lastLedToggle = 0;
      static bool ledState = false;
      if (millis() - lastLedToggle >= ledInterval) {
        lastLedToggle = millis();
        ledState = !ledState;
        analogWrite(LED_PIN, ledState ? 255 : 0);
      }

      static unsigned long loopStartTime = micros();
      const unsigned long loopInterval = 5000;
      unsigned long currentTime = micros();
      static bool ascentBufferDumped = false;

      if (currentTime - loopStartTime >= loopInterval) {
        loopStartTime = currentTime;
        float dt = 0.005f;

        float ax, ay, az, gx, gy, gz, mx, my, mz;
        getSensorData(ax, ay, az, gx, gy, gz, mx, my, mz);
        float relAlt = getRelativeAltitude();
        ekfPredict(ax, ay, az, dt);
        ekfUpdateBaro(relAlt);
        float x, y, z, vx, vy, vz;
        ekfGetState(x, y, z, vx, vy, vz);
        updateIntegratedAngles(gx, gy, gz, dt);
        float Roll, Pitch, Yaw;
        getIntegratedAngles(Roll, Pitch, Yaw);

        if (y > maxAltitude) {
          maxAltitude = y;
        }

        if (!ascentDetected && (vy > 20.0)) {
          ascentDetected = true;
          ascentStartTime = millis();
        }

        launchDetectTimestamp = micros();

        int indexOffset = 2 * 200;
        int twoSecIndex = ascentBufferIndex - indexOffset;
        if (twoSecIndex < 0) twoSecIndex += ASCENT_BUFFER_SIZE;
        ascentEntryTwoSecondsBeforeLaunchIndex = twoSecIndex;

        if (!ascentBufferDumped) {
          unsigned long timestamp = micros();
          String rawLine = String(timestamp) + "," +
                           String(ax, 3) + "," + String(ay, 3) + "," + String(az, 3) + "," +
                           String(gx, 3) + "," + String(gy, 3) + "," + String(gz, 3) + "," +
                           String(mx, 3) + "," + String(my, 3) + "," + String(mz, 3) + "," +
                           String(relAlt, 3);
          String filteredLine = String(timestamp) + "," +
                                String(x, 3) + "," + String(y, 3) + "," + String(z, 3) + "," +
                                String(vx, 3) + "," + String(vy, 3) + "," + String(vz, 3) + "," +
                                String(Roll, 3) + "," + String(Pitch, 3) + "," + String(Yaw, 3);

          ascentRollingBuffer[ascentBufferIndex].rawLine = rawLine;
          ascentRollingBuffer[ascentBufferIndex].filteredLine = filteredLine;
          ascentBufferIndex = (ascentBufferIndex + 1) % ASCENT_BUFFER_SIZE;
          if (ascentBufferIndex == 0) {
            ascentBufferFull = true;
          }

          if (ay > 6.0) {
            if (!ascentAccelActive) {
              ascentAccelActive = true;
              ascentAccelStartTime = millis();
            } else {
              if (millis() - ascentAccelStartTime >= 600) {
                Summarylog("LAUNCH DETECTED; dumping rolling buffer to SD.");
                int count = (ascentBufferFull ? ASCENT_BUFFER_SIZE : ascentBufferIndex);
                for (int i = 0; i < count; i++) {
                  rawDataFile.println(ascentRollingBuffer[i].rawLine);
                  filteredDataFile.println(ascentRollingBuffer[i].filteredLine);
                }
                rawDataFile.flush();
                filteredDataFile.flush();
                ascentBufferDumped = true;
                ascentBufferIndex = 0;
                ascentBufferFull = false;
                ascentAccelActive = false;
              }
            }
          } else {
            ascentAccelActive = false;
          }
        } else {
          logSensorData();
        }
        if (vy > ascentBufferVyMax) ascentBufferVyMax = vy;
        if (ay > ascentBufferAyMax) ascentBufferAyMax = ay;
        if (y > maxAltitudeSummary) maxAltitudeSummary = y;

        if (!apogeeDetected && ascentDetected) {
          if ((millis() - ascentStartTime >= 1000) &&
              (vy < -5.0) &&
              (maxAltitude > 10.0) &&
              (y < maxAltitude)) {
            apogeeDetected = true;
            apogeeTimestamp = millis();
            Summarylog("APOGEE DETECTED.");
            systemState = DESCENT;
            stateStartTime = millis();
          }
        }
      }
      break;
    }

    case DESCENT: {
      const unsigned long ledInterval = 250;
      static unsigned long lastLedToggle = 0;
      static bool ledState = false;
      if (millis() - lastLedToggle >= ledInterval) {
        lastLedToggle = millis();
        ledState = !ledState;
        analogWrite(LED_PIN, ledState ? 255 : 0);
      }

      static unsigned long loopStartTime = micros();
      const unsigned long loopInterval = 1000;
      unsigned long currentTime = micros();
      if (currentTime - loopStartTime >= loopInterval) {
        loopStartTime = currentTime;
        float dt = 0.001f;

        float ax, ay, az, gx, gy, gz, mx, my, mz;
        getSensorData(ax, ay, az, gx, gy, gz, mx, my, mz);
        float relAlt = getRelativeAltitude();
        ekfPredict(ax, ay, az, dt);
        ekfUpdateBaro(relAlt);
        float x, y, z, vx, vy, vz;
        ekfGetState(x, y, z, vx, vy, vz);
        updateIntegratedAngles(gx, gy, gz, dt);
        float Roll, Pitch, Yaw;
        getIntegratedAngles(Roll, Pitch, Yaw);

        static unsigned long lastLogTime = 0;
        if ((millis() - apogeeTimestamp < 5000) &&
            (fabs(vy) <= 20.0)) {
          if (millis() - lastLogTime >= 100) {
            logSensorData();
            lastLogTime = millis();
          }
        } else {
          logSensorData();
        }

        static float prevAltitude = y;
        static unsigned long landingTimerStart = millis();
        if ((fabs(y - prevAltitude) < 1.0) &&
            (fabs(relAlt - prevAltitude) < 1.0)) {
          if (millis() - landingTimerStart >= 5000) {
            Summarylog("Landing detected; transitioning to LANDED state.");
            systemState = LANDED;
            stateStartTime = millis();
            landingDetectTimestamp = millis();

          }
        } else {
          landingTimerStart = millis();
          prevAltitude = y;
        }

        Serial.print(">");
        Serial.print("altitude:"); Serial.print(y, 3);
        Serial.print(",vy:"); Serial.print(vy, 3);
        Serial.print(",roll:"); Serial.print(Roll, 3);
        Serial.print(",pitch:"); Serial.print(Pitch, 3);
        Serial.print(",yaw:"); Serial.print(Yaw, 3);
        Serial.print(",ay:"); Serial.print(ay, 3);
        Serial.println();
      }
      break;
    }

    case LANDED: {
      const unsigned long ledInterval = 500;
      static unsigned long lastLedToggle = 0;
      static bool ledState = false;
      if (millis() - lastLedToggle >= ledInterval) {
        lastLedToggle = millis();
        ledState = !ledState;
        analogWrite(LED_PIN, ledState ? 255 : 0);
      }
      logFilteredData();
      flushAllBuffers();
      Summarylog("Flight ended. Logging stopped.");
      Summarylog("========== FLIGHT SUMMARY ==========");
      Summarylog("Launch detected at: " + String(launchDetectTimestamp) + " \xC2\xB5s");
      String prelaunchLine = ascentRollingBuffer[ascentEntryTwoSecondsBeforeLaunchIndex].filteredLine;
      Summarylog("Entry 2s before launch: " + prelaunchLine);
      float apogeeTimeSec = (float)(apogeeTimestamp - launchDetectTimestamp) / 1000.0f;
      Summarylog("Apogee detected at: " + String(apogeeTimeSec, 2) + " s after launch");
      unsigned long landingTimeCorrected = landingDetectTimestamp - 5000;
      Summarylog("Landing detected at: " + String(landingTimeCorrected) + " ms");
      Summarylog("Max upward velocity (vy): " + String(ascentBufferVyMax, 3) + " m/s");
      Summarylog("Max upward acceleration (ay): " + String(ascentBufferAyMax, 3) + " m/s\xC2\xB2");
      Summarylog("Max altitude: " + String(maxAltitudeSummary, 3) + " m");
      Summarylog("====================================");

      while (1) {
      }
      break;
    }
  }
}

