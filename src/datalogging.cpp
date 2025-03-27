// datalogging.cpp
// This file handles logging sensor data to the SD card.
// It now logs two types of data:
// 1. Raw sensor data: direct IMU readings (accelerometer, gyroscope, magnetometer)
//    plus barometric altitude.
// 2. Filtered sensor data: EKF sensor fusion results (position and velocity)
//    and VQF orientation (roll, pitch, yaw).
//
// Data are buffered and then flushed to the SD card to reduce write overhead.

#include "ekf_sensor_fusion.h"    // For EKF functions like ekfGetState()
#include "orientation_VQF.h"      // For VQF functions (e.g., vqfGetEuler)
#include "sensor_setup.h"         // For getSensorData() and getRelativeAltitude()
#include <SD.h>
#include <SPI.h>
#include <Arduino.h>

// Number of log entries to buffer before flushing.
const int BUFFER_THRESHOLD = 100;

// Global SD File objects for raw and filtered sensor data.
File rawDataFile;       // File for raw IMU/barometer data.
File filteredDataFile;  // File for filtered EKF/VQF data.

// Global string buffers and counters for each log file.
String rawBuffer = "";
String filteredBuffer = "";
int rawCount = 0;
int filteredCount = 0;

// Flushes the provided buffer to its SD file and resets the counter.
void flushBuffer(File &file, String &buffer, int &count) {
  if (file) {
    file.print(buffer);
    file.flush();  // Ensure data is written to disk.
  }
  buffer = "";
  count = 0;
}

// Appends a log entry to the buffer and flushes if the threshold is reached.
void logData(File &file, String &buffer, int &count, const String &data) {
  buffer += data + "\n";
  count++;
  if (count >= BUFFER_THRESHOLD) {
    flushBuffer(file, buffer, count);
  }
}

// Setup the SD card files for logging.
// Raw sensor data is written to "Raw_Sensor_Data.txt".
// Filtered sensor data is written to "Filtered_Sensor_Data.txt".
// Adjust the chip-select pin (here 10 is assumed) as needed.
void setupFiles() {
  if (!SD.begin(10)) {
    Serial.println("SD initialization failed!");
    return;
  }
  rawDataFile = SD.open("Raw_Sensor_Data.txt", FILE_WRITE);
  filteredDataFile = SD.open("Filtered_Sensor_Data.txt", FILE_WRITE);
  
  if (!rawDataFile || !filteredDataFile) {
    Serial.println("Failed to open one or more log files!");
  }
}

// Logs raw sensor data from the IMU and barometer.
// Uses getSensorData() to get accelerometer, gyroscope, and magnetometer readings,
// and getRelativeAltitude() for the barometric altitude.
// The data are formatted as a CSV line.
void logRawSensorData() {
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  // Get raw IMU data (accelerometer, gyroscope, magnetometer).
  getSensorData(ax, ay, az, gx, gy, gz, mx, my, mz);
  
  // Get barometric altitude.
  float relAlt = getRelativeAltitude();
  
  // Timestamp in microseconds.
  unsigned long timestamp = micros();
  
  // CSV format: timestamp, ax, ay, az, gx, gy, gz, mx, my, mz, relative altitude
  String line = String(timestamp) + "," +
                String(ax, 3) + "," + String(ay, 3) + "," + String(az, 3) + "," +
                String(gx, 3) + "," + String(gy, 3) + "," + String(gz, 3) + "," +
                String(mx, 3) + "," + String(my, 3) + "," + String(mz, 3) + "," +
                String(relAlt, 3);
  
  logData(rawDataFile, rawBuffer, rawCount, line);
}

// Logs filtered sensor data from the EKF and VQF.
// The EKF provides the fused state (position and velocity) and
// VQF provides the filtered orientation (roll, pitch, yaw).
// The data are formatted as a CSV line.
void logFilteredSensorData() {
  float x, y, z, vx, vy, vz;
  // Get the current state from the EKF.
  ekfGetState(x, y, z, vx, vy, vz);
  
  float roll, pitch, yaw;
  // Get the orientation in Euler angles from VQF.
  vqfGetEuler(roll, pitch, yaw);
  
  // Timestamp in microseconds.
  unsigned long timestamp = micros();
  
  // CSV format: timestamp, x, y, z, vx, vy, vz, roll, pitch, yaw
  String line = String(timestamp) + "," +
                String(x, 3) + "," + String(y, 3) + "," + String(z, 3) + "," +
                String(vx, 3) + "," + String(vy, 3) + "," + String(vz, 3) + "," +
                String(roll, 3) + "," + String(pitch, 3) + "," + String(yaw, 3);
  
  logData(filteredDataFile, filteredBuffer, filteredCount, line);
}

// High-level function: logs both raw and filtered sensor data.
// This function is called in the main loop.
void logSensorData() {
  logRawSensorData();
  logFilteredSensorData();
}

// Flushes both raw and filtered data buffers (e.g., on shutdown or periodically).
void flushAllBuffers() {
  flushBuffer(rawDataFile, rawBuffer, rawCount);
  flushBuffer(filteredDataFile, filteredBuffer, filteredCount);
}