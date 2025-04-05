#include "ekf_sensor_fusion.h"       // For EKF functions like ekfGetState()
#include "orientation_estimation.h"  // For orientation estimation functions
#include "sensor_setup.h"            // For getSensorData() and getRelativeAltitude()
#include <SD.h>
#include <SPI.h>
#include <Arduino.h>

// Define BUILTIN_SDCARD if not already defined (Teensy 4.1 built-in SD card usually on pin 10)
#ifndef BUILTIN_SDCARD
  #define BUILTIN_SDCARD 10
#endif

// Use sdCardCS as the chip select pin for the built-in SD card
const int sdCardCS = BUILTIN_SDCARD;

// Number of log entries to buffer before flushing.
const int BUFFER_THRESHOLD = 100;

// Global SD File objects for raw and filtered sensor data.
File rawDataFile;       // File for raw sensor/barometer data.
File filteredDataFile;  // File for filtered EKF/orientation estimation data.

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

// Create a file with a base name and unique file number (e.g., "Raw_Sensor_Data_1.csv", "Raw_Sensor_Data_2.csv", etc.)
void createFile(File &file, const char *baseName) {
  int fileNumber = 1;
  char fileName[32];
  while (true) {
    snprintf(fileName, sizeof(fileName), "%s_%d.csv", baseName, fileNumber);
    if (!SD.exists(fileName)) {
      file = SD.open(fileName, FILE_WRITE);
      if (!file) {
        Serial.println("Failed to open file");
        while (1);  // Halt if file cannot be created.
      }
      Serial.print("Created file: ");
      Serial.println(fileName);
      break;
    }
    fileNumber++;
  }
}

// Setup the SD card files for logging using the built-in SD card on Teensy 4.1.
// This function creates unique files and writes header lines.
void setupFiles() {
  if (!SD.begin(sdCardCS)) {
    Serial.println("SD initialization failed!");
    return;
  }
  
  // Create unique log files using a base name.
  createFile(rawDataFile, "Raw_Sensor_Data");
  createFile(filteredDataFile, "Filtered_Sensor_Data");
  
  // Write header lines to each file.
  rawDataFile.println("Time,ax,ay,az,gx,gy,gz,mx,my,mz,relativeAltitude");
  rawDataFile.flush();
  
  filteredDataFile.println("Time,x,y,z,vx,vy,vz,roll,pitch,yaw");
  filteredDataFile.flush();
}

// Logs raw sensor data from the IMU and barometer.
void logRawSensorData() {
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  // Get raw sensor data.
  getSensorData(ax, ay, az, gx, gy, gz, mx, my, mz);
  // Get relative altitude.
  float relAlt = getRelativeAltitude();
  // Get timestamp.
  unsigned long timestamp = micros();
  
  // Format CSV: timestamp, ax, ay, az, gx, gy, gz, mx, my, mz, relative altitude
  String line = String(timestamp) + "," +
                String(ax, 3) + "," + String(ay, 3) + "," + String(az, 3) + "," +
                String(gx, 3) + "," + String(gy, 3) + "," + String(gz, 3) + "," +
                String(mx, 3) + "," + String(my, 3) + "," + String(mz, 3) + "," +
                String(relAlt, 3);
  
  logData(rawDataFile, rawBuffer, rawCount, line);
}

// Logs filtered sensor data from the EKF and integrated orientation estimation.
void logFilteredSensorData() {
  float x, y, z, vx, vy, vz;
  // Get the EKF state.
  ekfGetState(x, y, z, vx, vy, vz);
  
  float roll, pitch, yaw;
  // Get the integrated orientation (in degrees).
  getIntegratedAngles(roll, pitch, yaw);
  
  // Get timestamp.
  unsigned long timestamp = micros();
  
  // Format CSV: timestamp, x, y, z, vx, vy, vz, roll, pitch, yaw
  String line = String(timestamp) + "," +
                String(x, 3) + "," + String(y, 3) + "," + String(z, 3) + "," +
                String(vx, 3) + "," + String(vy, 3) + "," + String(vz, 3) + "," +
                String(roll, 3) + "," + String(pitch, 3) + "," + String(yaw, 3);
                
  logData(filteredDataFile, filteredBuffer, filteredCount, line);
}

// High-level function: logs both raw and filtered sensor data.
// This function is intended to be called in your main loop.
void logSensorData() {
  logRawSensorData();
  logFilteredSensorData();
}

// Flushes both raw and filtered data buffers (e.g., on shutdown or periodically).
void flushAllBuffers() {
  flushBuffer(rawDataFile, rawBuffer, rawCount);
  flushBuffer(filteredDataFile, filteredBuffer, filteredCount);
}
