// datalogging.cpp

#include "ekf_sensor_fusion.h"   // For EKF functions like ekfGetState()
#include "orientation_VQF.h"     // For vqfGetEuler() or other orientation-related functions
#include "sensor_setup.h"        // For getSensorData() and getRelativeAltitude()
#include <SD.h>
#include <SPI.h>
#include <Arduino.h>

// ----------------------------------------------------------
// 1) Global File objects (three files: raw data, filtered data, system log)
// ----------------------------------------------------------
File rawDataFile;       // Will store raw IMU + barometer data
File filteredDataFile;  // Will store EKF state + orientation data
File systemLogFile;     // Will store system log messages

// ----------------------------------------------------------
// 2) Buffers and flush thresholds for each file
// ----------------------------------------------------------

// We'll buffer lines to reduce SD writes. Adjust thresholds as needed.
String rawDataBuffer       = "";
String filteredDataBuffer  = "";
String systemLogBuffer     = "";

int rawDataCount           = 0;
int filteredDataCount      = 0;
int systemLogCount         = 0;

const int RAW_BUFFER_THRESHOLD      = 100;
const int FILTERED_BUFFER_THRESHOLD = 100;
const int SYSTEM_LOG_BUFFER_THRESHOLD = 100;

// ----------------------------------------------------------
// 3) Helper: flush any file's buffer
// ----------------------------------------------------------
void flushBuffer(File &file, String &buffer, int &count)
{
  if (file) {
    file.print(buffer);
    file.flush();
  }
  buffer = "";
  count  = 0;
}

// ----------------------------------------------------------
// 4) System log - a separate mechanism for log() calls
// ----------------------------------------------------------
void flushSystemLog()
{
  flushBuffer(systemLogFile, systemLogBuffer, systemLogCount);
}

// This replaces Serial.println(...). Timestamps use micros().
void log(const String &msg)
{
  unsigned long timestamp = micros();
  String line = String(timestamp) + ": " + msg;
  systemLogBuffer += line + "\n";
  systemLogCount++;

  // Auto-flush if threshold reached
  if (systemLogCount >= SYSTEM_LOG_BUFFER_THRESHOLD) {
    flushSystemLog();
  }
}

// ----------------------------------------------------------
// 5) Logging helpers for raw / filtered data
// ----------------------------------------------------------
// Append data to a given buffer, flush if threshold is reached
void logData(File &file, String &buffer, int &count, 
             const String &data, int threshold)
{
  buffer += data + "\n";
  count++;
  if (count >= threshold) {
    flushBuffer(file, buffer, count);
  }
}

// ----------------------------------------------------------
// 6) setupFiles() - open/create the 3 files on SD card
// ----------------------------------------------------------
void setupFiles()
{
  // Example: If using Teensy 4.1's built-in SD, you might do SD.begin(BUILTIN_SDCARD);
  if (!SD.begin(10)) {
    log("SD initialization failed!");
    return;
  }

  // Raw data
  rawDataFile = SD.open("RawDataFile.txt", FILE_WRITE);
  // Filtered data
  filteredDataFile = SD.open("FilteredDataFile.txt", FILE_WRITE);
  // System log
  systemLogFile = SD.open("SystemLogFile.txt", FILE_WRITE);

  if (!rawDataFile || !filteredDataFile || !systemLogFile) {
    log("Failed to open one or more log files!");
  } else {
    log("Successfully opened raw, filtered, and system log files.");
  }

  // Optionally write column headers
  if (rawDataFile) {
    rawDataFile.println("timestamp,ax,ay,az,gx,gy,gz,mx,my,mz,baroAlt");
    rawDataFile.flush();
  }
  if (filteredDataFile) {
    filteredDataFile.println("timestamp,x,y,z,vx,vy,vz,roll,pitch,yaw");
    filteredDataFile.flush();
  }
}

// ----------------------------------------------------------
// 7) Logging raw data (IMU + barometer)
// ----------------------------------------------------------
void logRawData()
{
  // Gather raw IMU data
  float ax, ay, az, gx, gy, gz, mx, my, mz;
  getSensorData(ax, ay, az, gx, gy, gz, mx, my, mz);
  
  // Get barometer altitude
  float baroAlt = getRelativeAltitude();

  // Build CSV line
  unsigned long timestamp = micros();
  String line = String(timestamp) + "," +
                String(ax, 3) + "," + String(ay, 3) + "," + String(az, 3) + "," +
                String(gx, 3) + "," + String(gy, 3) + "," + String(gz, 3) + "," +
                String(mx, 3) + "," + String(my, 3) + "," + String(mz, 3) + "," +
                String(baroAlt, 3);

  // Write to raw data file (with buffering)
  logData(rawDataFile, rawDataBuffer, rawDataCount, line, RAW_BUFFER_THRESHOLD);
}

// ----------------------------------------------------------
// 8) Logging filtered data (EKF + orientation)
// ----------------------------------------------------------
void logFilteredData()
{
  // Get EKF state
  float x, y, z, vx, vy, vz;
  ekfGetState(x, y, z, vx, vy, vz);

  // Get orientation angles (roll, pitch, yaw)
  float roll, pitch, yaw;
  vqfGetEuler(roll, pitch, yaw);

  // Build CSV line
  unsigned long timestamp = micros();
  String line = String(timestamp) + "," +
                String(x, 3) + "," + String(y, 3) + "," + String(z, 3) + "," +
                String(vx, 3) + "," + String(vy, 3) + "," + String(vz, 3) + "," +
                String(roll, 3) + "," + String(pitch, 3) + "," + String(yaw, 3);

  // Write to filtered data file (with buffering)
  logData(filteredDataFile, filteredDataBuffer, filteredDataCount, 
          line, FILTERED_BUFFER_THRESHOLD);
}

// ----------------------------------------------------------
// 9) Combined function to log everything once per loop
// ----------------------------------------------------------
void logSensorData()
{
  // Raw data
  logRawData();
  // Filtered data
  logFilteredData();
}

// ----------------------------------------------------------
// 10) Flush everything
// ----------------------------------------------------------
void flushAllBuffers()
{
  flushBuffer(rawDataFile,       rawDataBuffer,       rawDataCount);
  flushBuffer(filteredDataFile,  filteredDataBuffer,  filteredDataCount);
  flushSystemLog();  // flush the system log buffer
}
