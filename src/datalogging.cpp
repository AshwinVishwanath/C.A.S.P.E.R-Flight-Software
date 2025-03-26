// datalogging.cpp
#include "ekf_sensor_fusion.h"    // For EKF functions like ekfGetState()
#include "datalogging.h"          // Declarations for these logging functions
#include "orientation_VQF.h"      // For VQF functions (e.g., vqfGetEuler)
#include "sensor_setup.h"         // For getCorrectedIMUData() and getRelativeAltitude()
#include "filter.h"               // For EKF functions (ekfPredict, ekfUpdate, etc.)
#include <SD.h>
#include <SPI.h>
#include <Arduino.h>

// Set the number of log entries to buffer before flushing.
const int BUFFER_THRESHOLD = 100;

// Global SD File objects for each log file
File imuRawFile;
File imuFilteredFile;
File imuOrientationFile;
File barometerFile;
File ekfFile;

// Global String buffers and counters
String imuRawBuffer = "";
String imuFilteredBuffer = "";
String imuOrientationBuffer = "";
String barometerBuffer = "";
String ekfBuffer = "";
int imuRawCount = 0;
int imuFilteredCount = 0;
int imuOrientationCount = 0;
int barometerCount = 0;
int ekfCount = 0;

// Helper function: flush buffer for a given SD File and reset counter.
void flushBuffer(File &file, String &buffer, int &count) {
  if (file) {
    file.print(buffer);
    file.flush();  // Ensure data is written to disk
  }
  buffer = "";
  count = 0;
}

// Add a log entry to the specified buffer and flush when threshold is reached.
void logData(File &file, String &buffer, int &count, const String &data) {
  buffer += data + "\n";
  count++;
  if (count >= BUFFER_THRESHOLD) {
    flushBuffer(file, buffer, count);
  }
}

// Setup the SD files – open (or create) each file for writing.
// Adjust the chip–select pin (here 10 is assumed) as needed.
void setupFiles() {
  if (!SD.begin(10)) {
    Serial.println("SD initialization failed!");
    return;
  }
  imuRawFile = SD.open("IMU_Raw_Data.txt", FILE_WRITE);
  imuFilteredFile = SD.open("IMU_Filtered_Data.txt", FILE_WRITE);
  imuOrientationFile = SD.open("IMU_Raw_Orientation_Data.txt", FILE_WRITE);
  barometerFile = SD.open("Barometer_Raw_Data.txt", FILE_WRITE);
  ekfFile = SD.open("EKF_Data.txt", FILE_WRITE);

  if (!imuRawFile || !imuFilteredFile || !imuOrientationFile ||
      !barometerFile || !ekfFile) {
    Serial.println("Failed to open one or more log files!");
  }
}


// functions to retrieve data from other modules and log it.


// Log raw IMU sensor readings (accelerometer, gyroscope, magnetometer)
void logIMURawData() {
  sensors_event_t accelEvent, gyroEvent, magEvent;
  // Get sensor events directly from BNO055
  // (Assuming a global BNO055 object is available in sensor_setup.cpp)
  // If not, modify to use the proper instance.
  extern Adafruit_BNO055 bno;  
  bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
  bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  unsigned long timestamp = micros();
  String line = String(timestamp) + "," +
                String(accelEvent.acceleration.x, 3) + "," +
                String(accelEvent.acceleration.y, 3) + "," +
                String(accelEvent.acceleration.z, 3) + "," +
                String(gyroEvent.gyro.x, 3) + "," +
                String(gyroEvent.gyro.y, 3) + "," +
                String(gyroEvent.gyro.z, 3) + "," +
                String(magEvent.magnetic.x, 3) + "," +
                String(magEvent.magnetic.y, 3) + "," +
                String(magEvent.magnetic.z, 3);
  logData(imuRawFile, imuRawBuffer, imuRawCount, line);
}

// Log filtered IMU data (accelerometer rotated to NED and offsets applied)
void logIMUFilteredData() {
  float yaw, pitch, roll;
  float ax_ned, ay_ned, az_ned;
  getCorrectedIMUData(yaw, pitch, roll, ax_ned, ay_ned, az_ned);
  unsigned long timestamp = micros();
  String line = String(timestamp) + "," +
                String(yaw, 3) + "," +
                String(pitch, 3) + "," +
                String(roll, 3) + "," +
                String(ax_ned, 3) + "," +
                String(ay_ned, 3) + "," +
                String(az_ned, 3);
  logData(imuFilteredFile, imuFilteredBuffer, imuFilteredCount, line);
}

// Log orientation data from the VQF algorithm (as Euler angles).
// Here we assume orientation_VQF.h provides vqfGetEuler().
void logIMURawOrientationData() {
  float roll, pitch, yaw;
  vqfGetEuler(roll, pitch, yaw);
  unsigned long timestamp = micros();
  String line = String(timestamp) + "," +
                String(roll, 3) + "," +
                String(pitch, 3) + "," +
                String(yaw, 3);
  logData(imuOrientationFile, imuOrientationBuffer, imuOrientationCount, line);
}

// Log barometric altitude data (using getRelativeAltitude())
void logBarometerData() {
  float relAlt = getRelativeAltitude();
  unsigned long timestamp = micros();
  String line = String(timestamp) + "," + String(relAlt, 3);
  logData(barometerFile, barometerBuffer, barometerCount, line);
}

// Log EKF sensor fusion data (state estimate: position and velocity)
void logEKFData() {
  float x, y, z, vx, vy, vz;
  ekfGetState(x, y, z, vx, vy, vz);
  unsigned long timestamp = micros();
  String line = String(timestamp) + "," +
                String(x, 3) + "," +
                String(y, 3) + "," +
                String(z, 3) + "," +
                String(vx, 3) + "," +
                String(vy, 3) + "," +
                String(vz, 3);
  logData(ekfFile, ekfBuffer, ekfCount, line);
}

// High-level function: call this from your main loop to log all sensor data.
void logSensorData() {
  logIMURawData();
  logIMUFilteredData();
  logIMURawOrientationData();
  logBarometerData();
  logEKFData();
}

// Optionally flush all buffers (e.g. on shutdown or periodically)
void flushAllBuffers() {
  flushBuffer(imuRawFile, imuRawBuffer, imuRawCount);
  flushBuffer(imuFilteredFile, imuFilteredBuffer, imuFilteredCount);
  flushBuffer(imuOrientationFile, imuOrientationBuffer, imuOrientationCount);
  flushBuffer(barometerFile, barometerBuffer, barometerCount);
  flushBuffer(ekfFile, ekfBuffer, ekfCount);
}
