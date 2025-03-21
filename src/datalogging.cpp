#include "ekf_sensor_fusion.h"
#include "datalogging.h"
#include "orientation_VQF.h"
#include <SD.h>
#include <vector>
#include <string>
#include <fstream>

// This Part of the Code is responsible for the Data Logging of the IMU, Barometer and EKF Data
// The Data is logged in the form of a .txt file on the SD Card
// The aim is to increase speed and efficiency of the Data Logging Process, Whilst ensuring the size of each file
// is not too large, and data is not lost in the process
// Safety and resistance to power outages is also a key feature of this code
// The data is formatted in a way that is easy to read and understand, and can be imported into MATLAB with ease.

// Setup the File Creation System
void setupFiles() {

    //Initialize the IMU Files
    createFile("IMU_Raw_Data.txt");
    createFile("IMU_Filtered_Data.txt");
    createFile("IMU_Raw_Orientation_Data.txt");
    

    // Initialize Barometer Files
    createFile("Barometer_Raw_Data.txt");
    
    // Initialize EKF Files
    createFile("EKF_Data.txt");
}


// Assume these are member variables of your logger class:
std::vector<std::string> logBuffer;
const size_t BUFFER_THRESHOLD = 100; // Flush after 100 entries
std::ofstream logFile;

// Function to add a log entry
void logData(const std::string &data) {
    logBuffer.push_back(data);
    if (logBuffer.size() >= BUFFER_THRESHOLD) {
        flushBuffer();
    }
}

// Function to flush the buffer to the SD card
void flushBuffer() {
    // Write all buffered lines to the file
    for (const auto &line : logBuffer) {
        logFile << line << "\n";
    }
    logFile.flush(); // Ensures data is written to disk
    logBuffer.clear();
}