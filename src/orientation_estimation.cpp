#include "orientation_estimation.h"

// Static variables to hold the integrated angles (degrees)
static float integratedRoll = 0.0f;
static float integratedPitch = 0.0f;
static float integratedYaw = 0.0f;

// Low-pass filter state for gyroscope measurements
static float filteredGx = 0.0f;
static float filteredGy = 0.0f;
static float filteredGz = 0.0f;

// Reset function
void resetIntegratedAngles() {
    integratedRoll = 0.0f;
    integratedPitch = 0.0f;
    integratedYaw = 0.0f;
    
    filteredGx = 0.0f;
    filteredGy = 0.0f;
    filteredGz = 0.0f;
}

// Update integration from gyroscope readings (in rad/s) with low-pass filtering
void updateIntegratedAngles(float gx, float gy, float gz, float dt) {
    // Filter the gyroscope readings using an exponential filter.
    // Adjust filterAlpha between 0 (heavy filtering) and 1 (no filtering)
    const float filterAlpha = 0.5f;  
    filteredGx = filterAlpha * gx + (1 - filterAlpha) * filteredGx;
    filteredGy = filterAlpha * gy + (1 - filterAlpha) * filteredGy;
    filteredGz = filterAlpha * gz + (1 - filterAlpha) * filteredGz;
    
    // Integrate the filtered values. 
    // Note: RAD_TO_DEG converts radian/s to degrees/s. (Typically only one conversion is needed.)
    integratedRoll  += filteredGy * dt * RAD_TO_DEG * RAD_TO_DEG;
    integratedPitch += filteredGx * dt * RAD_TO_DEG * RAD_TO_DEG;
    integratedYaw   += filteredGz * dt * RAD_TO_DEG * RAD_TO_DEG;

    // Wrap angles to stay between 0 and 360 degrees
    while (integratedRoll < 0) integratedRoll += 360.0f;
    while (integratedRoll >= 360.0f) integratedRoll -= 360.0f;

    while (integratedPitch < 0) integratedPitch += 360.0f;
    while (integratedPitch >= 360.0f) integratedPitch -= 360.0f;

    while (integratedYaw < 0) integratedYaw += 360.0f;
    while (integratedYaw >= 360.0f) integratedYaw -= 360.0f;
}

// Function to read the integrated angles
void getIntegratedAngles(float &roll, float &pitch, float &yaw) {
    roll = integratedRoll;
    pitch = integratedPitch;
    yaw = integratedYaw;
}
