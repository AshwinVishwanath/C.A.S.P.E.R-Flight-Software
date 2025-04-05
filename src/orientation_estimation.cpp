#include "orientation_estimation.h"

// Static variables to hold the integrated angles (degrees)
static float integratedRoll = 0.0f;
static float integratedPitch = 0.0f;
static float integratedYaw = 0.0f;

// Reset function
void resetIntegratedAngles() {
    integratedRoll = 0.0f;
    integratedPitch = 0.0f;
    integratedYaw = 0.0f;
}

// Update integration from gyroscope readings (in rad/s)
void updateIntegratedAngles(float gx, float gy, float gz, float dt) {
    integratedRoll  += gy * dt * RAD_TO_DEG; // gy is roll rate in rad/s
    integratedPitch += gx * dt * RAD_TO_DEG; // gx is pitch rate in rad/s
    integratedYaw   += gz * dt * RAD_TO_DEG; // gz is yaw rate in rad/s

    // Wrap angles to stay between -180 and 180 degrees
    if (integratedRoll > 180.0f) integratedRoll -= 360.0f;
    if (integratedRoll < -180.0f) integratedRoll += 360.0f;

    if (integratedPitch > 180.0f) integratedPitch -= 360.0f;
    if (integratedPitch < -180.0f) integratedPitch += 360.0f;

    if (integratedYaw > 180.0f) integratedYaw -= 360.0f;
    if (integratedYaw < -180.0f) integratedYaw += 360.0f;
}

// Function to read the integrated angles
void getIntegratedAngles(float& roll, float& pitch, float& yaw) {
    roll = integratedRoll;
    pitch = integratedPitch;
    yaw = integratedYaw;
}
