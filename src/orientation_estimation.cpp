#include "orientation_estimation.h"

static float integratedRoll = 0.0f;
static float integratedPitch = 0.0f;
static float integratedYaw = 0.0f;

static float filteredGx = 0.0f;
static float filteredGy = 0.0f;
static float filteredGz = 0.0f;

void resetIntegratedAngles() {
    integratedRoll = 0.0f;
    integratedPitch = 0.0f;
    integratedYaw = 0.0f;

    filteredGx = 0.0f;
    filteredGy = 0.0f;
    filteredGz = 0.0f;
}

void updateIntegratedAngles(float gx, float gy, float gz, float dt) {
    const float filterAlpha = 0.7f;
    filteredGx = filterAlpha * gx + (1 - filterAlpha) * filteredGx;
    filteredGy = filterAlpha * gy + (1 - filterAlpha) * filteredGy;
    filteredGz = filterAlpha * gz + (1 - filterAlpha) * filteredGz;

    float k1_roll = filteredGy *  RAD_TO_DEG * RAD_TO_DEG;
    float k2_roll = (filteredGy) *  RAD_TO_DEG * RAD_TO_DEG;
    float k3_roll = (filteredGy) *  RAD_TO_DEG * RAD_TO_DEG;
    float k4_roll = (filteredGy) *  RAD_TO_DEG * RAD_TO_DEG;
    integratedRoll += (dt / 6.0f) * (k1_roll + 2*k2_roll + 2*k3_roll + k4_roll);

    float k1_pitch = filteredGx *  RAD_TO_DEG * RAD_TO_DEG;
    float k2_pitch = (filteredGx) *  RAD_TO_DEG * RAD_TO_DEG;
    float k3_pitch = (filteredGx) *  RAD_TO_DEG * RAD_TO_DEG;
    float k4_pitch = (filteredGx) * RAD_TO_DEG * RAD_TO_DEG;
    integratedPitch += (dt / 6.0f) * (k1_pitch + 2*k2_pitch + 2*k3_pitch + k4_pitch);

    float k1_yaw = filteredGz * RAD_TO_DEG * RAD_TO_DEG;
    float k2_yaw = (filteredGz) * RAD_TO_DEG * RAD_TO_DEG;
    float k3_yaw = (filteredGz) * RAD_TO_DEG * RAD_TO_DEG;
    float k4_yaw = (filteredGz) * RAD_TO_DEG * RAD_TO_DEG;
    integratedYaw += (dt / 6.0f) * (k1_yaw + 2*k2_yaw + 2*k3_yaw + k4_yaw);

    while (integratedRoll < 0) integratedRoll += 360.0f;
    while (integratedRoll >= 360.0f) integratedRoll -= 360.0f;

    while (integratedPitch < 0) integratedPitch += 360.0f;
    while (integratedPitch >= 360.0f) integratedPitch -= 360.0f;

    while (integratedYaw < 0) integratedYaw += 360.0f;
    while (integratedYaw >= 360.0f) integratedYaw -= 360.0f;
}

void getIntegratedAngles(float &roll, float &pitch, float &yaw) {
    roll = integratedRoll;
    pitch = integratedPitch;
    yaw = integratedYaw;
}

