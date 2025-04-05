#ifndef ORIENTATION_ESTIMATION_H
#define ORIENTATION_ESTIMATION_H

#include <Arduino.h> // For float, RAD_TO_DEG

// Functions
void resetIntegratedAngles();
void updateIntegratedAngles(float gx, float gy, float gz, float dt);
void getIntegratedAngles(float& roll, float& pitch, float& yaw);

#endif // ORIENTATION_ESTIMATION_H
