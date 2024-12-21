#ifndef ORIENTATION_VQF_H
#define ORIENTATION_VQF_H

#include <Arduino.h>

// Function declarations
void vqfInit();
void vqfPredict(float gx, float gy, float gz, float dt);
void vqfUpdate(float ax, float ay, float az, float mx, float my, float mz);
void vqfGetEuler(float &roll, float &pitch, float &yaw);
void vqfCalibrateOrientation();

#endif // ORIENTATION_VQF_H
