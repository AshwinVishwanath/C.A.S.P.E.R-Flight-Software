#ifndef EKF_SENSOR_FUSION_H
#define EKF_SENSOR_FUSION_H

void ekfInit(float x, float y, float z, float vx, float vy, float vz, float roll, float pitch, float yaw);
void ekfPredict(float ax, float ay, float az, float gx, float gy, float gz, float dt);
void ekfUpdate(float ax_meas, float ay_meas, float az_meas, float mx_meas, float my_meas, float mz_meas, float alt_meas);
void ekfGetState(float &x, float &y, float &z, float &vx, float &vy, float &vz, float &roll, float &pitch, float &yaw);

#endif // EKF_SENSOR_FUSION_H
