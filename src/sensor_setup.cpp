#include "sensor_setup.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <math.h>

float baselineAltitude = 0.0f;
float accel_offsets[3] = {0,0,0};
float mag_offsets[3] = {0,0,0};

// Internal variables
static Adafruit_BNO055 bno = Adafruit_BNO055(55);
static Adafruit_BMP3XX bmp;

static const float alpha_filter = 0.5f;
static float filteredAltitude = 0.0f;
static float lastAltitude = 0.0f;
static unsigned long lastPressureTime = 0;
static const unsigned long pressureInterval = 10; // ~100 Hz

// from previous code
static float getBaroAltitude() {
    unsigned long currentTime = millis();
    if (currentTime - lastPressureTime >= pressureInterval) {
        if (bmp.performReading()) {
            float alt = bmp.readAltitude(1013.25f);
            lastAltitude = alt;
        }
        lastPressureTime = currentTime;
    }
    return lastAltitude;
}

static bool isStationary(float accelSamples[][3], float gyroSamples[][3], float altSamples[], int count) {
    float ACCEL_VARIANCE_THRESHOLD = 0.1f;
    float GYRO_VARIANCE_THRESHOLD = 0.005f;
    float ALT_VARIANCE_THRESHOLD = 0.2f;

    float ax_mean=0, ay_mean=0, az_mean=0;
    float gx_mean=0, gy_mean=0, gz_mean=0;
    float alt_mean=0;

    for (int i=0;i<count;i++){
        ax_mean+=accelSamples[i][0];
        ay_mean+=accelSamples[i][1];
        az_mean+=accelSamples[i][2];
        gx_mean+=gyroSamples[i][0];
        gy_mean+=gyroSamples[i][1];
        gz_mean+=gyroSamples[i][2];
        alt_mean+=altSamples[i];
    }
    ax_mean/=count; ay_mean/=count; az_mean/=count;
    gx_mean/=count; gy_mean/=count; gz_mean/=count;
    alt_mean/=count;

    float ax_var=0, ay_var=0, az_var=0;
    float gx_var=0, gy_var=0, gz_var=0;
    float alt_var=0;

    for (int i=0;i<count;i++){
        float dax = accelSamples[i][0]-ax_mean;
        float day = accelSamples[i][1]-ay_mean;
        float daz = accelSamples[i][2]-az_mean;

        float dgx = gyroSamples[i][0]-gx_mean;
        float dgy = gyroSamples[i][1]-gy_mean;
        float dgz = gyroSamples[i][2]-gz_mean;

        float dalt = altSamples[i]-alt_mean;

        ax_var+=dax*dax; ay_var+=day*day; az_var+=daz*daz;
        gx_var+=dgx*dgx; gy_var+=dgy*dgy; gz_var+=dgz*dgz;
        alt_var+=dalt*dalt;
    }

    ax_var/=count; ay_var/=count; az_var/=count;
    gx_var/=count; gy_var/=count; gz_var/=count;
    alt_var/=count;

    bool accel_stationary = (ax_var<ACCEL_VARIANCE_THRESHOLD && ay_var<ACCEL_VARIANCE_THRESHOLD && az_var<ACCEL_VARIANCE_THRESHOLD);
    bool gyro_stationary = (gx_var<GYRO_VARIANCE_THRESHOLD && gy_var<GYRO_VARIANCE_THRESHOLD && gz_var<GYRO_VARIANCE_THRESHOLD);
    bool alt_stationary = (alt_var<ALT_VARIANCE_THRESHOLD);

    return accel_stationary && gyro_stationary && alt_stationary;
}

void setupSensors() {
    if (!bno.begin()) {
        Serial.println("BNO055 initialization failed!");
        while (1);
    }
    bno.setExtCrystalUse(true);

    if (!bmp.begin_I2C(0x76, &Wire1)) {
        Serial.println("BMP388 initialization failed!");
        while (1);
    }

    bmp.setPressureOversampling(BMP3_NO_OVERSAMPLING);
    bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
    bmp.setOutputDataRate(BMP3_ODR_100_HZ);
}

bool waitForCalibration() {
    Serial.println("Starting calibration process. Keep the sensor suite stationary...");
    static const int STATIONARY_WINDOW = 200;
    float accelSamples[STATIONARY_WINDOW][3];
    float gyroSamples[STATIONARY_WINDOW][3];
    float altSamples[STATIONARY_WINDOW];
    float magSamples[STATIONARY_WINDOW][3];

    while (true) {
        for (int i=0;i<STATIONARY_WINDOW;i++){
            sensors_event_t accelEvent, gyroEvent, magEvent;
            bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
            bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
            bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);

            float rawAltitude = getBaroAltitude();
            filteredAltitude = (1 - alpha_filter)*filteredAltitude + alpha_filter*rawAltitude;

            accelSamples[i][0]=accelEvent.acceleration.x;
            accelSamples[i][1]=accelEvent.acceleration.y;
            accelSamples[i][2]=accelEvent.acceleration.z;

            gyroSamples[i][0]=gyroEvent.gyro.x;
            gyroSamples[i][1]=gyroEvent.gyro.y;
            gyroSamples[i][2]=gyroEvent.gyro.z;

            magSamples[i][0]=magEvent.magnetic.x;
            magSamples[i][1]=magEvent.magnetic.y;
            magSamples[i][2]=magEvent.magnetic.z;

            altSamples[i]=filteredAltitude;

            delayMicroseconds(1000); // 1 ms per sample
        }

        if (isStationary(accelSamples, gyroSamples, altSamples, STATIONARY_WINDOW)) {
            Serial.println("Stationary detected. Computing offsets...");

            float ax_mean=0, ay_mean=0, az_mean=0;
            float mx_mean=0, my_mean=0, mz_mean=0;
            for (int i=0;i<STATIONARY_WINDOW;i++){
                ax_mean+=accelSamples[i][0];
                ay_mean+=accelSamples[i][1];
                az_mean+=accelSamples[i][2];

                mx_mean+=magSamples[i][0];
                my_mean+=magSamples[i][1];
                mz_mean+=magSamples[i][2];
            }
            ax_mean/=STATIONARY_WINDOW;
            ay_mean/=STATIONARY_WINDOW;
            az_mean/=STATIONARY_WINDOW;
            mx_mean/=STATIONARY_WINDOW;
            my_mean/=STATIONARY_WINDOW;
            mz_mean/=STATIONARY_WINDOW;

            float g = 9.80665f;
            accel_offsets[0] = ax_mean - 0.0f;
            accel_offsets[1] = ay_mean - 0.0f;
            accel_offsets[2] = az_mean - g;

            mag_offsets[0] = mx_mean;
            mag_offsets[1] = my_mean;
            mag_offsets[2] = mz_mean;

            // Baseline altitude
            float altSum=0.0f;
            int baselineCount=50; 
            for (int i=0;i<baselineCount;i++){
                float rawAltitude = getBaroAltitude();
                filteredAltitude = (1 - alpha_filter)*filteredAltitude + alpha_filter*rawAltitude;
                altSum+=filteredAltitude;
                delay(10); 
            }
            baselineAltitude = altSum/baselineCount;
            Serial.print("Baseline altitude set to: ");
            Serial.println(baselineAltitude,3);

            return true;
        } else {
            Serial.println("Not stationary yet, keep still...");
        }
    }
}

float getRelativeAltitude() {
    float rawAltitude = getBaroAltitude();
    filteredAltitude = (1 - alpha_filter)*filteredAltitude + alpha_filter*rawAltitude;
    return filteredAltitude - baselineAltitude;
}

// Convert body accel to NED using BNO055 orientation
// We'll get quaternion from BNO055 and rotate accelerations.
static void rotateToNED(float ax, float ay, float az, float &ax_ned, float &ay_ned, float &az_ned, float yaw, float pitch, float roll) {
    // Rotation from body to NED:
    // First, we assume BNO055 orientation gives yaw/pitch/roll w.r.t NED.
    // Rotation matrix R_ned_body = R_z(yaw)*R_y(pitch)*R_x(roll)
    // We'll invert it: since we have accel in body, to get NED: a_ned = R_ned_body * a_body
    // If yaw/pitch/roll is NED->body, we do the inverse. BNO055 typically gives orientation as Euler angles of sensor frame wrt NED.

    // Actually BNO055 orientation: yaw: about Z, pitch: about Y', roll: about X'' (Z-Y'-X'' sequence)
    // We can construct rotation matrix:

    float cy = cos(yaw); float sy = sin(yaw);
    float cp = cos(pitch); float sp = sin(pitch);
    float cr = cos(roll); float sr = sin(roll);

    // R_ned_body = Rz(yaw)*Ry(pitch)*Rx(roll)
    // a_ned = R_ned_body * a_body
    float R[3][3];
    R[0][0]=cy*cp; R[0][1]=cy*sp*sr - sy*cr; R[0][2]=cy*sp*cr + sy*sr;
    R[1][0]=sy*cp; R[1][1]=sy*sp*sr + cy*cr; R[1][2]=sy*sp*cr - cy*sr;
    R[2][0]=-sp;   R[2][1]=cp*sr;           R[2][2]=cp*cr;

    ax_ned = R[0][0]*ax + R[0][1]*ay + R[0][2]*az;
    ay_ned = R[1][0]*ax + R[1][1]*ay + R[1][2]*az;
    az_ned = R[2][0]*ax + R[2][1]*ay + R[2][2]*az;
}

void getCorrectedIMUData(float &yaw, float &pitch, float &roll, 
                         float &ax_ned, float &ay_ned, float &az_ned) {
    sensors_event_t accelEvent, gyroEvent, magEvent;
    bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);

    // Apply offsets to accel
    float ax_corrected = accelEvent.acceleration.x - accel_offsets[0];
    float ay_corrected = accelEvent.acceleration.y - accel_offsets[1];
    float az_corrected = accelEvent.acceleration.z - accel_offsets[2];

    // Get orientation
    imu::Quaternion q = bno.getQuat();
    float ysqr = q.y()*q.y();

    // Euler angles from quaternion
    float t0 = 2.0f*(q.w()*q.x() + q.y()*q.z());
    float t1 = 1.0f - 2.0f*(q.x()*q.x() + ysqr);
    roll = atan2f(t0, t1);

    float t2 = 2.0f*(q.w()*q.y()-q.z()*q.x());
    t2 = fmaxf(fminf(t2,1.0f),-1.0f);
    pitch = asinf(t2);

    float t3 = 2.0f*(q.w()*q.z() + q.x()*q.y());
    float t4 = 1.0f - 2.0f*(ysqr + q.z()*q.z());
    yaw = atan2f(t3,t4);

    // Rotate accel into NED
    rotateToNED(ax_corrected, ay_corrected, az_corrected, ax_ned, ay_ned, az_ned, yaw, pitch, roll);
}
