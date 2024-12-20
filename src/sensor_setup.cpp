#include "sensor_setup.h"
#include "BNO_LUT.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BMP3XX.h>
#include <Wire.h>

// Sensor objects
Adafruit_BNO055 bno = Adafruit_BNO055(55);
Adafruit_BMP3XX bmp;

// Global variables
float baselineAltitude = 0.0f; // Baseline for relative altitude
float altitudeBias = 0.0f;     // Bias for manual calibration of BMP388

void setupSensors() {
    Serial.println("Starting sensor setup...");

    // **BNO055 Setup**
    if (!bno.begin()) {
        Serial.println("BNO055 initialization failed!");
        while (1);
    }
    Serial.println("BNO055 initialized successfully.");

    // Set the BNO055 to use the external crystal for more accurate orientation readings
    bno.setExtCrystalUse(true);
    
    // Load and apply calibration data from BNO_LUT.h
    bno.setSensorOffsets(BNO_CALIBRATION_OFFSETS);
    Serial.println("BNO055 calibration offsets applied.");

    // **BMP388 Setup**
    if (!bmp.begin_I2C(0x76, &Wire1)) {
        Serial.println("BMP388 initialization failed!");
        while (1);
    }
    Serial.println("BMP388 initialized successfully.");

    // Configure BMP388 for optimal performance
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_16X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_15);
    bmp.setOutputDataRate(BMP3_ODR_100_HZ);

    // **Manual Calibration Step for BMP388**
    altitudeBias = manualCalibrateBMP388();
    Serial.print("Manual calibration complete. Altitude bias: ");
    Serial.println(altitudeBias, 3);

    // Set the baseline altitude as the starting reference point
    if (bmp.performReading()) {
        baselineAltitude = bmp.readAltitude(1013.25) - altitudeBias; // Remove the bias
        Serial.print("Baseline Altitude set to: ");
        Serial.println(baselineAltitude, 3);
    } else {
        Serial.println("Failed to read initial BMP388 altitude.");
    }
}

/**
 * Manually calibrate the BMP388 by taking 1000 readings and averaging them to compute a bias.
 * This bias is subtracted from all subsequent altitude readings.
 */
float manualCalibrateBMP388() {
    Serial.println("Starting manual calibration for BMP388...");
    const int numSamples = 1000;
    float totalAltitude = 0.0f;

    for (int i = 0; i < numSamples; i++) {
        if (bmp.performReading()) {
            float currentAltitude = bmp.readAltitude(1013.25); // Reference pressure at sea level
            totalAltitude += currentAltitude;
        } else {
            Serial.println("Failed to read BMP388 altitude during calibration.");
        }

        // Print progress every 100 samples
        if (i % 100 == 0) {
            Serial.print("Calibration progress: ");
            Serial.print((float)i / numSamples * 100, 1);
            Serial.println("%");
        }
    }

    float altitudeBias = totalAltitude / numSamples; // Average altitude is our bias
    Serial.print("BMP388 Calibration Complete. Altitude Bias: ");
    Serial.println(altitudeBias, 3);

    return altitudeBias;
}

/**
 * Get relative altitude from the BMP388 relative to the baseline altitude, with manual bias correction.
 */
float getRelativeAltitude() {
    if (bmp.performReading()) {
        float currentAltitude = bmp.readAltitude(1013.25); // Reference sea-level pressure
        float correctedAltitude = currentAltitude - altitudeBias; // Subtract the bias
        return correctedAltitude - baselineAltitude; // Return relative altitude
    } else {
        Serial.println("BMP388 failed to read altitude.");
        return 0.0f;
    }
}

void getCorrectedIMUData(float &yaw, float &pitch, float &roll, float &ax_ned, float &ay_ned, float &az_ned, float &mx, float &my, float &mz) {
    sensors_event_t accelEvent, gyroEvent, magEvent;

    // Get sensor data from BNO055
    bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);

    // Extract Yaw, Pitch, and Roll from BNO055
    yaw = gyroEvent.gyro.z;
    pitch = gyroEvent.gyro.y;
    roll = gyroEvent.gyro.x;

    // Extract NED accelerations from accelerometer
    ax_ned = accelEvent.acceleration.x;
    ay_ned = accelEvent.acceleration.y;
    az_ned = accelEvent.acceleration.z;

    // Extract magnetic field data from magnetometer
    mx = magEvent.magnetic.x;
    my = magEvent.magnetic.y;
    mz = magEvent.magnetic.z;
}
