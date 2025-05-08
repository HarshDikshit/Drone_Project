#include "MPU6050Sensor.h"
#include <Wire.h>

// MPU-6050 I2C address and register definitions
#define MPU6050_ADDR 0x68
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_GYRO_CONFIG 0x1B

// I2C pins for NodeMCU-32S
#define SDA_PIN 21
#define SCL_PIN 22

// Sensor scaling factors (based on MPU-6050 settings)
#define ACCEL_SCALE 16384.0f // ±2g range, LSB/g
#define GYRO_SCALE 131.0f    // ±250 deg/s range, LSB/(deg/s)

// Calibration offsets (based on stationary data)
#define ACCEL_X_OFFSET 0.0532f
#define ACCEL_Y_OFFSET -0.0181f
#define ACCEL_Z_OFFSET -1.2078f

MPU6050Sensor::MPU6050Sensor(float sampleFreq, float alpha)
    : filter(sampleFreq, alpha),
      gxOffset(0.0f), gyOffset(0.0f), gzOffset(0.0f),
      gxAvg(0.0f), gyAvg(0.0f), gzAvg(0.0f),
      sampleCount(0),
      lastUpdate(0),
      lastStationaryTime(0),
      isStationary(false) {
}

bool MPU6050Sensor::begin() {
    Serial.begin(115200);
    while (!Serial) {
        ; // Wait for serial port to connect
    }

    if (!initMPU6050()) {
        Serial.println("Failed to initialize MPU-6050");
        return false;
    }
    Serial.println("MPU-6050 initialized");

    calibrateGyro(gxOffset, gyOffset, gzOffset);
    delay(100);
    return true;
}

bool MPU6050Sensor::initMPU6050() {
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(0x00); // Wake up MPU-6050
    if (Wire.endTransmission(true) != 0) {
        return false;
    }

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_CONFIG);
    Wire.write(0x00); // AFS_SEL=0
    if (Wire.endTransmission(true) != 0) {
        return false;
    }

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_GYRO_CONFIG);
    Wire.write(0x00); // FS_SEL=0
    if (Wire.endTransmission(true) != 0) {
        return false;
    }

    return true;
}

bool MPU6050Sensor::readMPU6050(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
    static int16_t lastAccelX = 0, lastAccelY = 0, lastAccelZ = 0;
    static int16_t lastGyroX = 0, lastGyroY = 0, lastGyroZ = 0;

    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(MPU6050_ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    Wire.requestFrom((uint8_t)MPU6050_ADDR, (size_t)14, true); // Read 14 bytes
    if (Wire.available() < 14) {
        return false;
    }

    int16_t accelX = (Wire.read() << 8) | Wire.read();
    int16_t accelY = (Wire.read() << 8) | Wire.read();
    int16_t accelZ = (Wire.read() << 8) | Wire.read();
    Wire.read(); Wire.read(); // Skip temperature
    int16_t gyroX = (Wire.read() << 8) | Wire.read();
    int16_t gyroY = (Wire.read() << 8) | Wire.read();
    int16_t gyroZ = (Wire.read() << 8) | Wire.read();

    if (accelX == lastAccelX && accelY == lastAccelY && accelZ == lastAccelZ &&
        gyroX == lastGyroX && gyroY == lastGyroY && gyroZ == lastGyroZ) {
        Serial.println("Warning: Sensor readings unchanged");
        return false;
    }

    lastAccelX = accelX;
    lastAccelY = accelY;
    lastAccelZ = accelZ;
    lastGyroX = gyroX;
    lastGyroY = gyroY;
    lastGyroZ = gyroZ;

    ax = (accelX / ACCEL_SCALE) - ACCEL_X_OFFSET;
    ay = (accelY / ACCEL_SCALE) - ACCEL_Y_OFFSET;
    az = (accelZ / ACCEL_SCALE) - ACCEL_Z_OFFSET;
    gx = (gyroX / GYRO_SCALE) - gxOffset;
    gy = (gyroY / GYRO_SCALE) - gyOffset;
    gz = (gyroZ / GYRO_SCALE) - gzOffset;

    static unsigned long lastDebug = 0;
    if (millis() - lastDebug >= 500) {
        Serial.print("Raw Accel: "); Serial.print(accelX); Serial.print(", ");
        Serial.print(accelY); Serial.print(", "); Serial.println(accelZ);
        Serial.print("Raw Gyro: "); Serial.print(gyroX); Serial.print(", ");
        Serial.print(gyroY); Serial.print(", "); Serial.println(gyroZ);
        Serial.print("Converted Accel (g): "); Serial.print(ax, 4); Serial.print(", ");
        Serial.print(ay, 4); Serial.print(", "); Serial.println(az, 4);
        Serial.print("Converted Gyro (deg/s): "); Serial.print(gx, 4); Serial.print(", ");
        Serial.print(gy, 4); Serial.print(", "); Serial.println(gz, 4);
        lastDebug = millis();
    }

    return true;
}

void MPU6050Sensor::calibrateGyro(float& gxOffset, float& gyOffset, float& gzOffset) {
    Serial.println("Calibrating gyroscope...");
    float gxSum = 0.0f, gySum = 0.0f, gzSum = 0.0f;
    int samples = 200;
    int validSamples = 0;

    for (int i = 0; i < samples; i++) {
        float ax, ay, az, gx, gy, gz;
        if (readMPU6050(ax, ay, az, gx, gy, gz)) {
            gxSum += gx;
            gySum += gy;
            gzSum += gz;
            validSamples++;
        }
        delay(10);
    }

    if (validSamples > 0) {
        gxOffset = gxSum / validSamples;
        gyOffset = gySum / validSamples;
        gzOffset = gzSum / validSamples;
        Serial.print("Gyro offsets: ");
        Serial.print(gxOffset, 4); Serial.print(", ");
        Serial.print(gyOffset, 4); Serial.print(", ");
        Serial.println(gzOffset, 4);
    } else {
        Serial.println("Calibration failed: No valid samples");
        gxOffset = gyOffset = gzOffset = 0.0f;
    }
}

bool MPU6050Sensor::getAngles(float& roll, float& pitch, float& yaw) {
    unsigned long now = micros();
    if (now - lastUpdate >= 10000) { // 100 Hz
        float ax, ay, az, gx, gy, gz;

        if (readMPU6050(ax, ay, az, gx, gy, gz)) {
            // Update moving average for gyro
            gxAvg = (gxAvg * sampleCount + gx) / (sampleCount + 1);
            gyAvg = (gyAvg * sampleCount + gy) / (sampleCount + 1);
            gzAvg = (gzAvg * sampleCount + gz) / (sampleCount + 1);
            sampleCount = min(sampleCount + 1, avgSamples);

            // Check if stationary
            if (fabs(gxAvg) < 0.1f && fabs(gyAvg) < 0.1f && fabs(gzAvg) < 0.1f) {
                if (!isStationary) {
                    lastStationaryTime = millis();
                    isStationary = true;
                }
                if (millis() - lastStationaryTime >= 1000) { // 1 second stationary
                    filter.resetAngles(ax, ay, -az); // Invert az for reset
                    sampleCount = 0;
                    gxAvg = gyAvg = gzAvg = 0.0f;
                }
                static unsigned long lastStationaryDebug = 0;
                if (millis() - lastStationaryDebug >= 500) {
                    Serial.println("Stationary: Skipping filter update");
                    lastStationaryDebug = millis();
                }
            } else {
                isStationary = false;
                filter.update(gx, gy, gz, ax, ay, -az); // Invert az for update
            }

            filter.getEulerAngles(roll, pitch, yaw);
            lastUpdate = now;
            return true;
        } else {
            Serial.println("Failed to read MPU-6050");
            return false;
        }
    }
    return false; // Not ready for update
}