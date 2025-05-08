#include <Arduino.h>
#include "ComplementaryFilter.h"
#include <math.h>

ComplementaryFilter::ComplementaryFilter(float sampleFreq, float alphaVal) {
    dt = 1.0f / sampleFreq;
    alpha = (alphaVal >= 0.0f && alphaVal <= 1.0f) ? alphaVal : 0.7f;
    roll = 0.0f;
    pitch = 0.0f;
    yaw = 0.0f;
    isInitialized = false;
}

bool ComplementaryFilter::isValidData(float ax, float ay, float az, float gx, float gy, float gz) {
    if (isnan(ax) || isnan(ay) || isnan(az) || isnan(gx) || isnan(gy) || isnan(gz) ||
        isinf(ax) || isinf(ay) || isinf(az) || isinf(gx) || isinf(gy) || isinf(gz)) {
        return false;
    }

    float accelMag = sqrt(ax * ax + ay * ay + az * az);
    if (accelMag < 0.1f || accelMag > 2.0f) {
        return false;
    }

    if (fabs(gx) > 2000.0f || fabs(gy) > 2000.0f || fabs(gz) > 2000.0f) {
        return false;
    }

    return true;
}

void ComplementaryFilter::update(float gx, float gy, float gz, float ax, float ay, float az) {
    if (!isValidData(ax, ay, az, gx, gy, gz)) {
        Serial.println("Invalid sensor data in filter");
        return;
    }

    float accelMag = sqrt(ax * ax + ay * ay + az * az);
    if (accelMag > 0.0f) {
        ax /= accelMag;
        ay /= accelMag;
        az /= accelMag;
    } else {
        Serial.println("Accelerometer normalization failed");
        return;
    }

    static unsigned long lastNormDebug = 0;
    if (millis() - lastNormDebug >= 500) {
        Serial.print("Normalized Accel: "); Serial.print(ax, 4); Serial.print(", ");
        Serial.print(ay, 4); Serial.print(", "); Serial.println(az, 4);
        lastNormDebug = millis();
    }

    float accelRoll = atan2(ay, -az) * 180.0f / M_PI;
    float accelPitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;

    static unsigned long lastDebug = 0;
    if (millis() - lastDebug >= 500) {
        Serial.print("Accel Roll (rad): "); Serial.print(atan2(ay, -az), 4);
        Serial.print(" Accel Pitch (rad): "); Serial.println(atan2(-ax, sqrt(ay * ay + az * az)), 4);
        Serial.print("Accel Roll (deg): "); Serial.print(accelRoll, 2);
        Serial.print(" Accel Pitch (deg): "); Serial.println(accelPitch, 2);
        lastDebug = millis();
    }

    if (!isInitialized) {
        roll = accelRoll;
        pitch = accelPitch;
        yaw = 0.0f;
        isInitialized = true;
        Serial.println("Filter initialized");
        return;
    }

    roll += gx * dt;
    pitch += gy * dt;
    yaw += gz * dt;

    // Normalize angles to [-180, 180]
    while (roll > 180.0f) roll -= 360.0f;
    while (roll < -180.0f) roll += 360.0f;
    while (pitch > 180.0f) pitch -= 360.0f;
    while (pitch < -180.0f) pitch += 360.0f;
    while (yaw > 180.0f) yaw -= 360.0f;
    while (yaw < -180.0f) yaw += 360.0f;

    roll = alpha * roll + (1.0f - alpha) * accelRoll;
    pitch = alpha * pitch + (1.0f - alpha) * accelPitch;
}

void ComplementaryFilter::resetAngles(float ax, float ay, float az) {
    if (!isValidData(ax, ay, az, 0.0f, 0.0f, 0.0f)) {
        Serial.println("Invalid data for reset");
        return;
    }

    float accelMag = sqrt(ax * ax + ay * ay + az * az);
    if (accelMag > 0.0f) {
        ax /= accelMag;
        ay /= accelMag;
        az /= accelMag;
    } else {
        Serial.println("Reset: Accelerometer normalization failed");
        return;
    }

    roll = atan2(ay, -az) * 180.0f / M_PI;
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0f / M_PI;
    yaw = 0.0f;
    Serial.println("Angles reset");
}

void ComplementaryFilter::getEulerAngles(float& rollOut, float& pitchOut, float& yawOut) {
    rollOut = roll;
    pitchOut = pitch;
    yawOut = yaw;
}