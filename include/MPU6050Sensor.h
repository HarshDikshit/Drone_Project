#ifndef MPU6050_SENSOR_H
#define MPU6050_SENSOR_H

#include <Arduino.h>
#include "ComplementaryFilter.h"

class MPU6050Sensor {
public:
    MPU6050Sensor(float sampleFreq = 100.0f, float alpha = 0.989f);
    bool begin();
    bool getAngles(float& roll, float& pitch, float& yaw);

private:
    bool initMPU6050();
    bool readMPU6050(float& ax, float& ay, float& az, float& gx, float& gy, float& gz);
    void calibrateGyro(float& gxOffset, float& gyOffset, float& gzOffset);
    
    ComplementaryFilter filter;
    float gxOffset, gyOffset, gzOffset;
    float gxAvg, gyAvg, gzAvg;
    int sampleCount;
    static const int avgSamples = 10;
    unsigned long lastUpdate;
    unsigned long lastStationaryTime;
    bool isStationary;
};

#endif