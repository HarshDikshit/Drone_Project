#ifndef DRONE_CONTROLLER_H
#define DRONE_CONTROLLER_H

#include "PIDController.h"

class DroneController {
public:
    DroneController(float sampleFreq);
    bool begin();
    void update(float roll, float pitch, float yaw, float rollSetpoint, float pitchSetpoint, float throttle, float yawSetpoint);
    void setPIDTunings(float kpRoll, float kiRoll, float kdRoll, float kpPitch, float kiPitch, float kdPitch);
    void setMotorSpeeds(float speeds[4]);
    void arm();
    void disarm();
    bool isArmed() const;
    void getMotorSpeeds(float speeds[4]) const;

private:
    void initESCs();
    void writeMotorSpeeds();

    PIDController rollPID;
    PIDController pitchPID;
    float motorSpeeds[4]; // Front-left, front-right, back-left, back-right
    bool armed;
    static const int motorPins[4];
    static const float minThrottle;
    static const float maxThrottle;
};

#endif