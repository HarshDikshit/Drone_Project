#include "DroneController.h"
#include <Arduino.h>

const int DroneController::motorPins[4] = {25, 26, 27, 14}; // FL, FR, BL, BR
const float DroneController::minThrottle = 1000.0f; // µs
const float DroneController::maxThrottle = 2000.0f; // µs

DroneController::DroneController(float sampleFreq)
    : rollPID(2.0f, 0.1f, 0.5f, -500.0f, 500.0f, 1.0f / sampleFreq),
      pitchPID(2.0f, 0.1f, 0.5f, -500.0f, 500.0f, 1.0f / sampleFreq),
      armed(false) {
    for (int i = 0; i < 4; i++) {
        motorSpeeds[i] = minThrottle;
    }
}

bool DroneController::begin() {
    initESCs();
    disarm();
    return true;
}

void DroneController::initESCs() {
    for (int i = 0; i < 4; i++) {
        pinMode(motorPins[i], OUTPUT);
        // Initialize ESCs with minimum throttle
        analogWriteFrequency(50); // 50 Hz for ESC
        analogWrite(motorPins[i], map(minThrottle, 1000, 2000, 0, 255));
    }
    delay(2000); // Wait for ESCs to initialize
}

void DroneController::writeMotorSpeeds() {
    for (int i = 0; i < 4; i++) {
        int pwm = map(motorSpeeds[i], 1000, 2000, 0, 255);
        analogWrite(motorPins[i], pwm);
    }
}

void DroneController::update(float roll, float pitch, float yaw, float rollSetpoint, float pitchSetpoint, float throttle, float yawSetpoint) {
    if (!armed) {
        for (int i = 0; i < 4; i++) {
            motorSpeeds[i] = minThrottle;
        }
        writeMotorSpeeds();
        return;
    }

    // Compute PID outputs
    float rollOutput = rollPID.compute(rollSetpoint, roll);
    float pitchOutput = pitchPID.compute(pitchSetpoint, pitch);

    // Map throttle (0–100%) to PWM (1000–2000 µs)
    float baseThrottle = minThrottle + (maxThrottle - minThrottle) * (throttle / 100.0f);
    if (baseThrottle < minThrottle) baseThrottle = minThrottle;
    if (baseThrottle > maxThrottle) baseThrottle = maxThrottle;

    // Yaw control (differential throttle)
    float yawOutput = yawSetpoint * 2.0f; // Scale yaw setpoint (±50°/s)

    // Motor mixing for "X" configuration
    // Front-left: +roll, -pitch, +yaw
    motorSpeeds[0] = baseThrottle + rollOutput - pitchOutput + yawOutput;
    // Front-right: -roll, -pitch, -yaw
    motorSpeeds[1] = baseThrottle - rollOutput - pitchOutput - yawOutput;
    // Back-left: +roll, +pitch, -yaw
    motorSpeeds[2] = baseThrottle + rollOutput + pitchOutput - yawOutput;
    // Back-right: -roll, +pitch, +yaw
    motorSpeeds[3] = baseThrottle - rollOutput + pitchOutput + yawOutput;

    // Clamp motor speeds
    for (int i = 0; i < 4; i++) {
        if (motorSpeeds[i] < minThrottle) motorSpeeds[i] = minThrottle;
        if (motorSpeeds[i] > maxThrottle) motorSpeeds[i] = maxThrottle;
    }

    writeMotorSpeeds();
}

void DroneController::setPIDTunings(float kpRoll, float kiRoll, float kdRoll, float kpPitch, float kiPitch, float kdPitch) {
    rollPID.setTunings(kpRoll, kiRoll, kdRoll);
    pitchPID.setTunings(kpPitch, kiPitch, kdPitch);
}

void DroneController::setMotorSpeeds(float speeds[4]) {
    if (!armed) return;
    for (int i = 0; i < 4; i++) {
        motorSpeeds[i] = speeds[i];
        if (motorSpeeds[i] < minThrottle) motorSpeeds[i] = minThrottle;
        if (motorSpeeds[i] > maxThrottle) motorSpeeds[i] = maxThrottle;
    }
    writeMotorSpeeds();
}

void DroneController::arm() {
    armed = true;
    for (int i = 0; i < 4; i++) {
        motorSpeeds[i] = minThrottle;
    }
    writeMotorSpeeds();
}

void DroneController::disarm() {
    armed = false;
    for (int i = 0; i < 4; i++) {
        motorSpeeds[i] = minThrottle;
    }
    writeMotorSpeeds();
}

bool DroneController::isArmed() const {
    return armed;
}

void DroneController::getMotorSpeeds(float speeds[4]) const {
    for (int i = 0; i < 4; i++) {
        speeds[i] = motorSpeeds[i];
    }
}