#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float minOutput, float maxOutput, float sampleTime)
    : kp(kp), ki(ki), kd(kd), minOutput(minOutput), maxOutput(maxOutput), sampleTime(sampleTime),
      integral(0.0f), previousError(0.0f), firstCompute(true) {
}

void PIDController::setTunings(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

float PIDController::compute(float setpoint, float input) {
    float error = setpoint - input;
    
    // Proportional term
    float pTerm = kp * error;

    // Integral term with anti-windup
    integral += ki * error * sampleTime;
    if (integral > maxOutput) integral = maxOutput;
    else if (integral < minOutput) integral = minOutput;

    // Derivative term
    float dTerm = 0.0f;
    if (!firstCompute) {
        dTerm = kd * (error - previousError) / sampleTime;
    }
    previousError = error;
    firstCompute = false;

    // Compute output and clamp
    float output = pTerm + integral + dTerm;
    if (output > maxOutput) output = maxOutput;
    else if (output < minOutput) output = minOutput;

    return output;
}

void PIDController::reset() {
    integral = 0.0f;
    previousError = 0.0f;
    firstCompute = true;
}