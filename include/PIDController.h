#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float minOutput, float maxOutput, float sampleTime);
    void setTunings(float kp, float ki, float kd);
    float compute(float setpoint, float input);
    void reset();

private:
    float kp, ki, kd;
    float minOutput, maxOutput;
    float sampleTime;
    float integral, previousError;
    bool firstCompute;
};

#endif