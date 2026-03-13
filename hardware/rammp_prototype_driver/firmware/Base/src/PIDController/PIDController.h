#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float min_out, float max_out);

    // Compute PID output given setpoint, measured value, and dt
    float compute(float setpoint, float measured, float dt);

    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min_out, float max_out);
    void reset();

    float kp, ki, kd;
    float min_out, max_out;
    float integral;
    float prev_error;
};

#endif
