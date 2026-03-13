#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float min_out, float max_out)
    : kp(kp), ki(ki), kd(kd), min_out(min_out), max_out(max_out), integral(0.0f), prev_error(0.0f) {}

float PIDController::compute(float setpoint, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - measured;
    
    // Proportional term
    float p_out = kp * error;

    // Integral term with anti-windup (only integrate if output is not saturated)
    integral += error * dt;
    float i_out = ki * integral;

    // Derivative term
    float derivative = (error - prev_error) / dt;
    float d_out = kd * derivative;

    // Compute total output
    float output = p_out + i_out + d_out;

    // Apply output limits and anti-windup clamping
    if (output > max_out) {
        output = max_out;
        integral -= error * dt; // Undo integration
    } else if (output < min_out) {
        output = min_out;
        integral -= error * dt; // Undo integration
    }

    // Save previous error
    prev_error = error;

    return output;
}

void PIDController::setGains(float kp, float ki, float kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

void PIDController::setOutputLimits(float min_out, float max_out) {
    this->min_out = min_out;
    this->max_out = max_out;
}

void PIDController::reset() {
    integral = 0.0f;
    prev_error = 0.0f;
}
