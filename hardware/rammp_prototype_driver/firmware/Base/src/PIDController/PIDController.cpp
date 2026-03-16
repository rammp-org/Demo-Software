#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(float kp, float ki, float kd, float kff,
                             float min_out, float max_out, float scaling)
    : kp(kp), ki(ki), kd(kd), kff(kff), min_out(min_out), max_out(max_out),
      integral(0.0f), prev_error(0.0f), scaling(scaling) {}

float PIDController::compute(float setpoint, float measured, float dt) {
  if (dt <= 0.0f)
    return 0.0f;

  float error = setpoint - measured;

  // Feed-forward term (direct contribution from setpoint)
  float ff_out = kff / scaling * setpoint;

  // Proportional term
  float p_out = kp / scaling * error;

  // Integral term with anti-windup (only integrate if output is not saturated)
  integral += error * dt;
  float i_out = ki / scaling * integral;

  // Derivative term
  float derivative = (error - prev_error) / dt;
  float d_out = kd / scaling * derivative;

  // Compute total output (feed-forward + PID)
  float output = ff_out + p_out + i_out + d_out;

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

void PIDController::setFeedForward(float kff) { this->kff = kff; }

void PIDController::setOutputLimits(float min_out, float max_out) {
  this->min_out = min_out;
  this->max_out = max_out;
}

void PIDController::reset() {
  integral = 0.0f;
  prev_error = 0.0f;
}
