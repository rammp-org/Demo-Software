#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(float kp, float ki, float kd, float kff,
                             float min_out, float max_out, float scaling)
    : kp(kp), ki(ki), kd(kd), kff(kff), min_out(min_out), max_out(max_out),
      max_ramp_rate(0.0f), integral(0.0f), prev_error(0.0f), scaling(scaling) {}

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

  bool clamped = false;

  // Apply output limits
  if (output > max_out) {
    output = max_out;
    clamped = true;
  } else if (output < min_out) {
    output = min_out;
    clamped = true;
  }

  // Apply trapezoidal ramp rate limit if enabled
  if (max_ramp_rate > 0.0f) {
    float max_change = max_ramp_rate * dt;
    if (output - _prev_output > max_change) {
      output = _prev_output + max_change;
      clamped = true;
    } else if (output - _prev_output < -max_change) {
      output = _prev_output - max_change;
      clamped = true;
    }
  }

  // Anti-windup: undo integration if we hit any limit
  if (clamped) {
    integral -= error * dt;
  }

  // Save state
  _prev_output = output;
  prev_error = error;

  // Apply Low Pass Filter to the output
  _filtered_output += lpf_alpha * (output - _filtered_output);

  return _filtered_output;
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

void PIDController::setRampRate(float max_ramp_rate) {
  this->max_ramp_rate = max_ramp_rate;
}

void PIDController::reset() {
  integral = 0.0f;
  prev_error = 0.0f;
  _filtered_output = 0.0f;
  _prev_output = 0.0f;
}

void PIDController::setLpfAlpha(float alpha) {
  if (alpha < 0.0f)
    alpha = 0.0f;
  if (alpha > 1.0f)
    alpha = 1.0f;
  this->lpf_alpha = alpha;
}

float PIDController::getLpfAlpha() const { return this->lpf_alpha; }
