#include "PIDController.h"
#include <Arduino.h>

PIDController::PIDController(float kp, float ki, float kd, float min_out,
                             float max_out)
    : kp(kp), ki(ki), kd(kd), min_out(min_out), max_out(max_out),
      integral(0.0f), prev_error(0.0f) {}

float PIDController::compute(float setpoint, float measured, float dt) {
  if (dt <= 0.0f)
    return 0.0f;

  float error = setpoint - measured;

  // Proportional term
  float p_out = kp / 10000 * error;

  // Integral term with anti-windup (only integrate if output is not saturated)
  integral += error * dt;
  float i_out = ki / 10000 * integral;

  // Derivative term
  float derivative = (error - prev_error) / dt;
  float d_out = kd / 10000 * derivative;

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

  Serial.print("DEBUG: p_out, error, setpoint, measured, dt, output: ");
  Serial.print(p_out);
  Serial.print(", ");
  Serial.print(error);
  Serial.print(", ");
  Serial.print(setpoint);
  Serial.print(", ");
  Serial.print(measured);
  Serial.print(", ");
  Serial.print(dt);
  Serial.print(", ");
  Serial.println(output);

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
