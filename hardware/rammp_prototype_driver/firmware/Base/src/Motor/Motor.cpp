#include "Motor.h"

Motor::Motor() : MotorBase() {}

void Motor::setMode(ControlMode new_mode) {
  if (mode != new_mode) {
    pos_pid.reset();
    vel_pid.reset();
  }
  mode = new_mode;
}

void Motor::disable() {
  target_pwm = 0.0f;
  target_vel = 0.0f;
  target_pos = current_pos;
  pos_pid.reset();
  vel_pid.reset();
  setMode(DISABLED);
}

void Motor::updateSensorData(float current_pos, float dt) {
  // Multiply by encoder direction to allow reversing logical sensor axis
  float raw_pos = current_pos * encoder_dir;

  // position input LPF
  this->current_pos =
      this->current_pos + this->lpf_input_alpha * (raw_pos - this->current_pos);

  if (dt > 0.0f) {
    float raw_vel = (this->current_pos - this->prev_pos) / dt;
    // velocity input LPF
    this->current_vel = this->current_vel +
                        this->lpf_input_alpha * (raw_vel - this->current_vel);
  }
  this->prev_pos = this->current_pos;
}
