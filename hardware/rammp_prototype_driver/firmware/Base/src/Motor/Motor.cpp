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
