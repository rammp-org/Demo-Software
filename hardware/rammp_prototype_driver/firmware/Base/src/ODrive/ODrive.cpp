#include "ODrive.h"

ODrive::ODrive(ODriveUART &odrive, int axis_direction)
    : MotorBase(), odrive(odrive) {
  direction = (axis_direction >= 0) ? 1 : -1;
}

void ODrive::updateSensorData(float current_pos, float dt) {
  this->current_pos = this->odrive.getPosition() * this->direction;

  if (dt > 0.0f) {
    this->current_vel = (this->current_pos - this->prev_pos) / dt;
  }
  this->prev_pos = this->current_pos;
}

void ODrive::setMode(ControlMode new_mode) {
  this->mode = new_mode;
  switch (new_mode) {
  case OPEN_LOOP:
  case DISABLED:
    while (this->odrive.getState() != AXIS_STATE_IDLE) {
      Serial.println("ODrive setMode DISABLED still waiting!!");
      this->odrive.clearErrors();
      this->odrive.setState(AXIS_STATE_IDLE);
      delay(10);
    }
    break;

  case VELOCITY_CONTROL:
    while (this->odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
      this->odrive.clearErrors();
      this->odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
      delay(10);
    }

    while (this->odrive.getParameterAsInt(
               "axis0.controller.config.control_mode") != 2) {
      this->odrive.clearErrors();
      this->odrive.setParameter("axis0.controller.config.control_mode",
                                CONTROL_MODE_VELOCITY_CONTROL);
      delay(10);
    }
    break;
  case POSITION_CONTROL:
    while (this->odrive.getState() != AXIS_STATE_CLOSED_LOOP_CONTROL) {
      Serial.println("ODrive setMode POSITION_CONTROL still waiting!!");
      this->odrive.clearErrors();
      this->odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
      delay(10);
    }

    while (this->odrive.getParameterAsInt(
               "axis0.controller.config.control_mode") != 3) {
      this->odrive.clearErrors();
      this->odrive.setParameter("axis0.controller.config.control_mode",
                                CONTROL_MODE_POSITION_CONTROL);
      delay(10);
    }
    break;
  }
}

//
float ODrive::getTargetVelocity() { return target_vel * this->direction; }

void ODrive::disable() { this->setMode(DISABLED); }

float ODrive::getTargetPosition() { return target_pos * this->direction; }

float ODrive::getCurrentPosition() { return current_pos; }

float ODrive::getCurrentTorque() {
  return this->odrive.getParameterAsFloat("axis0.motor.torque_estimate") *
         static_cast<float>(this->direction);
}
