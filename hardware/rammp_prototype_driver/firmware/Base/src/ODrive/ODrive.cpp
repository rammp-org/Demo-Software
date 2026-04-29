#include "ODrive.h"

ODrive::ODrive(ODriveUART &odrive) : odrive(odrive) {}

void ODrive::setMode(DriveMode new_mode) {
  this->mode = new_mode;
  switch (new_mode) {
  case DISABLED:
    while (this->odrive.getState() != AXIS_STATE_IDLE) {
      this->odrive.clearErrors();
      this->odrive.setState(AXIS_STATE_IDLE);
      delay(10);
    }
    break;
  case OPEN_LOOP:
    // there is no open loop mode for odrive, idle??
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

void ODrive::setTargetPosition(float target_pos) {
  this->odrive.setPosition(target_pos);
}

void ODrive::setTargetVelocity(float target_vel) {
  this->odrive.setVelocity(target_vel);
}

void ODrive::disable() {
  float target_pos = this->odrive.getPosition();
  this->setMode(POSITION_CONTROL);
  this->odrive.setPosition(target_pos);
  this->setMode(DISABLED)
}
