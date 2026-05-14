#include "ODrive.h"

ODrive::ODrive(ODriveUART &odrive, int axis_direction)
    : odrive(odrive), direction(axis_direction) {}

void ODrive::updateEncoderReadings() {
  current_pos = this->odrive.getPosition() * this->direction;
}

void ODrive::setMode(DriveMode new_mode) {
  // Serial.println("ODrive setMode called");
  this->mode = new_mode;
  // Serial.println(new_mode);
  switch (new_mode) {
  case OPEN_LOOP:
  case DISABLED:
    while (this->odrive.getState() != AXIS_STATE_IDLE) {
      Serial.println("ODrive setMode POSITION_CONTROL still waiting!!");
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
    // Serial.println("ODrive setMode POSITION_CONTROL called");
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

void ODrive::setTargetPosition(float pos) { target_pos = pos; }

void ODrive::disable() {
  // getCurrentPosition() is robot frame; UART setPosition expects hardware.
  const float robot_pos = this->getCurrentPosition();
  const float hw_pos = robot_pos / static_cast<float>(direction);
  this->setMode(DISABLED);
  this->odrive.setPosition(hw_pos);
  this->setMode(DISABLED);
}

float ODrive::getTargetPosition() { return target_pos * this->direction; }

float ODrive::getCurrentPosition() {
  this->updateEncoderReadings();
  return current_pos;
}

float ODrive::getCurrentTorque() {
  return this->odrive.getParameterAsFloat("axis0.motor.torque_estimate") *
         static_cast<float>(this->direction);
}
