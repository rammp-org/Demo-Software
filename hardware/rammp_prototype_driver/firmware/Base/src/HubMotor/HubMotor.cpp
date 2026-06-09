#include "HubMotor.h"

HubMotor::HubMotor() : MotorBase() {
  this->direction = (axis_direction >= 0) ? 1 : -1;
}

unsigned short HubMotor::crc16(const unsigned char *buf, unsigned int len) {
  unsigned int i;
  unsigned short cksum = 0;
  for (i = 0; i < len; i++) {
    cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
  }
  return cksum;
}

uint8_t HubMotor::buildMessage(uint8_t payload, int payload_len) {
  uint8_t message = ((uint8_t)FRAME_HEADER);
  message += (uint8_t)payload_len;
  message += (uint8_t)payload;
  uint16_t crc = crc16(payload, payload_len);
  message += (uint8_t)(crc >> 8);
  message += (uint8_t)(crc & 0xFF);
  message += (uint8_t)FRAME_TAIL;
  return message;
}

void HubMotor::setMode(ControlMode mode) {
  this->mode = mode;
  switch (mode) {
  case OPEN_LOOP:
    break;
  case DISABLED:
    break;
  case POSITION_CONTROL:
    break;
  }
}

float HubMotor::getTargetVelocity() { return target_vel * this->direction; }

void HubMotor::disable() { this->setMode(DISABLED); }

float HubMotor::getTargetPosition() { return target_pos * this->direction; }

float HubMotor::getCurrentPosition() { return current_pos; }
