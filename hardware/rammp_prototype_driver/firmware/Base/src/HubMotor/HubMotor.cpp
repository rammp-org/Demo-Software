#include "HubMotor.h"

HubMotor::HubMotor(int axis_direction = 1, HardwareSerial &motor)
    : MotorBase(), motor(motor) {
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

void HubMotor::writeMotorCommand(uint8_t paylod, uint8_t payload_len) {
  uint16_t crc = crc16(payload, payload_len);
  motor.write((uint8_t)FRAME_HEADER);
  motor.write((uint8_t)payload_len);
  motor.write(payload, payload_len);
  motor.write((uint8_t)(crc >> 8));
  motor.write((uint8_t)(crc & 0xFF));
  motor.write((uint8_t)FRAME_TAIL);
}

void HubMotor::writePWM(float pwm) {
  pwm = direction * pwm * pwm_scale;
  int32_t duty = pwm;
  uint8_t payload[] = {0x46, (uint8_t)(duty >> 24), (uint8_t)(duty >> 16),
                       (uint8_t)(duty >> 8), (uint8_t)(duty)};
  this->writeMotorCommand(payload, 5);
}

void HubMotor::writeTargetPos() {
  int32_t position = target_pos * pos_scale * 360;
  uint8_t payload[] = {(uint8_t)(position >> 24), (uint8_t)(position >> 16),
                       (uint8_t)(position >> 8), (uint8_t)(position)};
  this->writeMotorCommand(payload, 4);
}

void HubMotor::writeZeroCurrent() {
  int32_t current = 0;
  uint8_t payload[] = {0x47, (uint8_t)(current >> 24), (uint8_t)(current >> 16),
                       (uint8_t)(current >> 8), (uint8_t)(current)};
  this->writeMotorCommand(payload, 5);
}

void HubMotor::setMode(ControlMode mode) {
  this->mode = mode;
  switch (mode) {
  case OPEN_LOOP:
    break;
  case DISABLED:
    this->writeZeroCurrent();
    break;
  case POSITION_CONTROL:
    break;
  }
}

void HubMotor::updateSensorData(float current_pos, float dt) {
  motor.write(getPos, sizeof(getPos));

  uint8_t message[10];
  uint8_t len = 0;

  // wait for header
  unsigned long timeout = millis() + 200;
  while (millis() < timeout) {
    if (motor.available()) {
      uint8_t b = motor.read();
      if (b == 0xAA) {
        message[len++] = b;
        break;
      }
    }
  }

  // collect rest
  timeout = millis() + 50;
  while (millis() < timeout && len < sizeof(message)) {
    if (motor.available()) {
      message[len++] = motor.read();
    }
  }

  // grab position read payload and convert to float
  if (len == 10 && message[0] == 0xAA && message[2] == 0x57 &&
      message[9] == 0xBB) {
    uint32_t raw = ((uint32_t)message[3] << 24) | ((uint32_t)message[4] << 16) |
                   ((uint32_t)message[5] << 8) | (uint32_t)message[6];
    float position;
    memcpy(&position, &raw, 4);

    this->current_pos = position * this->direction;
    this->prev_pos = this->current_pos;
  }
}

float HubMotor::getTargetVelocity() { return target_vel * this->direction; }

void HubMotor::disable() { this->setMode(DISABLED); }

float HubMotor::getTargetPosition() { return target_pos * this->direction; }

float HubMotor::getCurrentPosition() { return current_pos; }
