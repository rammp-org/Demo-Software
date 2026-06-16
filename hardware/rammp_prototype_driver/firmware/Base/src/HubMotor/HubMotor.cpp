#include "HubMotor.h"

static const unsigned short CRC16_TAB[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108,
    0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210,
    0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b,
    0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401,
    0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee,
    0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6,
    0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d,
    0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b, 0x5af5,
    0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
    0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4,
    0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd,
    0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13,
    0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a,
    0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e,
    0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e, 0x02b1,
    0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256, 0xb5ea, 0xa5cb,
    0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3, 0x14a0,
    0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
    0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657,
    0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9,
    0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882,
    0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
    0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e,
    0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9, 0x7c26, 0x6c07,
    0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1, 0xef1f, 0xff3e, 0xcf5d,
    0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36, 0x4e55, 0x5e74,
    0x2e93, 0x3eb2, 0x0ed1, 0x1ef0};

HubMotor::HubMotor(int axis_direction, HardwareSerial &motor)
    : MotorBase(), motor(motor) {
  this->direction = (axis_direction >= 0) ? 1 : -1;
}

unsigned short HubMotor::crc16(const unsigned char *buf, unsigned int len) {
  unsigned int i;
  unsigned short cksum = 0;
  for (i = 0; i < len; i++) {
    cksum = CRC16_TAB[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
  }
  return cksum;
}

void HubMotor::writeMotorCommand(uint8_t *payload, uint8_t payload_len) {
  // flush the buffer
  while (motor.available())
    motor.read();

  uint16_t crc = crc16(payload, payload_len);
  motor.write((uint8_t)FRAME_HEADER);
  motor.write((uint8_t)payload_len);
  motor.write(payload, payload_len);
  motor.write((uint8_t)(crc >> 8));
  motor.write((uint8_t)(crc & 0xFF));
  motor.write((uint8_t)FRAME_TAIL);
}

void HubMotor::writePWM() {
  float pwm = direction * this->target_pwm * pwm_scale;
  int32_t duty = pwm;
  uint8_t payload[] = {0x46, (uint8_t)(duty >> 24), (uint8_t)(duty >> 16),
                       (uint8_t)(duty >> 8), (uint8_t)(duty)};
  this->writeMotorCommand(payload, 5);
}

void HubMotor::writeTargetPos() {
  int32_t pos = direction * this->target_pos * pos_scale;
  int32_t spd = 20000;   // ERPM
  int32_t accel = 60000; // ERPM/s
  Serial.print("DEBUG MSG: hub motor target position: ");
  Serial.println(pos);
  uint32_t raw;

  uint8_t spd_payload[13];
  spd_payload[0] = 0x3C;
  spd_payload[1] = (uint8_t)(pos >> 24);
  spd_payload[2] = (uint8_t)(pos >> 16);
  spd_payload[3] = (uint8_t)(pos >> 8);
  spd_payload[4] = (uint8_t)(pos);
  spd_payload[5] = (uint8_t)(spd >> 24);
  spd_payload[6] = (uint8_t)(spd >> 16);
  spd_payload[7] = (uint8_t)(spd >> 8);
  spd_payload[8] = (uint8_t)(spd);
  spd_payload[9] = (uint8_t)(accel >> 24);
  spd_payload[10] = (uint8_t)(accel >> 16);
  spd_payload[11] = (uint8_t)(accel >> 8);
  spd_payload[12] = (uint8_t)(accel);

  this->writeMotorCommand(spd_payload, sizeof(spd_payload));
}

void HubMotor::writeTargetVel() {
  int32_t vel = direction * this->target_vel * vel_scale;
  uint8_t payload[] = {(uint8_t)COMM_SET_RPM, (uint8_t)(vel >> 24),
                       (uint8_t)(vel >> 16), (uint8_t)(vel >> 8),
                       (uint8_t)(vel)};

  this->writeMotorCommand(payload, sizeof(payload));
}

void HubMotor::writeZeroCurrent() {
  int32_t current = 0;
  uint8_t payload[] = {0x47, (uint8_t)(current >> 24), (uint8_t)(current >> 16),
                       (uint8_t)(current >> 8), (uint8_t)(current)};

  this->writeMotorCommand(payload, 5);
}

void HubMotor::setMode(ControlMode mode) {
  if (this->mode != mode) {
    pos_pid.reset();
    vel_pid.reset();
  }

  this->mode = mode;
  switch (mode) {
  case OPEN_LOOP:
    break;
  case VELOCITY_CONTROL:
    break;
  case DISABLED:
    this->writeZeroCurrent();
    break;
  case POSITION_CONTROL:
    break;
  }
}

void HubMotor::updateSensorData(float current_pos, float dt) {
  unsigned long now = millis();
  if (now - last_pos_poll_ms < 100)
    return;
  last_pos_poll_ms = now;
  while (motor.available())
    motor.read();
  motor.write(getPos, sizeof(getPos));
  uint8_t message[10];
  uint8_t len = 0;
  unsigned long header_timeout = now + 30;
  while (millis() < header_timeout) {
    if (motor.available()) {
      uint8_t b = motor.read();
      if (b == FRAME_HEADER) {
        message[len++] = b;
        break;
      }
    }
  }

  unsigned long timeout = millis() + 30;
  while (millis() < timeout && len > 0 && len < 10) {
    if (motor.available())
      message[len++] = motor.read();
  }
  if (len == 10 && message[0] == 0xAA && message[2] == 0x57 &&
      message[9] == 0xBB) {
    uint32_t raw = ((uint32_t)message[3] << 24) | ((uint32_t)message[4] << 16) |
                   ((uint32_t)message[5] << 8) | (uint32_t)message[6];
    float position;
    memcpy(&position, &raw, 4);

    this->current_pos = position;
    this->prev_pos = this->current_pos;
  }
}

void HubMotor::disable() {
  target_pwm = 0.0f;
  target_vel = 0.0f;
  target_pos = current_pos;
  pos_pid.reset();
  vel_pid.reset();
  this->setMode(DISABLED);
}

void HubMotor::setOrigin() {
  uint8_t payload[] = {0x40};
  this->writeMotorCommand(payload, 1);
  this->current_pos = 0;
  this->target_pos = 0;
  this->prev_pos = 0;
  this->current_vel = 0;
  last_pos_sample_ms = 0;
}
