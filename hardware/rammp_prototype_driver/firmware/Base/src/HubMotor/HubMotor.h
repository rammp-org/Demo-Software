#ifndef HUB_MOTOR_H
#define HUB_MOTOR_H

#include "../MotorBase/MotorBase.h"

#define COMM_GET_VALUES 69  // request all motor parameters
#define COMM_SET_DUTY 70    // set motor duty cycle
#define COMM_SET_POS 74     // set motor position
#define COMM_SET_POS_SPD 60 // motor operates in pos/vel loop mode
#define COMM_SET_RPM 73     // set motor velocity

class HubMotor : public MotorBase {
public:
  HubMotor(int axis_direction, HardwareSerial &motor);

  static constexpr uint8_t FRAME_HEADER = 0xAA;
  static constexpr uint8_t FRAME_TAIL = 0xBB;
  static constexpr uint8_t GET_POS_CMD_LEN = 7;

  int pwm_scale = 100000;
  int vel_scale = 100000;
  int pos_scale = 1000;
  HardwareSerial &motor;
  uint8_t getPos[GET_POS_CMD_LEN] = {0xAA, 0x02, 0x4C, 0x04, 0x08, 0x25, 0xBB};

  unsigned short crc16(const unsigned char *buf, unsigned int len);

  void setMode(ControlMode mode) override;
  void disable() override;
  void updateSensorData(float current_pos, float dt) override;

  void writeMotorCommand(uint8_t *payload, uint8_t payload_len);
  void writeZeroCurrent();
  void writePWM();
  void writeTargetPos();
  void setOrigin() override;
  void writeTargetVel();

private:
  unsigned long last_pos_poll_ms = 0;
  unsigned long last_pos_sample_ms = 0;
};

#endif
