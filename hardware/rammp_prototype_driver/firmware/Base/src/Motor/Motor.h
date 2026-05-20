#ifndef MOTOR_H
#define MOTOR_H

#include "../MotorBase/MotorBase.h"

class Motor : public MotorBase {
public:
  Motor();
  ~Motor() override = default;

  void setMode(ControlMode mode) override;
  void disable() override;
};

#endif
