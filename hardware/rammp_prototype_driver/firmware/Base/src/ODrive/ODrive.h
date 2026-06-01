#ifndef ODRIVE_H
#define ODRIVE_H

#include <ODriveUART.h>
#include "ODriveEnums.h"

#include "../MotorBase/MotorBase.h"

class ODrive : public MotorBase {
public:
  ODrive(ODriveUART &odrive, int axis_direction = 1);

  void setMode(ControlMode new_mode) override;
  void disable() override;
  void updateSensorData(float current_pos, float dt) override;

  float getTargetPosition();
  float getTargetVelocity();
  float getCurrentPosition();
  float getCurrentTorque();

private:
  ODriveUART &odrive;
};

#endif
