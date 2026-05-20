#ifndef ODRIVE_H
#define ODRIVE_H

#include <ODriveUART.h>
#include "ODriveEnums.h"

#include "../MotorBase/MotorBase.h"

class ODrive : public MotorBase {
public:
  /// axis_direction: +1 or -1 maps ODrive hardware position into robot
  ///  frame (read/write). SequencePlayer uses robot frame.
  ODrive(ODriveUART &odrive, int axis_direction = 1);

  void setMode(ControlMode new_mode) override;
  void disable() override;

  // only odrive functions
  void updateEncoderReadings();
  float getTargetPosition();
  float getTargetVelocity();
  float getCurrentPosition();
  float getCurrentTorque();
  // may need to add absolute vs relative target position compute function
private:
  ODriveUART &odrive;
  //   int direction;
};

#endif
