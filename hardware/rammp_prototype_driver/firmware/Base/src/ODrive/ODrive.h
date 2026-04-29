#include <ODriveUART.h>
#include "ODriveEnums.h"

class ODrive {
public:
  enum DriveMode { DISABLED, OPEN_LOOP, VELOCITY_CONTROL, POSITION_CONTROL };

  ODrive(ODriveUART &odrive);
  ODriveUART &odrive;
  DriveMode mode;
  void setMode(DriveMode mode);
  void setTargetPosition(float target_pos);
  void setTargetVelocity(float target_vel);
  void disable();
};
