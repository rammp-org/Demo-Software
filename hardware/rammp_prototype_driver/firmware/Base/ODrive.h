#include <ODriveUART.h>
#include "ODriveEnums.h"

class ODrive {
public:
  enum DriveMode { DISABLED, POSITION_CONTROL, VELOCITY_CONTROL, OPEN_LOOP };

  ODrive(ODriveUART &odrive);
  DriveMode mode;
  float current_pos;
  float target_pos;
  void updateEncoderReadings();
  void setMode(DriveMode new_mode);
  void setTargetPosition(float target_pos);
  float getTargetPosition();
  float getCurrentPosition();
  void disable();
  // may need to add absolute vs relative target position compute function
private:
  ODriveUART &odrive;
};
