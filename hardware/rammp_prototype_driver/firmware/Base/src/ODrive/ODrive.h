#include <ODriveUART.h>
#include "ODriveEnums.h"

class ODrive {
public:
  enum DriveMode { DISABLED, POSITION_CONTROL, VELOCITY_CONTROL, OPEN_LOOP };

  /// \p axis_direction: +1 or -1 maps ODrive hardware position into robot
  /// frame (read/write). SequencePlayer uses robot frame.
  ODrive(ODriveUART &odrive, int axis_direction = 1);
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
  int direction;
};
