#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>

// Phase 4: State Machine
enum SystemState {
  INIT,
  IDLE,
  TUNER_MODE,
  ESTOP,
  SELF_LEVELING,
  CONFIGURATION,
  AUTO_CURB_CLIMBING
};

// Phase 4: System Telemetry
// Motor order for all 6-element arrays: [rc, fc, ml, mr, ml_carriage,
// mr_carriage] Limit switch order for limit_switches[4]: [ml_fwd, ml_bwd,
// mr_fwd, mr_bwd] IMU order for imu[6]: [pitch, roll, yaw, ax, ay, az]
// Quaternion order for quat[4]: [w, x, y, z]
// Leveling order for leveling[5]: [pitch_err, roll_err, z_ml, z_rc, z_mr]
// Strain gauge order for sg[4]: [rc, fc, ml, mr]
struct SystemTelemetry {
  SystemState state;
  float positions[6];
  float velocities[6];
  float pwms[6];
  int directions[6];
  int enc_directions[6];
  bool limit_switches[4];
  float imu[6];
  float quat[4];
  float leveling[5];
  float sg[4];
  int modes[6];
  float drive_positions[2];
  float drive_velocities[2];
  float drive_pwms[2];
};

// Extern declarations for globals accessed by telemetry functions
// (defined in Base.ino)
class Motor;
class IMU_Class;
class StrainGauge;

extern SystemState current_state;
extern SystemTelemetry telemetry;
extern Motor rc, fc, ml, mr, ml_carriage, mr_carriage, ml_drive, mr_drive;
extern IMU_Class IMU;
extern StrainGauge sg_rc, sg_fc, sg_ml, sg_mr;
extern bool ml_fwd_limit, ml_bwd_limit, mr_fwd_limit, mr_bwd_limit;

// Update telemetry struct from current sensor/motor state
void updateTelemetry();

// Send telemetry as CSV line over Serial
void sendTelemetry();

#endif // TELEMETRY_H
