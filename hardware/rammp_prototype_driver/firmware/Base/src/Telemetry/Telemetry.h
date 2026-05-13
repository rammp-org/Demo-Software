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
  AUTO_CURB_CLIMBING,
  CALIBRATING,
  UNCALIBRATED
};

// System Telemetry
// Motor order for all 6-element arrays: [rc, fc, ml, mr, ml_carriage,
// mr_carriage] Limit switch order: [ml_fwd, ml_bwd, mr_fwd, mr_bwd] IMU order:
// [pitch, roll, yaw, ax, ay, az] Quaternion order: [w, x, y, z] Leveling order:
// [pitch_err, roll_err, z_ml, z_rc, z_mr] Strain gauge order: [rc, fc, ml, mr]
// Drive arrays [2]: [drive_fb (avg), drive_lr (diff)]
// drive_modes: control mode for drive_fb/drive_lr
// raw_enc: direction-corrected individual ML/MR encoder readings
// drive_directions: motor direction for drive_fb/drive_lr
// drive_enc_directions: encoder direction for drive_fb/drive_lr
// odrive_l/r: ODrive axis position (logical frame after ODrive wrapper sign)
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
  int drive_modes[2];
  float raw_enc_positions[2];
  float raw_enc_velocities[2];
  int drive_directions[2];
  int drive_enc_directions[2];
  float odrive_l_position;
  float odrive_r_position;
};

// Extern declarations for globals accessed by telemetry functions
// (defined in Base.ino)
class Motor;
class IMU_Class;
class StrainGauge;
class ODrive;

extern SystemState current_state;
extern SystemTelemetry telemetry;
extern bool calibrated;
extern Motor rc, fc, ml, mr, ml_carriage, mr_carriage, drive_fb, drive_lr;
extern float raw_ml_enc_pos, raw_mr_enc_pos;
extern float raw_ml_enc_vel, raw_mr_enc_vel;
extern IMU_Class IMU;
extern StrainGauge sg_rc, sg_fc, sg_ml, sg_mr;
extern bool ml_fwd_limit, ml_bwd_limit, mr_fwd_limit, mr_bwd_limit;
extern ODrive ODriveR;
extern ODrive ODriveL;

// Update telemetry struct from current sensor/motor state
void updateTelemetry();

// Send telemetry as CSV line over Serial
void sendTelemetry();

#endif // TELEMETRY_H
