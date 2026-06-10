#include <Arduino.h>
#include "Telemetry.h"
#include "../Motor/Motor.h"
#include "../ODrive/ODrive.h"
#include "../IMU_Class/IMU_Class.h"
#include "../StrainGauge/StrainGauge.h"

// Map firmware ControlMode enum to GUI mode integers (0=Open Loop, 1=Velocity,
// 2=Position)
static inline int toGuiMode(Motor::ControlMode m) {
  switch (m) {
  case Motor::VELOCITY_CONTROL:
    return 1;
  case Motor::POSITION_CONTROL:
    return 2;
  default:
    return 0; // DISABLED or OPEN_LOOP
  }
}

// Helper to update telemetry — iterates motor/gauge arrays to avoid per-field
// repetition
void updateTelemetry() {
  Motor *motors[6] = {&rc, &fc, &ml, &mr, &ml_carriage, &mr_carriage};
  StrainGauge *gauges[4] = {&sg_rc, &sg_fc, &sg_ml, &sg_mr};

  telemetry.state = current_state;

  for (int i = 0; i < 6; i++) {
    telemetry.positions[i] = motors[i]->current_pos;
    telemetry.velocities[i] = motors[i]->current_vel;
    telemetry.pwms[i] = motors[i]->target_pwm;
    telemetry.directions[i] = motors[i]->getDirection();
    telemetry.enc_directions[i] = motors[i]->getEncoderDirection();
    telemetry.modes[i] = toGuiMode(motors[i]->mode);
  }

  for (int i = 0; i < 4; i++) {
    telemetry.sg[i] = gauges[i]->getValue();
  }

  telemetry.limit_switches[0] = ml_fwd_limit;
  telemetry.limit_switches[1] = ml_bwd_limit;
  telemetry.limit_switches[2] = mr_fwd_limit;
  telemetry.limit_switches[3] = mr_bwd_limit;

  // IMU: [pitch, roll, yaw, ax, ay, az]
  telemetry.imu[0] = IMU.pitchf;
  telemetry.imu[1] = IMU.rollf;
  telemetry.imu[2] = IMU.yaw;
  telemetry.imu[3] = IMU.ax;
  telemetry.imu[4] = IMU.ay;
  telemetry.imu[5] = IMU.az;

  // Quaternion: [w, x, y, z]
  telemetry.quat[0] = IMU.current_quat.w();
  telemetry.quat[1] = IMU.current_quat.x();
  telemetry.quat[2] = IMU.current_quat.y();
  telemetry.quat[3] = IMU.current_quat.z();

  telemetry.drive_positions[0] = drive_fb.current_pos;
  telemetry.drive_positions[1] = drive_lr.current_pos;
  telemetry.drive_velocities[0] = drive_fb.current_vel;
  telemetry.drive_velocities[1] = drive_lr.current_vel;

  // Only report drive PWMs when the system is actively commanding the wheels
  // (AUTO_CURB_CLIMBING).  In all other states, force zero so stale PID output
  // never leaks to the joystick spoofer.
  if (current_state == AUTO_CURB_CLIMBING) {
    telemetry.drive_pwms[0] = drive_fb.target_pwm * drive_fb.getDirection();
    telemetry.drive_pwms[1] = drive_lr.target_pwm * drive_lr.getDirection();
  } else {
    telemetry.drive_pwms[0] = 0;
    telemetry.drive_pwms[1] = 0;
  }
  telemetry.drive_modes[0] = toGuiMode(drive_fb.mode);
  telemetry.drive_modes[1] = toGuiMode(drive_lr.mode);
  telemetry.raw_enc_positions[0] = raw_ml_enc_pos;
  telemetry.raw_enc_positions[1] = raw_mr_enc_pos;
  telemetry.raw_enc_velocities[0] = raw_ml_enc_vel;
  telemetry.raw_enc_velocities[1] = raw_mr_enc_vel;
  telemetry.drive_directions[0] = drive_fb.getDirection();
  telemetry.drive_directions[1] = drive_lr.getDirection();
  telemetry.drive_enc_directions[0] = drive_fb.getEncoderDirection();
  telemetry.drive_enc_directions[1] = drive_lr.getEncoderDirection();

  telemetry.odrive_positions[0] = ODriveR.current_pos;
  telemetry.odrive_positions[1] = ODriveL.current_pos;
  telemetry.odrive_torques[0] = ODriveR.getCurrentTorque();
  telemetry.odrive_torques[1] = ODriveL.getCurrentTorque();

  // Optional: populated by other modules; defaults to 0.
  telemetry.carriage_return_direction = carriage_return_direction;
}

// Helper to send telemetry — builds the full CSV line into a buffer, single
// Serial.print Packet format (80 comma-separated values after the header):
//   TELEMETRY,<ms>,<state>,
//   <6 positions>,<6 velocities>,<6 pwms>,
//   <6 motor dirs>,<6 enc dirs>,<4 limit switches>,
//   <3 imu angles>,<3 imu accel>,<4 quaternion>,
//   <5 leveling debug>,<4 strain gauges>,<6 control modes>,
//   <2 drive positions>,<2 drive velocities>,<2 drive pwms>,
//   <2 drive control modes>,<2 raw enc positions>,<2 raw enc velocities>,
//   <2 drive directions>,<2 drive enc directions>,
//   <odrive_r_pos>,<odrive_l_pos>,<odrive_r_torque>,<odrive_l_torque>,
//   <carriage_return_direction>
void sendTelemetry() {
  char buf[800];
  int n = 0;

  // Header
  n += snprintf(buf + n, sizeof(buf) - n, "TELEMETRY,%lu,%d", millis(),
                (int)telemetry.state);

  // Per-motor groups (6 values each)
  for (int i = 0; i < 6; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%.2f", telemetry.positions[i]);
  for (int i = 0; i < 6; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%.2f", telemetry.velocities[i]);
  for (int i = 0; i < 6; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%.2f", telemetry.pwms[i]);
  for (int i = 0; i < 6; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%d", telemetry.directions[i]);
  for (int i = 0; i < 6; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%d", telemetry.enc_directions[i]);

  // Limit switches (4)
  for (int i = 0; i < 4; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%d",
                  telemetry.limit_switches[i] ? 1 : 0);

  // IMU angles (3 × 2dp) then accel (3 × 3dp)
  for (int i = 0; i < 3; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%.2f", telemetry.imu[i]);
  for (int i = 3; i < 6; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%.3f", telemetry.imu[i]);

  // Quaternion (4 × 4dp)
  for (int i = 0; i < 4; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%.4f", telemetry.quat[i]);

  // Leveling debug (5 × 4dp)
  for (int i = 0; i < 5; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%.4f", telemetry.leveling[i]);

  // Strain gauges (4 × 2dp)
  for (int i = 0; i < 4; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%.2f", telemetry.sg[i]);

  // Control modes (6)
  for (int i = 0; i < 6; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%d", telemetry.modes[i]);

  for (int i = 0; i < 2; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%.2f",
                  telemetry.drive_positions[i]);
  for (int i = 0; i < 2; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%.2f",
                  telemetry.drive_velocities[i]);
  for (int i = 0; i < 2; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%.2f", telemetry.drive_pwms[i]);
  for (int i = 0; i < 2; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%d", telemetry.drive_modes[i]);
  for (int i = 0; i < 2; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%.2f",
                  telemetry.raw_enc_positions[i]);
  for (int i = 0; i < 2; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%.2f",
                  telemetry.raw_enc_velocities[i]);

  for (int i = 0; i < 2; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%d",
                  telemetry.drive_directions[i]);
  for (int i = 0; i < 2; i++)
    n += snprintf(buf + n, sizeof(buf) - n, ",%d",
                  telemetry.drive_enc_directions[i]);

  n += snprintf(buf + n, sizeof(buf) - n, ",%.4f",
                telemetry.odrive_positions[0]);
  n += snprintf(buf + n, sizeof(buf) - n, ",%.4f",
                telemetry.odrive_positions[1]);
  n += snprintf(buf + n, sizeof(buf) - n, ",%.4f", telemetry.odrive_torques[0]);
  n += snprintf(buf + n, sizeof(buf) - n, ",%.4f", telemetry.odrive_torques[1]);

  n += snprintf(buf + n, sizeof(buf) - n, ",%d",
                telemetry.carriage_return_direction);

  n += snprintf(buf + n, sizeof(buf) - n, "\n");

  Serial.print(buf);
}
