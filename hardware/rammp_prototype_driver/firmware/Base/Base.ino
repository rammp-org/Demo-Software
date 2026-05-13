#include <Arduino.h>
#include "src/Constants/Constants.h"
#include "src/EncoderContainer/EncoderContainer.h"
#include "src/IMU_Class/IMU_Class.h"
#include "src/ConfigStorage/ConfigStorage.h"
#include <SD.h>
#include <SPI.h>
#include "src/Timer/Timer.h"
// TODO: Timer::updateTime() prints to Serial every loop — remove debug output
// to reduce serial noise
#include <Wire.h>
#include "src/RoboClaw/RoboClaw.h"
// TODO: Upgrade to basicmicro_arduino library (RoboClaw library is deprecated
// per vendor README)
#include <utility/imumaths.h>

#include "src/Motor/Motor.h"
#include "src/CommandParser/CommandParser.h"
// TODO: CommandParser uses Arduino String — replace with fixed-size char buffer
// to avoid heap fragmentation
#include "src/MotorMap/MotorMap.h"
#include "src/SequencePlayer/SequencePlayer.h"
#include "src/Telemetry/Telemetry.h"
#include "src/CommandDispatch/CommandDispatch.h"
#include "src/PIDController/PIDController.h"
#include "src/StrainGauge/StrainGauge.h"

#include <ODriveUART.h>
#include "ODriveEnums.h"
#include "src/ODrive/ODrive.h"
#define DEBUG_MODE 0

// Drive motor position deadzone. When the FB position error is within this
// many ticks, the target is snapped to current position and both PIDs are
// cleared so the telemetry PWM output (read by the RNET joystick spoofer)
// stays exactly zero rather than hunting.
#define DRIVE_DEADZONE_TICKS 300.0f

#define CAL_NUM_MOTORS 6
#define CAL_MIN_DRIVE_MS 6000
#define CAL_VEL_THRESHOLD 2.0f
// TODO: Make DEBUG_MODE runtime-configurable via serial command

// SystemState enum and SystemTelemetry struct moved to
// src/Telemetry/Telemetry.h

// Global State
SystemState current_state = INIT;
SystemTelemetry telemetry;
bool calibrated = false;

// Sequence player state moved to src/SequencePlayer/

// Self Leveling Targets
float target_pitch = 0.0f;
float target_roll = 0.0f;

// Hardware Objects
Adafruit_BNO055 bno = Adafruit_BNO055(55);
IMU_Class IMU = IMU_Class(bno);
EncoderContainer EContr;
Timer timer;
CommandParser parser(2000);

// Limit switch states (global for telemetry)
bool ml_fwd_limit = false;
bool ml_bwd_limit = false;
bool mr_fwd_limit = false;
bool mr_bwd_limit = false;

// Initialize RoboClaw Controllers
RoboClaw roboclaw_carriages(&Serial3, 10000); // Serial3
RoboClaw roboclaw_casters(&Serial4, 10000);   // Serial4
RoboClaw roboclaw_main(&Serial5, 10000);      // Serial5

// Init ODrive motors
HardwareSerial &odriveR_serial = Serial1;
HardwareSerial &odriveL_serial = Serial6;

// ODriveUART odriveL(odriveL_serial);
// ODriveUART odriveR(odriveR_serial);

// Instantiate the 6 actuated Motor objects + 2 body-frame drive controllers
Motor rc;
Motor fc;
Motor ml;
Motor mr;
Motor ml_carriage;
Motor mr_carriage;
Motor drive_fb;
Motor drive_lr;
ODriveUART odriveR(odriveR_serial);
ODriveUART odriveL(odriveL_serial);
ODrive ODriveR(odriveR, -1); // hardware == robot +X
ODrive ODriveL(odriveL);     // flip hardware vs robot frame

int8_t ml_enc_dir = 1;
int8_t mr_enc_dir = 1;

float raw_ml_enc_pos = 0, raw_mr_enc_pos = 0;
float raw_ml_enc_vel = 0, raw_mr_enc_vel = 0;

// Centralized motor-encoder mapping table (declared extern in MotorMap.h)
MotorEntry motor_map[8] = {
    {&rc, 3, &roboclaw_casters, 1, true, true, "rc"},
    {&fc, 2, &roboclaw_casters, 2, true, true, "fc"},
    {&ml, 7, &roboclaw_main, 1, true, true, "ml"},
    {&mr, 5, &roboclaw_main, 2, true, true, "mr"},
    {&ml_carriage, 11, &roboclaw_carriages, 2, true, true, "ml_carriage"},
    {&mr_carriage, 12, &roboclaw_carriages, 1, true, true, "mr_carriage"},
    {&drive_fb, 9, nullptr, 0, false, false, "drive_fb"},
    {&drive_lr, 10, nullptr, 0, false, false, "drive_lr"},
};

// Strain gauge objects — one per load cell (default lpf_alpha = 0.5)
StrainGauge sg_rc(RC_LOADCELL_PIN, 0.8f);
StrainGauge sg_fc(FC_LOADCELL_PIN, 0.8f);
StrainGauge sg_ml(ML_LOADCELL_PIN, 0.8f);
StrainGauge sg_mr(MR_LOADCELL_PIN, 0.8f);

int16_t scaled_mlc_pwm;
int16_t scaled_mrc_pwm;

int16_t scaled_ml_pwm;
int16_t scaled_mr_pwm;

// IMU offset
float pitch_trim_deg = 3.0f;
float roll_trim_deg = 2.0f;

float getPitchTrim() { return pitch_trim_deg; }
void setPitchTrim(float val) { pitch_trim_deg = val; }

float getRollTrim() { return roll_trim_deg; }
void setRollTrim(float val) { roll_trim_deg = val; }

// --- Self Leveling Kinematics ---
// Ported from legacy Base_old_self_leveling.ino

const float CARRIAGE_LEVEL_TARGET = 100.0f;
const float CARRIAGE_LEVEL_TOLERANCE = 200.0f;
const unsigned long LEVEL_BLEND_MS = 2000;

void runSelfLeveling(float dt) {
  static bool ik_was_active = false;
  static unsigned long blend_start = 0;
  static float hold_rc, hold_ml, hold_mr, hold_fc;

  rc.setMode(Motor::POSITION_CONTROL);
  ml.setMode(Motor::POSITION_CONTROL);
  mr.setMode(Motor::POSITION_CONTROL);
  ml_carriage.setMode(Motor::POSITION_CONTROL);
  mr_carriage.setMode(Motor::POSITION_CONTROL);
  fc.setMode(Motor::POSITION_CONTROL);

  ml_carriage.setTargetPosition(CARRIAGE_LEVEL_TARGET);
  mr_carriage.setTargetPosition(CARRIAGE_LEVEL_TARGET);

  bool ml_carr_ready = fabs(ml_carriage.current_pos - CARRIAGE_LEVEL_TARGET) <
                       CARRIAGE_LEVEL_TOLERANCE;
  bool mr_carr_ready = fabs(mr_carriage.current_pos - CARRIAGE_LEVEL_TARGET) <
                       CARRIAGE_LEVEL_TOLERANCE;

  if (!ml_carr_ready || !mr_carr_ready) {
    ik_was_active = false;
    rc.setTargetPosition(rc.current_pos);
    ml.setTargetPosition(ml.current_pos);
    mr.setTargetPosition(mr.current_pos);
    fc.setTargetPosition(fc.current_pos);
    return;
  }

  // Carriages ready — on first frame, capture hold positions for gradual blend
  if (!ik_was_active) {
    ik_was_active = true;
    blend_start = millis();
    hold_rc = rc.current_pos;
    hold_ml = ml.current_pos;
    hold_mr = mr.current_pos;
    hold_fc = fc.current_pos;
  }

  // offset IMU reading
  double pitch_trim_rad = (getPitchTrim() * PI / 180.0) / 2.0;
  imu::Quaternion q_trim_pitch(cos(pitch_trim_rad), sin(pitch_trim_rad), 0.0,
                               0.0);

  double roll_trim_rad = (getRollTrim() * PI / 180.0) / 2.0;
  imu::Quaternion q_trim_roll(cos(roll_trim_rad), 0.0, sin(roll_trim_rad), 0.0);

  // Apply trim to measured orientation before error calculation
  imu::Quaternion q_trim = q_trim_pitch * q_trim_roll;
  imu::Quaternion q_meas = q_trim * IMU.current_quat;

  // Construct target quaternion manually to avoid API differences in
  // fromAxisAngle Note: BNO055 defines Pitch as rotation around X, Roll as
  // rotation around Y. Because the IMU is mounted upside down, Roll is
  // physically offset by 180 deg.
  double p_rad = (target_pitch * PI / 180.0) / 2.0;
  imu::Quaternion q_target_pitch(cos(p_rad), sin(p_rad), 0.0, 0.0);

  double r_rad = (target_roll * PI / 180.0) / 2.0;
  imu::Quaternion q_target_roll(cos(r_rad), 0.0, sin(r_rad), 0.0);

  // Target orientation
  imu::Quaternion q_target = q_target_pitch * q_target_roll;

  // Calculate the rotation required to go from current to target
  imu::Quaternion q_err = q_target * q_meas.conjugate();

  // Extract the Pitch and Roll error directly from the Error Quaternion
  // Because the error is small, this will never hit gimbal lock or +/- 180
  // boundaries!
  double sinr_cosp = 2.0 * (q_err.w() * q_err.x() + q_err.y() * q_err.z());
  double cosr_cosp =
      1.0 - 2.0 * (q_err.x() * q_err.x() + q_err.y() * q_err.y());
  double err_x =
      atan2(sinr_cosp, cosr_cosp) * (180.0 / PI); // Roll error mapped to X

  double sinp = 2.0 * (q_err.w() * q_err.y() - q_err.z() * q_err.x());
  double err_y;
  if (abs(sinp) >= 1)
    err_y = copysign(M_PI / 2, sinp) * (180.0 / PI);
  else
    err_y = asin(sinp) * (180.0 / PI); // Pitch error mapped to Y

  // Convert exact, continuous error angles to radians
  float dpitchrd = err_x / DG; // BNO X = Robot Pitch
  float drollrd = err_y / DG;  // BNO Y = Robot Roll

  // Deadband to prevent jitter
  if (fabs(dpitchrd) < 0.01)
    dpitchrd = 0.0;
  if (fabs(drollrd) < 0.01)
    drollrd = 0.0;

  // --- Forward Kinematics offset ---
  // Compute the pitch/roll that the current wheel heights impose on the
  // chassis geometry.  Without this offset the controller commands all legs
  // to a uniform baseline (9.5 cm) when the IMU error reaches zero, which
  // undoes any slope correction.  Adding the FK offset keeps the rotation-
  // matrix targets consistent with the current leg differential on uneven
  // ground so the motors hold position once the chassis is level.
  //
  // Derivation (linearised from the mebot geometry matrix):
  //   z ≈ -pitch·x + roll·y + baseline
  //   ML  (x=-34, y=-31):  z_ml = 34·p - 31·r + c
  //   RC  (x=+34, y=  0):  z_rc = -34·p       + c   (avg of RC_L / RC_R)
  //   MR  (x=-34, y=+31):  z_mr = 34·p + 31·r + c
  //   ⇒  pitch_fk = ((z_ml + z_mr)/2 − z_rc) / 68
  //   ⇒  roll_fk  = (z_mr − z_ml) / 62
  float z_cur_ml = ml.current_pos / ML_CM_TO_TICKS;
  float z_cur_rc = rc.current_pos / RC_CM_TO_TICKS;
  float z_cur_mr = mr.current_pos / MR_CM_TO_TICKS;

  float pitch_fk = -((z_cur_ml + z_cur_mr) / 2.0f - z_cur_rc) / 68.0f;
  float roll_fk = (z_cur_mr - z_cur_ml) / 62.0f;

  // Combine IMU error with FK offset for slope-aware correction
  float dpitch_total = dpitchrd + pitch_fk;
  float droll_total = drollrd + roll_fk;

  // Build rotation matrix (combining pitch and roll)
  double rotm[4][4] = {0};
  rotm[0][0] = cos(dpitch_total);
  rotm[0][1] = 0.0;
  rotm[0][2] = sin(dpitch_total);
  rotm[0][3] = 0.0;

  rotm[1][0] = sin(droll_total) * sin(dpitch_total);
  rotm[1][1] = cos(droll_total);
  rotm[1][2] = -1 * sin(droll_total) * cos(dpitch_total);
  rotm[1][3] = 0.0;

  rotm[2][0] = -1 * cos(droll_total) * sin(dpitch_total);
  rotm[2][1] = sin(droll_total);
  rotm[2][2] = cos(droll_total) * cos(dpitch_total);
  rotm[2][3] = 8; // Baseline Z height (cm)

  rotm[3][0] = 0.0;
  rotm[3][1] = 0.0;
  rotm[3][2] = 0.0;
  rotm[3][3] = 1.0;

  // Chassis geometry matrix (mebot)
  // Columns: ML, RC_L, RC_R, MR
  double mebot[4][4] = {
      {-34, 34, 34, -34}, // X
      {-31, -11, 11, 31}, // Y
      {0, 0, 0, 0},       // Z
      {1, 1, 1, 1}        // Homogeneous
  };

  // Multiply rotm * mebot to get new coordinates
  double newmebot[4][4] = {0};
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      for (int inner = 0; inner < 4; inner++) {
        newmebot[row][col] += rotm[row][inner] * mebot[inner][col];
      }
    }
  }

  // Extract Z-heights for each actuator (row 2)
  float z_target_ml = newmebot[2][0];
  float z_target_rc = (newmebot[2][1] + newmebot[2][2]) /
                      2.0; // Average left/right caster height
  float z_target_mr = newmebot[2][3];

  // Blend from hold positions to IK targets over LEVEL_BLEND_MS
  // to prevent violent jerk when IK first engages.
  float blend =
      min(1.0f, (float)(millis() - blend_start) / (float)LEVEL_BLEND_MS);

  float ik_ml = z_target_ml * ML_CM_TO_TICKS;
  float ik_mr = z_target_mr * MR_CM_TO_TICKS;
  float ik_rc = z_target_rc * RC_CM_TO_TICKS;
  float ik_fc = FC_MAX_TICKS;

  ml.setTargetPosition(hold_ml + blend * (ik_ml - hold_ml));
  mr.setTargetPosition(hold_mr + blend * (ik_mr - hold_mr));
  rc.setTargetPosition(hold_rc + blend * (ik_rc - hold_rc));
  fc.setTargetPosition(hold_fc + blend * (ik_fc - hold_fc));

  // Store debug data for telemetry — leveling[]: [pitch_err, roll_err, z_ml,
  // z_rc, z_mr]
  telemetry.leveling[0] = err_y;
  telemetry.leveling[1] = err_x;
  telemetry.leveling[2] = z_target_ml;
  telemetry.leveling[3] = z_target_rc;
  telemetry.leveling[4] = z_target_mr;
}

// Save all 6 motor configs (PID, dirs, limits, current position) to EEPROM
void saveAllMotorConfigs() {
  Motor *all_motors[8] = {&rc,          &fc,          &ml,       &mr,
                          &ml_carriage, &mr_carriage, &drive_fb, &drive_lr};
  for (int i = 0; i < 8; i++) {
    Motor *m = all_motors[i];
    int motor_id = i + 1;
    MotorConfig conf = ConfigStorage::loadMotorConfig(motor_id);
    conf.motor_dir = m->getDirection();
    // Drive wheels: encoder_dir is tracked by ml_enc_dir/mr_enc_dir at runtime
    // because motor->encoder_dir is always reset to 1 after command handling.
    if (motor_id == 7) {
      conf.encoder_dir = ml_enc_dir;
    } else if (motor_id == 8) {
      conf.encoder_dir = mr_enc_dir;
    } else {
      conf.encoder_dir = m->getEncoderDirection();
    }
    conf.lpf_input_alpha = m->lpf_input_alpha;
    conf.pos_p = m->pos_pid.kp;
    conf.pos_i = m->pos_pid.ki;
    conf.pos_d = m->pos_pid.kd;
    conf.pos_ff = m->pos_pid.kff;
    conf.pos_lpf_alpha = m->pos_pid.getLpfAlpha();
    conf.pos_max_ramp_rate = m->pos_pid.max_ramp_rate;
    conf.vel_p = m->vel_pid.kp;
    conf.vel_i = m->vel_pid.ki;
    conf.vel_d = m->vel_pid.kd;
    conf.vel_ff = m->vel_pid.kff;
    conf.vel_lpf_alpha = m->vel_pid.getLpfAlpha();
    conf.vel_max_ramp_rate = m->vel_pid.max_ramp_rate;
    conf.saved_position = m->current_pos;
    conf.pos_limit_min = m->pos_limit_min;
    conf.pos_limit_max = m->pos_limit_max;
    ConfigStorage::saveMotorConfig(motor_id, conf);
  }
}

// Save a single motor's config (PID, dirs, limits, current position) to EEPROM
void saveMotorConfig(int motor_id, Motor *m) {
  MotorConfig conf = ConfigStorage::loadMotorConfig(motor_id);
  conf.motor_dir = m->getDirection();
  // Drive wheels: encoder_dir is tracked by ml_enc_dir/mr_enc_dir at runtime
  // because motor->encoder_dir is always reset to 1 after command handling.
  if (motor_id == 7) {
    conf.encoder_dir = ml_enc_dir;
  } else if (motor_id == 8) {
    conf.encoder_dir = mr_enc_dir;
  } else {
    conf.encoder_dir = m->getEncoderDirection();
  }
  conf.lpf_input_alpha = m->lpf_input_alpha;
  conf.pos_p = m->pos_pid.kp;
  conf.pos_i = m->pos_pid.ki;
  conf.pos_d = m->pos_pid.kd;
  conf.pos_ff = m->pos_pid.kff;
  conf.pos_lpf_alpha = m->pos_pid.getLpfAlpha();
  conf.pos_max_ramp_rate = m->pos_pid.max_ramp_rate;
  conf.vel_p = m->vel_pid.kp;
  conf.vel_i = m->vel_pid.ki;
  conf.vel_d = m->vel_pid.kd;
  conf.vel_ff = m->vel_pid.kff;
  conf.vel_lpf_alpha = m->vel_pid.getLpfAlpha();
  conf.vel_max_ramp_rate = m->vel_pid.max_ramp_rate;
  conf.saved_position = m->current_pos;
  conf.pos_limit_min = m->pos_limit_min;
  conf.pos_limit_max = m->pos_limit_max;
  ConfigStorage::saveMotorConfig(motor_id, conf);
}

// --- Calibration ---
// Drives all 6 actuated motors open-loop until they stall against their
// mechanical limits, then zeros the encoders. Requires at least
// CAL_MIN_DRIVE_MS of driving before checking for stall.

unsigned long cal_start_ms = 0;
float cal_pwm = 0.0f;
bool cal_done[CAL_NUM_MOTORS] = {};

void startCalibration(float pwm) {
  cal_start_ms = millis();
  cal_pwm = pwm;
  Motor *cal_motors[CAL_NUM_MOTORS] = {&rc, &fc,          &ml,
                                       &mr, &ml_carriage, &mr_carriage};
  for (int i = 0; i < CAL_NUM_MOTORS; i++) {
    cal_done[i] = false;
    cal_motors[i]->setMode(Motor::OPEN_LOOP);
    cal_motors[i]->updateLimits(-9999999, cal_motors[i]->pos_limit_max);
    cal_motors[i]->setTargetPWM(cal_pwm);
  }
  if (DEBUG_MODE)
    Serial.println("CAL: Started calibration");
}

void runCalibration(float dt) {
  Motor *cal_motors[CAL_NUM_MOTORS] = {&rc, &fc,          &ml,
                                       &mr, &ml_carriage, &mr_carriage};
  unsigned long elapsed = millis() - cal_start_ms;
  bool all_done = true;

  for (int i = 0; i < CAL_NUM_MOTORS; i++) {
    if (cal_done[i])
      continue;

    all_done = false;

    if (elapsed >= CAL_MIN_DRIVE_MS &&
        fabsf(cal_motors[i]->current_vel) < CAL_VEL_THRESHOLD) {
      cal_motors[i]->setTargetPWM(0);

      int enc_idx = motor_map[i].encoder_index;
      EContr.zeroEncoder(enc_idx);
      cal_motors[i]->pos_pid.reset();
      cal_motors[i]->vel_pid.reset();
      cal_motors[i]->target_pos = 0;
      cal_motors[i]->current_pos = 0;
      cal_motors[i]->prev_pos = 0;

      cal_done[i] = true;

      if (DEBUG_MODE) {
        Serial.print("CAL: Homed ");
        Serial.println(motor_map[i].name);
      }
    }
  }

  if (all_done) {
    for (int i = 0; i < CAL_NUM_MOTORS; i++) {
      cal_motors[i]->disable();
      if (i == 4 || i == 5) {
        cal_motors[i]->updateLimits(100, cal_motors[i]->pos_limit_max);
      } else {
        cal_motors[i]->updateLimits(20, cal_motors[i]->pos_limit_max);
      }
    }
    calibrated = true;
    current_state = IDLE;
    Serial.println("CAL_DONE");
  }
}

void abortCalibration() {
  Motor *cal_motors[CAL_NUM_MOTORS] = {&rc, &fc,          &ml,
                                       &mr, &ml_carriage, &mr_carriage};
  for (int i = 0; i < CAL_NUM_MOTORS; i++)
    cal_motors[i]->disable();
  current_state = UNCALIBRATED;
  if (DEBUG_MODE)
    Serial.println("CAL: Aborted -> UNCALIBRATED");
}

void setup() {
  Serial.begin(460800);  // jetson
  Serial3.begin(460800); // roboclaw 1
  Serial4.begin(460800); // roboclaw 2
  Serial5.begin(460800); // roboclaw 3
  Serial1.begin(460800); // odrive right
  // Serial7.begin(460800); // unknown serial port
  Serial6.begin(460800); // odrive left

  // set up limit switches
  pinMode(CARRIAGE_SW1_PIN, INPUT_PULLUP);
  pinMode(CARRIAGE_SW2_PIN, INPUT_PULLUP);
  pinMode(CARRIAGE_SW3_PIN, INPUT_PULLUP);
  pinMode(CARRIAGE_SW4_PIN, INPUT_PULLUP);

  delay(1000);

  // Initialize IMU
  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected!");
  } else {
    IMU.initialize_BNO055_sensor();
    Serial.println("IMU initialized");
  }

  // Initialize ConfigStorage and load saved motor configurations
  ConfigStorage::begin();

  auto safe_f = [](float v) -> float {
    return (isnan(v) || isinf(v)) ? 0.0f : v;
  };

  Motor *all_motors[8] = {&rc,          &fc,          &ml,       &mr,
                          &ml_carriage, &mr_carriage, &drive_fb, &drive_lr};
  for (int i = 0; i < 8; i++) {
    MotorConfig conf = ConfigStorage::loadMotorConfig(i + 1);
    all_motors[i]->setDirection(conf.motor_dir);
    all_motors[i]->setEncoderDirection(conf.encoder_dir);
    // Restore drive wheel kinematics encoder direction from EEPROM.
    // ml_enc_dir/mr_enc_dir are the runtime source of truth for drive wheel
    // encoder direction; they must match what was saved.
    if (i == 6)
      ml_enc_dir = conf.encoder_dir;
    if (i == 7)
      mr_enc_dir = conf.encoder_dir;
    all_motors[i]->setInputLpfAlpha(safe_f(conf.lpf_input_alpha));
    all_motors[i]->pos_pid.kp = safe_f(conf.pos_p);
    all_motors[i]->pos_pid.ki = safe_f(conf.pos_i);
    all_motors[i]->pos_pid.kd = safe_f(conf.pos_d);
    all_motors[i]->pos_pid.setFeedForward(safe_f(conf.pos_ff));
    all_motors[i]->pos_pid.setLpfAlpha(safe_f(conf.pos_lpf_alpha));
    all_motors[i]->pos_pid.setRampRate(safe_f(conf.pos_max_ramp_rate));

    all_motors[i]->vel_pid.kp = safe_f(conf.vel_p);
    all_motors[i]->vel_pid.ki = safe_f(conf.vel_i);
    all_motors[i]->vel_pid.kd = safe_f(conf.vel_d);
    all_motors[i]->vel_pid.setFeedForward(safe_f(conf.vel_ff));
    all_motors[i]->vel_pid.setLpfAlpha(safe_f(conf.vel_lpf_alpha));
    all_motors[i]->vel_pid.setRampRate(safe_f(conf.vel_max_ramp_rate));

    all_motors[i]->updateLimits(conf.pos_limit_min, conf.pos_limit_max);
    // TODO: @alex explain this code or fix it, this looks insane VVVVV

    // Restore encoder offset so the filtered position resumes from
    // saved_position. Map motor index (0-5) to encoder container index,
    // matching updateSensorData().
    int enc_idx = motor_map[i].encoder_index;

    // saved_position is the logical position (after encoder_dir flip).
    // Divide by encoder_dir to recover the raw tick count, then set the
    // offset so that (raw_reading - offset) == saved_position.
    // Guard: encoder_dir is validated to ±1 by loadMotorConfig; check anyway.
    if (conf.encoder_dir != 0 && !isnan(conf.saved_position) &&
        !isinf(conf.saved_position)) {
      EContr.encoder_offset[enc_idx] =
          EContr.getRawReading(enc_idx) -
          (signed long)(conf.saved_position / (float)conf.encoder_dir);
    }
  }

  rc.attachStrainGauge(&sg_rc);
  fc.attachStrainGauge(&sg_fc);
  ml.attachStrainGauge(&sg_ml);
  mr.attachStrainGauge(&sg_mr);

  // Drive wheel motor objects keep encoder_dir=1 at all times.
  // The actual direction is tracked in ml_enc_dir/mr_enc_dir globals,
  // which were already loaded from EEPROM in the loop above (lines 429-430).
  drive_fb.setEncoderDirection(1);
  drive_lr.setEncoderDirection(1);

  Serial.println(
      "EEPROM CONFIG LOADED: All motor configs restored from EEPROM.");
  current_state =
      IDLE; // TODO: change back to UNCALIBRATED after odrive testing done
  calibrated = true; // TODO: change back to false after odrive testing done
  // Serial.println("STATE: UNCALIBRATED — calibration required before
  // operation");
}

void loop() {
  // Serial.println("LOOP: Entered loop");
  timer.updateTime();
  float dt = timer.elapsed_time;

  // Track whether the host has ever sent commands this session, so we only
  // auto-save on a real disconnect (not on first boot with no connection yet).
  static bool was_connected = false;

  // 1. Read Sensors
  EContr.retrieve_readings();
  IMU.retrieve_readings();

  // Strain gauges
  sg_rc.update(dt);
  sg_fc.update(dt);
  sg_ml.update(dt);
  sg_mr.update(dt);
  rc.updateLoad();
  fc.updateLoad();
  ml.updateLoad();
  mr.updateLoad();

  // ODrive note: idk yet if I should add odrive update encoder reading function
  // here
  ODriveR.updateEncoderReadings();
  ODriveL.updateEncoderReadings();

  rc.updateSensorData(EContr.encoderf[3], dt);
  fc.updateSensorData(EContr.encoderf[2], dt);
  ml.updateSensorData(EContr.encoderf[7], dt);
  mr.updateSensorData(EContr.encoderf[5], dt);
  ml_carriage.updateSensorData(EContr.encoderf[11], dt);
  mr_carriage.updateSensorData(EContr.encoderf[12], dt);
  {
    static float prev_ml = 0, prev_mr = 0;
    float ml_enc = EContr.encoderf[9] * ml_enc_dir;
    float mr_enc = EContr.encoderf[10] * mr_enc_dir;

    drive_fb.updateSensorData((ml_enc + mr_enc) / 2.0f, dt);
    drive_lr.updateSensorData((ml_enc - mr_enc), dt);

    raw_ml_enc_vel = (dt > 0) ? (ml_enc - prev_ml) / dt : 0;
    raw_mr_enc_vel = (dt > 0) ? (mr_enc - prev_mr) / dt : 0;
    prev_ml = ml_enc;
    prev_mr = mr_enc;
    raw_ml_enc_pos = ml_enc;
    raw_mr_enc_pos = mr_enc;
  }

  // 2. Parse Comms
  // TODO: Add checksums/framing to serial protocol for reliability
  RobotCommand cmd = parser.parse(Serial);

  if (DEBUG_MODE && cmd.type != CMD_NONE && cmd.type != CMD_UNKNOWN) {
    Serial.print("DEBUG: Received CMD type=");
    Serial.print(cmd.type);
    Serial.print(" id=");
    Serial.print(cmd.actuator_id);
    Serial.print(" val=");
    Serial.println(cmd.value, 4);
  }

  // Mark connected whenever the watchdog is being fed (commands are flowing)
  if (!parser.isTimedOut()) {
    was_connected = true;
  }

  // 3. Update State Machine
  if (parser.isTimedOut() && current_state != ESTOP) {
    current_state = ESTOP;
    Serial.println("WATCHDOG TIMEOUT -> ESTOP");
    // Auto-save all motor configs on disconnect (fires once per disconnect)
    if (was_connected) {
      saveAllMotorConfigs();
      Serial.println("AUTO-SAVE: All motor configs saved on disconnect.");
      was_connected = false;
    }
  }

  if (cmd.type == CMD_Z) {
    if (current_state != ESTOP) {
      current_state = ESTOP;
      if (DEBUG_MODE)
        Serial.println("DEBUG: Manual ESTOP Triggered");
    }
  } else if (cmd.type == CMD_C && current_state == ESTOP) {
    current_state = calibrated ? IDLE : UNCALIBRATED;
    parser.feedWatchdog();
    was_connected = true; // Re-arm auto-save for next disconnect
    if (DEBUG_MODE) {
      if (calibrated)
        Serial.println("DEBUG: ESTOP Cleared, entering IDLE");
      else
        Serial.println("DEBUG: ESTOP Cleared, entering UNCALIBRATED");
    }
  } else if (cmd.type == CMD_SEQ_MODE) {
    // All 8 motors are position-controlled during sequences (including drive
    // wheels).
    // ODrive note: add odrives to the motor array
    // Serial.println("SEQ: Entered SEQ_MODE");
    Motor *seq_motors[SEQ_NUM_MOTORS] = {
        &rc, &fc, &ml, &mr, &ml_carriage, &mr_carriage, &drive_fb, &drive_lr};
    ODrive *seq_odrives[SEQ_NUM_ODRIVES] = {&ODriveR, &ODriveL};
    if (cmd.actuator_id == 1) {
      // Serial.println("SEQ: B1:1 / B1:0 — enter or exit sequence mode");
      // B1:1 / B1:0 — enter or exit sequence mode
      if (cmd.value > 0.5f) {
        // Serial.println("SEQ: B1:1 — entering AUTO_CURB_CLIMBING mode");
        current_state = AUTO_CURB_CLIMBING;
        sequenceEnter(seq_motors, seq_odrives);
        Serial.println("SEQ: Entered AUTO_CURB_CLIMBING mode");
      } else {
        Serial.println("SEQ: Exited AUTO_CURB_CLIMBING mode");
        current_state = calibrated ? IDLE : UNCALIBRATED;
        sequenceExit(seq_motors, seq_odrives);
        // Serial.println("SEQ: Exited AUTO_CURB_CLIMBING mode");
      }
    } else if (cmd.actuator_id == 2) {
      // B2:1 / B2:0 — enable or disable auto-run
      sequenceSetAutoRun(cmd.value > 0.5f);
    }
  } else if (cmd.type == CMD_CALIBRATE) {
    if (cmd.value != 0.0f) {
      current_state = CALIBRATING;
      startCalibration(cmd.value);
    } else if (current_state == CALIBRATING) {
      abortCalibration();
    }
  } else if (cmd.type == CMD_LEVEL_MODE) {
    if (cmd.value > 0.5) {
      // If jumping from AUTO_CURB_CLIMBING, clean up sequence state first
      // so drive wheels don't stay in POSITION_CONTROL with stale targets.
      if (current_state == AUTO_CURB_CLIMBING) {
        Motor *seq_motors[SEQ_NUM_MOTORS] = {
            &rc,          &fc,          &ml,       &mr,
            &ml_carriage, &mr_carriage, &drive_fb, &drive_lr};
        ODrive *seq_odrives[SEQ_NUM_ODRIVES] = {&ODriveR, &ODriveL};
        sequenceExit(seq_motors, seq_odrives);
      }
      current_state = SELF_LEVELING;
      if (DEBUG_MODE)
        Serial.println("DEBUG: Entering SELF_LEVELING mode");
    } else {
      // Disable all position-controlled motors (zero power, PIDs reset).
      // Motors are not backdrivable so holding position is unnecessary.
      rc.disable();
      fc.disable();
      ml.disable();
      mr.disable();
      ml_carriage.disable();
      mr_carriage.disable();
      current_state = calibrated ? IDLE : UNCALIBRATED;
      if (DEBUG_MODE)
        Serial.println(calibrated
                           ? "DEBUG: Exiting SELF_LEVELING mode"
                           : "DEBUG: Exiting SELF_LEVELING -> UNCALIBRATED");
    }
  } else if (cmd.type == CMD_LEVEL_PITCH) {
    target_pitch = cmd.value;
    if (DEBUG_MODE)
      Serial.print("DEBUG: Set target pitch: ");
    Serial.println(target_pitch);
  } else if (cmd.type == CMD_LEVEL_ROLL) {
    target_roll = cmd.value;
    if (DEBUG_MODE)
      Serial.print("DEBUG: Set target roll: ");
    Serial.println(target_roll);
  } else if (cmd.type != CMD_NONE && current_state == IDLE) {
    current_state = TUNER_MODE;
    if (DEBUG_MODE)
      Serial.println("DEBUG: Entering TUNER_MODE");
  } else if (cmd.type != CMD_NONE && current_state == UNCALIBRATED &&
             cmd.type != CMD_CALIBRATE && cmd.type != CMD_GET_CONFIG) {
    // Block motor commands in UNCALIBRATED state — calibration required first
    if (DEBUG_MODE)
      Serial.println("DEBUG: Command rejected — calibration required");
  }

  // Config reads are safe during any state (including E-Stop).
  if (cmd.type == CMD_GET_CONFIG && cmd.type != CMD_NONE) {
    const MotorEntry *cfg_entry = getMotorEntry(cmd.actuator_id);
    Motor *cfg_m = cfg_entry ? cfg_entry->motor : nullptr;
    if (cfg_m != nullptr) {
      CommandContext cfg_ctx = {cfg_m,     (uint8_t)cmd.actuator_id,
                                cmd.value, parser.last_payload,
                                EContr,    cfg_entry};
      dispatchCommand(cmd, cfg_ctx);
    }
  }

  if (current_state == TUNER_MODE && cmd.type != CMD_NONE) {
    if (cmd.type == CMD_ODRIVE_POS) {
      parser.feedWatchdog();
      const int axis = cmd.actuator_id;
      const float pos = cmd.value;
      auto apply_od = [&](ODrive &od) {
        od.setMode(ODrive::POSITION_CONTROL);
        od.setTargetPosition(pos);
      };
      if (axis == 0) {
        apply_od(ODriveR);
        apply_od(ODriveL);
      } else if (axis == 1) {
        apply_od(ODriveL);
      } else if (axis == 2) {
        apply_od(ODriveR);
      }
    } else if (cmd.type == CMD_SAVE_CONFIG && cmd.actuator_id == 0) {
      saveAllMotorConfigs();
      if (DEBUG_MODE) {
        Serial.println("DEBUG: Saved config for ALL motors (K0)");
      }
    } else {
      const MotorEntry *entry = getMotorEntry(cmd.actuator_id);
      Motor *m = entry ? entry->motor : nullptr;

      if (m != nullptr) {
        // Build dispatch context and delegate to table-driven handler
        CommandContext ctx = {m,         (uint8_t)cmd.actuator_id,
                              cmd.value, parser.last_payload,
                              EContr,    entry};
        dispatchCommand(cmd, ctx);
      }
    }
  }

  // Sequence command dispatch (delegated to SequencePlayer module)
  if (current_state == AUTO_CURB_CLIMBING && cmd.type != CMD_NONE) {
    Motor *seq_motors[SEQ_NUM_MOTORS] = {
        &rc,          &fc,       &ml,      &mr, &ml_carriage,
        &mr_carriage, &drive_fb, &drive_lr}; // indices 0-5: position-mode; 6-7:
                                             // velocity-mode (drive wheels)
    ODrive *seq_odrives[SEQ_NUM_ODRIVES] = {&ODriveR, &ODriveL};
    sequenceHandleCommand(cmd, seq_motors, seq_odrives, parser.last_payload);
  }

  // 4. Update Motors
  if (current_state == ESTOP) {
    // Stop all motors safely via DISABLED mode
    rc.disable();
    fc.disable();
    ml.disable();
    mr.disable();
    ml_carriage.disable();
    mr_carriage.disable();
    drive_fb.disable();
    drive_lr.disable();
    // ODrive note: need to disable odrives
    ODriveR.disable();
  } else if (current_state == SELF_LEVELING) {
    // Drive wheels are not used during leveling — disable every tick to prevent
    // stale PID output from leaking to the joystick (e.g. if a prior mode left
    // drive_fb in POSITION_CONTROL with a stale target).
    drive_fb.disable();
    drive_lr.disable();
    // ODrive note: need to disable odrives
    ODriveR.disable();
    runSelfLeveling(dt);
  } else if (current_state == AUTO_CURB_CLIMBING) {
    // ODrive note: need to add odrives to the motor array
    Motor *seq_motors[SEQ_NUM_MOTORS] = {
        &rc,          &fc,       &ml,      &mr, &ml_carriage,
        &mr_carriage, &drive_fb, &drive_lr}; // indices 0-5: position-mode; 6-7:
                                             // velocity-mode (drive wheels)
    ODrive *seq_odrives[SEQ_NUM_ODRIVES] = {&ODriveR, &ODriveL};
    sequenceUpdate(seq_motors, seq_odrives);
  } else if (current_state == CALIBRATING) {
    runCalibration(dt);
  }

  float rc_pwm = rc.update(dt);
  float fc_pwm = fc.update(dt);
  float ml_pwm = ml.update(dt);
  float mr_pwm = mr.update(dt);
  float mlc_pwm = ml_carriage.update(dt);
  float mrc_pwm = mr_carriage.update(dt);

  // Drive motor deadzone — applied only in position control mode.
  // If the position error is within ±DRIVE_DEADZONE_TICKS, snap the target to
  // the current position and clear both PIDs so the PWM output sent over
  // telemetry to the RNET joystick spoofer is exactly zero. Without this,
  // integrator windup near the setpoint produces a non-zero joystick command
  // and the wheelchair creeps even when it should be stationary.
  if (drive_fb.mode == Motor::POSITION_CONTROL &&
      fabsf(drive_fb.target_pos - drive_fb.current_pos) <
          DRIVE_DEADZONE_TICKS) {
    drive_fb.target_pos = drive_fb.current_pos;
    drive_fb.pos_pid.reset();
    drive_fb.vel_pid.reset();
  }
  if (drive_lr.mode == Motor::POSITION_CONTROL &&
      fabsf(drive_lr.target_pos - drive_lr.current_pos) <
          DRIVE_DEADZONE_TICKS) {
    drive_lr.target_pos = drive_lr.current_pos;
    drive_lr.pos_pid.reset();
    drive_lr.vel_pid.reset();
  }

  float mld_pwm = drive_fb.update(dt);
  float mrd_pwm = drive_lr.update(dt);

  // read limit switches (store in globals for telemetry)
  ml_fwd_limit = !digitalRead(CARRIAGE_SW1_PIN); // Active low
  ml_bwd_limit = !digitalRead(CARRIAGE_SW2_PIN);
  mr_fwd_limit = !digitalRead(CARRIAGE_SW3_PIN);
  mr_bwd_limit = !digitalRead(CARRIAGE_SW4_PIN);

  // Stop carriages if limit is hit
  if (ml_fwd_limit && mlc_pwm > 0) {
    mlc_pwm = 0;
  }

  if (ml_bwd_limit && mlc_pwm < 0) {
    mlc_pwm = 0;
  }

  if (mr_fwd_limit && mrc_pwm > 0) {
    mrc_pwm = 0;
  }

  if (mr_bwd_limit && mrc_pwm < 0) {
    mrc_pwm = 0;
  }

  // TODO: Check return values from RoboClaw DutyM1/M2 for communication error
  // detection Write PWM to RoboClaws (constrained strictly to 16-bit signed int
  // +/- 32767) roboclaw_main: M1 = Main Left, M2 = Main Right
  // TODO: change main wheel controls back to default
  roboclaw_main.DutyM1(0x80, (int16_t)ml_pwm);

  roboclaw_main.DutyM2(0x80, (int16_t)mr_pwm);

  // roboclaw_casters: M1 = Rear Caster, M2 = Front Caster
  roboclaw_casters.DutyM1(0x80, (int16_t)rc_pwm);
  roboclaw_casters.DutyM2(0x80, (int16_t)fc_pwm);

  roboclaw_carriages.DutyM1(0x80, (int16_t)mrc_pwm);

  roboclaw_carriages.DutyM2(0x80, (int16_t)mlc_pwm);

  odriveR.setPosition(ODriveR.getTargetPosition());
  odriveL.setPosition(ODriveL.getTargetPosition());
  // 5. Send Telemetry
  updateTelemetry();

  static unsigned long last_telem_time = 0;
  if (millis() - last_telem_time >= 100) { // Fixed 10Hz telemetry
    last_telem_time = millis();
    // sendTelemetry();
  }

  // TODO: Replace delayMicroseconds with hardware timer for deterministic loop
  delayMicroseconds(5000);
}
