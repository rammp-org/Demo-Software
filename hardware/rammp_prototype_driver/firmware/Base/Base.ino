#include <Arduino.h>
#include "src/Constants/Constants.h"
#include "src/EncoderContainer/EncoderContainer.h"
#include "src/IMU_Class/IMU_Class.h"
#include "src/ConfigStorage/ConfigStorage.h"
#include <SD.h>
#include <SPI.h>
#include "src/Timer/Timer.h"
#include <Wire.h>
#include "src/RoboClaw/RoboClaw.h"
#include <utility/imumaths.h>

#include "src/Motor/Motor.h"
#include "src/CommandParser/CommandParser.h"
#include "src/PIDController/PIDController.h"
#include "src/StrainGauge/StrainGauge.h"

#define DEBUG_MODE 1

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
// Motor order for all 6-element arrays: [rc, fc, ml, mr, ml_carriage, mr_carriage]
// Limit switch order for limit_switches[4]: [ml_fwd, ml_bwd, mr_fwd, mr_bwd]
// IMU order for imu[6]: [pitch, roll, yaw, ax, ay, az]
// Quaternion order for quat[4]: [w, x, y, z]
// Leveling order for leveling[5]: [pitch_err, roll_err, z_ml, z_rc, z_mr]
// Strain gauge order for sg[4]: [rc, fc, ml, mr]
struct SystemTelemetry {
  SystemState state;
  float positions[6];
  float velocities[6];
  float pwms[6];
  int   directions[6];
  int   enc_directions[6];
  bool  limit_switches[4];
  float imu[6];
  float quat[4];
  float leveling[5];
  float sg[4];
  int   modes[6];
};

// Global State
SystemState current_state = INIT;
SystemTelemetry telemetry;

// --- Sequence / Trajectory (AUTO_CURB_CLIMBING mode) ---
#define MAX_SEQ_KEYFRAMES 20
#define SEQ_NUM_MOTORS 6

struct Keyframe {
  float    targets[SEQ_NUM_MOTORS];
  bool     active[SEQ_NUM_MOTORS];
  uint32_t duration_ms;
};

Keyframe seq_keyframes[MAX_SEQ_KEYFRAMES];
int      seq_length        = 0;
int      seq_current       = -1;
bool     seq_interpolating = false;
unsigned long seq_interp_start = 0;
float    seq_start_pos[SEQ_NUM_MOTORS];

// Self Leveling Targets
float target_pitch = 0.0f;
float target_roll = 0.0f;

// Hardware Objects
Adafruit_BNO055 bno = Adafruit_BNO055(55);
IMU_Class IMU = IMU_Class(bno);
EncoderContainer EContr;
Timer timer;
CommandParser parser(60000);

// Limit switch states (global for telemetry)
bool ml_fwd_limit = false;
bool ml_bwd_limit = false;
bool mr_fwd_limit = false;
bool mr_bwd_limit = false;

// Initialize RoboClaw Controllers
RoboClaw roboclaw_carriages(&Serial3, 10000); // Serial3
RoboClaw roboclaw_casters(&Serial4, 10000);   // Serial4
RoboClaw roboclaw_main(&Serial5, 10000);      // Serial5

// Instantiate the 6 Motor objects
Motor rc;
Motor fc;
Motor ml;
Motor mr;
Motor ml_carriage;
Motor mr_carriage;

// Strain gauge objects — one per load cell (default lpf_alpha = 0.5)
StrainGauge sg_rc(RC_LOADCELL_PIN, 0.5f);
StrainGauge sg_fc(FC_LOADCELL_PIN, 0.6f);
StrainGauge sg_ml(ML_LOADCELL_PIN, 0.7f);
StrainGauge sg_mr(MR_LOADCELL_PIN, 0.8f);

int16_t scaled_mlc_pwm;
int16_t scaled_mrc_pwm;

int16_t scaled_ml_pwm;
int16_t scaled_mr_pwm;

// Map firmware ControlMode enum to GUI mode integers (0=Open Loop, 1=Velocity, 2=Position)
static inline int toGuiMode(Motor::ControlMode m) {
  switch (m) {
    case Motor::VELOCITY_CONTROL:  return 1;
    case Motor::POSITION_CONTROL:  return 2;
    default:                       return 0; // DISABLED or OPEN_LOOP
  }
}

// Helper to update telemetry — iterates motor/gauge arrays to avoid per-field repetition
void updateTelemetry() {
  Motor      *motors[6] = {&rc, &fc, &ml, &mr, &ml_carriage, &mr_carriage};
  StrainGauge *gauges[4] = {&sg_rc, &sg_fc, &sg_ml, &sg_mr};

  telemetry.state = current_state;

  for (int i = 0; i < 6; i++) {
    telemetry.positions[i]    = motors[i]->current_pos;
    telemetry.velocities[i]   = motors[i]->current_vel;
    telemetry.pwms[i]         = motors[i]->target_pwm;
    telemetry.directions[i]   = motors[i]->getDirection();
    telemetry.enc_directions[i] = motors[i]->getEncoderDirection();
    telemetry.modes[i]        = toGuiMode(motors[i]->mode);
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
}

// Helper to send telemetry — builds the full CSV line into a buffer, single Serial.print
// Packet format (59 comma-separated values after the header):
//   TELEMETRY,<ms>,<state>,
//   <6 positions>,<6 velocities>,<6 pwms>,
//   <6 motor dirs>,<6 enc dirs>,<4 limit switches>,
//   <3 imu angles>,<3 imu accel>,<4 quaternion>,
//   <5 leveling debug>,<4 strain gauges>,<6 control modes>
void sendTelemetry() {
  char buf[640];
  int  n = 0;

  // Header
  n += snprintf(buf + n, sizeof(buf) - n, "TELEMETRY,%lu,%d",
                millis(), (int)telemetry.state);

  // Per-motor groups (6 values each)
  for (int i = 0; i < 6; i++) n += snprintf(buf + n, sizeof(buf) - n, ",%.2f", telemetry.positions[i]);
  for (int i = 0; i < 6; i++) n += snprintf(buf + n, sizeof(buf) - n, ",%.2f", telemetry.velocities[i]);
  for (int i = 0; i < 6; i++) n += snprintf(buf + n, sizeof(buf) - n, ",%.2f", telemetry.pwms[i]);
  for (int i = 0; i < 6; i++) n += snprintf(buf + n, sizeof(buf) - n, ",%d",   telemetry.directions[i]);
  for (int i = 0; i < 6; i++) n += snprintf(buf + n, sizeof(buf) - n, ",%d",   telemetry.enc_directions[i]);

  // Limit switches (4)
  for (int i = 0; i < 4; i++) n += snprintf(buf + n, sizeof(buf) - n, ",%d", telemetry.limit_switches[i] ? 1 : 0);

  // IMU angles (3 × 2dp) then accel (3 × 3dp)
  for (int i = 0; i < 3; i++) n += snprintf(buf + n, sizeof(buf) - n, ",%.2f", telemetry.imu[i]);
  for (int i = 3; i < 6; i++) n += snprintf(buf + n, sizeof(buf) - n, ",%.3f", telemetry.imu[i]);

  // Quaternion (4 × 4dp)
  for (int i = 0; i < 4; i++) n += snprintf(buf + n, sizeof(buf) - n, ",%.4f", telemetry.quat[i]);

  // Leveling debug (5 × 4dp)
  for (int i = 0; i < 5; i++) n += snprintf(buf + n, sizeof(buf) - n, ",%.4f", telemetry.leveling[i]);

  // Strain gauges (4 × 2dp)
  for (int i = 0; i < 4; i++) n += snprintf(buf + n, sizeof(buf) - n, ",%.2f", telemetry.sg[i]);

  // Control modes (6)
  for (int i = 0; i < 6; i++) n += snprintf(buf + n, sizeof(buf) - n, ",%d",   telemetry.modes[i]);

  // Terminate with newline
  n += snprintf(buf + n, sizeof(buf) - n, "\n");

  Serial.print(buf);
}

// --- Self Leveling Kinematics ---
// Ported from legacy Base_old_self_leveling.ino
void runSelfLeveling(float dt) {
  // Set all actively controlled motors to POSITION_CONTROL mode
  rc.setMode(Motor::POSITION_CONTROL);
  ml.setMode(Motor::POSITION_CONTROL);
  mr.setMode(Motor::POSITION_CONTROL);
  ml_carriage.setMode(Motor::POSITION_CONTROL);
  mr_carriage.setMode(Motor::POSITION_CONTROL);
  fc.setMode(Motor::POSITION_CONTROL);

  // Get raw orientation from IMU
  imu::Quaternion q_meas = IMU.current_quat;

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
  if (fabs(dpitchrd) < 0.001)
    dpitchrd = 0.0;
  if (fabs(drollrd) < 0.001)
    drollrd = 0.0;

  // Build rotation matrix (combining pitch and roll)
  double rotm[4][4] = {0};
  rotm[0][0] = cos(dpitchrd);
  rotm[0][1] = 0.0;
  rotm[0][2] = sin(dpitchrd);
  rotm[0][3] = 0.0;

  rotm[1][0] = sin(drollrd) * sin(dpitchrd);
  rotm[1][1] = cos(drollrd);
  rotm[1][2] = -1 * sin(drollrd) * cos(dpitchrd);
  rotm[1][3] = 0.0;

  rotm[2][0] = -1 * cos(drollrd) * sin(dpitchrd);
  rotm[2][1] = sin(drollrd);
  rotm[2][2] = cos(drollrd) * cos(dpitchrd);
  rotm[2][3] = 9.5; // Baseline Z height (cm)

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

  // Dispatch targets in ticks
  ml.setTargetPosition(z_target_ml * ML_CM_TO_TICKS);
  mr.setTargetPosition(z_target_mr * MR_CM_TO_TICKS);
  rc.setTargetPosition(z_target_rc * RC_CM_TO_TICKS);

  // Hold carriages steady
  // TODO: Convert encoder ticks to ticks/cm
  ml_carriage.setTargetPosition(12000);
  mr_carriage.setTargetPosition(12000);

  // FC is hardcoded to top of range
  fc.setTargetPosition(FC_MAX_TICKS);

  // Store debug data for telemetry — leveling[]: [pitch_err, roll_err, z_ml, z_rc, z_mr]
  telemetry.leveling[0] = err_y;
  telemetry.leveling[1] = err_x;
  telemetry.leveling[2] = z_target_ml;
  telemetry.leveling[3] = z_target_rc;
  telemetry.leveling[4] = z_target_mr;
}

// Save all 6 motor configs (PID, dirs, limits, current position) to EEPROM
void saveAllMotorConfigs() {
  Motor *all_motors[6] = {&rc, &fc, &ml, &mr, &ml_carriage, &mr_carriage};
  for (int i = 0; i < 6; i++) {
    Motor *m = all_motors[i];
    MotorConfig conf = ConfigStorage::loadMotorConfig(i + 1);
    conf.motor_dir         = m->getDirection();
    conf.encoder_dir       = m->getEncoderDirection();
    conf.lpf_input_alpha   = m->lpf_input_alpha;
    conf.pos_p             = m->pos_pid.kp;
    conf.pos_i             = m->pos_pid.ki;
    conf.pos_d             = m->pos_pid.kd;
    conf.pos_ff            = m->pos_pid.kff;
    conf.pos_lpf_alpha     = m->pos_pid.getLpfAlpha();
    conf.pos_max_ramp_rate = m->pos_pid.max_ramp_rate;
    conf.vel_p             = m->vel_pid.kp;
    conf.vel_i             = m->vel_pid.ki;
    conf.vel_d             = m->vel_pid.kd;
    conf.vel_ff            = m->vel_pid.kff;
    conf.vel_lpf_alpha     = m->vel_pid.getLpfAlpha();
    conf.vel_max_ramp_rate = m->vel_pid.max_ramp_rate;
    conf.saved_position    = m->current_pos;
    conf.pos_limit_min     = m->pos_limit_min;
    conf.pos_limit_max     = m->pos_limit_max;
    ConfigStorage::saveMotorConfig(i + 1, conf);
  }
}

// Save a single motor's config (PID, dirs, limits, current position) to EEPROM
void saveMotorConfig(int motor_id, Motor *m) {
  MotorConfig conf = ConfigStorage::loadMotorConfig(motor_id);
  conf.motor_dir         = m->getDirection();
  conf.encoder_dir       = m->getEncoderDirection();
  conf.lpf_input_alpha   = m->lpf_input_alpha;
  conf.pos_p             = m->pos_pid.kp;
  conf.pos_i             = m->pos_pid.ki;
  conf.pos_d             = m->pos_pid.kd;
  conf.pos_ff            = m->pos_pid.kff;
  conf.pos_lpf_alpha     = m->pos_pid.getLpfAlpha();
  conf.pos_max_ramp_rate = m->pos_pid.max_ramp_rate;
  conf.vel_p             = m->vel_pid.kp;
  conf.vel_i             = m->vel_pid.ki;
  conf.vel_d             = m->vel_pid.kd;
  conf.vel_ff            = m->vel_pid.kff;
  conf.vel_lpf_alpha     = m->vel_pid.getLpfAlpha();
  conf.vel_max_ramp_rate = m->vel_pid.max_ramp_rate;
  conf.saved_position    = m->current_pos;
  conf.pos_limit_min     = m->pos_limit_min;
  conf.pos_limit_max     = m->pos_limit_max;
  ConfigStorage::saveMotorConfig(motor_id, conf);
}

// Parse "t1,t2,t3,t4,t5,t6,a1,a2,a3,a4,a5,a6,dur_ms" into a Keyframe.
// Returns true on success.
bool parseKeyframePayload(const String &payload, Keyframe &kf) {
  float vals[13];
  int count = 0;
  int start = 0;
  for (int i = 0; i <= (int)payload.length() && count < 13; i++) {
    char c = (i < (int)payload.length()) ? payload.charAt(i) : ',';
    if (c == ',') {
      vals[count++] = payload.substring(start, i).toFloat();
      start = i + 1;
    }
  }
  if (count < 13) return false;
  for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
    kf.targets[i] = vals[i];
    kf.active[i]  = (vals[6 + i] > 0.5f);
  }
  kf.duration_ms = (uint32_t)vals[12];
  return true;
}

void setup() {
  Serial.begin(460800);  // jetson
  Serial3.begin(460800); // roboclaw 1
  Serial4.begin(460800); // roboclaw 2
  Serial5.begin(460800); // roboclaw 3

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

  Motor *all_motors[6] = {&rc, &fc, &ml, &mr, &ml_carriage, &mr_carriage};
  for (int i = 0; i < 6; i++) {
    MotorConfig conf = ConfigStorage::loadMotorConfig(i + 1);
    all_motors[i]->setDirection(conf.motor_dir);
    all_motors[i]->setEncoderDirection(conf.encoder_dir);
    all_motors[i]->setInputLpfAlpha(conf.lpf_input_alpha);
    // Load PID values into PIDController objects
    all_motors[i]->pos_pid.kp = conf.pos_p;
    all_motors[i]->pos_pid.ki = conf.pos_i;
    all_motors[i]->pos_pid.kd = conf.pos_d;
    all_motors[i]->pos_pid.setFeedForward(conf.pos_ff);
    all_motors[i]->pos_pid.setLpfAlpha(conf.pos_lpf_alpha);
    all_motors[i]->pos_pid.setRampRate(conf.pos_max_ramp_rate);

    all_motors[i]->vel_pid.kp = conf.vel_p;
    all_motors[i]->vel_pid.ki = conf.vel_i;
    all_motors[i]->vel_pid.kd = conf.vel_d;
    all_motors[i]->vel_pid.setFeedForward(conf.vel_ff);
    all_motors[i]->vel_pid.setLpfAlpha(conf.vel_lpf_alpha);
    all_motors[i]->vel_pid.setRampRate(conf.vel_max_ramp_rate);

    all_motors[i]->updateLimits(conf.pos_limit_min, conf.pos_limit_max);

    // Restore encoder offset so the filtered position resumes from
    // saved_position. Map motor index (0-5) to encoder container index,
    // matching updateSensorData().
    int enc_idx = 0;
    switch (i) {
    case 0:
      enc_idx = 3;
      break; // rc  -> encoderf[3]
    case 1:
      enc_idx = 2;
      break; // fc  -> encoderf[2]
    case 2:
      enc_idx = 7;
      break; // ml  -> encoderf[7]
    case 3:
      enc_idx = 5;
      break; // mr  -> encoderf[5]
    case 4:
      enc_idx = 11;
      break; // ml_carriage -> encoderf[11]
    case 5:
      enc_idx = 12;
      break; // mr_carriage -> encoderf[12]
    }

    // saved_position is the logical position (after encoder_dir flip).
    // Divide by encoder_dir to recover the raw tick count, then set the
    // offset so that (raw_reading - offset) == saved_position.
    // Guard: encoder_dir is validated to ±1 by loadMotorConfig; check anyway.
    if (conf.encoder_dir != 0) {
      EContr.encoder_offset[enc_idx] =
          EContr.getRawReading(enc_idx) -
          (signed long)(conf.saved_position / (float)conf.encoder_dir);
    }
  }

  Serial.println("EEPROM CONFIG LOADED: All motor configs restored from EEPROM.");
  current_state = IDLE;
}

void loop() {
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

  // TODO @alex : verify map encoders to motor positions (I took a guess, but
  // I'm unsure)
  rc.updateSensorData(EContr.encoderf[3], dt);
  fc.updateSensorData(EContr.encoderf[2], dt);
  ml.updateSensorData(EContr.encoderf[7], dt);
  mr.updateSensorData(EContr.encoderf[5], dt);
  ml_carriage.updateSensorData(EContr.encoderf[11], dt);
  mr_carriage.updateSensorData(EContr.encoderf[12], dt);

  // 2. Parse Comms
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
    current_state = IDLE;
    parser.feedWatchdog();
    was_connected = true; // Re-arm auto-save for next disconnect
    if (DEBUG_MODE)
      Serial.println("DEBUG: ESTOP Cleared, entering IDLE");
  } else if (cmd.type == CMD_SEQ_MODE) {
    if (cmd.value > 0.5f) {
      current_state = AUTO_CURB_CLIMBING;
      seq_length = 0;
      seq_current = -1;
      seq_interpolating = false;
      Motor *motors[SEQ_NUM_MOTORS] = {&rc, &fc, &ml, &mr, &ml_carriage, &mr_carriage};
      for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
        motors[i]->setMode(Motor::POSITION_CONTROL);
        seq_start_pos[i] = motors[i]->current_pos;
      }
      Serial.println("SEQ: Entered AUTO_CURB_CLIMBING mode");
    } else {
      current_state = IDLE;
      Serial.println("SEQ: Exited AUTO_CURB_CLIMBING mode");
    }
  } else if (cmd.type == CMD_LEVEL_MODE) {
    if (cmd.value > 0.5) {
      current_state = SELF_LEVELING;
      if (DEBUG_MODE)
        Serial.println("DEBUG: Entering SELF_LEVELING mode");
    } else {
      current_state = IDLE; // Fall back to IDLE so next cmd kicks to TUNER_MODE
      if (DEBUG_MODE)
        Serial.println("DEBUG: Exiting SELF_LEVELING mode");
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
  }

  // Process specific tuning commands if in TUNER_MODE
  if (current_state == TUNER_MODE && cmd.type != CMD_NONE) {
    // Special case: Save all motors (K0)
    if (cmd.type == CMD_SAVE_CONFIG && cmd.actuator_id == 0) {
      saveAllMotorConfigs();
      if (DEBUG_MODE) {
        Serial.println("DEBUG: Saved config for ALL motors (K0)");
      }
    } else {
      Motor *m = nullptr;
      switch (cmd.actuator_id - 1) {
      case 0:
        m = &rc;
        break;
      case 1:
        m = &fc;
        break;
      case 2:
        m = &ml;
        break;
      case 3:
        m = &mr;
        break;
      case 4:
        m = &ml_carriage;
        break;
      case 5:
        m = &mr_carriage;
        break;
      }

      if (m != nullptr) {
        switch (cmd.type) {
        case CMD_M:
          if (cmd.value == 0)
            m->setMode(Motor::OPEN_LOOP);
          else if (cmd.value == 1)
            m->setMode(Motor::VELOCITY_CONTROL);
          else if (cmd.value == 2)
            m->setMode(Motor::POSITION_CONTROL);
          if (DEBUG_MODE) {
            Serial.print("DEBUG: Set Mode to ");
            Serial.println(cmd.value);
          }
          break;
        case CMD_T:
          if (m->mode == Motor::OPEN_LOOP)
            m->setTargetPWM(cmd.value);
          else if (m->mode == Motor::VELOCITY_CONTROL)
            m->setTargetVelocity(cmd.value);
          else if (m->mode == Motor::POSITION_CONTROL)
            m->setTargetPosition(cmd.value);
          if (DEBUG_MODE) {
            Serial.print("DEBUG: Set Target to ");
            Serial.println(cmd.value, 4);
          }
          break;
        case CMD_POS_P:
          m->pos_pid.kp = cmd.value;
          saveMotorConfig(cmd.actuator_id, m);
          if (DEBUG_MODE)
            Serial.println("DEBUG: Set Pos P");
          break;
        case CMD_POS_I:
          m->pos_pid.ki = cmd.value;
          saveMotorConfig(cmd.actuator_id, m);
          if (DEBUG_MODE)
            Serial.println("DEBUG: Set Pos I");
          break;
        case CMD_POS_D:
          m->pos_pid.kd = cmd.value;
          saveMotorConfig(cmd.actuator_id, m);
          if (DEBUG_MODE)
            Serial.println("DEBUG: Set Pos D");
          break;
        case CMD_POS_FF:
          m->pos_pid.setFeedForward(cmd.value);
          saveMotorConfig(cmd.actuator_id, m);
          if (DEBUG_MODE)
            Serial.println("DEBUG: Set Pos FF");
          break;
        case CMD_VEL_P:
          m->vel_pid.kp = cmd.value;
          saveMotorConfig(cmd.actuator_id, m);
          if (DEBUG_MODE)
            Serial.println("DEBUG: Set Vel P");
          break;
        case CMD_VEL_I:
          m->vel_pid.ki = cmd.value;
          saveMotorConfig(cmd.actuator_id, m);
          if (DEBUG_MODE)
            Serial.println("DEBUG: Set Vel I");
          break;
        case CMD_VEL_D:
          m->vel_pid.kd = cmd.value;
          saveMotorConfig(cmd.actuator_id, m);
          if (DEBUG_MODE)
            Serial.println("DEBUG: Set Vel D");
          break;
        case CMD_VEL_FF:
          m->vel_pid.setFeedForward(cmd.value / 10000);
          saveMotorConfig(cmd.actuator_id, m);
          if (DEBUG_MODE)
            Serial.println("DEBUG: Set Vel FF");
          break;
        case CMD_INPUT_LPF:
          m->setInputLpfAlpha(cmd.value);
          saveMotorConfig(cmd.actuator_id, m);
          if (DEBUG_MODE)
            Serial.println("DEBUG: Set Input LPF");
          break;
        case CMD_POS_LPF:
          m->pos_pid.setLpfAlpha(cmd.value);
          saveMotorConfig(cmd.actuator_id, m);
          if (DEBUG_MODE)
            Serial.println("DEBUG: Set Pos LPF");
          break;
        case CMD_VEL_LPF:
          m->vel_pid.setLpfAlpha(cmd.value);
          saveMotorConfig(cmd.actuator_id, m);
          if (DEBUG_MODE)
            Serial.println("DEBUG: Set Vel LPF");
          break;
        case CMD_POS_RAMP:
          m->pos_pid.setRampRate(cmd.value);
          saveMotorConfig(cmd.actuator_id, m);
          if (DEBUG_MODE)
            Serial.println("DEBUG: Set Pos max ramp rate");
          break;
        case CMD_VEL_RAMP:
          m->vel_pid.setRampRate(cmd.value);
          saveMotorConfig(cmd.actuator_id, m);
          if (DEBUG_MODE)
            Serial.println("DEBUG: Set Vel max ramp rate");
          break;
        case CMD_R:
          m->pos_pid.reset();
          m->vel_pid.reset();
          if (DEBUG_MODE)
            Serial.println("DEBUG: Reset PID state (cleared integrator)");
          break;
        case CMD_HOME: {
          // Zero encoder for this joint
          // Map joint ID to encoder index based on updateSensorData mapping
          int enc_idx = 0;
          switch (cmd.actuator_id) {
          case 1:
            enc_idx = 3;
            break; // rc -> encoderf[3]
          case 2:
            enc_idx = 2;
            break; // fc -> encoderf[2]
          case 3:
            enc_idx = 7;
            break; // ml -> encoderf[7]
          case 4:
            enc_idx = 5;
            break; // mr -> encoderf[5]
          case 5:
            enc_idx = 11;
            break; // ml_carriage -> encoderf[11]
          case 6:
            enc_idx = 12;
            break; // mr_carriage -> encoderf[12]
          }
          EContr.zeroEncoder(enc_idx);
          m->pos_pid.reset();
          m->vel_pid.reset();
          m->target_pos = 0; // Set target to new zero
          if (DEBUG_MODE) {
            Serial.print("DEBUG: Homed encoder for joint ");
            Serial.println(cmd.actuator_id);
          }
          break;
        }
        case CMD_OFFSET: {
          int enc_idx = 0;
          switch (cmd.actuator_id) {
          case 1:
            enc_idx = 3;
            break;
          case 2:
            enc_idx = 2;
            break;
          case 3:
            enc_idx = 7;
            break;
          case 4:
            enc_idx = 5;
            break;
          case 5:
            enc_idx = 11;
            break;
          case 6:
            enc_idx = 12;
            break;
          }

          if (enc_idx > 0) {
            float raw_pos = (float)EContr.getRawReading(enc_idx);
            // Apply motor encoder direction to the raw reading logic?
            // The GUI targets "logical position".
            // In updateSensorData: raw_pos = current_pos * encoder_dir
            // We want: current_logical = cmd.value
            // (raw_pos - new_offset) * encoder_dir = cmd.value
            // raw_pos - new_offset = cmd.value / encoder_dir
            // new_offset = raw_pos - (cmd.value / encoder_dir)
            float encoder_dir = m->getEncoderDirection();
            signed long new_offset =
                (signed long)(raw_pos - (cmd.value / encoder_dir));

            EContr.setOffset(enc_idx, new_offset);

            m->pos_pid.reset();
            m->vel_pid.reset();
            m->target_pos = cmd.value;
            m->current_pos = cmd.value; // prevent jump
            m->prev_pos = cmd.value;    // prevent velocity jump

            if (DEBUG_MODE) {
              Serial.print("DEBUG: Set offset J");
              Serial.print(cmd.actuator_id);
              Serial.print(": new logical pos=");
              Serial.println(cmd.value);
            }

            // Auto-save
            ConfigStorage::save_position(cmd.actuator_id, cmd.value);
          }
          break;
        }
        case CMD_DIR: {
          // Toggle motor direction and save to EEPROM
          m->toggleDirection();
          MotorConfig conf = ConfigStorage::loadMotorConfig(cmd.actuator_id);
          conf.motor_dir = m->getDirection();
          ConfigStorage::saveMotorConfig(cmd.actuator_id, conf);
          if (DEBUG_MODE) {
            Serial.print("DEBUG: Toggled direction for motor ");
            Serial.print(cmd.actuator_id);
            Serial.print(" to ");
            Serial.println(m->getDirection());
          }
          break;
        }
        case CMD_ENC_DIR: {
          // Toggle encoder direction and save to EEPROM
          m->toggleEncoderDirection();
          MotorConfig conf = ConfigStorage::loadMotorConfig(cmd.actuator_id);
          conf.encoder_dir = m->getEncoderDirection();
          ConfigStorage::saveMotorConfig(cmd.actuator_id, conf);
          if (DEBUG_MODE) {
            Serial.print("DEBUG: Toggled enc direction for motor ");
            Serial.print(cmd.actuator_id);
            Serial.print(" to ");
            Serial.println(m->getEncoderDirection());
          }
          break;
        }
        case CMD_SAVE_CONFIG: {
          saveMotorConfig(cmd.actuator_id, m);
          if (DEBUG_MODE) {
            Serial.print("DEBUG: Saved config for motor ");
            Serial.println(cmd.actuator_id);
          }
          break;
        }
        case CMD_POS_MIN: {
          m->updateLimits(cmd.value, m->pos_limit_max);
          saveMotorConfig(cmd.actuator_id, m);
          if (DEBUG_MODE) {
            Serial.print("DEBUG: Set min limit to ");
            Serial.println(cmd.value);
          }
          break;
        }
        case CMD_POS_MAX: {
          m->updateLimits(m->pos_limit_min, cmd.value);
          saveMotorConfig(cmd.actuator_id, m);
          if (DEBUG_MODE) {
            Serial.print("DEBUG: Set max limit to ");
            Serial.println(cmd.value);
          }
          break;
        }
        case CMD_GET_CONFIG: {
          // Print config back to GUI
          Serial.print("CONFIG,");
          Serial.print(cmd.actuator_id);
          Serial.print(",");
          Serial.print(m->pos_pid.kp, 4);
          Serial.print(",");
          Serial.print(m->pos_pid.ki, 4);
          Serial.print(",");
          Serial.print(m->pos_pid.kd, 4);
          Serial.print(",");
          Serial.print(m->pos_pid.kff, 4);
          Serial.print(",");
          Serial.print(m->vel_pid.kp, 4);
          Serial.print(",");
          Serial.print(m->vel_pid.ki, 4);
          Serial.print(",");
          Serial.print(m->vel_pid.kd, 4);
          Serial.print(",");
          Serial.print(m->vel_pid.kff, 4);
          Serial.print(",");
          Serial.print(m->pos_pid.getLpfAlpha(), 4);
          Serial.print(",");
          Serial.print(m->vel_pid.getLpfAlpha(), 4);
          Serial.print(",");
          Serial.print(m->lpf_input_alpha, 4);
          Serial.print(",");
          Serial.print(m->pos_limit_min);
          Serial.print(",");
          Serial.print(m->pos_limit_max);
          Serial.print(",");
          Serial.print(m->pos_pid.max_ramp_rate, 4);
          Serial.print(",");
          Serial.println(m->vel_pid.max_ramp_rate, 4);
          break;
        }
        default:
          break;
        }
      }
    }
  }

  // Sequence command dispatch (valid in AUTO_CURB_CLIMBING mode)
  if (current_state == AUTO_CURB_CLIMBING && cmd.type != CMD_NONE) {
    if (cmd.type == CMD_SEQ_KEYFRAME) {
      int idx = cmd.actuator_id;
      if (idx >= 0 && idx < MAX_SEQ_KEYFRAMES) {
        Keyframe kf;
        if (parseKeyframePayload(parser.last_payload, kf)) {
          seq_keyframes[idx] = kf;
          if (idx >= seq_length) seq_length = idx + 1;
          Serial.print("SEQ_ACK,");
          Serial.println(idx);
        } else {
          Serial.print("SEQ_ERR,bad_payload,");
          Serial.println(idx);
        }
      }
    } else if (cmd.type == CMD_SEQ_STEP_FWD) {
      if (!seq_interpolating && seq_current < seq_length - 1) {
        seq_current++;
        Motor *motors[SEQ_NUM_MOTORS] = {&rc, &fc, &ml, &mr, &ml_carriage, &mr_carriage};
        for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
          seq_start_pos[i] = motors[i]->current_pos;
        }
        seq_interp_start = millis();
        seq_interpolating = true;
        Serial.print("SEQ_STATUS,"); Serial.print(seq_current);
        Serial.print(","); Serial.print(seq_length);
        Serial.println(",1");
      }
    } else if (cmd.type == CMD_SEQ_STEP_BWD) {
      if (!seq_interpolating && seq_current > 0) {
        seq_current--;
        Motor *motors[SEQ_NUM_MOTORS] = {&rc, &fc, &ml, &mr, &ml_carriage, &mr_carriage};
        for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
          seq_start_pos[i] = motors[i]->current_pos;
        }
        seq_interp_start = millis();
        seq_interpolating = true;
        Serial.print("SEQ_STATUS,"); Serial.print(seq_current);
        Serial.print(","); Serial.print(seq_length);
        Serial.println(",1");
      }
    } else if (cmd.type == CMD_SEQ_MODE && cmd.value <= 0.5f) {
      // handled above in state transitions
    }
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
  } else if (current_state == SELF_LEVELING) {
    runSelfLeveling(dt);
  } else if (current_state == AUTO_CURB_CLIMBING) {
    if (seq_interpolating && seq_current >= 0 && seq_current < seq_length) {
      Keyframe &kf = seq_keyframes[seq_current];
      unsigned long elapsed = millis() - seq_interp_start;
      float t = (kf.duration_ms == 0)
                  ? 1.0f
                  : min(1.0f, (float)elapsed / (float)kf.duration_ms);
      Motor *motors[SEQ_NUM_MOTORS] = {&rc, &fc, &ml, &mr, &ml_carriage, &mr_carriage};
      for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
        if (kf.active[i]) {
          float target = seq_start_pos[i] + t * (kf.targets[i] - seq_start_pos[i]);
          motors[i]->setTargetPosition(target);
        }
      }
      if (t >= 1.0f) {
        seq_interpolating = false;
        Serial.print("SEQ_STATUS,"); Serial.print(seq_current);
        Serial.print(","); Serial.print(seq_length);
        Serial.println(",0");
      }
    }
  }

  float rc_pwm = rc.update(dt);
  float fc_pwm = fc.update(dt);
  float ml_pwm = ml.update(dt);
  float mr_pwm = mr.update(dt);
  float mlc_pwm = ml_carriage.update(dt);
  float mrc_pwm = mr_carriage.update(dt);

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

  // Write PWM to RoboClaws (constrained strictly to 16-bit signed int +/-
  // 32767) roboclaw_main: M1 = Main Left, M2 = Main Right
  // TODO: change main wheel controls back to default
  roboclaw_main.DutyM1(0x80, (int16_t)ml_pwm);

  roboclaw_main.DutyM2(0x80, (int16_t)mr_pwm);

  // roboclaw_casters: M1 = Rear Caster, M2 = Front Caster
  roboclaw_casters.DutyM1(0x80, (int16_t)rc_pwm);
  roboclaw_casters.DutyM2(0x80, (int16_t)fc_pwm);

  roboclaw_carriages.DutyM1(0x80, (int16_t)mlc_pwm);

  roboclaw_carriages.DutyM2(0x80, (int16_t)mrc_pwm);

  // 5. Send Telemetry
  updateTelemetry();

  static unsigned long last_telem_time = 0;
  if (millis() - last_telem_time >= 100) { // Fixed 10Hz telemetry
    last_telem_time = millis();
    sendTelemetry();
  }

  // TODO: Stabilize timing loop
  delayMicroseconds(5000);
}
