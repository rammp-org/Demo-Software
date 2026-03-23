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
struct SystemTelemetry {
  SystemState state;
  float rc_pos, fc_pos;
  float ml_pos, mr_pos;
  float ml_carriage_pos, mr_carriage_pos;
  float rc_vel, fc_vel;
  float ml_vel, mr_vel;
  float ml_carriage_vel, mr_carriage_vel;
  float imu_pitch, imu_roll, imu_yaw;
  float leveling_pitch_err;
  float leveling_roll_err;
  float z_target_ml;
  float z_target_rc;
  float z_target_mr;
  // Strain gauge (load cell) readings — filtered ADC counts
  float sg_rc_value;
  float sg_fc_value;
  float sg_ml_value;
  float sg_mr_value;
};

// Global State
SystemState current_state = INIT;
SystemTelemetry telemetry;

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
StrainGauge sg_rc(RC_LOADCELL_PIN);
StrainGauge sg_fc(FC_LOADCELL_PIN);
StrainGauge sg_ml(ML_LOADCELL_PIN);
StrainGauge sg_mr(MR_LOADCELL_PIN);

int16_t scaled_mlc_pwm;
int16_t scaled_mrc_pwm;

int16_t scaled_ml_pwm;
int16_t scaled_mr_pwm;

// Helper to update telemetry
void updateTelemetry() {
  telemetry.state = current_state;
  telemetry.rc_pos = rc.current_pos;
  telemetry.fc_pos = fc.current_pos;
  telemetry.ml_pos = ml.current_pos;
  telemetry.mr_pos = mr.current_pos;
  telemetry.ml_carriage_pos = ml_carriage.current_pos;
  telemetry.mr_carriage_pos = mr_carriage.current_pos;

  telemetry.rc_vel = rc.current_vel;
  telemetry.fc_vel = fc.current_vel;
  telemetry.ml_vel = ml.current_vel;
  telemetry.mr_vel = mr.current_vel;
  telemetry.ml_carriage_vel = ml_carriage.current_vel;
  telemetry.mr_carriage_vel = mr_carriage.current_vel;

  // Strain gauges
  telemetry.sg_rc_value = sg_rc.getValue();
  telemetry.sg_fc_value = sg_fc.getValue();
  telemetry.sg_ml_value = sg_ml.getValue();
  telemetry.sg_mr_value = sg_mr.getValue();
}

// Helper to send telemetry
// Extended format: 44 values (18 original + 6 dirs + 4 limits + 6 imu + 4 quat)
void sendTelemetry() {
  Serial.print("TELEMETRY,");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(telemetry.state);
  Serial.print(",");
  // Positions (6)
  Serial.print(telemetry.rc_pos);
  Serial.print(",");
  Serial.print(telemetry.fc_pos);
  Serial.print(",");
  Serial.print(telemetry.ml_pos);
  Serial.print(",");
  Serial.print(telemetry.mr_pos);
  Serial.print(",");
  Serial.print(telemetry.ml_carriage_pos);
  Serial.print(",");
  Serial.print(telemetry.mr_carriage_pos);
  Serial.print(",");
  // Velocities (6)
  Serial.print(telemetry.rc_vel);
  Serial.print(",");
  Serial.print(telemetry.fc_vel);
  Serial.print(",");
  Serial.print(telemetry.ml_vel);
  Serial.print(",");
  Serial.print(telemetry.mr_vel);
  Serial.print(",");
  Serial.print(telemetry.ml_carriage_vel);
  Serial.print(",");
  Serial.print(telemetry.mr_carriage_vel);
  Serial.print(",");
  // PWM Targets (6)
  Serial.print(rc.target_pwm);
  Serial.print(",");
  Serial.print(fc.target_pwm);
  Serial.print(",");
  Serial.print(ml.target_pwm);
  Serial.print(",");
  Serial.print(mr.target_pwm);
  Serial.print(",");
  Serial.print(ml_carriage.target_pwm);
  Serial.print(",");
  Serial.print(mr_carriage.target_pwm);
  Serial.print(",");
  // Motor directions (6)
  Serial.print(rc.getDirection());
  Serial.print(",");
  Serial.print(fc.getDirection());
  Serial.print(",");
  Serial.print(ml.getDirection());
  Serial.print(",");
  Serial.print(mr.getDirection());
  Serial.print(",");
  Serial.print(ml_carriage.getDirection());
  Serial.print(",");
  Serial.print(mr_carriage.getDirection());
  Serial.print(",");
  // Encoder directions (6)
  Serial.print(rc.getEncoderDirection());
  Serial.print(",");
  Serial.print(fc.getEncoderDirection());
  Serial.print(",");
  Serial.print(ml.getEncoderDirection());
  Serial.print(",");
  Serial.print(mr.getEncoderDirection());
  Serial.print(",");
  Serial.print(ml_carriage.getEncoderDirection());
  Serial.print(",");
  Serial.print(mr_carriage.getEncoderDirection());
  Serial.print(",");
  // Limit switches (4)
  Serial.print(ml_fwd_limit ? 1 : 0);
  Serial.print(",");
  Serial.print(ml_bwd_limit ? 1 : 0);
  Serial.print(",");
  Serial.print(mr_fwd_limit ? 1 : 0);
  Serial.print(",");
  Serial.print(mr_bwd_limit ? 1 : 0);
  Serial.print(",");
  // IMU data (6)
  Serial.print(IMU.pitchf, 2);
  Serial.print(",");
  Serial.print(IMU.rollf, 2);
  Serial.print(",");
  Serial.print(IMU.yaw, 2);
  Serial.print(",");
  Serial.print(IMU.ax, 3);
  Serial.print(",");
  Serial.print(IMU.ay, 3);
  Serial.print(",");
  Serial.print(IMU.az, 3);
  Serial.print(",");
  // Raw Quaternion (4)
  Serial.print(IMU.current_quat.w(), 4);
  Serial.print(",");
  Serial.print(IMU.current_quat.x(), 4);
  Serial.print(",");
  Serial.print(IMU.current_quat.y(), 4);
  Serial.print(",");
  Serial.print(IMU.current_quat.z(), 4);
  Serial.print(",");
  // Leveling Debug (5)
  Serial.print(telemetry.leveling_pitch_err, 4);
  Serial.print(",");
  Serial.print(telemetry.leveling_roll_err, 4);
  Serial.print(",");
  Serial.print(telemetry.z_target_ml, 4);
  Serial.print(",");
  Serial.print(telemetry.z_target_rc, 4);
  Serial.print(",");
  Serial.print(telemetry.z_target_mr, 4);
  Serial.print(",");
  // Strain gauges (4)
  Serial.print(telemetry.sg_rc_value, 2);
  Serial.print(",");
  Serial.print(telemetry.sg_fc_value, 2);
  Serial.print(",");
  Serial.print(telemetry.sg_ml_value, 2);
  Serial.print(",");
  Serial.println(telemetry.sg_mr_value, 2);
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

  // Construct target quaternion manually to avoid API differences in fromAxisAngle
  // Note: BNO055 defines Pitch as rotation around X, Roll as rotation around Y.
  // Because the IMU is mounted upside down, Roll is physically offset by 180 deg.
  double p_rad = (target_pitch * PI / 180.0) / 2.0;
  imu::Quaternion q_target_pitch(cos(p_rad), sin(p_rad), 0.0, 0.0);
  
  double r_rad = ((target_roll - 180.0) * PI / 180.0) / 2.0;
  imu::Quaternion q_target_roll(cos(r_rad), 0.0, sin(r_rad), 0.0);
  
  // Target orientation
  imu::Quaternion q_target = q_target_pitch * q_target_roll;

  // Calculate the rotation required to go from current to target
  imu::Quaternion q_err = q_target * q_meas.conjugate();

  // Extract the Pitch and Roll error directly from the Error Quaternion
  // Because the error is small, this will never hit gimbal lock or +/- 180 boundaries!
  double sinr_cosp = 2.0 * (q_err.w() * q_err.x() + q_err.y() * q_err.z());
  double cosr_cosp = 1.0 - 2.0 * (q_err.x() * q_err.x() + q_err.y() * q_err.y());
  double err_x = atan2(sinr_cosp, cosr_cosp) * (180.0 / PI); // Roll error mapped to X

  double sinp = 2.0 * (q_err.w() * q_err.y() - q_err.z() * q_err.x());
  double err_y;
  if (abs(sinp) >= 1)
    err_y = copysign(M_PI / 2, sinp) * (180.0 / PI);
  else
    err_y = asin(sinp) * (180.0 / PI); // Pitch error mapped to Y

  // Convert exact, continuous error angles to radians
  float dpitchrd = 1.0f * (err_x / DG); // BNO X = Robot Pitch
  float drollrd  = 1.0f * (err_y / DG); // BNO Y = Robot Roll

  // Deadband to prevent jitter
  if (fabs(dpitchrd) < 0.001) dpitchrd = 0.0;
  if (fabs(drollrd) < 0.001) drollrd = 0.0;

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
  float z_target_rc = (newmebot[2][1] + newmebot[2][2]) / 2.0; // Average left/right caster height
  float z_target_mr = newmebot[2][3];

  // Dispatch targets in ticks
  ml.setTargetPosition(z_target_ml * ML_CM_TO_TICKS);
  mr.setTargetPosition(z_target_mr * MR_CM_TO_TICKS);
  rc.setTargetPosition(z_target_rc * RC_CM_TO_TICKS);
  
  // Hold carriages steady
  ml_carriage.setTargetPosition(0.1f * CARRIAGE_CM_TO_TICKS);
  mr_carriage.setTargetPosition(0.1f * CARRIAGE_CM_TO_TICKS);
  
  // FC is hardcoded to top of range
  fc.setTargetPosition(FC_MAX_TICKS);
  
  // Store debug data for telemetry
  telemetry.leveling_pitch_err = err_y;
  telemetry.leveling_roll_err = err_x;
  telemetry.z_target_ml = z_target_ml;
  telemetry.z_target_rc = z_target_rc;
  telemetry.z_target_mr = z_target_mr;
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
  
  Motor* all_motors[6] = {&rc, &fc, &ml, &mr, &ml_carriage, &mr_carriage};
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

    // Restore encoder offset so the filtered position resumes from saved_position.
    // Map motor index (0-5) to encoder container index, matching updateSensorData().
    int enc_idx = 0;
    switch (i) {
      case 0: enc_idx = 3; break;  // rc  -> encoderf[3]
      case 1: enc_idx = 2; break;  // fc  -> encoderf[2]
      case 2: enc_idx = 7; break;  // ml  -> encoderf[7]
      case 3: enc_idx = 5; break;  // mr  -> encoderf[5]
      case 4: enc_idx = 11; break; // ml_carriage -> encoderf[11]
      case 5: enc_idx = 12; break; // mr_carriage -> encoderf[12]
    }

    // saved_position is the logical position (after encoder_dir flip).
    // Divide by encoder_dir to recover the raw tick count, then set the
    // offset so that (raw_reading - offset) == saved_position.
    // Guard: encoder_dir is validated to ±1 by loadMotorConfig; check anyway.
    if (conf.encoder_dir != 0) {
      EContr.encoder_offset[enc_idx] = EContr.getRawReading(enc_idx) -
          (signed long)(conf.saved_position / (float)conf.encoder_dir);
    }
  }

  current_state = IDLE;
}

void loop() {
  timer.updateTime();
  float dt = timer.elapsed_time;

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

  // 3. Update State Machine
  if (parser.isTimedOut() && current_state != ESTOP) {
    current_state = ESTOP;
    Serial.println("WATCHDOG TIMEOUT -> ESTOP");
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
    if (DEBUG_MODE)
      Serial.println("DEBUG: ESTOP Cleared, entering IDLE");
  } else if (cmd.type == CMD_LEVEL_MODE) {
    if (cmd.value > 0.5) {
      current_state = SELF_LEVELING;
      if (DEBUG_MODE) Serial.println("DEBUG: Entering SELF_LEVELING mode");
    } else {
      current_state = IDLE; // Fall back to IDLE so next cmd kicks to TUNER_MODE
      if (DEBUG_MODE) Serial.println("DEBUG: Exiting SELF_LEVELING mode");
    }
  } else if (cmd.type == CMD_LEVEL_PITCH) {
    target_pitch = cmd.value;
    if (DEBUG_MODE) Serial.print("DEBUG: Set target pitch: "); Serial.println(target_pitch);
  } else if (cmd.type == CMD_LEVEL_ROLL) {
    target_roll = cmd.value;
    if (DEBUG_MODE) Serial.print("DEBUG: Set target roll: "); Serial.println(target_roll);
  } else if (cmd.type != CMD_NONE && current_state == IDLE) {
    current_state = TUNER_MODE;
    if (DEBUG_MODE)
      Serial.println("DEBUG: Entering TUNER_MODE");
  }

  // Process specific tuning commands if in TUNER_MODE
  if (current_state == TUNER_MODE && cmd.type != CMD_NONE) {
    // Special case: Save all motors (K0)
    if (cmd.type == CMD_SAVE_CONFIG && cmd.actuator_id == 0) {
      Motor* all_motors[6] = {&rc, &fc, &ml, &mr, &ml_carriage, &mr_carriage};
      for (int i = 0; i < 6; i++) {
        Motor* m_i = all_motors[i];
        MotorConfig conf = ConfigStorage::loadMotorConfig(i + 1);
        conf.motor_dir = m_i->getDirection();
        conf.encoder_dir = m_i->getEncoderDirection();
        conf.lpf_input_alpha = m_i->lpf_input_alpha;
        conf.pos_p = m_i->pos_pid.kp;
        conf.pos_i = m_i->pos_pid.ki;
        conf.pos_d = m_i->pos_pid.kd;
        conf.pos_ff = m_i->pos_pid.kff;
        conf.pos_lpf_alpha = m_i->pos_pid.getLpfAlpha();
        conf.pos_max_ramp_rate = m_i->pos_pid.max_ramp_rate;
        conf.vel_p = m_i->vel_pid.kp;
        conf.vel_i = m_i->vel_pid.ki;
        conf.vel_d = m_i->vel_pid.kd;
        conf.vel_ff = m_i->vel_pid.kff;
        conf.vel_lpf_alpha = m_i->vel_pid.getLpfAlpha();
        conf.vel_max_ramp_rate = m_i->vel_pid.max_ramp_rate;
        conf.saved_position = m_i->current_pos;
        conf.pos_limit_min = m_i->pos_limit_min;
        conf.pos_limit_max = m_i->pos_limit_max;
        ConfigStorage::saveMotorConfig(i + 1, conf);
      }
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
        if (DEBUG_MODE)
          Serial.println("DEBUG: Set Pos P");
        break;
      case CMD_POS_I:
        m->pos_pid.ki = cmd.value;
        if (DEBUG_MODE)
          Serial.println("DEBUG: Set Pos I");
        break;
      case CMD_POS_D:
        m->pos_pid.kd = cmd.value;
        if (DEBUG_MODE)
          Serial.println("DEBUG: Set Pos D");
        break;
      case CMD_POS_FF:
        m->pos_pid.setFeedForward(cmd.value);
        if (DEBUG_MODE)
          Serial.println("DEBUG: Set Pos FF");
        break;
      case CMD_VEL_P:
        m->vel_pid.kp = cmd.value;
        if (DEBUG_MODE)
          Serial.println("DEBUG: Set Vel P");
        break;
      case CMD_VEL_I:
        m->vel_pid.ki = cmd.value;
        if (DEBUG_MODE)
          Serial.println("DEBUG: Set Vel I");
        break;
      case CMD_VEL_D:
        m->vel_pid.kd = cmd.value;
        if (DEBUG_MODE)
          Serial.println("DEBUG: Set Vel D");
        break;
      case CMD_VEL_FF:
        m->vel_pid.setFeedForward(cmd.value / 10000);
        if (DEBUG_MODE)
          Serial.println("DEBUG: Set Vel FF");
        break;
      case CMD_INPUT_LPF:
        m->setInputLpfAlpha(cmd.value);
        if (DEBUG_MODE)
          Serial.println("DEBUG: Set Input LPF");
        break;
      case CMD_POS_LPF:
        m->pos_pid.setLpfAlpha(cmd.value);
        if (DEBUG_MODE)
          Serial.println("DEBUG: Set Pos LPF");
        break;
      case CMD_VEL_LPF:
        m->vel_pid.setLpfAlpha(cmd.value);
        if (DEBUG_MODE)
          Serial.println("DEBUG: Set Vel LPF");
        break;
      case CMD_POS_RAMP:
        m->pos_pid.setRampRate(cmd.value);
        if (DEBUG_MODE)
          Serial.println("DEBUG: Set Pos max ramp rate");
        break;
      case CMD_VEL_RAMP:
        m->vel_pid.setRampRate(cmd.value);
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
          case 1: enc_idx = 3; break;  // rc -> encoderf[3]
          case 2: enc_idx = 2; break;  // fc -> encoderf[2]
          case 3: enc_idx = 7; break;  // ml -> encoderf[7]
          case 4: enc_idx = 5; break;  // mr -> encoderf[5]
          case 5: enc_idx = 11; break; // ml_carriage -> encoderf[11]
          case 6: enc_idx = 12; break; // mr_carriage -> encoderf[12]
        }
        EContr.zeroEncoder(enc_idx);
        m->pos_pid.reset();
        m->vel_pid.reset();
        m->target_pos = 0;  // Set target to new zero
        if (DEBUG_MODE) {
          Serial.print("DEBUG: Homed encoder for joint ");
          Serial.println(cmd.actuator_id);
        }
        break;
      }
      case CMD_OFFSET: {
        int enc_idx = 0;
        switch (cmd.actuator_id) {
          case 1: enc_idx = 3; break;
          case 2: enc_idx = 2; break;
          case 3: enc_idx = 7; break;
          case 4: enc_idx = 5; break;
          case 5: enc_idx = 11; break;
          case 6: enc_idx = 12; break;
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
          signed long new_offset = (signed long)(raw_pos - (cmd.value / encoder_dir));
          
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
        // Load existing config first so any fields not set here are preserved
        MotorConfig conf = ConfigStorage::loadMotorConfig(cmd.actuator_id);
        conf.motor_dir = m->getDirection();
        conf.encoder_dir = m->getEncoderDirection();
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
        ConfigStorage::saveMotorConfig(cmd.actuator_id, conf);
        if (DEBUG_MODE) {
          Serial.print("DEBUG: Saved config for motor ");
          Serial.println(cmd.actuator_id);
        }
        break;
      }
      case CMD_POS_MIN: {
        m->updateLimits(cmd.value, m->pos_limit_max);
        if (DEBUG_MODE) {
          Serial.print("DEBUG: Set min limit to ");
          Serial.println(cmd.value);
        }
        break;
      }
      case CMD_POS_MAX: {
        m->updateLimits(m->pos_limit_min, cmd.value);
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
        Serial.print(m->pos_pid.kp, 4); Serial.print(",");
        Serial.print(m->pos_pid.ki, 4); Serial.print(",");
        Serial.print(m->pos_pid.kd, 4); Serial.print(",");
        Serial.print(m->pos_pid.kff, 4); Serial.print(",");
        Serial.print(m->vel_pid.kp, 4); Serial.print(",");
        Serial.print(m->vel_pid.ki, 4); Serial.print(",");
        Serial.print(m->vel_pid.kd, 4); Serial.print(",");
        Serial.print(m->vel_pid.kff, 4); Serial.print(",");
        Serial.print(m->pos_pid.getLpfAlpha(), 4); Serial.print(",");
        Serial.print(m->vel_pid.getLpfAlpha(), 4); Serial.print(",");
        Serial.print(m->lpf_input_alpha, 4); Serial.print(",");
        Serial.print(m->pos_limit_min); Serial.print(",");
        Serial.print(m->pos_limit_max); Serial.print(",");
        Serial.print(m->pos_pid.max_ramp_rate, 4); Serial.print(",");
        Serial.println(m->vel_pid.max_ramp_rate, 4);
        break;
      }
      default:
        break;
      }
    }
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
  }

  float rc_pwm = rc.update(dt);
  float fc_pwm = fc.update(dt);
  float ml_pwm = ml.update(dt);
  float mr_pwm = mr.update(dt);
  float mlc_pwm = ml_carriage.update(dt);
  float mrc_pwm = mr_carriage.update(dt);

  // read limit switches (store in globals for telemetry)
  ml_fwd_limit = !digitalRead(CARRIAGE_SW1_PIN);  // Active low
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
  // scaled_ml_pwm = (int16_t)constrain(ml_pwm * PWM_SCALE, -32767, 32767);
  // Serial.print("DEBUG: scaled_ml_pwm = ");
  // Serial.println(scaled_ml_pwm);
  roboclaw_main.DutyM1(0x80, (int16_t)ml_pwm);

  // scaled_mr_pwm = (int16_t)constrain(mr_pwm * PWM_SCALE, -32767, 32767);
  // Serial.print("DEBUG: scaled_mr_pwm = ");
  // Serial.println(scaled_mr_pwm);
  roboclaw_main.DutyM2(0x80, (int16_t)mr_pwm);

  // roboclaw_casters: M1 = Rear Caster, M2 = Front Caster
  roboclaw_casters.DutyM1(0x80, (int16_t)rc_pwm);
  roboclaw_casters.DutyM2(0x80, (int16_t)fc_pwm);

  //   roboclaw_carriages: M1 = Left Carriage, M2 = Right Carriage
  //   Serial.print("DEBUG: mlc_pwm = ");
  //   Serial.print(mlc_pwm);
  //   Serial.print("; mrc_pwm = ");
  //   Serial.println(mrc_pwm);

  // scaled_mlc_pwm = (int16_t)constrain(mlc_pwm * PWM_SCALE, -32767, 32767);
  Serial.print("DEBUG: scaled_mlc_pwm = ");
  Serial.println(mlc_pwm);
  roboclaw_carriages.DutyM1(0x80, (int16_t)mlc_pwm);

  // scaled_mrc_pwm = (int16_t)constrain(mrc_pwm * PWM_SCALE, -32767, 32767);
  Serial.print("DEBUG: scaled_mrc_pwm = ");
  Serial.println(mrc_pwm);
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
