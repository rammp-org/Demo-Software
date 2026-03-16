#include <Arduino.h>
#include "src/Constants/Constants.h"
#include "src/EncoderContainer/EncoderContainer.h"
#include "src/IMU_Class/IMU_Class.h"
#include <SD.h>
#include <SPI.h>
#include "src/Timer/Timer.h"
#include <Wire.h>
#include "src/RoboClaw/RoboClaw.h"
#include <utility/imumaths.h>

#include "src/Motor/Motor.h"
#include "src/CommandParser/CommandParser.h"
#include "src/PIDController/PIDController.h"

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
};

// Global State
SystemState current_state = INIT;
SystemTelemetry telemetry;

// Hardware Objects
Adafruit_BNO055 bno = Adafruit_BNO055(55);
// IMU_Class IMU = IMU_Class(bno);
EncoderContainer EContr;
Timer timer;
CommandParser parser(60000);

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
}

// Helper to send telemetry
// TODO: Eventually all 12 encoders will need to be sent out
void sendTelemetry() {
  Serial.print("TELEMETRY,");
  Serial.print(millis());
  Serial.print(",");
  Serial.print(telemetry.state);
  Serial.print(",");
  // Positions
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
  // Velocities
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
  // PWM Targets
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
  Serial.println();
}

void setup() {
  Serial.begin(115200);  // jetson
  Serial3.begin(460800); // roboclaw 1
  Serial4.begin(460800); // roboclaw 2
  Serial5.begin(460800); // roboclaw 3

  delay(1000);

  current_state = IDLE;
}

void loop() {
  timer.updateTime();
  float dt = timer.elapsed_time;

  // 1. Read Sensors
  EContr.retrieve_readings();

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
  } else if (cmd.type != CMD_NONE && current_state == IDLE) {
    current_state = TUNER_MODE;
    if (DEBUG_MODE)
      Serial.println("DEBUG: Entering TUNER_MODE");
  }

  // Process specific tuning commands if in TUNER_MODE
  if (current_state == TUNER_MODE && cmd.type != CMD_NONE) {
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
      default:
        break;
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
  }

  float rc_pwm = rc.update(dt);
  float fc_pwm = fc.update(dt);
  float ml_pwm = ml.update(dt);
  float mr_pwm = mr.update(dt);
  float mlc_pwm = ml_carriage.update(dt);
  float mrc_pwm = mr_carriage.update(dt);

  // Write PWM to RoboClaws (constrained strictly to 16-bit signed int +/-
  // 32767) roboclaw_main: M1 = Main Left, M2 = Main Right
  // TODO: change main wheel controls back to default
  // scaled_ml_pwm = (int16_t)constrain(ml_pwm * PWM_SCALE, -32767, 32767);
  // Serial.print("DEBUG: scaled_ml_pwm = ");
  // Serial.println(scaled_ml_pwm);
  roboclaw_main.DutyM1(0x80, ml_pwm);

  // scaled_mr_pwm = (int16_t)constrain(mr_pwm * PWM_SCALE, -32767, 32767);
  // Serial.print("DEBUG: scaled_mr_pwm = ");
  // Serial.println(scaled_mr_pwm);
  roboclaw_main.DutyM2(0x80, mr_pwm);

  // roboclaw_casters: M1 = Rear Caster, M2 = Front Caster
  roboclaw_casters.DutyM1(0x80, (int16_t)constrain(rc_pwm, -32767, 32767));
  roboclaw_casters.DutyM2(0x80, (int16_t)constrain(fc_pwm, -32767, 32767));

  // roboclaw_carriages: M1 = Left Carriage, M2 = Right Carriage
  // Serial.print("DEBUG: mlc_pwm = ");
  // Serial.print(mlc_pwm);
  // Serial.print("; mrc_pwm = ");
  // Serial.println(mrc_pwm);

  // scaled_mlc_pwm = (int16_t)constrain(mlc_pwm * PWM_SCALE, -32767, 32767);
  Serial.print("DEBUG: scaled_mlc_pwm = ");
  Serial.println(mlc_pwm);
  roboclaw_carriages.DutyM1(0x80, mlc_pwm);

  // scaled_mrc_pwm = (int16_t)constrain(mrc_pwm * PWM_SCALE, -32767, 32767);
  Serial.print("DEBUG: scaled_mr_pwm = ");
  Serial.println(mrc_pwm);
  roboclaw_carriages.DutyM2(0x80, mrc_pwm);

  // 5. Send Telemetry
  updateTelemetry();

  static unsigned long last_telem_time = 0;
  if (millis() - last_telem_time >= 100) { // Fixed 10Hz telemetry
    last_telem_time = millis();
    sendTelemetry();
  }

  // delay(5);
}
