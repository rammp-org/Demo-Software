/*
  For referesetnces of these header files, please head to the MEBot github
  repository and look those header files inside the open library folder.
  SD.h, utility/imumaths.h and SPI.h can be find online

  Pinout for the MCP23S17, LINK:
  https://ww1.microchip.com/downloads/en/devicedoc/20001952c.pdf Data sheet for
  the Adafruit_BNO055, LINK:
  https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/downloads
  Teensy 4.0 PinoutL, LINK: https://www.pjrc.com/teensy/pinout.html

  Documentation for js.x:
  1. range 0 to 127 will make the robot turn right, higher number means faster
  turning rate

  slow                fast
  |                 |
  [0----right turn----127]

  2. range 128 to 255 will make the robot turn left, lower number means faster
  turning rate fast                slow |                    | [128----left
  turn----255]


  Documentation for js.y:
  1. range 0 to 127 will make the robot go forward, higher number means faster
  speed

  slow                fast
  |                   |
  [0----go forward----127]

  2. range 128 to 255 will make the robot go backward, lower number means faster
  speed

  fast                  slow
  |                     |
  [128----go backward----255]

  packet string format:
  "curb_height + distance_of_interest + angle_initial_turn + sign_initial_turn +
  angle_final_turn + sign_final_turn" 0.00(meter)   0.00(meter) 000.00(degree)
  0(-) / 1(+)         000.00(degree)     0(-)/1(+) 4             4 6 1 6 1 total
  of 22 charactors

  main functions:
  displaydata: Used to print out variables in Serial Monitor.
  manual_features: Four seat functions: Elevation, Tilt Back-Forth, Lateral
  Tilt, Drive wheel configuration advanced_features: Self-leveling, Curb
  Climbing, Curb Descending, Pause, Reset
*/

#include <Carriage.h>
#include <Caster.h>
#include <Component.h>
#include <Constants.h>
#include <EncoderContainer.h>
#include <IMU_Class.h>
#include <JoyStick.h>
#include <MotorController.h>
#include <SD.h>
#include <SPI.h>
#include <Timer.h>
#include <Wheel.h>
#include <Wire.h>
#include <RoboClaw.h>
#include <utility/imumaths.h>
// new line

// Class Definitions (Move these to other files later)
//----------------------------------------------------

// Class Instances
//-----------------------------
Adafruit_BNO055 bno;

IMU_Class IMU = IMU_Class(bno);

EncoderContainer EContr;

Caster RC = Caster(RC_LOADCELL_PIN, MOTOR_RC);
Caster FC = Caster(FC_LOADCELL_PIN, MOTOR_FC);

Carriage ML_Carriage = Carriage(MOTOR_ML_CARRIAGE, CARRIAGE_SW1_PIN,
                                CARRIAGE_SW2_PIN, EContr.encoderf[11]);

// make ML and connect ML Carriage to it
Wheel ML = Wheel(ML_LOADCELL_PIN, MOTOR_ML, ML_Carriage, EContr.encoderf[7],
                 EContr.encoder[9]);

Carriage MR_Carriage = Carriage(MOTOR_MR_CARRIAGE, CARRIAGE_SW3_PIN,
                                CARRIAGE_SW4_PIN, EContr.encoderf[12]);

// make MR and connect MR Carriage to it
Wheel MR = Wheel(MR_LOADCELL_PIN, MOTOR_MR, MR_Carriage, EContr.encoderf[5],
                 EContr.encoderf[10]);

Timer timer;

bool self_leveling_on = false;
char action;
int CA_flag = 1;
MotorController motor_controller =
    MotorController(RC, FC, MR, ML, timer, self_leveling_on, action, CA_flag);

// Initialize RoboClaw Controllers
RoboClaw roboclaw_casters(&Serial3, 10000);   // Serial3
RoboClaw roboclaw_main(&Serial2, 10000);      // Serial4
RoboClaw roboclaw_carriages(&Serial1, 10000); // Serial5

JoyStick js;

//------------------------------
int profile_counter = 0;
int speed_counter = 0;
int sl_counter = 0;

int dummy_speedcounter = 0;

int mode = 0;

float user_weight = 0.0;

float dist1 = 0.0, dist2 = 0.0;

int commaIndex = 0;

// carriage switches, sw1 = leftback, sw2 = leftfront, sw3 = rightfront, sw4 =
// rightback int carriage_sw1 = 0, carriage_sw2 = 0, carriage_sw3 = 0,
// carriage_sw4 = 0;

// MEBot length: 68cm, DWs width: 62cm, RC width: 22cm
double mebot[4][4] = {
    {-34, 34, 34, -34}, // original
    //  {-50, 50, 50, -50}, //DWs are ok; but desired RC are lower than should
    //  {-45, 23, 23, -45}, //offset. Works but makes front casters touch at 10°
    {-31, -11, 11, 31},
    {0, 0, 0, 0},
    {1, 1, 1, 1}};
double rotm[4][4] = {
    {0., 0., 0., 0.}, {0., 0., 0., 0.}, {0., 0., 0., 0.}, {0., 0., 0., 0.}};
double newmebot[4][4] = {{0.0, 0.0, 0.0, 0.0},
                         {0.0, 0.0, 0.0, 0.0},
                         {0.0, 0.0, 0.0, 0.0},
                         {0.0, 0.0, 0.0, 0.0}};
float drollrd = 0.0, dpitchrd = 0.0;

// Variables to obtain angles from pneumatics
float pneupitch = 0.0, pneuroll = 0.0;

// ************************************************

// Make a CA class (curb automation)
bool curb_climb_automation = false;
bool curb_descend_automation = false;
bool CN_profile = false;
bool SL_profile = false;
bool done_reading_data = false;
bool bypass_3_step = false; // Should be set to false initially
bool did_once = false;
bool set_CA = false;
bool done_travel = false;
bool done_initial_turn = false;
bool done_final_turn = false;
int state = 0;
bool yaw_begin_travel_stored = false;
int task = 0; // To keep a track of which task is being performed throughout the
              // curb detection process

// Not used
bool yaw_begin_final_turn_stored = false;

unsigned long second_counter;
unsigned long delta = 10;
int read_index; // the index of current reading

int count = 0; // updated but not used for anything
int c_h = 0;   // being set but not used for anything

float distance_traveled = 0.0;
float distance_to_travel = 0.0;
float dis_two_third = 0.0;
float CA_init_pos = 0;
float IMUyaw_initial = 0.0, IMU_diff = 0.0;
float yaw_initial_turn = 0.0, yaw_after_initial_turn = 0.0;
float yaw_begin_travel = 0.0, yaw_travel = 0.0, yaw_after_travel = 0.0;
float yaw_final_turn = 0.0, yaw_after_final_turn = 0.0;

// to store input from the real-sense
String input = "";
String test_with_real_sense = "1"; // Change it back to "1"

// arrays to store the readings from real-sense
float height_readings[NUM_READINGS];
float distance_readings[NUM_READINGS];
float theta_readings[NUM_READINGS];
float initial_angle_readings[NUM_READINGS];
float final_angle_readings[NUM_READINGS];
float minInitialAngle =
    -1; // Initialize to a value that won't interfere with actual data

// variables to do moving averages
// float cur_theta = 0.0; // Not Used
float cur_dis = 0.0;
float cur_hei = 0.0;
float cur_ini = 0.0; // ini means initial
float cur_fin = 0.0; // fin means final

float ini_total = 0.0; // ini means initial
float fin_total = 0.0; // fin means final
float dis_total = 0.0;
float hei_total = 0.0;
// float theta_total = 0.0;  // Replaces ini and fin for now // Not Used

// float initial_average = 0.0; // Not Used
// float final_average = 0.0; // Not Used
float ini_average = 0.0;
float fin_average = 0.0;
float dis_average = 0.0;
float hei_average = 0.0;
float theta_average = 0.0;

// float curb_dis = 0.5; // Not Used
// float curb_dis_POI = 0.0; // Not Used
// float ini_angle = 0.0; // Not Used

// Variables for turning
float initial_angle_to_turn = 0.0;
float initial_angle_to_stop = 0.0;
float final_angle_to_turn = 0.0;
float final_angle_to_stop = 0.0;

// Variables for wheel postions
float init_posr = 0.0; // the initial position of the right wheel
// float cur_posr = 0.0;   // the current position of the right wheel // Not
// Used

// CACD variables
// Not being used for anything
// float posdesFR = 0, posdesFL = 0, posdesMR = 0;
// float posdesML = 0, posdesRR = 0, posdesRL = 0;
float MLpreloadcell = 0, MRpreloadcell = 0;

int front_caster_counter = 0;

// String sign = '0';
// String sign_theta = "0"; // Not Used
String sign_initial_angle = "0";
String sign_final_angle = "0";

// ************************************************

String serial_input;
String BTserial_input;

// variables for Curb climbing
bool cc_profile = false;
bool curb_climb_prestep = false;
bool raise_chair = false;
bool curb_height_measure = false;
float curb_height = 0.0;
float FC_state = 0.0;

bool drive_forward = false;

// Setup routine runs once when you press reset:
void setup() {
  Serial.begin(115200); // jetson
  Serial1.begin(38400); // roboclaw 1
  Serial2.begin(38400); // roboclaw 2
  Serial3.begin(38400); // roboclaw 3

  set_calculation_constants();
  initialize_digital_pins();
  IMU.initialize_BNO055_sensor();

  delay(1000);
  bno.setExtCrystalUse(true);
  initialize_all_readings();
}

// Loop routine runs over and over again indefinitely:
void loop() {
  //  if (PI_MOTORS.available() > 0)
  //  {
  //  // read the string data from the real-sense
  //    input = PI_MOTORS.readStringUntil('\n');
  //  }
  //  Serial.println(input);
  timer.updateTime();

  // get_GUI_input_from_bluetooth_joystick();

  js.retrieve_omni2_iom_readings();

  MR.retrieve_lc_reading();
  ML.retrieve_lc_reading();
  FC.retrieve_lc_reading();
  RC.retrieve_lc_reading();

  IMU.retrieve_readings();
  EContr.retrieve_readings();

  // carriage switches, sw1 = leftback, sw2 = leftfront, sw3 = rightfront, sw4 =
  // rightback
  MR.carriage.retrieve_readings();
  ML.carriage.retrieve_readings();

  MR.calculate_main_wheels_positions(timer.elapsed_time);
  ML.calculate_main_wheels_positions(timer.elapsed_time);
  calculate_casters_positions();

  MR.carriage.calculate_carriages_position();
  ML.carriage.calculate_carriages_position();

  calculate_pitch_and_roll_angle();

  get_GUI_input_from_serial();
  Serial.print("Action: ");
  Serial.println(action);

  /* The method below is for getting input using bluetooth, keeping it for now
   */
  // get_GUI_input_from_bluetooth();
  // manual_features();

  select_mode_based_on_GUI_command();
  select_controller_based_on_model();

  motor_controller.carriage_limits_switch();
  motor_controller.set_positions_for_MWs_and_RC();

  displaydata();

  // Reduce delay to increase frequency. Currently set to 8ms
  delay(5);
  reset_newmebot_array();
}

/*
 * ***********************************************************
 * ********* below are helper finctions for setup()***********
 */
void set_calculation_constants() {
  ML_Carriage.carriage_ticks = 12000;
  MR_Carriage.carriage_ticks = -12531;

  ML.pos_ticks = -390;
  ML.wheel_pos_ticks = 15212;
  ML.angle_ticks = -390;
  ML.eha_norm = 39.4;

  MR.pos_ticks = 380;
  MR.wheel_pos_ticks = -15212;
  MR.angle_ticks = 356;
  MR.eha_norm = 35.47;
}

void initialize_digital_pins() {
  // EHA and DW carriage direction and pwms
  MR.initialize_pins();
  MR.carriage.initialize_pins();
  ML.initialize_pins();
  ML.carriage.initialize_pins();
  RC.initialize_pins();
  FC.initialize_pins();

  // Omni2-CC pins
  pinMode(OMNI2_PROFILE_PIN, OUTPUT);
  pinMode(OMNI2_FORWARD_PIN, OUTPUT);
  pinMode(OMNI2_LEFT_PIN, OUTPUT);
  pinMode(OMNI2_RIGHT_PIN, OUTPUT);
  pinMode(OMNI2_REVERSE_PIN, OUTPUT);

  // Omni2-SL pins
  pinMode(JS_SD_PIN, INPUT);
  pinMode(JS_PFILE_PIN, INPUT);
  pinMode(JS_SUP_PIN, INPUT);

  // set up initial joystick I/O values
  pinMode(JS_SW_PIN, OUTPUT);
  digitalWrite(JS_SW_PIN, LOW); // Profile
  analogWrite(JS_X_PIN, 0);     // Speed UP
  analogWrite(JS_Y_PIN, 0);     // Speed DOWN
  // Serial.println("omni2_variables OFF");

  digitalWrite(OMNI2_PROFILE_PIN, LOW);
  digitalWrite(OMNI2_FORWARD_PIN, LOW);
  digitalWrite(OMNI2_LEFT_PIN, LOW);
  digitalWrite(OMNI2_RIGHT_PIN, LOW);
  digitalWrite(OMNI2_REVERSE_PIN, LOW);
}

/*
 * ********* end of helper finctions for setup()**************
 * ***********************************************************
 */

/*
 * ***********************************************************
 * ********* below are helper finctions for loop()************
 */

void calculate_casters_positions() {
  // Height: DWs: 21cm, RC: 20cm, FC: 23cm (off ground)
  // GC: DWs: 3cm, RC: 1cm, FC: 5cm (off ground)
  /* Calculation w.r.t bottom of the base */

  RC.pos = (float)EContr.encoderf[3] * -18.4 / 680.0; // 0 - 18.4
  // RC.pos = 3.0 + 26.8 * sin((-7.0 + (float)EContr.encoderf[3] * -60 / 710) /
  // DG);

  // raw min-max encoder value to limit eha motor movement
  RC.eha = fabs(EContr.encoderf[1]) / 85.62; // actuator stroke 0 - 10.15 cm

  FC.pos = ((float)EContr.encoderf[2]); // max encoder value to z-height
  FC.pos = 1.0 + ((float)EContr.encoderf[2] * 0.031); // 2.5 to 22.5 cm

  // NOT WORKING PROPERLY
  // raw min-max encoder value to limit eha motor movement
  FC.eha = fabs(EContr.encoderf[4]) / 83.6; // actuator stroke 0 - 10.15 cm
  // RC.angle = -7.0 + (float)EContr.encoderf[3] * -60 / 710;
}

void calculate_pitch_and_roll_angle() {
  // Obtain pitch and roll angles from pneumatics
  float xac1 = mebot[0][3] - mebot[0][1];
  float yac1 = mebot[1][3] - mebot[1][1];
  float zac1 = ML.pos - RC.pos;
  float xbd1 = mebot[0][2] - mebot[0][0];
  float ybd1 = mebot[1][2] - mebot[1][0];
  float zbd1 = RC.pos - MR.pos;

  float xac2 = (xac1) / sqrt((xac1 * xac1) + (yac1 * yac1) + (zac1 * zac1));
  float yac2 = (yac1) / sqrt((xac1 * xac1) + (yac1 * yac1) + (zac1 * zac1));
  float zac2 = (zac1) / sqrt((xac1 * xac1) + (yac1 * yac1) + (zac1 * zac1));

  float xbd2 = (xbd1) / sqrt((xbd1 * xbd1) + (ybd1 * ybd1) + (zbd1 * zbd1));
  float ybd2 = (ybd1) / sqrt((xbd1 * xbd1) + (ybd1 * ybd1) + (zbd1 * zbd1));
  float zbd2 = (zbd1) / sqrt((xbd1 * xbd1) + (ybd1 * ybd1) + (zbd1 * zbd1));

  float u[3] = {0, 0, 0};
  u[0] = yac2 * zbd2 - zac2 * ybd2;
  u[1] = zac2 * xbd2 - xac2 * zbd2;
  u[2] = xac2 * ybd2 - yac2 * xbd2;

  pneuroll = atan2(MR.pos - ML.pos, 62) * DG;
  pneupitch = atan2((ML.pos + MR.pos) / 2 - RC.pos, 68) * DG;
}

void get_GUI_input_from_bluetooth() {
  if (PI_BT.available() > 0) {
    String serial_input = PI_BT.readStringUntil('\n');
    action = serial_input.charAt(0);
    // Serial.println(action);
  }
}

void select_mode_based_on_GUI_command() {
  switch (action) {
  case '1': // Control each motor with keyboard
    set_mode(1);
    self_leveling_on = false;
    break;

  case '2': // Control motor with desired position
    set_mode(2);
    self_leveling_on = false;
    break;

  case '3': // Self-leveling control using IMU as input
    set_mode(3);
    break;

  case '4': //  Manual curb climb control
    set_mode(4);
    break;

  case '5': // Manual features -> Carriage
    set_mode(5);
    break;

  case '6': // Manual features incremental
    set_mode(6);
    break;

  default:
    break;
  }
}

void select_controller_based_on_model() {
  IMU.am = IMU.am + 0.005;
  switch (mode) {
  case 1: // dev mode
    individual_motor_FF();
    break;

  case 2: // manual control
    motor_controller.manual_features_proportional();
    break;

  case 3: // advanced features
    advanced_features();
    break;

    // case 4: // manual curb climb control
    //     // Eventually change to CD
    //     manual_curb_climb_v2();
    //     break;

  case 5: // manual control for carriage -> set_mode(5)
    motor_controller.manual_features_carriage();
    analogWrite(JS_Y_PIN, 0);
    analogWrite(JS_Y_PIN, 0);
    analogWrite(JS_Y_PIN, 0);
    break;

  case 6:
    motor_controller.manual_features();
    break;

  default:
    break;
  }
}

void displaydata() {
  String PIout = "[";

  // IMU
  PIout += String(IMU.pitchf + 3.0) + ',';
  PIout += String(IMU.rollf) + ',';
  PIout += String(IMU.ax) + ',';
  PIout += String(IMU.ay) + ',';
  PIout += String(IMU.az) + ',';

  // Encoders
  PIout += String(FC.pos) + ',';
  PIout += String(RC.pos) + ',';
  PIout += String(MR.pos) + ',';
  PIout += String(ML.pos) + ',';
  PIout += String(MR.carriage.pos) + ',';
  PIout += String(ML.carriage.pos) + ',';
  PIout += String(ML.wheel_pos) + ',';
  PIout += String(MR.wheel_pos) + ',';

  // loadcell readings
  PIout += String(FC.loadcell) + ',';
  PIout += String(MR.loadcell) + ',';
  PIout += String(ML.loadcell) + ',';

  // ca_flag
  PIout += String(CA_flag) + ',';

  // apptime
  PIout += "0,"; // placeholder

  // velocity
  PIout += String(ML.speed_drivef) + ',';
  PIout += String(MR.speed_drivef) + ',';

  // acceleration will be calculated in sensor_data_pub
  // tilt can be calculated using pitch and roll in sensor_data_pub
  // measure height will be calculated in sensor_data_pub using encoder data

  PIout += "]";
  Serial.println(PIout);
}

void reset_newmebot_array() {
  MR.last_err = MR.err;
  ML.last_err = ML.err;
  RC.last_err = RC.err;
  FC.last_err = FC.err;
  MR.carriage.last_err = MR.carriage.err;
  ML.carriage.last_err = ML.carriage.err;
  ML.wheel_pos_pre = ML.wheel_pos;
  MR.wheel_pos_pre = MR.wheel_pos;
  RC.des_pre = RC.des;
  ML.des_pre = ML.des;
  MR.des_pre = MR.des;
  FC.des_pre = FC.des;
  MR.carriage.des_pre = MR.carriage.des;
  ML.carriage.des_pre = ML.carriage.des;

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      newmebot[i][j] = 0.0;
    }
  }
}

/*
 * ********* end of helper finctions for loop()***************
 * ***********************************************************
 */

/*
 * ************************************************************************
 * ********* below are helper finctions used in other functions************
 */

// select_mode_based_on_GUI_command helper function
void set_mode(int num) {
  ML.des = ML.pos;
  MR.des = MR.pos;
  RC.des = RC.pos;
  FC.des = FC.pos;
  ML.carriage.des = ML.carriage.pos;
  MR.carriage.des = MR.carriage.pos;
  mode = num;
}

// select_controller_based_on_model helper functions
void individual_motor_FF() {
  // Serial.println("I'm here in Individual Motor FF");
  CA_flag = 1;
  // ML UP
  if (action == 'q') {
    motor_controller.ML_UP_1s(); // raise ML up for 1 second
  }
  // ML DOWN
  else if (action == 'a') {
    motor_controller.ML_DOWN_1s();
  }
  // RC UP, activate relay 2 forward
  else if (action == 'w') {
    motor_controller.RC_UP_1s();
  }
  // RC DOWN, activate relay 2 reverse
  else if (action == 's') {
    motor_controller.RC_DOWN_1s();
  }
  // MR UP
  else if (action == 'e') {
    motor_controller.MR_UP_1s();
  }
  // MR DOWN
  else if (action == 'd') {
    motor_controller.MR_DOWN_1s();
  }
  // FC UP
  else if (action == 'r') {
    motor_controller.FC_UP_1s();
  }
  // FC DOWN
  else if (action == 'f') {
    motor_controller.FC_DOWN_1s();
  }
  // left carriage Forward
  else if (action == 't') {
    motor_controller.LEFT_CARRIAGE_FORWARD_point4s();
  }
  // left side carriage Backward
  else if (action == 'g') {
    motor_controller.LEFT_CARRIAGE_BACKWARD_point4s();
  }
  // right side carriage Forward
  else if (action == 'y') {
    motor_controller.RIGHT_CARRIAGE_FORWARD_point4s();
  }
  // right side carriage Backward
  else if (action == 'h') {
    motor_controller.RIGHT_CARRIAGE_BACKWARD_point4s();
  }
  // right side carriage Forward
  else if (action == 'j') {
    motor_controller.BOTH_CARRIAGE_FORWARD_4s();
  }
  // right side carriage Backward
  else if (action == 'u') {
    motor_controller.BOTH_CARRIAGE_BACKWARD_4s();
  } else if (action == 'z') {
    motor_controller.NO_MOVEMENT();
    js.x = 0;
    js.y = 0;
    digitalWrite(JS_SW_PIN, 0);
    analogWrite(JS_X_PIN, 0);
    analogWrite(JS_Y_PIN, 0);
  }
}

void initialize_all_readings() {
  for (int i = 0; i < NUM_READINGS; i++) {
    height_readings[i] = 0.0;
    distance_readings[i] = 0.0;
    theta_readings[i] = 0.0;
    initial_angle_readings[i] = 0.0;
    final_angle_readings[i] = 0.0;
  }
}

float normalize_angle(float angle) {
  angle = fmod(angle, 360);
  if (angle < 0)
    angle += 360;
  return angle;
}

void reset_all_automation_variables() {
  // Curb negotiation variables
  raise_chair = false;
  curb_climb_prestep = false;
  cc_profile = false;
  curb_height_measure = false;

  // Not used
  //  js.x = 0;
  //  js.y = 0;

  // These are not used
  hei_average = 0.0;
  ini_average = 0.0;
  dis_average = 0.0;
  fin_average = 0.0;
  theta_average = 0.0;

  init_posr = 0.0;

  IMUyaw_initial = 0.0;
  IMU_diff = 0.0;
  initial_angle_to_turn = 0.0;
  initial_angle_to_stop = 0.0;
  yaw_initial_turn = 0.0;
  yaw_after_initial_turn = 0.0;

  yaw_begin_travel = 0.0;
  distance_to_travel = 0.0;
  distance_traveled = 0.0;
  yaw_travel = 0.0;
  yaw_after_travel = 0.0;

  final_angle_to_turn = 0.0;
  final_angle_to_stop = 0.0;
  yaw_final_turn = 0.0;
  yaw_after_final_turn = 0.0;

  sign_initial_angle = "0";
  sign_final_angle = "0";
  done_travel = false;
  done_initial_turn = false;
  done_final_turn = false;
  bypass_3_step = false;
  curb_climb_automation = false;
  curb_descend_automation = false;
  self_leveling_on = false;

  done_reading_data = false;
  did_once = false;
  set_CA = false;
  yaw_begin_final_turn_stored = false;
  yaw_begin_travel_stored = false;

  CA_flag = 1;
  second_counter = 0;

  dis_two_third = 0.0;
  task = 0;
  profile_counter = 0;
  speed_counter = 0;
  front_caster_counter = 0;
}

void advanced_features() {
  // Self leveling
  if (action == 's') {
    SL_profile = true;
    CN_profile = false;
    self_leveling_on = true;
    CA_flag = 1;
  }
  // Curb climbing Automation
  else if (action == 'c') {
    self_leveling_on = false;
    CN_profile = true;
    raise_chair = true;
    CA_flag = 1;
  }
  // Curb descend Automation
  else if (action == 'd') {
    self_leveling_on = false;
    CN_profile = true;
    curb_descend_automation = true;
  }
  // Reset the chair to the initial state
  else if (action == 'r') {
    CA_flag = 1;
    self_leveling_on = false;
    sl_counter = 0;
    FC.des = 2.0;
    motor_controller.move_each_wheel(3.0, 0.3, 3.0);
    MR.carriage.des = 0.1;
    ML.carriage.des = 0.1;
    js.set_joystick_speed(0, 0);

    // Reset all curb climb variables
    reset_all_automation_variables();
  }

  // Steps for curb descending automation
  if (curb_descend_automation == true) {
    // Serial.println("Curb Drop Automation");

    // Reset all curb descend variables
    // reset_all_automation_variables();
    // auto_curb_descend();
  }

  // Elevate RC/DWs to their maximum position. Front caster to lowest position
  // Chair must be near curb before starting this step.
  // Otherwise, next step may not work as intended.
  if (raise_chair == true) {
    // Assume curb climb selection was selected with joystick --> change to CN
    // profile: if (CN_profile == true)
    // {
    //     profile_counter = profile_counter + 1;
    //     if (profile_counter < 20 * 1)
    //     {
    //         digitalWrite(JS_SW_PIN, LOW);
    //     }
    //     else if (profile_counter >= 20 * 1 && profile_counter < 20 * 2)
    //     {
    //         Serial.println("profile 1");
    //         digitalWrite(JS_SW_PIN, HIGH);
    //     }
    //     else if (profile_counter >= 20 * 2 && profile_counter < 20 * 3)
    //     {
    //         digitalWrite(JS_SW_PIN, LOW);
    //     }
    //     else
    //     {
    //         CN_profile = false;
    //         profile_counter = 0;
    //     }
    // }

    // Elevate RC/DWs to their maximum position. Front caster to lowest position
    front_caster_counter = front_caster_counter + 1;
    if (front_caster_counter < 200) {
      FC.des = 2.0; // Front caster to lowest position
    } else {
      front_caster_counter = 200;
      FC.des = FC.pos;
    }
    ML.carriage.des = 0.1;
    MR.carriage.des = 0.1;
    RC.des = 8.0; // initial value 7.0
    ML.des = 20.0;
    MR.des = 20.0;
    // record MR.pos, ML.pos values to calculate distance to be traveled later
    if (MR.pos > 19.0 && ML.pos > 19.0) {
      CA_flag = 2;
      ML.wheel_pos_init = ML.wheel_pos;
      MR.wheel_pos_init = MR.wheel_pos;
      MLpreloadcell = ML.loadcell;
      MRpreloadcell = MR.loadcell;
      curb_climb_prestep = true;
    }
  }
  // MEBot takes control
  if (curb_climb_prestep == true) {
    raise_chair = false;
    cc_profile = true;
    auto_curb_climb();
  }

  if (self_leveling_on) {
    // change to SL profile:
    if (SL_profile == true) {
      profile_counter = profile_counter + 1;
      if (profile_counter < 20 * 1) {
        digitalWrite(JS_SW_PIN, LOW);
      } else if (profile_counter >= 20 * 1 && profile_counter < 20 * 2) {
        Serial.println("profile 1");
        digitalWrite(JS_SW_PIN, HIGH);
      } else if (profile_counter >= 20 * 2 && profile_counter < 20 * 3) {
        digitalWrite(JS_SW_PIN, LOW);
      } else if (profile_counter >= 20 * 3 && profile_counter < 20 * 4) {
        Serial.println("profile 2");
        digitalWrite(JS_SW_PIN, HIGH);
      } else if (profile_counter >= 20 * 4 && profile_counter < 20 * 5) {
        digitalWrite(JS_SW_PIN, LOW);
      } else {
        SL_profile = false;
        profile_counter = 0;
      }
    }
    if (SL_profile == false) {
      self_leveling_application();
    }
    // Serial.println("SL ON within advanced features");
  }
  motor_controller.individual_motor_PID();
}

// advanced_features helper functions
void self_leveling_application() {
  //    dpitchrd = 1 * (pneupitch / DG - IMU.pitchrd);
  //    drollrd = 1 * (pneuroll / DG - IMU.rollrd);
  //    // Self-leveling starting angle at 3 degrees
  //    // dpitchrd = 1 * (pneupitch / DG - IMU.pitchrd) + 3.0 / DG;
  //    if (fabs(dpitchrd) < 0.001)
  //    {
  //        dpitchrd = 0.0;
  //    }
  //    if (fabs(drollrd) < 0.001)
  //    {
  //        drollrd = 0.0;
  //    }
  //
  //    // construct_rotm_array(dpitchrd, drollrd);
  //    rotm[0][0] = cos(dpitchrd);
  //    rotm[0][1] = 0.0;
  //    rotm[0][2] = sin(dpitchrd);
  //    rotm[0][3] = 0.0;
  //    rotm[1][0] = sin(drollrd) * sin(dpitchrd);
  //    rotm[1][1] = cos(drollrd);
  //    rotm[1][2] = -1 * sin(drollrd) * cos(dpitchrd);
  //    rotm[1][3] = 0.0;
  //    rotm[2][0] = -1 * cos(drollrd) * sin(dpitchrd);
  //    rotm[2][1] = sin(drollrd);
  //    rotm[2][2] = cos(drollrd) * cos(dpitchrd);
  //    rotm[2][3] = 9.5; // was 10
  //    rotm[3][0] = 0.0;
  //    rotm[3][1] = 0.0;
  //    rotm[3][2] = 0.0;
  //    rotm[3][3] = 1.0;
  //
  //    // set_newmebot_array_base_on_rotm_and_mebot();
  //    for (int row = 0; row < 4; row++)
  //    {
  //        for (int col = 0; col < 4; col++)
  //        {
  //            for (int inner = 0; inner < 4; inner++)
  //            {
  //                newmebot[row][col] += rotm[row][inner] * mebot[inner][col];
  //            }
  //        }
  //    }
  //
  //    // Initialize self-leveling for 5 seconds
  //    sl_counter = sl_counter + 1;
  //    if (sl_counter < 80 * 5)
  //    {
  //        // calculate_desired_values_for_Mws_RC_FC();
  //        ML.des = newmebot[2][0];
  //        RC.des = (newmebot[2][1] + newmebot[2][2]) / 2;
  //        MR.des = newmebot[2][3];
  //        // stop DW carriage and front casters if previous action moved them
  //        ML.carriage.des = 0.1;
  //        MR.carriage.des = 0.1;
  //        FC.des = 1.0;
  //    }
  //    else
  //    {
  //        sl_counter = 80 * 5;
  //        // Inhibit EHA movement if IMU.pitch is positive (driving up)
  //        if (dpitchrd * DG < 6.0 && fabs(drollrd * DG) < 2.0)
  //        {
  //            ML.des = ML.des_pre;
  //            RC.des = RC.des_pre;
  //            MR.des = MR.des_pre;
  //            FC.des = FC.pos;
  //            // Serial.println("No change in SL des values");
  //        }
  //        else
  //        {
  //            // calculate_desired_values_for_Mws_RC_FC();
  //            ML.des = newmebot[2][0];
  //            RC.des = (newmebot[2][1] + newmebot[2][2]) / 2;
  //            MR.des = newmebot[2][3];
  //            // stop DW carriage and front casters if previous action moved
  //            them ML.carriage.des = ML.carriage.pos; MR.carriage.des =
  //            MR.carriage.pos; FC.des = FC.pos;
  //
  //            // COMMENTED FOR RAMMP DEMO
  //            //  Reduce speed in positive pitch and all roll (going down or
  //            side slopes)
  //            //  if (dpitchrd * DG > 6.0 || drollrd * DG > 3.0 || drollrd *
  //            DG < -3.0)
  //            //  {
  //            //      if (js.speed_counter > 1)
  //            //      {
  //            //          speed_counter = speed_counter + 1;
  //            //          if (speed_counter < 20 * 1 && speed_counter >= 1)
  //            //          {
  //            //              // Serial.println("no change");
  //            //              analogWrite(JS_X_PIN, 0);
  //            //          }
  //            //          else if (speed_counter >= 20 * 1 && speed_counter <
  //            20 * 2)
  //            //          {
  //            //              // Serial.println(" speed CHANGEEE");
  //            //              analogWrite(JS_X_PIN, 255);
  //            //          }
  //            //          else if (speed_counter >= 20 * 2 && speed_counter <=
  //            20 * 3)
  //            //          {
  //            //              analogWrite(JS_X_PIN, 0);
  //            //          }
  //            //          else
  //            //          {
  //            //              // Serial.println("nothing");
  //            //              speed_counter = 0;
  //            //          }
  //            //      }
  //            //      else if (js.speed_counter == 1)
  //            //      {
  //            //          analogWrite(JS_X_PIN, 0);
  //            //          analogWrite(JS_Y_PIN, 0);
  //            //          speed_counter = 0;
  //            //      }
  //            //  }
  //            //  Speed DOWN. Use it with dpitchrd*DG in if-statement
  //            //  else {
  //            //    if (js.speed_counter < 3) {
  //            //      test_counter = test_counter + 1;
  //            //      if (test_counter < 20 * 1 && test_counter >= 1) {
  //            //        analogWrite(JS_Y_PIN, 0);
  //            //      } else if (test_counter >= 20 * 1 && test_counter < 20 *
  //            2) {
  //            //        analogWrite(JS_Y_PIN, 255);
  //            //      } else if (test_counter >= 20 * 2 && test_counter < 20 *
  //            3) {
  //            //        analogWrite(JS_Y_PIN, 0);
  //            //      } else {
  //            //        Serial.println("keep me at this speed");
  //            //        analogWrite(JS_Y_PIN, 0);
  //            //        analogWrite(JS_X_PIN, 0);
  //            //        test_counter = 0;
  //            //      }
  //            //    } else if (js.speed_counter >= 3) {
  //            //      analogWrite(JS_Y_PIN, 0);
  //            //      analogWrite(JS_X_PIN, 0);
  //            //      test_counter = 0;
  //            //    }
  //            //  }
  //        }
  //    }
}

void get_GUI_input_from_serial() {
  if (Serial.available() > 0) {
    String serial_input = Serial.readStringUntil('\n');
    action = serial_input.charAt(0);
    //        Serial.println(action);
  }
}

//======================================================================
// Potential function for automatic curb climbing with omni2 controller
// void auto_curb_climb_omni2() {
//     // Change to CC profile
//     if (cc_profile == true) {
//         test_counter = test_counter + 1;
//         if (test_counter < 50 * 1) {
//             digitalWrite(OMNI2_PROFILE_PIN, LOW);
//         } else if (test_counter > 50 * 1 && test_counter < 50 * 2) {
//             digitalWrite(OMNI2_PROFILE_PIN, HIGH);
//         } else if (test_counter > 50 * 2 && test_counter < 50 * 3) {
//             digitalWrite(OMNI2_PROFILE_PIN, LOW);
//         } else {
//             test_counter = 0;
//             curb_height_measure = true;
//             cc_profile = false;
//             CA_flag = 0;
//         }
//     }
//     // place front casters down to measure height
//     if (curb_height_measure == true) {
//         FC.des = FC.pos + 0.01;
//         if (FC.loadcell < 430) {
//             FC.des = FC.pos;
//             FC_state = FC.pos;
//             curb_height = (MR.pos + RC.pos) / 2 - FC.pos;  // in cm
//             curb_height_measure = false;
//             CA_flag = 2;
//         }
//     }
//     if (CA_flag == 2) {
//         // step three, push the frame onto the curb followed by elevating
//         ML,MR MR.carriage.des = 30.0; ML.carriage.des = 30.0; FC.des =
//         FC_state; if (ML.carriage.pos > 15.0) {
//             motor_controller.move_each_wheel(3.5, 21.0, 3.5);
//         }
//         if (MR.carriage.pos > 29.5 && ML.carriage.pos > 29.5) {
//             CA_flag = 3;
//             drive_forward = true;
//         }
//     } else if (CA_flag == 3) {
//         // spin the wheel while moving the carriage half-way
//         FC.des = FC_state;
//         MR.carriage.des = 20.0;
//         ML.carriage.des = 20.0;
//         motor_controller.move_each_wheel(3.5, RC.pos, 3.5);
//         if (drive_forward == true) {
//             test_counter = test_counter + 1;
//             if (test_counter < 50 * 1) {
//                 digitalWrite(OMNI2_FORWARD_PIN, LOW);
//             } else if (test_counter > 50 * 1 && test_counter < 50 * 4) {
//                 digitalWrite(OMNI2_FORWARD_PIN, HIGH);
//             } else if (test_counter > 50 * 4 && test_counter < 50 * 7) {
//                 digitalWrite(OMNI2_FORWARD_PIN, LOW);
//             } else {
//                 drive_forward = false;
//                 test_counter = 0;
//                 CA_flag = 4;
//             }
//         }
//     } else if (CA_flag == 4) {
//         // step six, lift rear casters and move the carriage back to the edge
//         of the curb to hold the weight FC.des = FC_state; MR.carriage.des
//         = 30.0; ML.carriage.des = 30.0;
//         motor_controller.move_each_wheel(3.5, 2.0, 3.5);
//         if (drive_forward == true) {
//             test_counter = test_counter + 1;
//             if (test_counter < 50 * 1) {
//                 digitalWrite(OMNI2_FORWARD_PIN, LOW);
//             } else if (test_counter > 50 * 1 && test_counter < 50 * 4) {
//                 digitalWrite(OMNI2_FORWARD_PIN, HIGH);
//             } else if (test_counter > 50 * 4 && test_counter < 50 * 7) {
//                 digitalWrite(OMNI2_FORWARD_PIN, LOW);
//             } else {
//                 drive_forward = false;
//                 test_counter = 0;
//                 CA_flag = 5;
//             }
//         }
//     }

//     else if (CA_flag == 5) {
//         // step eight, move both carriage back to original position
//         FC.des = -2.5;
//         MR.carriage.des = 0.5;
//         ML.carriage.des = 0.5;
//         motor_controller.move_each_wheel(3.5, 2.0, 3.5);
//         if (MR.carriage.pos < 15.0) {
//             if (drive_forward == true) {
//                 test_counter = test_counter + 1;
//                 if (test_counter < 50 * 1) {
//                     digitalWrite(OMNI2_FORWARD_PIN, LOW);
//                 } else if (test_counter > 50 * 1 && test_counter < 50 * 4) {
//                     digitalWrite(OMNI2_FORWARD_PIN, HIGH);
//                 } else if (test_counter > 50 * 4 && test_counter < 50 * 7) {
//                     digitalWrite(OMNI2_FORWARD_PIN, LOW);
//                 } else {
//                     drive_forward = false;
//                     test_counter = 0;
//                     CA_flag = 6;
//                 }
//             }
//         }
//     }
//     // STOP any movement
//     else if (CA_flag == 6) {
//         FC.des = FC.pos;
//         motor_controller.move_each_wheel(ML.pos, RC.pos, MR.pos);
//         MR.carriage.des = MR.carriage.pos;
//         ML.carriage.des = ML.carriage.pos;
//         CA_flag = 0;
//         curb_climb_prestep = false;
//     }

//     // Reset all curb climb variables
//     reset_all_automation_variables();
// }

//======================================================================
// // Automated CC. Manual drive. Both drive wheels lift together
void auto_curb_climb() {
  if (CA_flag == 2) {
    // step two, drive forward till hit the curb
    motor_controller.move_each_wheel(20.5, RC.pos, 20.5);
    FC.des = 2.0;
    // Hit curb to start CC automation
    if (abs(MLpreloadcell - ML.loadcell) > 20 ||
        abs(MRpreloadcell - MR.loadcell) > 20) {
      CA_flag = 3;
    }

    // MR.wheel_traveled = MR.wheel_pos - MR.wheel_pos_init;
    // if (MR.wheel_traveled > 30.0)
    //     CA_flag = 3;
    // }
  } else if (CA_flag == 3) {
    // step three, push the carriage onto the curb and elevating ML,MR
    MR.carriage.des = 30.0;
    ML.carriage.des = 30.0;
    FC.des = 2.0;
    // if (ML.carriage.pos > 10.0)
    // {
    motor_controller.move_each_wheel(ML.pos, 21.0, MR.pos);
    // }

    if (MR.carriage.pos > 29.5 && ML.carriage.pos > 29.5) {
      CA_flag = 4;
    }
  } else if (CA_flag == 4) {
    // step four, raise the main wheels
    // and spin the wheel a bit to reduce friction if the wheel is touching the
    // curb
    motor_controller.move_each_wheel(3.0, 21.0, 3.0);
    FC.des = 4.0;
    MR.carriage.des = 30.0;
    ML.carriage.des = 30.0;
    //         js.set_joystick_speed(0, 25);
    if (MR.pos < 4.0 && ML.pos < 4.0) {
      CA_flag = 5;
      //             js.set_joystick_speed(0, 0);
    }
  } else if (CA_flag == 5) {
    // step five, move the the driving wheels and the carriage onto the curb
    motor_controller.move_each_wheel(3.0, RC.pos, 3.0);
    //         js.set_joystick_speed(245, 30);
    MR.carriage.des = 13.0;
    ML.carriage.des = 13.0;
    FC.des = 6.0;
    if (MR.carriage.pos < 13.5 && ML.carriage.pos < 13.5) {
      CA_flag = 6;
      //             js.set_joystick_speed(0, 0);
    }
  } else if (CA_flag == 6) {
    // step six, move the carriage back to the edge of the curb to hold the
    // weight
    motor_controller.move_each_wheel(ML.pos, RC.pos, MR.pos);
    FC.des = 1.0;
    //         js.set_joystick_speed(245, 20);
    MR.carriage.des = 31.0;
    ML.carriage.des = 31.0;
    if (MR.carriage.pos > 30.0 && ML.carriage.pos > 30.0) {
      CA_flag = 7;
      //             js.set_joystick_speed(0, 0);
    }
  } else if (CA_flag == 7) {
    // step seven, raise rear casters
    motor_controller.move_each_wheel(3.5, 3.0, 3.5);
    FC.des = 1.0;
    MR.carriage.des = MR.carriage.pos;
    ML.carriage.des = ML.carriage.pos;
    //         js.set_joystick_speed(245, 30);
    if (RC.pos < 4.0) {
      CA_flag = 8;
      ML.wheel_pos_init = ML.wheel_pos;
      MR.wheel_pos_init = MR.wheel_pos;
      //             js.set_joystick_speed(0, 0);
    }
  } else if (CA_flag == 8) {
    // step eight, move both carriage back to original position
    motor_controller.move_each_wheel(3.5, 3.0, 3.5);
    FC.des = FC.pos;
    MR.wheel_traveled = MR.wheel_pos - MR.wheel_pos_init;
    if (MR.wheel_traveled < 15.0) {
      //             js.set_joystick_speed(245, 50);
    } else {
      MR.carriage.des = 0.5;
      ML.carriage.des = 0.5;
      //             js.set_joystick_speed(245, 10);
    }
    if (MR.carriage.pos < 1.0 && ML.carriage.pos < 1.0) {
      CA_flag = 9;
      //             js.set_joystick_speed(0, 0);
    }
  }
  // Raise front casters and STOP any movement
  else if (CA_flag == 9) {
    front_caster_counter = front_caster_counter + 1;
    if (front_caster_counter < 200) {
      FC.des = 0.0; // Front caster to lowest position
    } else {
      front_caster_counter = 200;
      FC.des = FC.pos;
    }
    motor_controller.move_each_wheel(ML.pos, RC.pos, MR.pos);
    MR.carriage.des = MR.carriage.pos;
    ML.carriage.des = ML.carriage.pos;
    CA_flag = 0;
  } else if (CA_flag == 0) {
    // Reset all curb climb variables
    reset_all_automation_variables();
  }
}

//======================================================================

// void approach_to_curb() {
//     if (bypass_3_step == true) {
//         if (!set_CA) {
//             CA_flag = 1;  // is it the same as CA in Siva?
//             CA_init_pos = MR.wheel_pos;
//             set_CA = true;
//         }
//         state = 4;
//         // auto_curb_climb();
//     } else {
//         // save the initial IMU.yaw value
//         if (!did_once)  // flag, so this if statement will only run once
//         {
//             IMUyaw_initial = IMU.yaw;  // record the initial IMU.yaw reading
//             init_posr = MR.wheel_pos;  // record the initial postion of the
//             right wheel give_sign_to_angles();

//             // calculate the angle to stop
//             initial_angle_to_stop = normalize_angle(IMUyaw_initial -
//             initial_angle_to_turn);

//             did_once = true;
//         }

//         if (!done_initial_turn) {
//             // level_the_wheels(); // doesn't do anything other than set
//             variables that aren't even being used anywhere state = 1;
//             execute_initial_turn_real_sense();
//         } else {
//             if (!done_travel) {
//                 state = 2;
//                 travel();
//             } else {
//                 // if done traveling, start turning toward the curb
//                 if (!done_final_turn) {
//                     state = 3;
//                     execute_final_turn_real_sense();
//                 } else {
//                     //        yaw_final_stop = IMU.yaw;
//                     //        curb_climb_automation = false;// need to be
//                     commented for curb climbing sequence to happen

//                     // BELOW IS THE ACTUAL CLIMBING SEQUENCE, comment out if
//                     only needs to test the 3 step automation if (!set_CA) {
//                         // CA = 1 only happen once, this will triger the
//                         climbing sequence CA_flag = 1;  // Is it the same as
//                         CA_flag in Siva? CA_init_pos = MR.wheel_pos; set_CA =
//                         true;
//                     }
//                     state = 4;
//                     // auto_curb_climb();
//                 }
//             }
//         }
//     }
// }

//======================================================================

// void get_GUI_input_from_bluetooth_joystick() {
//     if (PI_BT.available() > 0) {
//         serial_input = PI_BT.readStringUntil('\n');
//         //    Serial.print(serial_input); Serial.print("\t");
//     }

//     // These aren't being used anywhere
//     // PI_joyx = serial_input.substring(0, 2);
//     // PI_joyy = serial_input.substring(3, 5);
//     // PI_pot = serial_input.substring(6, 7);
//     // PI_switch = serial_input.substring(8, 9);
//     // PI_joyswitch = serial_input.substring(10, 11);
// }

//======================================================================

// void omni2test() {
//     // test Omni2 signals for CC: profile assign and directions
//     test_counter = test_counter + 1;
//     if (test_counter < 300 * 1) {
//         digitalWrite(OMNI2_PROFILE_PIN, HIGH);
//         Serial.print(" OMNI2_PROFILE_PIN HIGH ");  // Profile
//     } else if (test_counter >= 300 * 1 && test_counter < 300 * 2) {
//         digitalWrite(OMNI2_PROFILE_PIN, LOW);
//         Serial.print(" OMNI2_PROFILE_PIN LOW ");
//     } else if (test_counter >= 300 * 2 && test_counter < 300 * 3) {
//         digitalWrite(OMNI2_FORWARD_PIN, HIGH);
//         Serial.print(" OMNI2_FORWARD_PIN HIGH ");  // Forward
//     } else if (test_counter >= 300 * 3 && test_counter < 300 * 4) {
//         digitalWrite(OMNI2_FORWARD_PIN, LOW);
//         Serial.print(" OMNI2_FORWARD_PIN LOW ");
//     } else if (test_counter >= 300 * 4 && test_counter < 300 * 5) {
//         digitalWrite(OMNI2_LEFT_PIN, HIGH);  // Left
//         Serial.print(" OMNI2_LEFT_PIN HIGH ");
//     } else if (test_counter >= 300 * 5 && test_counter < 300 * 6) {
//         digitalWrite(OMNI2_LEFT_PIN, LOW);
//         Serial.print(" OMNI2_LEFT_PIN LOW ");
//     } else if (test_counter >= 300 * 6 && test_counter < 300 * 7) {
//         digitalWrite(OMNI2_RIGHT_PIN, HIGH);  // Right
//         Serial.print(" OMNI2_RIGHT_PIN HIGH ");
//     } else if (test_counter >= 300 * 7 && test_counter < 300 * 8) {
//         digitalWrite(OMNI2_RIGHT_PIN, LOW);
//         Serial.print(" OMNI2_RIGHT_PIN LOW ");
//     } else if (test_counter >= 300 * 8 && test_counter < 300 * 11) {
//         digitalWrite(OMNI2_REVERSE_PIN, HIGH);  // Reverse
//         Serial.print(" OMNI2_REVERSE_PIN HIGH ");
//     } else if (test_counter >= 300 * 11 && test_counter < 300 * 12) {
//         digitalWrite(OMNI2_REVERSE_PIN, LOW);
//         Serial.print(" OMNI2_REVERSE_PIN LOW ");
//     } else {
//         test_counter = 0;
//     }
//     Serial.println(test_counter);
// }

//======================================================================

// void joystickiotest() {
//     test_counter = test_counter + 1;
//     if (test_counter < 300 * 1) {
//         digitalWrite(JS_SW_PIN, HIGH);  // Profile
//         Serial.print(" joystick profile HIGH ");
//     } else if (test_counter >= 300 * 1 && test_counter < 300 * 2) {
//         digitalWrite(JS_SW_PIN, LOW);
//         Serial.print(" joystick profile LOW ");
//     } else if (test_counter >= 300 * 2 && test_counter < 300 * 3) {
//         analogWrite(JS_Y_PIN, 255);  // Speed UP
//         Serial.print("speed up HIGH ");
//     } else if (test_counter >= 300 * 3 && test_counter < 300 * 4) {
//         analogWrite(JS_Y_PIN, 0);  // Speed UP
//         Serial.print("speed up LOW");
//     } else if (test_counter >= 300 * 4 && test_counter < 300 * 5) {
//         analogWrite(JS_X_PIN, 255);  // Speed DOWN
//         Serial.print("speed down HIGH ");
//     } else if (test_counter >= 300 * 5 && test_counter < 300 * 6) {
//         analogWrite(JS_X_PIN, 0);  // Speed DOWN
//         Serial.print("speed down LOW ");
//     } else {
//         test_counter = 0;
//     }
//     Serial.println(test_counter);
// }

//======================================================================
// void read_data() {
//     // Read data from real sense for 1 second
//     if (second_counter < 10000) {
//         // Update task number/order
//         task = 2;

//         second_counter += delta;
//         check_input_source(test_with_real_sense);
//         count += 1;
//         // Check the length of input
//         if (PACKET_LENGTH <= input.length() && input.length() <=
//         PACKET_LENGTH + 6)  //(input.length() == PACKET_LENGTH)
//         {
//             extract_data_from_input_real_sense();  // Task 3

//             calculate_moving_average_real_sense();  // Task 4
//         } else {
//             // Update task number/order
//             task = 5;

//             input = "";
//         }
//     } else {
//         // Update task number/order
//         task = 6;

//         set_curb_climb_automation_parameters_real_sense();
//         done_reading_data = true;
//     }
// }

//======================================================================

// void extract_data_from_input_real_sense() {
//     /*
//       packet string format:
//       "curb_height + distance_of_interest + angle_initial_turn +
//       sign_initial_turn + angle_final_turn + sign_final_turn" 0.00(meter)
//       0.00(meter)            000.00(degree)       0(-) / 1(+) 000.00(degree)
//       0(-)/1(+) 4              4                      6                    1
//       6                  1          total of 22 charactors

//       Break down:
//       packet: 0.00|0.00|000.00|0 |000.00|0
//       index: 0--3|4--7|8---13|14|15--20|21
//     */
//     task = 3;
//     // Serial.println(input);

//     // index 0 to 4 should give the height
//     String hei = input.substring(0, 4);
//     // index 4 to 8 should gice the distance
//     String dis = input.substring(4, 8);
//     // index 8 to 13 should give the initial angle
//     String ini = input.substring(8, 14);
//     // index 13 to 14 should give the direaction, "0" means right, "1" means
//     left sign_initial_angle = input.substring(14, 15);
//     // index 14 to 19 should give the final angle
//     String fin = input.substring(15, 21);
//     // index 19 to 20 should give the direction, "0" means right, "1" means
//     left sign_final_angle = input.substring(21, 22);

//     // Serial.println(input.length());

//     input = "";  // clear the variable for next iteration

//     // local varaible to store the current reading
//     cur_dis = dis.toFloat();
//     cur_hei = hei.toFloat();
//     float currentIni = cur_ini = ini.toFloat();
//     cur_fin = fin.toFloat();

//     // Check if the minimum initial angle has not been set or if the current
//     angle is smaller than the stored minimum if (minInitialAngle == -1 ||
//     fabs(currentIni - minInitialAngle) < ANGLE_THRESHOLD) {
//         // If the difference between the current and stored values is within
//         the threshold, update the minimum minInitialAngle = currentIni;
//         cur_ini = minInitialAngle;  // Update cur_ini to be the smallest
//         initial angle
//     }

//     // debug print
//     Serial.print("c_hei:");
//     Serial.print(cur_hei);

//     Serial.print(" dis:");
//     Serial.print(cur_dis);

//     Serial.print(" c_ini:");
//     Serial.print(cur_ini);

//     Serial.print(" sign_initial_sign:");
//     Serial.print(sign_initial_angle);

//     Serial.print(" c_fin:");
//     Serial.print(cur_fin);

//     Serial.print(" sign_final_sign:");
//     Serial.println(sign_final_angle);
// }

//======================================================================

// void calculate_moving_average_real_sense() {
//     // Update task number/order
//     task = 4;

//     // subtract the last reading:
//     dis_total = dis_total - distance_readings[read_index];
//     hei_total = hei_total - height_readings[read_index];
//     ini_total = ini_total - initial_angle_readings[read_index];
//     fin_total = fin_total - final_angle_readings[read_index];

//     // add the current reading to the reading array
//     distance_readings[read_index] = cur_dis;
//     height_readings[read_index] = cur_hei;
//     initial_angle_readings[read_index] = cur_ini;
//     final_angle_readings[read_index] = cur_fin;

//     // add the current reading to the total
//     hei_total = hei_total + height_readings[read_index];
//     dis_total = dis_total + distance_readings[read_index];
//     ini_total = ini_total + initial_angle_readings[read_index];
//     fin_total = fin_total + final_angle_readings[read_index];
//     // advance to the next position in the array
//     read_index++;

//     // if we're at the end of the array...
//     if (read_index >= NUM_READINGS) {
//         // ...wrap around to the beginning
//         read_index = 0;
//     }

//     // calculate the average
//     hei_average = (hei_total / NUM_READINGS) * 100;
//     dis_average = (dis_total / NUM_READINGS) * 100;
//     ini_average = ini_total / NUM_READINGS;
//     fin_average = fin_total / NUM_READINGS;

//     // debug
//     Serial.print("Individual Readings:");
//     for (int i = 0; i < NUM_READINGS; i++) {
//         Serial.print(" (");
//         Serial.print(distance_readings[i]);
//         Serial.print(", ");
//         Serial.print(height_readings[i]);
//         Serial.print(", ");
//         Serial.print(initial_angle_readings[i]);
//         Serial.print(", ");
//         Serial.print(final_angle_readings[i]);
//         Serial.print(")");
//     }
//     Serial.println();

//     Serial.print("Averaged Values: ");
//     Serial.print("ch_average:");
//     Serial.print(hei_average);

//     Serial.print(" dis_average:");
//     Serial.print(dis_average);

//     Serial.print(" ini_average:");
//     Serial.print(ini_average);

//     Serial.print(" ini_sign:");
//     Serial.print(sign_initial_angle);

//     Serial.print(" fin_averages:");
//     Serial.print(fin_average);

//     Serial.print(" fin_sign:");
//     Serial.println(sign_final_angle);
// }

//======================================================================

// void set_curb_climb_automation_parameters_real_sense() {
//     // Update task number/order
//     task = 7;

//     // first check the readings from real-sense to see if we can climb the
//     curb directly if (dis_average <= (0.2 * 100) && fin_average - ini_average
//     <= 4) {
//         bypass_3_step = true;
//     }

//     // assign the averaged readings from real-sense to parameters used by the
//     3-step process c_h = hei_average * 0.3937;  // convert centimeter into
//     inch distance_to_travel = dis_average; initial_angle_to_turn =
//     ini_average;
//     //  initial_angle_to_turn = ini_angle;
//     final_angle_to_turn = fin_average;
//     dis_two_third = distance_to_travel / 3 * 2;
// }

//======================================================================

// void check_input_source(String source) {
//     if (test_with_real_sense == "1") {
//         if (PI_MOTORS.available() > 0) {
//             // read the string data from the real-sense
//             input = PI_MOTORS.readStringUntil('\n');
//         }
//     } else {
//         // use hardcoded string data
//         input = TEST_INPUT_CURB_150CM_L45D;
//     }
// }

// void turn_right(float imu_start, float angle_in_degree)
//  {
//      IMU_diff = IMU.yaw - imu_start;
//      if (IMU_diff < 0)
//      {
//          IMU_diff = IMU_diff + 360.00;
//      }

//     js.y = RIGHT_TURN_JOY_Y;
//     if (fabs(IMU_diff) <= fabs(angle_in_degree / 2))
//     {
//         js.x = RIGHT_TURN_JOY_X;
//     }
//     else if (fabs(IMU_diff) > fabs(angle_in_degree / 2) && fabs(IMU_diff) <
//     fabs(angle_in_degree))
//     {
//         if (js.x > SLOW_RIGHT_TURN_JOY_X)
//         {
//             js.x -= JOY_X_DELTA;
//         }
//         else
//         {
//             js.x = SLOW_RIGHT_TURN_JOY_X;
//         }
//     }
// }

// void turn_left(float imu_start, float angle_in_degree)
// {
//     IMU_diff = IMU.yaw - imu_start;
//     if (IMU_diff > 0)
//     {
//         IMU_diff = IMU_diff - 360.00;
//     }

//     js.y = LEFT_TURN_JOY_Y;
//     if (fabs(IMU_diff) <= fabs(angle_in_degree / 2))
//     {
//         js.x = LEFT_TURN_JOY_X;
//     }
//     else if (fabs(IMU_diff) > fabs(angle_in_degree / 2) && fabs(IMU_diff) <
//     fabs(angle_in_degree))
//     {
//         if (js.x < SLOW_LEFT_TURN_JOY_X)
//         {
//             js.x += JOY_X_DELTA;
//         }
//         else
//         {
//             js.x = SLOW_LEFT_TURN_JOY_X;
//         }
//     }
// }

// void execute_initial_turn_real_sense()
// {
//     yaw_initial_turn = IMU.yaw;
//     // if IMU.yaw is not within the range of  degree of the target, keep
//     turning if (IMU.yaw > initial_angle_to_stop + 1 || IMU.yaw <
//     initial_angle_to_stop - 1)
//     {
//         // check_which_way_to_turn();
//         if (sign_initial_angle == "0")
//         { // turning right
//             turn_right(IMUyaw_initial, initial_angle_to_turn);
//         }
//         else
//         { // turning left
//             turn_left(IMUyaw_initial, initial_angle_to_turn);
//         }
//     }
//     else
//     {
//         yaw_after_initial_turn = IMU.yaw;
//         init_posr = MR.wheel_pos;
//         js.x = 0;
//         js.y = 0;
//         IMU_diff = 0.0;
//         done_initial_turn = true;
//     }
// }

// void execute_final_turn_real_sense()
// {
//     yaw_final_turn = IMU.yaw;
//     // if IMU.yaw is not within the range of  degree of the target, keep
//     turning if (IMU.yaw > final_angle_to_stop + 1 || IMU.yaw <
//     final_angle_to_stop - 1)
//     {
//         // check_which_way_to_turn();
//         if (sign_final_angle == "0")
//         { // turning right
//             turn_right(yaw_after_travel, final_angle_to_turn);
//         }
//         else
//         { // turning left
//             turn_left(yaw_after_travel, final_angle_to_turn);
//         }
//     }
//     else
//     {
//         yaw_after_final_turn = IMU.yaw;
//         js.x = 0;
//         js.y = 0;
//         done_final_turn = true;
//     }
// }

// void speed_smoother()
// {
//     js.x = 0;
//     js.y = 60 * (distance_to_travel - distance_traveled) + 10; // was 80
//     if (js.y > 40)
//     {
//         js.y = 40;
//     }
//     if (js.y < 10)
//     {
//         js.y = 10;
//     }
// }

// void travel()
// {
//     if (!yaw_begin_travel_stored)
//     {
//         yaw_begin_travel = IMU.yaw;
//         yaw_begin_travel_stored = true;
//     }

//     yaw_travel = IMU.yaw;

//     distance_traveled = MR.wheel_pos - init_posr;
//     // stop when reaching the distance
//     if (distance_traveled <= distance_to_travel)
//     {
//         speed_smoother();
//     }
//     else
//     {
//         yaw_after_travel = IMU.yaw;
//         final_angle_to_stop = normalize_angle(yaw_begin_travel -
//         final_angle_to_turn); js.y = 0; js.x = 0; done_travel = true;
//     }
// }

// void give_sign_to_angles()
// {
//     // the sign of the angle follows the right hand rule
//     if (sign_initial_angle == "0")
//     {
//         initial_angle_to_turn = -1 * initial_angle_to_turn;
//     }

//     if (sign_final_angle == "0")
//     {
//         final_angle_to_turn = -1 * final_angle_to_turn;
//     }
// }
