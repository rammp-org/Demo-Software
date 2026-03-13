#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Caster.h>
#include <Timer.h>
#include <Wheel.h>

// Define states for the state machine
enum State { NEUTRAL, CARRIAGE_FORWARD, CARRIAGE_BACKWARD };

class MotorController {
public:
  Caster &RC;
  Caster &FC;
  Wheel &MR;
  Wheel &ML;
  Timer &timer;
  // Initialize state variable
  State currentState = NEUTRAL;

  bool set_new_height = false;
  bool &self_leveling_on;
  char &action;
  int &CA_flag;

  MotorController(Caster &, Caster &, Wheel &, Wheel &, Timer &, bool &, char &,
                  int &);

  // control_motor_with_desired_position helper function
  void set_wheels_height(float, float, float);
  void set_wheels_height(float, float, float, float);
  void move_each_wheel(float, float, float);
  // individual_motor_FF helper functions
  void ML_UP_1s();
  void ML_DOWN_1s();
  void RC_UP_1s();
  void RC_DOWN_1s();
  void MR_UP_1s();
  void MR_DOWN_1s();
  void FC_UP_1s();
  void FC_DOWN_1s();
  void LEFT_CARRIAGE_FORWARD_point4s();
  void LEFT_CARRIAGE_BACKWARD_point4s();
  void RIGHT_CARRIAGE_FORWARD_point4s();
  void RIGHT_CARRIAGE_BACKWARD_point4s();
  void BOTH_CARRIAGE_FORWARD_4s();
  void BOTH_CARRIAGE_BACKWARD_4s();
  void NO_MOVEMENT();
  void count_ticks(int);
  void set_positions_for_MWs_and_RC();
  void carriage_limits_switch();
  void set_motorpwm(int, int, int, int, int, int);
  void set_motordir(int, int, int, int, int, int);
  void individual_motor_PID_proportional();
  void individual_motor_PID();
  void normalize_acceleration();
  void manual_features_proportional();
  void manual_features_carriage();
  void manual_features();
};

#endif
