#ifndef WHEEL_H
#define WHEEL_H

#include <Carriage.h>
#include <Component.h>

class Wheel : public Component {
public:
  // Member Variables
  float wheel_pos = 0.0;
  float wheel_pos_pre = 0.0;
  float wheel_pos_init = 0.0;
  float wheel_traveled = 0.0;
  float speed_drivef = 0.0;
  float speed_drive = 0.0;

  // calculation stuff
  float Kdrive = 0.3;
  int pos_ticks = 1;
  int wheel_pos_ticks = 1;
  int angle_ticks = 1;
  float eha_norm = 1;
  signed long &encoder_val_1;
  signed long &encoder_val_2;

  Carriage &carriage;

  Wheel(int, MotorID, Carriage &, signed long &, signed long &, bool);
  void calculate_main_wheels_positions(float &);
  void proportional_PID(float &, bool &, bool &);
};

#endif
