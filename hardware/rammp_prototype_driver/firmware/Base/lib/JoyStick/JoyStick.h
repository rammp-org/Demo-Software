#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <Arduino.h>

class JoyStick {
public:
  int speed_up = 1, speed_down = 1, profile = 1;
  int speed_counter = 3, profile_counter = 2;
  int profile_pre = 1, speed_up_pre = 1, speed_down_pre = 1;
  int x = 0, y = 0;

  void retrieve_omni2_iom_readings();
  // manual_curb_climb helper functions
  void set_joystick_speed(int, int);
};

#endif
