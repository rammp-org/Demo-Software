#include <Constants.h>
#include <JoyStick.h>

void JoyStick::retrieve_omni2_iom_readings() {
  // Test Omni2 direction of wheels for CC
  //  omni2test();
  // test joystick input/output signals for SL: read and write joystick buttons
  //  joystickiotest();

  // Read power button to record last profile and inhibit reading profile button
  // or speed down/up buttons R-net configuration needs to be changed to start
  // in Profile 1 (indoor), speed 3 We are assuming that the chair starts in
  // this profile and is never turn off

  profile = digitalRead(JS_PFILE_PIN);
  speed_down = digitalRead(JS_SD_PIN);
  speed_up = digitalRead(JS_SUP_PIN);

  // keep track of the selected joystick profile, speed up and down
  if (profile != profile_pre && profile == 1) {
    profile_counter = profile_counter + 1;
    delay(150);
  }
  profile_pre = profile;
  if (profile_counter == 5) {
    profile_counter = 1;
  }

  if (profile_counter <= 2) {
    if (speed_down != speed_down_pre && speed_down == 1) {
      speed_counter = speed_counter - 1;
      delay(150);
    }
    speed_down_pre = speed_down;

    if (speed_up != speed_up_pre && speed_up == 1) {
      speed_counter = speed_counter + 1;
      delay(150);
    }
    speed_up_pre = speed_up;

    if (speed_counter <= 1) {
      speed_counter = 1;
    }
    if (speed_counter >= 5) {
      speed_counter = 5;
    }
  } else {
  }
}

// manual_curb_climb helper functions
void JoyStick::set_joystick_speed(int new_x, int new_y) {
  x = new_x;
  y = new_y;
}
