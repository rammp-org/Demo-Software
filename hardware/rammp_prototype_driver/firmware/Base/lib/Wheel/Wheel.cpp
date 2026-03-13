#include <Arduino.h>
#include <Wheel.h>

Wheel::Wheel(int lc_pin, MotorID motor_id, Carriage &carriageRef,
             signed long &encoder1, signed long &encoder2, bool fwd_is_positive)
    : Component(lc_pin, motor_id, fwd_is_positive), encoder_val_1(encoder1),
      encoder_val_2(encoder2), carriage(carriageRef){};

void Wheel::calculate_main_wheels_positions(float &elapsed_time) {
  // Height: DWs: 21cm, RC: 20cm, FC: 23cm (off ground)
  // GC: DWs: 3cm, RC: 1cm, FC: 5cm (off ground)

  /* Calculation w.r.t bottom of the base */
  pos = 2.5 +
        33.0 * sin((float)encoder_val_1 * 33.0 / pos_ticks / DG); //  2.5-21.0
  angle = (float)encoder_val_1 * 33.0 / angle_ticks;
  eha = fabs(encoder_val_1) / eha_norm; // actuator stroke 0 - 10.15cm

  /* Load cell calculation */
  // weight = 0.68 * loadcell + 0.74 * angle - 285.19;

  // Calculation DW speed/linear position
  wheel_pos =
      -34.3 * 3.14 * (float)encoder_val_2 /
      wheel_pos_ticks; // 1 rev = 13.5" =34.3 cm = 15212 ticks (forward +)

  speed_drive = (wheel_pos - wheel_pos_pre) / (100 * elapsed_time); // m/s
  speed_drivef = speed_drivef + Kdrive * (speed_drive - speed_drivef);
};

void Wheel::proportional_PID(float &elapsed_time, bool &self_leveling_on,
                             bool &set_new_height) {
  // ML
  err = pos - des;

  if (err > 0.0) {
    Kp = 25.0; // 5.0
    Ki = 1.0;  // 0.01
  } else {
    Kp = 25.0; // 20.0
    Ki = 1.0;  // 1.0
  }

  if (err > 0.2 && eha <= 0.3) {
    motor_PWM = 0;
    Kacc = 0;
  } else if (err < -0.2 && eha >= 9.8) {
    motor_PWM = 0;
    Kacc = 0;
  } else {
    if (des_pre != des && self_leveling_on == false) {
      Kacc = 0;
    }
    // Error for Integrate
    cum_err += fabs(err) * elapsed_time;
    // Error for Derivative
    rate_err = fabs((err - last_err)) / elapsed_time;
    motor_PWM = Kacc * fabs(20 + Kp * fabs(err) + Ki * cum_err + Kd * rate_err);
  }

  // Change Direction
  if (err > 0.2) {
    motor_dir = 1;
  } else if (err < -0.2) {
    motor_dir = 0;
  } else {
    cum_err = 0.0, rate_err = 0.0;
    motor_PWM = 0;
    Kacc = 0;
  }
  if (err < 0.7 && err > -0.7) {
    set_new_height = false;
  }
}
