#include <MotorController.h>

MotorController::MotorController(Caster &rc, Caster &fc, Wheel &mr, Wheel &ml,
                                 Timer &timer_obj, bool &self_level_bool,
                                 char &act, int &ca_flag)
    : RC(rc), FC(fc), MR(mr), ML(ml), timer(timer_obj),
      self_leveling_on(self_level_bool), action(act), CA_flag(ca_flag){};

// control_motor_with_desired_position helper function
void MotorController::set_wheels_height(float mlh, float rch, float mrh) {
  if (mlh > 21.0) {
    mlh = 21.0;
  }
  if (rch > 21.0) {
    rch = 21.0;
  }
  if (mrh > 21.0) {
    mrh = 21.0;
  }

  if (mlh < 3.0) {
    mlh = 3.0;
  }
  if (rch < 1.0) {
    rch = 1.0;
  }
  if (mrh < 3.0) {
    mrh = 3.0;
  }

  move_each_wheel(mlh, rch, mrh);

  // if (!set_new_height) {
  //   move_each_wheel(mlh, rch, mrh);
  //   set_new_height = true;
  //   action = '0';
  // }
}

void MotorController::set_wheels_height(float mlh, float rch, float mrh,
                                        float fch) {
  set_wheels_height(mlh, rch, mrh);

  if (fch < 2.0) {
    fch = 2.0;
  } else if (FC.des > 15.0) {
    fch = 15.0;
  }

  FC.des = fch;
}

void MotorController::move_each_wheel(float height_ML, float height_RC,
                                      float height_MR) {
  RC.des = height_RC;
  ML.des = height_ML;
  MR.des = height_MR;
  //  FC.des = FC.pos;
}

// individual_motor_FF helper functions
void MotorController::ML_UP_1s() {
  count_ticks(100);
  set_motorpwm(0, 100, 0, 0, 0, 0);
  set_motordir(0, 1, 0, 0, 0, 0);
}

void MotorController::ML_DOWN_1s() {
  count_ticks(100);
  set_motorpwm(0, 100, 0, 0, 0, 0);
  set_motordir(0, 0, 0, 0, 0, 0);
}

void MotorController::RC_UP_1s() {
  count_ticks(100);
  set_motorpwm(0, 0, 0, 100, 0, 0);
  set_motordir(0, 0, 0, 0, 0, 0);
}

void MotorController::RC_DOWN_1s() {
  count_ticks(100);
  set_motorpwm(0, 0, 0, 100, 0, 0);
  set_motordir(0, 0, 0, 1, 0, 0);
}

void MotorController::MR_UP_1s() {
  count_ticks(100);
  set_motorpwm(0, 0, 100, 0, 0, 0);
  set_motordir(0, 0, 1, 0, 0, 0);
}

void MotorController::MR_DOWN_1s() {
  count_ticks(100);
  set_motorpwm(0, 0, 100, 0, 0, 0);
  set_motordir(0, 0, 0, 0, 0, 0);
}

void MotorController::FC_UP_1s() {
  count_ticks(100);
  set_motorpwm(100, 0, 0, 0, 0, 0);
  set_motordir(1, 0, 0, 0, 0, 0);
}

void MotorController::FC_DOWN_1s() {
  count_ticks(100);
  set_motorpwm(100, 0, 0, 0, 0, 0);
  set_motordir(0, 0, 0, 0, 0, 0);
}

void MotorController::LEFT_CARRIAGE_FORWARD_point4s() {
  count_ticks(300);
  set_motorpwm(0, 0, 0, 0, 100, 0);
  set_motordir(0, 0, 0, 0, 1, 0);
}

void MotorController::LEFT_CARRIAGE_BACKWARD_point4s() {
  count_ticks(300);
  set_motorpwm(0, 0, 0, 0, 100, 0);
  set_motordir(0, 0, 0, 0, 0, 0);
  // Serial.println("RELAY1 DOWN");
}

void MotorController::RIGHT_CARRIAGE_FORWARD_point4s() {
  count_ticks(300);
  set_motorpwm(0, 0, 0, 0, 0, 100);
  set_motordir(0, 0, 0, 0, 0, 1);
}

void MotorController::RIGHT_CARRIAGE_BACKWARD_point4s() {
  count_ticks(300);
  set_motorpwm(0, 0, 0, 0, 0, 100);
  set_motordir(0, 0, 0, 0, 0, 0);
}

void MotorController::BOTH_CARRIAGE_FORWARD_4s() {
  count_ticks(300);
  set_motorpwm(0, 0, 0, 0, 100, 100);
  set_motordir(0, 0, 0, 0, 0, 1);
}

void MotorController::BOTH_CARRIAGE_BACKWARD_4s() {
  count_ticks(300);
  set_motorpwm(0, 0, 0, 0, 100, 100);
  set_motordir(0, 0, 0, 0, 1, 0);
}

void MotorController::NO_MOVEMENT() {
  set_motorpwm(0, 0, 0, 0, 0, 0);
  set_motordir(0, 0, 0, 0, 0, 0);
}

void MotorController::count_ticks(int num) {
  timer.counter = timer.counter + 1;
  if (timer.counter == num) {
    action = 'z';
    timer.counter = 0;
  }
}

void MotorController::set_positions_for_MWs_and_RC() {
  MR.carriage.move();
  ML.carriage.move();
  RC.move();
  MR.move();
  ML.move();
  FC.move();
  //  if (ML.eha<0.1 && ML.posf>4.5 && ML.err>0.3){ML.motor_PWM=0.0;}
}

void MotorController::carriage_limits_switch() {
  ML.carriage.limit_switch();
  MR.carriage.limit_switch();
}

void MotorController::set_motorpwm(int pwm_1, int pwm_2, int pwm_3, int pwm_4,
                                   int pwm_5, int pwm_6) {
  FC.motor_PWM = pwm_1;
  ML.motor_PWM = pwm_2;
  MR.motor_PWM = pwm_3;
  RC.motor_PWM = pwm_4;
  ML.carriage.motor_PWM = pwm_5;
  MR.carriage.motor_PWM = pwm_6;
}

void MotorController::set_motordir(int dir_1, int dir_2, int dir_3, int dir_4,
                                   int dir_5, int dir_6) {
  FC.motor_dir = dir_1;
  ML.motor_dir = dir_2;
  MR.motor_dir = dir_3;
  RC.motor_dir = dir_4;
  ML.carriage.motor_dir = dir_5;
  MR.carriage.motor_dir = dir_6;
}

void MotorController::individual_motor_PID_proportional() {
  // RC
  RC.err = RC.pos - RC.des;

  if (RC.err > 0.0) {
    RC.Kp = 25.0; // 20.0
    RC.Ki = 2.0;  // 1.8
  } else {
    RC.Kp = 25.0; // 25
    RC.Ki = 2.0;  // 1.5
  }

  if (RC.des_pre != RC.des && self_leveling_on == false) {
    RC.Kacc = 0;
  }
  // Error for Integrate
  RC.cum_err += fabs(RC.err) * timer.elapsed_time;
  // Error for Derivative
  RC.rate_err = fabs((RC.err - RC.last_err)) / timer.elapsed_time;
  RC.motor_PWM = RC.Kacc * (fabs(20 + RC.Kp * fabs(RC.err) +
                                 RC.Ki * RC.cum_err + RC.Kd * RC.rate_err));
  ///  }
  if (RC.err < 0.5 && RC.err > -0.7) {
    set_new_height = false;
  }

  // Change Direction
  if (RC.err > 0.2) {
    RC.motor_dir = 0;
  } else if (RC.err < -0.2) {
    RC.motor_dir = 1;
  } else {
    RC.cum_err = 0.0, RC.rate_err = 0.0;
    RC.motor_PWM = 0;
    RC.Kacc = 0;
  }

  // FC
  FC.err = FC.pos - FC.des;

  if (FC.err > 0.0) {
    FC.Kp = 25.0; // 20.0
    FC.Ki = 2.0;  // 1.8
  } else {
    FC.Kp = 25.0; // 17
    FC.Ki = 2.0;  // 1.5
  }

  if (FC.err > 0.3 && FC.eha <= 0.5) {
    FC.motor_PWM = 0;
    FC.Kacc = 0;
  } else if (FC.err < -0.3 && FC.eha >= 9.9) {
    FC.motor_PWM = 0;
    FC.Kacc = 0;
  } else {
    if (FC.des_pre != FC.des && self_leveling_on == false) {
      FC.Kacc = 0;
    }
    // Error for Integrate
    FC.cum_err += fabs(FC.err) * timer.elapsed_time;
    // Error for Derivative
    FC.rate_err = fabs((FC.err - FC.last_err)) / timer.elapsed_time;
    FC.motor_PWM = FC.Kacc * (fabs(20 + FC.Kp * fabs(FC.err) +
                                   FC.Ki * FC.cum_err + FC.Kd * FC.rate_err));
  }
  // Change Direction
  if (FC.err > 0.2) {
    FC.motor_dir = 1;
  }
  // else if (RC.err < -0.2 && RC.eha <= 10.00)
  else if (FC.err < -0.2) {
    FC.motor_dir = 0;
  } else {
    FC.cum_err = 0.0, FC.rate_err = 0.0;
    //    FC.motor_PWM = 0;
    FC.Kacc = 0;
  }

  ML.proportional_PID(timer.elapsed_time, self_leveling_on, set_new_height);
  MR.proportional_PID(timer.elapsed_time, self_leveling_on, set_new_height);

  MR.carriage.proportional_PID(timer.elapsed_time, self_leveling_on);

  if (MR.carriage.err > 0.2) {
    MR.carriage.motor_dir = 1;
  } else if (MR.carriage.err < -0.2) {
    MR.carriage.motor_dir = 0;
  } else {
    MR.carriage.cum_err = 0.0, MR.carriage.rate_err = 0.0;
    MR.carriage.motor_PWM = 0;
    MR.carriage.Kacc = 0;
  }

  ML.carriage.proportional_PID(timer.elapsed_time, self_leveling_on);

  if (ML.carriage.err > 0.2) {
    ML.carriage.motor_dir = 0;
  } else if (ML.carriage.err < -0.2) {
    ML.carriage.motor_dir = 1;
  } else {
    ML.carriage.cum_err = 0.0, ML.carriage.rate_err = 0.0;
    ML.carriage.motor_PWM = 0;
    ML.carriage.Kacc = 0;
  }

  MR.carriage.Kacc += 0.005;
  ML.carriage.Kacc += 0.005;
  RC.Kacc += 0.003; // 0.05
  MR.Kacc += 0.003; // 0.05
  ML.Kacc += 0.003; // 0.05
  FC.Kacc += 0.003; // 0.05

  normalize_acceleration();
}

void MotorController::individual_motor_PID() {
  // RC
  RC.err = RC.pos - RC.des;

  if (RC.err > 0.0) {
    RC.Kp = 25.0; // 20.0
    RC.Ki = 2.0;  // 1.8
  } else {
    RC.Kp = 25.0; // 25
    RC.Ki = 2.0;  // 1.5
  }

  // if (RC.err > 0.3 && RC.eha <= 0.3)
  // {
  //   RC.motor_PWM = 0;
  //   RC.Kacc = 0;
  // }
  // else if (RC.err < -0.3 && RC.eha >= 9.9)
  // {
  //   RC.motor_PWM = 0;
  //   RC.Kacc = 0;
  // }
  //  if (RC.err > 0.3)
  //  {
  //    RC.motor_PWM = 0;
  //    RC.Kacc = 0;
  //  }
  //  else if (RC.err < -0.3)
  //  {
  //    RC.motor_PWM = 0;
  //    RC.Kacc = 0;
  //  }
  //  else
  //  {
  if (RC.des_pre != RC.des && self_leveling_on == false) {
    RC.Kacc = 0;
  }
  // Error for Integrate
  RC.cum_err += fabs(RC.err) * timer.elapsed_time;
  // Error for Derivative
  RC.rate_err = fabs((RC.err - RC.last_err)) / timer.elapsed_time;
  RC.motor_PWM = RC.Kacc * (fabs(20 + RC.Kp * fabs(RC.err) +
                                 RC.Ki * RC.cum_err + RC.Kd * RC.rate_err));
  ///  }
  if (RC.err < 0.5 && RC.err > -0.7) {
    set_new_height = false;
  }

  // Change Direction
  if (RC.err > 0.2) {
    RC.motor_dir = 0;
  } else if (RC.err < -0.2) {
    RC.motor_dir = 1;
  } else {
    RC.cum_err = 0.0, RC.rate_err = 0.0;
    RC.motor_PWM = 0;
    RC.Kacc = 0;
  }

  // FC
  FC.err = FC.pos - FC.des;

  if (FC.err > 0.0) {
    FC.Kp = 25.0; // 20.0
    FC.Ki = 2.0;  // 1.8
  } else {
    FC.Kp = 25.0; // 17
    FC.Ki = 2.0;  // 1.5
  }

  if (FC.err > 0.3 && FC.eha <= 0.5) {
    FC.motor_PWM = 0;
    FC.Kacc = 0;
  } else if (FC.err < -0.3 && FC.eha >= 9.9) {
    FC.motor_PWM = 0;
    FC.Kacc = 0;
  } else {
    if (FC.des_pre != FC.des && self_leveling_on == false) {
      FC.Kacc = 0;
    }
    // Error for Integrate
    FC.cum_err += fabs(FC.err) * timer.elapsed_time;
    // Error for Derivative
    FC.rate_err = fabs((FC.err - FC.last_err)) / timer.elapsed_time;
    FC.motor_PWM = FC.Kacc * (fabs(20 + FC.Kp * fabs(FC.err) +
                                   FC.Ki * FC.cum_err + FC.Kd * FC.rate_err));
  }
  // Change Direction
  if (FC.err > 0.2) {
    FC.motor_dir = 1;
  }
  // else if (RC.err < -0.2 && RC.eha <= 10.00)
  else if (FC.err < -0.2) {
    FC.motor_dir = 0;
  } else {
    FC.cum_err = 0.0, FC.rate_err = 0.0;
    //    FC.motor_PWM = 0;
    FC.Kacc = 0;
  }

  // ML
  ML.proportional_PID(timer.elapsed_time, self_leveling_on, set_new_height);

  // MR
  MR.proportional_PID(timer.elapsed_time, self_leveling_on, set_new_height);

  // MRcarriage
  MR.carriage.proportional_PID(timer.elapsed_time, self_leveling_on);

  if (MR.carriage.err > 0.2) {
    MR.carriage.motor_dir = 1;
  } else if (MR.carriage.err < -0.2) {
    MR.carriage.motor_dir = 0;
  } else {
    MR.carriage.cum_err = 0.0, MR.carriage.rate_err = 0.0;
    MR.carriage.motor_PWM = 0;
    MR.carriage.Kacc = 0;
  }

  // MLcarriage
  ML.carriage.proportional_PID(timer.elapsed_time, self_leveling_on);

  if (ML.carriage.err > 0.2) {
    ML.carriage.motor_dir = 0;
  } else if (ML.carriage.err < -0.2) {
    ML.carriage.motor_dir = 1;
  } else {
    ML.carriage.cum_err = 0.0, ML.carriage.rate_err = 0.0;
    ML.carriage.motor_PWM = 0;
    ML.carriage.Kacc = 0;
  }

  MR.carriage.Kacc += 0.01;
  ML.carriage.Kacc += 0.01;
  RC.Kacc += 0.01; // 0.05
  MR.Kacc += 0.01; // 0.05
  ML.Kacc += 0.01; // 0.05
  FC.Kacc += 0.01; // 0.05

  normalize_acceleration();
}

void MotorController::normalize_acceleration() {
  if (MR.Kacc >= 1.0) {
    MR.Kacc = 1.0;
  }
  if (ML.Kacc >= 1.0) {
    ML.Kacc = 1.0;
  }
  if (RC.Kacc >= 1.0) {
    RC.Kacc = 1.0;
  }
  if (ML.carriage.Kacc >= 1.0) {
    ML.carriage.Kacc = 1.0;
  }
  if (MR.carriage.Kacc >= 1.0) {
    MR.carriage.Kacc = 1.0;
  }
}

void MotorController::manual_features_proportional() {
  // Serial.println("I'm Here in Manual Features Proportional");
  CA_flag = 1;

  switch (action) {
  case 'q':
    // Serial.println("Elevate");
    set_motorpwm(0, 100, 100, 100, 0, 0);
    set_motordir(0, 0, 0, 1, 0, 0);
    ML.carriage.des = 0.1;
    MR.carriage.des = 0.1;
    set_wheels_height(20, 15, 20, 15);
    break;
  case 'a':
    // Serial.println("Lower seat");
    set_motorpwm(0, 100, 100, 100, 0, 0);
    set_motordir(0, 1, 1, 0, 0, 0);
    ML.carriage.des = 0.1;
    MR.carriage.des = 0.1;
    set_wheels_height(4, 1, 4, 2);
    break;
  case 'w':
    // Serial.println("Tilt Back");
    set_motorpwm(100, 100, 100, 100, 0, 0);
    set_motordir(0, 0, 0, 0, 0, 0);
    ML.carriage.des = 0.1;
    MR.carriage.des = 0.1;
    set_wheels_height(19, 1, 19, 2);
    break;
  case 's':
    // Serial.println("Tilt Forward");
    set_motorpwm(100, 100, 100, 100, 0, 0);
    set_motordir(1, 1, 1, 1, 0, 0);
    ML.carriage.des = 0.1;
    MR.carriage.des = 0.1;
    set_wheels_height(4, 15, 4, 2);
    break;
  case 'r':
    // Serial.println("Tilt Left");
    set_motorpwm(0, 100, 0, 100, 0, 0);
    set_motordir(0, 0, 0, 1, 0, 0);
    ML.carriage.des = 0.1;
    MR.carriage.des = 0.1;
    set_wheels_height(4, 1, 19, 2);
    break;
  case 'f':
    // Serial.println("Tilt Right");
    set_motorpwm(0, 0, 100, 100, 0, 0);
    set_motordir(0, 0, 0, 1, 0, 0);
    ML.carriage.des = 0.1;
    MR.carriage.des = 0.1;
    set_wheels_height(19, 1, 4, 2);
    break;
  case 'z':
    // Serial.println("Neutral");
    ML.carriage.des = ML.carriage.pos;
    MR.carriage.des = MR.carriage.pos;
    set_wheels_height(ML.pos, RC.pos, MR.pos, FC.pos);
    // currentState = NEUTRAL; // Reset state to NEUTRAL
    break;
  }

  // move
  individual_motor_PID_proportional();
}

void MotorController::manual_features_carriage() {
  // Serial.println("I'm Here in Manual Features -> Carriage");
  CA_flag = 1;
  switch (action) {
  case 'u':
    // Serial.println("Carriage Forward");
    if (currentState != CARRIAGE_FORWARD) {
      // Relocate Drive wheels forward
      set_wheels_height(4.0, 2.0, 4.0);
      ML.carriage.des -= 15.0;
      MR.carriage.des -= 15.0;
      FC.des = 2.0;
      if (ML.carriage.des < 0.1) {
        ML.carriage.des = 0.05;
      } else if (ML.carriage.des > 30.0) {
        ML.carriage.des = 30.0;
      }
      if (MR.carriage.des < 0.1) {
        MR.carriage.des = 0.05;
      } else if (MR.carriage.des > 30.0) {
        MR.carriage.des = 30.0;
      }
      currentState = CARRIAGE_FORWARD;
    }
    break;
  case 'j':
    // Serial.println("Carriage Backward");
    if (currentState != CARRIAGE_BACKWARD) {
      // Relocate Drive wheels back
      set_wheels_height(4.0, 2.0, 4.0);
      ML.carriage.des += 15.0;
      MR.carriage.des += 15.0;
      FC.des = 2.0;
      if (ML.carriage.des < 0.05) {
        ML.carriage.des = 0.05;
      } else if (ML.carriage.des > 30.0) {
        ML.carriage.des = 30.0;
      }
      if (MR.carriage.des < 0.1) {
        MR.carriage.des = 0.05;
      } else if (MR.carriage.des > 30.0) {
        MR.carriage.des = 30.0;
      }
      currentState = CARRIAGE_BACKWARD;
    }
    break;
  case 'z':
    // STOP any movement
    NO_MOVEMENT();
    currentState = NEUTRAL; // Reset state to NEUTRAL
    break;
  }
  // move
  individual_motor_PID_proportional();

  if (MR.carriage.motor_PWM == 0 && ML.carriage.motor_PWM == 0) {
    action = 'z';
  }
}

void MotorController::manual_features() {
  // Serial.println("I'm Here in Manual Features");
  CA_flag = 1;

  // Elevate chair
  if (action == 'q') {
    ML.carriage.des = 0.1;
    MR.carriage.des = 0.1;
    set_wheels_height(ML.pos + 5.0, ML.pos + 5.0, ML.pos + 5.0, FC.des + 5.0);
  }
  // Lower chair
  else if (action == 'a') {
    ML.carriage.des = 0.1;
    MR.carriage.des = 0.1;
    set_wheels_height(ML.pos - 5.0, ML.pos - 5.0, ML.pos - 5.0, FC.des - 5.0);
  }
  // Tilt back
  else if (action == 'w') {
    ML.carriage.des = 0.1;
    MR.carriage.des = 0.1;
    set_wheels_height(ML.pos + 5.0, RC.pos - 5.0, ML.pos + 5.0, 2.0);
  }
  // Tilt forward
  else if (action == 's') {
    ML.carriage.des = 0.1;
    MR.carriage.des = 0.1;
    set_wheels_height(ML.pos - 5.0, RC.pos + 5.0, ML.pos - 5.0, 2.0);
  }
  // Tilt left
  else if (action == 'r') {
    ML.carriage.des = 0.1;
    MR.carriage.des = 0.1;
    set_wheels_height(ML.pos - 4.0, RC.pos - 2.0, MR.pos + 4.0, 2.0);
  }
  // Tilt right
  else if (action == 'f') {
    ML.carriage.des = 0.1;
    MR.carriage.des = 0.1;
    set_wheels_height(ML.pos + 4.0, RC.pos - 2.0, MR.pos - 4.0, 2.0);
  }
  // Relocate Drive wheels forward
  else if (action == 'u') {
    set_wheels_height(4.0, 4.0, 4.0);
    ML.carriage.des -= 15.0;
    MR.carriage.des -= 15.0;
    if (ML.carriage.des < 0.1) {
      ML.carriage.des = 0.05;
    } else if (ML.carriage.des > 30.0) {
      ML.carriage.des = 30.0;
    }
    if (MR.carriage.des < 0.1) {
      MR.carriage.des = 0.05;
    } else if (MR.carriage.des > 30.0) {
      MR.carriage.des = 30.0;
    }
    if (ML.carriage.des >= 10) {
      FC.des = 0.0;
    } else {
      FC.des = -2.0;
    }
  }
  // Relocate Drive wheels back
  else if (action == 'j') {
    set_wheels_height(4.0, 4.0, 4.0);
    ML.carriage.des += 15.0;
    MR.carriage.des += 15.0;
    if (ML.carriage.des < 0.05) {
      ML.carriage.des = 0.05;
    } else if (ML.carriage.des > 30.0) {
      ML.carriage.des = 30.0;
    }
    if (MR.carriage.des < 0.1) {
      MR.carriage.des = 0.05;
    } else if (MR.carriage.des > 30.0) {
      MR.carriage.des = 30.0;
    }
    if (ML.carriage.des >= 10) {
      FC.des = 0.0;
    } else {
      FC.des = -2.0;
    }
  }
  // STOP any movement
  else if (action == 'z') {
    NO_MOVEMENT();
  }

  // move
  individual_motor_PID();
}
