#ifndef MOTOR_H
#define MOTOR_H

#include "../PIDController/PIDController.h"
#include "../StrainGauge/StrainGauge.h"
#include <Arduino.h>

class Motor {
public:
  enum ControlMode { DISABLED, OPEN_LOOP, VELOCITY_CONTROL, POSITION_CONTROL };

  Motor();

  // Initialize with PIDs
  void initPIDs(float p_kp, float p_ki, float p_kd, float p_min, float p_max,
                float v_kp, float v_ki, float v_kd, float v_min, float v_max);

  void setMode(ControlMode mode);

  // Disable motor (sets to DISABLED mode, zeros PWM)
  void disable();

  // Set target state
  void setTargetPosition(float target_pos);
  void setTargetVelocity(float target_vel);
  void setTargetPWM(float target_pwm);

  // Provide sensory feedback
  void updateSensorData(float current_pos, float dt);

  void attachStrainGauge(StrainGauge *sg);
  void updateLoad();

  // Compute cascaded control, returns PWM required
  float update(float dt);

  // Motor direction control
  void setDirection(int8_t dir);
  void toggleDirection();
  int8_t getDirection() const;

  float current_pos;
  float current_vel;
  float prev_pos;
  float prev_vel;
  float target_pos;
  float target_vel;
  float target_pwm;
  float scaled_target_pwm;
  float current_load = 0.0f;
  float lpf_input_alpha = 0.5f;

  void setInputLpfAlpha(float alpha);

  const float PWM_SCALE = 32767;

  // Motor direction: 1 = normal, -1 = inverted
  int8_t direction = 1;

  // Encoder direction: 1 = normal, -1 = inverted
  int8_t encoder_dir = 1;

  // Encoder direction control
  void setEncoderDirection(int8_t dir);
  void toggleEncoderDirection();
  int8_t getEncoderDirection() const;

  int32_t pos_limit_min = 0;
  int32_t pos_limit_max = 0;
  bool limits_enabled = false;
  void updateLimits(int32_t min, int32_t max);

  PIDController pos_pid;
  PIDController vel_pid;
  ControlMode mode;

private:
  StrainGauge *_strain_gauge = nullptr;
};

#endif
