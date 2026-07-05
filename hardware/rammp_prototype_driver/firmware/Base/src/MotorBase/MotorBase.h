#ifndef MOTOR_BASE_H
#define MOTOR_BASE_H

#include "../PIDController/PIDController.h"
#include "../StrainGauge/StrainGauge.h"
#include <Arduino.h>

class MotorBase {
public:
  enum ControlMode { DISABLED, OPEN_LOOP, VELOCITY_CONTROL, POSITION_CONTROL };

  MotorBase();
  virtual ~MotorBase() = default;

  // --- Pure virtual: subclasses MUST define ---
  virtual void setMode(ControlMode mode) = 0;
  virtual void disable() = 0;
  virtual void updateSensorData(float current_pos, float dt) = 0;

  // --- Shared implementations ---
  virtual float update(float dt);

  virtual void initPIDs(float p_kp, float p_ki, float p_kd, float p_min,
                        float p_max, float v_kp, float v_ki, float v_kd,
                        float v_min, float v_max);

  virtual void setTargetPosition(float target_pos);
  virtual void setTargetVelocity(float target_vel);
  virtual void setTargetPWM(float target_pwm);

  virtual void attachStrainGauge(StrainGauge *sg);
  virtual void updateLoad();

  virtual void setDirection(int8_t dir);
  virtual void toggleDirection();
  virtual int8_t getDirection() const;

  virtual void setEncoderDirection(int8_t dir);
  virtual void toggleEncoderDirection();
  virtual int8_t getEncoderDirection() const;

  virtual void setInputLpfAlpha(float alpha);
  virtual void updateLimits(int32_t min, int32_t max);
  virtual void setOrigin();

  // --- State (public for direct access from control loops) ---
  float current_pos = 0.0f;
  float current_vel = 0.0f;
  float prev_pos = 0.0f;
  float prev_vel = 0.0f;
  float target_pos = 0.0f;
  float target_vel = 0.0f;
  float target_pwm = 0.0f;
  float scaled_target_pwm = 0.0f;
  float current_load = 0.0f;
  float lpf_input_alpha = 0.5f;

  static constexpr float PWM_SCALE = 32767.0f;

  int8_t direction = 1;
  int8_t encoder_dir = 1;

  int32_t pos_limit_min = 0;
  int32_t pos_limit_max = 0;
  bool limits_enabled = false;

  PIDController pos_pid;
  PIDController vel_pid;
  ControlMode mode = DISABLED;

protected:
  StrainGauge *_strain_gauge = nullptr;
};

#endif
