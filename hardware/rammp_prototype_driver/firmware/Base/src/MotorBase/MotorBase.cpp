#include "MotorBase.h"

MotorBase::MotorBase()
    : pos_pid(0.0f, 0.0f, 0.0f, 0.0f, -10000.0f, 10000.0f, 1),
      vel_pid(0.0f, 0.0f, 0.0f, 0.0f, -1.0f, 1.0f, 10000) {}

float MotorBase::update(float dt) {
  if (dt <= 0.0f)
    return 0.0f;

  switch (mode) {
  case DISABLED:
    target_pwm = 0.0f;
    return 0.0f;

  case POSITION_CONTROL:
    if (limits_enabled) {
      if (target_pos < pos_limit_min)
        target_pos = pos_limit_min;
      else if (target_pos > pos_limit_max)
        target_pos = pos_limit_max;
    }
    target_vel = pos_pid.compute(target_pos, current_pos, dt);
    // Fallthrough to velocity control

  case VELOCITY_CONTROL:
    target_pwm = vel_pid.compute(target_vel, current_vel, dt);
    // Fallthrough to PWM output

  case OPEN_LOOP:
  default:
    if (limits_enabled) {
      if (current_pos <= pos_limit_min && target_pwm < 0) {
        target_pwm = 0.0f;
        vel_pid.reset();
      } else if (current_pos >= pos_limit_max && target_pwm > 0) {
        target_pwm = 0.0f;
        vel_pid.reset();
      }
    }

    // Apply direction multiplier to final PWM output
    scaled_target_pwm = (int16_t)constrain(
        target_pwm * direction * this->PWM_SCALE, -32767, 32767);
    return scaled_target_pwm;
  }
}

void MotorBase::initPIDs(float p_kp, float p_ki, float p_kd, float p_min,
                         float p_max, float v_kp, float v_ki, float v_kd,
                         float v_min, float v_max) {
  pos_pid.setGains(p_kp, p_ki, p_kd);
  pos_pid.setOutputLimits(p_min, p_max);
  vel_pid.setGains(v_kp, v_ki, v_kd);
  vel_pid.setOutputLimits(v_min, v_max);
}

void MotorBase::setTargetPosition(float pos) { target_pos = pos; }
void MotorBase::setTargetVelocity(float vel) { target_vel = vel; }
void MotorBase::setTargetPWM(float pwm) { target_pwm = pwm; }

void MotorBase::attachStrainGauge(StrainGauge *sg) { _strain_gauge = sg; }

void MotorBase::updateLoad() {
  if (_strain_gauge != nullptr) {
    current_load = _strain_gauge->getValue();
  }
}

void MotorBase::setDirection(int8_t dir) { direction = (dir >= 0) ? 1 : -1; }
void MotorBase::toggleDirection() { direction = -direction; }
int8_t MotorBase::getDirection() const { return direction; }

void MotorBase::setEncoderDirection(int8_t dir) {
  encoder_dir = (dir >= 0) ? 1 : -1;
}
void MotorBase::toggleEncoderDirection() { encoder_dir = -encoder_dir; }
int8_t MotorBase::getEncoderDirection() const { return encoder_dir; }

void MotorBase::setInputLpfAlpha(float alpha) {
  if (alpha < 0.0f)
    alpha = 0.0f;
  if (alpha > 1.0f)
    alpha = 1.0f;
  lpf_input_alpha = alpha;
}

void MotorBase::updateLimits(int32_t min, int32_t max) {
  pos_limit_min = min;
  pos_limit_max = max;
  limits_enabled = (min != max);
}

void MotorBase::setOrigin() {}
