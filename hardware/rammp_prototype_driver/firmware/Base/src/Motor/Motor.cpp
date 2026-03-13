#include "Motor.h"

Motor::Motor() 
    : current_pos(0.0f), current_vel(0.0f), target_pos(0.0f), target_vel(0.0f), target_pwm(0.0f),
      pos_pid(0.0f, 0.0f, 0.0f, -1.0f, 1.0f), 
      vel_pid(0.0f, 0.0f, 0.0f, -1.0f, 1.0f), 
      mode(DISABLED) {}

void Motor::initPIDs(float p_kp, float p_ki, float p_kd, float p_min, float p_max,
                     float v_kp, float v_ki, float v_kd, float v_min, float v_max) {
    pos_pid.setGains(p_kp, p_ki, p_kd);
    pos_pid.setOutputLimits(p_min, p_max);
    
    vel_pid.setGains(v_kp, v_ki, v_kd);
    vel_pid.setOutputLimits(v_min, v_max);
}

void Motor::setMode(ControlMode new_mode) {
    if (this->mode != new_mode) {
        pos_pid.reset();
        vel_pid.reset();
    }
    this->mode = new_mode;
}

void Motor::disable() {
    target_pwm = 0.0f;
    target_vel = 0.0f;
    target_pos = current_pos; // Prevent jump when re-enabling
    setMode(DISABLED);
}

void Motor::setTargetPosition(float pos) {
    target_pos = pos;
}

void Motor::setTargetVelocity(float vel) {
    target_vel = vel;
}

void Motor::setTargetPWM(float pwm) {
    target_pwm = pwm;
}

void Motor::updateSensorData(float current_pos, float current_vel) {
    this->current_pos = current_pos;
    this->current_vel = current_vel;
}

float Motor::update(float dt) {
    if (dt <= 0.0f) return 0.0f;

    switch (mode) {
        case DISABLED:
            target_pwm = 0.0f;
            return 0.0f;

        case POSITION_CONTROL:
            // Output of position PID is target velocity
            target_vel = pos_pid.compute(target_pos, current_pos, dt);
            // Fallthrough to velocity control

        case VELOCITY_CONTROL:
            // Output of velocity PID is target PWM
            target_pwm = vel_pid.compute(target_vel, current_vel, dt);
            // Fallthrough to PWM output

        case OPEN_LOOP:
        default:
            return target_pwm;
    }
}
