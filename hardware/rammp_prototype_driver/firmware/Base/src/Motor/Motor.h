#ifndef MOTOR_H
#define MOTOR_H

#include "../PIDController/PIDController.h"

class Motor {
public:
    enum ControlMode {
        DISABLED,
        OPEN_LOOP,
        VELOCITY_CONTROL,
        POSITION_CONTROL
    };

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
    void updateSensorData(float current_pos, float current_vel);

    // Compute cascaded control, returns PWM required
    float update(float dt);

    float current_pos;
    float current_vel;
    float target_pos;
    float target_vel;
    float target_pwm;

    PIDController pos_pid;
    PIDController vel_pid;
    ControlMode mode;
};

#endif
