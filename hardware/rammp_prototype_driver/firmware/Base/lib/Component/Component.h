#ifndef COMPONENT_H
#define COMPONENT_H

enum MotorID {
    MOTOR_RC = 0,
    MOTOR_FC = 1,
    MOTOR_ML = 2,
    MOTOR_MR = 3,
    MOTOR_ML_CARRIAGE = 4,
    MOTOR_MR_CARRIAGE = 5
};

class Component
{
public:
    const int LOADCELL_PIN;
    // const int PWM_PIN;
    // const int DIR_PIN;
    const MotorID MOTOR_ID;     // specify which motor this is
    // Member variables
    float pos = 0.0, err = 0.0, des = 3.0, eha = 0.0;
    float des_pre = 0.0;
    float weight = 0.0;
    float angle = 0.0;
    float motor_PWM = 0;
    int motor_dir = 0;
    float cum_err = 0.0;
    float rate_err = 0.0;
    float last_err = 0.0;
    float Kp = 10.0, Ki = 1.0, Kd = 0.8; // P = 10, I=1, D=0.5
    float Kacc = 0.0;
    float loadcell = 0.0;
    float lc = 0.0;

    Component(int, MotorID);
    virtual void initialize_pins();
    void move();
    void retrieve_lc_reading();
};

float normalize_value(float);

#endif