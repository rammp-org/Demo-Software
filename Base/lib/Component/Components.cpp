#include <Arduino.h>
#include <Component.h>
#include <Constants.h>

Component::Component(int lc_pin, int motor_id) : LOADCELL_PIN(lc_pin), MOTOR_ID(motor_id) {};

void Component::initialize_pins() {
    // pinMode(DIR_PIN, OUTPUT);
    // analogWriteFrequency(PWM_PIN, 20000);
};

void Component::move() {
    motor_PWM = normalize_value(motor_PWM);

    int duty;
    if (motor_dir == 0) {
        duty = -int((motor_PWM * 32767) / 255);
    } else {
        duty = int((motor_PWM * 32767) / 255);
    }

    switch(MOTOR_ID) {
        case 0:
            roboclaw_casters.DutyM1(0x80, duty);
            break;
        case 1:
            roboclaw_casters.DutyM2(0x80, duty);
            break;
        case 2:
            roboclaw_main.DutyM1(0x80, duty);
            break;
        case 3:
            roboclaw_main.DutyM2(0x80, duty);
            break;
        case 4:
            roboclaw_carriages.DutyM1(0x80, duty);
            break;
        case 5:
            roboclaw_carriages.DutyM2(0x80, duty);
            break;
        default:
            break;
    }
};

// old move function
// void Component::move() {
//     motor_PWM = normalize_value(motor_PWM);
//     analogWrite(PWM_PIN, int(motor_PWM));
//     digitalWrite(DIR_PIN, motor_dir);
// };

void Component::retrieve_lc_reading() {
    if (LOADCELL_PIN == NO_PIN) {
        return;
    }

    lc = analogRead(LOADCELL_PIN);
    //  lc = 282.25 - 0.67 * analogRead(A9);
    loadcell = loadcell + 0.03 * (lc - loadcell);
};

float normalize_value(float val) {
    if (val > 255) {
        val = 255;
    }
    if (val < 0) {
        val = 0;
    }

    return val;
}