#include <Arduino.h>
#include <Carriage.h>

Carriage::Carriage(int motor_id, int sw_1_pin, int sw_2_pin, signed long& encoder) :  Component(NO_PIN, motor_id),
                                                                        SW1_PIN(sw_1_pin),
                                                                        SW2_PIN(sw_2_pin),
                                                                        encoder_val(encoder) { des = 0.0; };

void Carriage::initialize_pins() {
    Component::initialize_pins();
    pinMode(SW1_PIN, INPUT);
    pinMode(SW2_PIN, INPUT);
};

void Carriage::retrieve_readings() {
    sw1 = digitalRead(SW1_PIN);
    sw2 = digitalRead(SW2_PIN);
};

void Carriage::limit_switch() {
    // if (sw1 == 0 && motor_dir == 1) {
    //     analogWrite(PWM_PIN, 0);
    // }
    // if (sw2 == 0 && motor_dir == 0) {
    //     analogWrite(PWM_PIN, 0);
    // }
};

void Carriage::calculate_carriages_position() {
    // Calculation of drive wheel carriage position
    pos = 34.3 * (float)encoder_val / carriage_ticks;  // 13.5" travel (ticks: carriage_ticks)
}

void Carriage::proportional_PID(float& elapsed_time, bool& self_leveling_on) {
    // MRcarriage
    err = pos - des;

    Kp = 15.0;  // was 10.0
    Ki = 1.0;   // was 1.0

    if (des_pre != des && self_leveling_on == false) {
        Kacc = 0;
    }

    // Error for Integrate
    cum_err += fabs(err) * elapsed_time;
    // Error for Derivative
    rate_err = fabs((err - last_err)) / elapsed_time;
    motor_PWM = Kacc * fabs(20 + Kp * fabs(err) + Ki * cum_err + Kd * rate_err);
}