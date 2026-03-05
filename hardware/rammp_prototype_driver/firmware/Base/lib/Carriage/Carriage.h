#ifndef CARRIAGE_H
#define CARRIAGE_H

#include <Component.h>
#include <Constants.h>

class Carriage : public Component {
   public:
    const int SW1_PIN;
    const int SW2_PIN;
    int sw1 = 0, sw2 = 0;
    signed long& encoder_val;
    int carriage_ticks = 1;

    Carriage(MotorID, int, int, signed long&);
    void initialize_pins() override;
    void retrieve_readings();
    void limit_switch();
    void calculate_carriages_position();
    void proportional_PID(float&, bool&);
};

#endif