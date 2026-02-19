#ifndef CASTER_H
#define CASTER_H

#include <Component.h>

class Caster : public Component {
   public:
    // Member Variables
    float angle_top = 0.0;
    Caster(int, MotorID);
};

#endif