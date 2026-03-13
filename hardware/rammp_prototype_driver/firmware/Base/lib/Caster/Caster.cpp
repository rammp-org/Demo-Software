#include <Caster.h>

Caster::Caster(int lc_pin, MotorID motor_id, bool fwd_is_positive)
    : Component::Component(lc_pin, motor_id, fwd_is_positive){};
