#ifndef FC_MOTOR_CONFIG_H
#define FC_MOTOR_CONFIG_H

// Front-caster hardware on actuators 9 (L) and 10 (R), Serial1 / Serial7.
//   1 = ODrive L/R
//   2 = hub motors L/R
#define fc_motor_id 2

#if (fc_motor_id != 1) && (fc_motor_id != 2)
#error "fc_motor_id must be 1 (ODrive) or 2 (hub motors)"
#endif

#endif // FC_MOTOR_CONFIG_H
