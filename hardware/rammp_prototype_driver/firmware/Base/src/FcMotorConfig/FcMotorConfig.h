#ifndef FC_MOTOR_CONFIG_H
#define FC_MOTOR_CONFIG_H

// Front-caster hardware on actuators 9 (L) and 10 (R), Serial1 / Serial7.
//   1 = ODrive L/R
//   2 = hub motors L/R
#define fc_motor_id 2

#if (fc_motor_id != 1) && (fc_motor_id != 2)
#error "fc_motor_id must be 1 (ODrive) or 2 (hub motors)"
#endif

// Sequence table slots 8–9: front-caster R then L (matches motor list in
// Base.ino).
#define SEQ_FC_START 8

// Robot-frame axis sign for FC commands and encoder feedback (+1 or -1).
// Left is negated so both casters roll the same way when s:<vel> is sent.
// Tune here if bench motion still opposes; EEPROM motor_dir does not apply.
#define FC_MOTOR_L_AXIS_DIR (-1)
#define FC_MOTOR_R_AXIS_DIR (1)

#endif // FC_MOTOR_CONFIG_H
