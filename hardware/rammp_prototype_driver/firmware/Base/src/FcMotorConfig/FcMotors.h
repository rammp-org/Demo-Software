#ifndef FC_MOTORS_H
#define FC_MOTORS_H

#include "FcMotorConfig.h"
#include <string.h>
#include "../MotorBase/MotorBase.h"

#ifndef SEQ_NUM_MOTORS
#define SEQ_NUM_MOTORS 10
#endif

// Sequence table slots 8–9: R then L (matches motor list in Base.ino).
#define SEQ_FC_START 8

#if (fc_motor_id == 1)
#define FC_MOTOR_SERIAL_BAUD 460800
#elif (fc_motor_id == 2)
#define FC_MOTOR_SERIAL_BAUD 921600
#else
#error "fc_motor_id must be 1 (ODrive) or 2 (hub motors); see FcMotorConfig.h"
#endif

// Use in Base.ino after FC motor globals are declared.
#define FILL_SEQ_MOTORS(dst, rc_, fc_, ml_, mr_, mlc_, mrc_, dfb_, dlr_,       \
                        fc_r_, fc_l_)                                          \
  do {                                                                         \
    MotorBase *_src[SEQ_NUM_MOTORS] = {(rc_),   (fc_),  (ml_),  (mr_),         \
                                       (mlc_),  (mrc_), (dfb_), (dlr_),        \
                                       (fc_r_), (fc_l_)};                      \
    memcpy((dst), _src, sizeof(_src));                                         \
  } while (0)

#define FILL_ALL_MOTORS(dst, rc_, fc_, ml_, mr_, mlc_, mrc_, dfb_, dlr_,       \
                        fc_r_, fc_l_)                                          \
  do {                                                                         \
    MotorBase *_src[10] = {(rc_),  (fc_),  (ml_),  (mr_),   (mlc_),            \
                           (mrc_), (dfb_), (dlr_), (fc_r_), (fc_l_)};          \
    memcpy((dst), _src, sizeof(_src));                                         \
  } while (0)

#endif // FC_MOTORS_H
