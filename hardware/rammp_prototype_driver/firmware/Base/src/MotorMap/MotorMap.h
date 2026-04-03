#ifndef MOTOR_MAP_H
#define MOTOR_MAP_H

#include <Arduino.h>

class Motor;
class RoboClaw;

// Centralized motor-encoder mapping for the RAMMP mobile base firmware.
// Replaces 4+ duplicated switch statements that map motor IDs to encoder indices.
//
// Reference mapping (defined in Base.ino):
// Index 0 (id=1): rc,          enc=3,  roboclaw_casters,  M1, offset=true,  seq=true,  "rc"
// Index 1 (id=2): fc,          enc=2,  roboclaw_casters,  M2, offset=true,  seq=true,  "fc"
// Index 2 (id=3): ml,          enc=7,  roboclaw_main,     M1, offset=true,  seq=true,  "ml"
// Index 3 (id=4): mr,          enc=5,  roboclaw_main,     M2, offset=true,  seq=true,  "mr"
// Index 4 (id=5): ml_carriage, enc=11, roboclaw_carriages,M1, offset=true,  seq=true,  "ml_carriage"
// Index 5 (id=6): mr_carriage, enc=12, roboclaw_carriages,M2, offset=true,  seq=true,  "mr_carriage"
// Index 6 (id=7): drive_fb,    enc=9,  nullptr,           0,  offset=false, seq=false, "drive_fb"
// Index 7 (id=8): drive_lr,    enc=10, nullptr,           0,  offset=false, seq=false, "drive_lr"

constexpr uint8_t NUM_MOTORS = 8;
constexpr uint8_t NUM_SEQ_MOTORS = 6;

struct MotorEntry {
  Motor* motor;              // Pointer to global Motor object (populated at runtime)
  uint8_t encoder_index;     // Encoder array index in EncoderContainer
  RoboClaw* controller;      // RoboClaw instance (nullptr for encoder-only motors)
  uint8_t roboclaw_channel;  // 1=M1, 2=M2, 0=no controller
  bool supports_offset;      // true for motors 1-6, false for drive wheels 7-8
  bool in_sequence;          // true for 6 motors in AUTO_CURB_CLIMBING sequence
  const char* name;          // Human-readable name for debug
};

extern MotorEntry motor_map[8];

inline const MotorEntry* getMotorEntry(uint8_t actuator_id) {
  if (actuator_id < 1 || actuator_id > NUM_MOTORS) {
    return nullptr;
  }
  return &motor_map[actuator_id - 1];
}

inline uint8_t getEncoderIndex(uint8_t actuator_id) {
  const MotorEntry* entry = getMotorEntry(actuator_id);
  return entry ? entry->encoder_index : 0;
}

inline Motor* getMotor(uint8_t actuator_id) {
  const MotorEntry* entry = getMotorEntry(actuator_id);
  return entry ? entry->motor : nullptr;
}

#endif // MOTOR_MAP_H
