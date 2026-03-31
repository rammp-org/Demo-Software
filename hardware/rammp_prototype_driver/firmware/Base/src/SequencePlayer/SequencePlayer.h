#ifndef SEQUENCE_PLAYER_H
#define SEQUENCE_PLAYER_H

#include <Arduino.h>

class Motor;
struct RobotCommand;

#define MAX_SEQ_KEYFRAMES 20
#define SEQ_NUM_MOTORS 8
#define SEQ_NUM_POS_MOTORS 6

struct Keyframe {
  float targets[SEQ_NUM_MOTORS];
  bool active[SEQ_NUM_MOTORS];
  uint32_t duration_ms;
};

// Parse "t1,t2,t3,t4,t5,t6,a1,a2,a3,a4,a5,a6,dur_ms" into a Keyframe.
// Returns true on success.
bool parseKeyframePayload(const String &payload, Keyframe &kf);

// Initialize sequence state when entering AUTO_CURB_CLIMBING mode.
// Motors array must be: {rc, fc, ml, mr, ml_carriage, mr_carriage, drive_fb, drive_lr}
void sequenceEnter(Motor* motors[SEQ_NUM_MOTORS]);

// Zero drive motor velocities and restore position control on sequence end.
// SAFETY: must be called on ALL exit paths to prevent uncontrolled wheelchair motion.
void sequenceExit(Motor* motors[SEQ_NUM_MOTORS]);

// Handle a sequence command (CMD_SEQ_KEYFRAME, CMD_SEQ_STEP_FWD/BWD, CMD_SEQ_GOTO).
// Motors array must be: {rc, fc, ml, mr, ml_carriage, mr_carriage}
void sequenceHandleCommand(const RobotCommand& cmd, Motor* motors[SEQ_NUM_MOTORS],
                           const String& payload);

// Update interpolation during AUTO_CURB_CLIMBING (called every loop).
// Motors array must be: {rc, fc, ml, mr, ml_carriage, mr_carriage}
void sequenceUpdate(Motor* motors[SEQ_NUM_MOTORS]);

// Accessors for state used by Base.ino
bool sequenceIsInterpolating();
int sequenceCurrentStep();
int sequenceLength();

#endif // SEQUENCE_PLAYER_H
