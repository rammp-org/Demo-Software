#ifndef SEQUENCE_PLAYER_H
#define SEQUENCE_PLAYER_H

#include <Arduino.h>

class Motor;
struct RobotCommand;

#define MAX_SEQ_KEYFRAMES 20
#define SEQ_NUM_MOTORS 8
#define SEQ_NUM_POS_MOTORS 6

// Position-based completion: all active motors must be within this deadzone
// of their target (encoder ticks) before a keyframe is considered complete.
#define SEQ_COMPLETION_DEADZONE 100.0f

// Safety timeout (ms) for position-based completion.  If motors cannot reach
// their targets within this window the keyframe is force-completed and a
// SEQ_TIMEOUT message is sent so the GUI can alert the operator.
#define SEQ_COMPLETION_TIMEOUT_MS 10000

struct Keyframe {
  float targets[SEQ_NUM_MOTORS];
  bool active[SEQ_NUM_MOTORS];
  bool relative[SEQ_NUM_MOTORS];          // true  = target is offset from start pos
  uint32_t duration_ms[SEQ_NUM_MOTORS];   // per-motor interpolation durations
};

// Parse CSV payload into a Keyframe.
// New format  (32 values): t1..t8, a1..a8, r1..r8, d1..d8
// Legacy fmt  (17 values): t1..t8, a1..a8, dur_ms   (all absolute, shared duration)
bool parseKeyframePayload(const String &payload, Keyframe &kf);

// Initialize sequence state when entering AUTO_CURB_CLIMBING mode.
// ALL 8 motors are placed in POSITION_CONTROL for the duration of the sequence.
void sequenceEnter(Motor* motors[SEQ_NUM_MOTORS]);

// Cleanup on mode exit.  Drive wheels are restored to VELOCITY_CONTROL.
// SAFETY: must be called on ALL exit paths to prevent uncontrolled motion.
void sequenceExit(Motor* motors[SEQ_NUM_MOTORS]);

// Handle incoming sequence commands (keyframe upload, step, goto).
void sequenceHandleCommand(const RobotCommand& cmd, Motor* motors[SEQ_NUM_MOTORS],
                           const String& payload);

// Tick interpolation / settling / auto-run (called every loop).
void sequenceUpdate(Motor* motors[SEQ_NUM_MOTORS]);

// Auto-run: automatically advance to the next keyframe on completion.
void sequenceSetAutoRun(bool enable);
bool sequenceIsAutoRunning();

// State accessors for telemetry / Base.ino
bool sequenceIsInterpolating();
int sequenceCurrentStep();
int sequenceLength();

#endif // SEQUENCE_PLAYER_H
