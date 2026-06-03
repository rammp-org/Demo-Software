#ifndef SEQUENCE_PLAYER_H
#define SEQUENCE_PLAYER_H

#include <Arduino.h>

class MotorBase;
struct RobotCommand;

#define MAX_SEQ_KEYFRAMES 20
#define SEQ_NUM_MOTORS 10
#define SEQ_NUM_POS_MOTORS 6
#define SEQ_DRIVE_START                                                        \
  6 // drive_fb, drive_lr (synthetic encoder origin zeroed on enter)
#define SEQ_ODRIVE_START 8 // ODriveR, ODriveL

static const float SEQ_COMPLETION_DEADZONE[SEQ_NUM_MOTORS] = {
    50.0f,   // 0: RC
    50.0f,   // 1: FC
    50.0f,   // 2: ML
    50.0f,   // 3: MR
    500.0f,  // 4: ML_Car
    500.0f,  // 5: MR_Car
    2000.0f, // 6: Drive_FB
    2000.0f, // 7: Drive_LR
    0.1f,    // 8: ODriveR (turns)
    0.1f,    // 9: ODriveL (turns)
};

// Safety timeout (ms) for position-based completion.  If motors cannot reach
// their targets within this window the keyframe is force-completed and a
// SEQ_TIMEOUT message is sent so the GUI can alert the operator.
#define SEQ_COMPLETION_TIMEOUT_MS 5000

enum GuardCondition {
  GUARD_NONE = 0,
  GUARD_GREATER_THAN = 1,
  GUARD_LESS_THAN = 2
};

struct Keyframe {
  float targets[SEQ_NUM_MOTORS];
  bool active[SEQ_NUM_MOTORS];
  bool relative[SEQ_NUM_MOTORS]; // true  = target is offset from start pos
  uint32_t duration_ms[SEQ_NUM_MOTORS]; // per-motor interpolation durations
  int32_t carriage_return; // LUCI forward/back for this keyframe (-1, 0, 1)
  float guard_threshold[SEQ_NUM_MOTORS];
  uint8_t guard_condition[SEQ_NUM_MOTORS];
};

// Parse CSV payload into a Keyframe (SEQ_NUM_MOTORS = 10 only).
// Standard  (41 values): t1..t10, a1..a10, r1..r10, d1..d10, cr
// Guarded   (61 values): + guard_threshold + guard_condition per motor
// Compact   (22 values): t1..t10, a1..a10, global_duration_ms, cr
bool parseKeyframePayload(const String &payload, Keyframe &kf);

// Initialize sequence state when entering AUTO_CURB_CLIMBING mode.
// All SEQ_NUM_MOTORS actuators use POSITION_CONTROL for the sequence.
void sequenceEnter(MotorBase *motors[SEQ_NUM_MOTORS]);

// Cleanup on mode exit.  All motors are disabled (zero power, PIDs reset).
// SAFETY: must be called on ALL exit paths to prevent uncontrolled motion.
void sequenceExit(MotorBase *motors[SEQ_NUM_MOTORS]);

// Handle incoming sequence commands (keyframe upload, step, goto).
void sequenceHandleCommand(const RobotCommand &cmd,
                           MotorBase *motors[SEQ_NUM_MOTORS],
                           const String &payload);

// Tick interpolation / settling / auto-run (called every loop).
void sequenceUpdate(MotorBase *motors[SEQ_NUM_MOTORS]);

// Auto-run: automatically advance to the next keyframe on completion.
void sequenceSetAutoRun(bool enable);
bool sequenceIsAutoRunning();

// State accessors for telemetry / Base.ino
bool sequenceIsInterpolating();
int sequenceCurrentStep();
int sequenceLength();

#endif // SEQUENCE_PLAYER_H
