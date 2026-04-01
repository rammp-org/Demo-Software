#include <Arduino.h>
#include "SequencePlayer.h"
#include "../Motor/Motor.h"
#include "../CommandParser/CommandParser.h"

// ---------------------------------------------------------------------------
//  Module-scope state
// ---------------------------------------------------------------------------
static Keyframe seq_keyframes[MAX_SEQ_KEYFRAMES];
static int seq_length = 0;
static int seq_current = -1;

// Interpolation phase: lerp toward targets using per-motor durations.
static bool seq_interpolating = false;
static unsigned long seq_interp_start = 0;
static float seq_start_pos[SEQ_NUM_MOTORS];

// Settling phase: lerp finished, waiting for motors to physically arrive.
static bool seq_settling = false;
static unsigned long seq_settle_start = 0;

// Auto-run: advance automatically when a keyframe completes.
static bool seq_auto_run = false;

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------

// Compute the final target position for motor i in the current keyframe.
static inline float finalTarget(const Keyframe &kf, int i) {
  if (kf.relative[i])
    return seq_start_pos[i] + kf.targets[i];   // relative delta
  return kf.targets[i];                         // absolute position
}

// Begin interpolation toward the current keyframe.
static void beginInterp(Motor* motors[SEQ_NUM_MOTORS]) {
  for (int i = 0; i < SEQ_NUM_MOTORS; i++)
    seq_start_pos[i] = motors[i]->current_pos;
  seq_interp_start = millis();
  seq_interpolating = true;
  seq_settling = false;

  Serial.print("SEQ_STATUS,");
  Serial.print(seq_current);
  Serial.print(",");
  Serial.print(seq_length);
  Serial.println(",1");   // 1 = interpolating
}

// ---------------------------------------------------------------------------
//  Payload parser (supports new 32-value and legacy 17-value formats)
// ---------------------------------------------------------------------------
bool parseKeyframePayload(const String &payload, Keyframe &kf) {
  // Maximum possible values: 8*4 = 32
  const int MAX_VALS = SEQ_NUM_MOTORS * 4;
  float vals[MAX_VALS];
  int count = 0;
  int start = 0;

  for (int i = 0; i <= (int)payload.length() && count < MAX_VALS; i++) {
    char c = (i < (int)payload.length()) ? payload.charAt(i) : ',';
    if (c == ',') {
      vals[count++] = payload.substring(start, i).toFloat();
      start = i + 1;
    }
  }

  // New format: 32 values  (targets, active, relative, durations)
  if (count == SEQ_NUM_MOTORS * 4) {
    for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
      kf.targets[i]     = vals[i];
      kf.active[i]      = (vals[SEQ_NUM_MOTORS     + i] > 0.5f);
      kf.relative[i]    = (vals[SEQ_NUM_MOTORS * 2 + i] > 0.5f);
      kf.duration_ms[i] = (uint32_t)vals[SEQ_NUM_MOTORS * 3 + i];
    }
    return true;
  }

  // Legacy format: 17 values  (targets, active, one global duration)
  if (count == SEQ_NUM_MOTORS * 2 + 1) {
    uint32_t global_dur = (uint32_t)vals[SEQ_NUM_MOTORS * 2];
    for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
      kf.targets[i]     = vals[i];
      kf.active[i]      = (vals[SEQ_NUM_MOTORS + i] > 0.5f);
      kf.relative[i]    = false;
      kf.duration_ms[i] = global_dur;
    }
    return true;
  }

  return false;   // unrecognised format
}

// ---------------------------------------------------------------------------
//  Enter / Exit
// ---------------------------------------------------------------------------
void sequenceEnter(Motor* motors[SEQ_NUM_MOTORS]) {
  seq_length = 0;
  seq_current = -1;
  seq_interpolating = false;
  seq_settling = false;
  seq_auto_run = false;

  // ALL motors — including drive wheels — run position control during sequences.
  for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
    motors[i]->setMode(Motor::POSITION_CONTROL);
    motors[i]->setTargetPosition(motors[i]->current_pos);
    seq_start_pos[i] = motors[i]->current_pos;
  }
}

void sequenceExit(Motor* motors[SEQ_NUM_MOTORS]) {
  seq_auto_run = false;
  seq_settling = false;
  seq_interpolating = false;

  // Restore drive wheels: zero velocity, hold current position for safety.
  for (int i = SEQ_NUM_POS_MOTORS; i < SEQ_NUM_MOTORS; i++) {
    motors[i]->setTargetVelocity(0);
    motors[i]->setMode(Motor::POSITION_CONTROL);
    motors[i]->setTargetPosition(motors[i]->current_pos);
  }
}

// ---------------------------------------------------------------------------
//  Command handler
// ---------------------------------------------------------------------------
void sequenceHandleCommand(const RobotCommand& cmd, Motor* motors[SEQ_NUM_MOTORS],
                           const String& payload) {
  // ---- Keyframe upload ----
  if (cmd.type == CMD_SEQ_KEYFRAME) {
    int idx = cmd.actuator_id;
    if (idx >= 0 && idx < MAX_SEQ_KEYFRAMES) {
      Keyframe kf;
      if (parseKeyframePayload(payload, kf)) {
        seq_keyframes[idx] = kf;
        if (idx >= seq_length)
          seq_length = idx + 1;
        Serial.print("SEQ_ACK,");
        Serial.println(idx);
      } else {
        Serial.print("SEQ_ERR,bad_payload,");
        Serial.println(idx);
      }
    }
    return;
  }

  // ---- Step forward ----
  if (cmd.type == CMD_SEQ_STEP_FWD) {
    if (!seq_interpolating && !seq_settling && seq_current < seq_length - 1) {
      seq_current++;
      beginInterp(motors);
    }
    return;
  }

  // ---- Step backward ----
  if (cmd.type == CMD_SEQ_STEP_BWD) {
    if (!seq_interpolating && !seq_settling && seq_current > 0) {
      seq_current--;
      beginInterp(motors);
    }
    return;
  }

  // ---- Go to index ----
  if (cmd.type == CMD_SEQ_GOTO) {
    int target_step = cmd.actuator_id;
    if (!seq_interpolating && !seq_settling &&
        target_step >= 0 && target_step < seq_length) {
      seq_current = target_step;
      beginInterp(motors);
    }
    return;
  }
}

// ---------------------------------------------------------------------------
//  Update (called every loop iteration while in AUTO_CURB_CLIMBING)
// ---------------------------------------------------------------------------
void sequenceUpdate(Motor* motors[SEQ_NUM_MOTORS]) {
  if (!seq_interpolating || seq_current < 0 || seq_current >= seq_length)
    return;

  Keyframe &kf = seq_keyframes[seq_current];

  // ==== Phase 1: Interpolation (lerp toward targets) ====================
  if (!seq_settling) {
    unsigned long elapsed = millis() - seq_interp_start;
    bool all_lerps_done = true;

    for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
      if (!kf.active[i]) continue;

      float t_i = (kf.duration_ms[i] == 0)
                      ? 1.0f
                      : min(1.0f, (float)elapsed / (float)kf.duration_ms[i]);
      if (t_i < 1.0f) all_lerps_done = false;

      float dest = finalTarget(kf, i);
      float pos  = seq_start_pos[i] + t_i * (dest - seq_start_pos[i]);
      motors[i]->setTargetPosition(pos);
    }

    if (all_lerps_done) {
      // Ensure every motor's PID is chasing the exact final target.
      for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
        if (kf.active[i])
          motors[i]->setTargetPosition(finalTarget(kf, i));
      }
      seq_settling = true;
      seq_settle_start = millis();

      Serial.print("SEQ_STATUS,");
      Serial.print(seq_current);
      Serial.print(",");
      Serial.print(seq_length);
      Serial.println(",2");   // 2 = settling
    }
    return;
  }

  // ==== Phase 2: Settling (wait for motors to physically arrive) ========
  bool all_settled = true;
  for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
    if (!kf.active[i]) continue;

    float dest = finalTarget(kf, i);
    motors[i]->setTargetPosition(dest);   // keep commanding exact target

    if (fabs(motors[i]->current_pos - dest) > SEQ_COMPLETION_DEADZONE)
      all_settled = false;
  }

  bool timed_out = (millis() - seq_settle_start) > SEQ_COMPLETION_TIMEOUT_MS;

  if (all_settled || timed_out) {
    // ---- Keyframe complete ----
    seq_settling = false;

    if (timed_out)
      Serial.println("SEQ_TIMEOUT");

    // Auto-run: advance to next keyframe if available.
    if (seq_auto_run && seq_current < seq_length - 1) {
      seq_current++;
      beginInterp(motors);                // sends SEQ_STATUS ...,1
    } else {
      seq_interpolating = false;
      Serial.print("SEQ_STATUS,");
      Serial.print(seq_current);
      Serial.print(",");
      Serial.print(seq_length);
      Serial.println(",0");               // 0 = idle / complete
    }
  }
}

// ---------------------------------------------------------------------------
//  Auto-run
// ---------------------------------------------------------------------------
void sequenceSetAutoRun(bool enable) {
  seq_auto_run = enable;
  Serial.print("SEQ_AUTO_RUN,");
  Serial.println(enable ? "1" : "0");
}

bool sequenceIsAutoRunning() { return seq_auto_run; }

// ---------------------------------------------------------------------------
//  State accessors
// ---------------------------------------------------------------------------
bool sequenceIsInterpolating() { return seq_interpolating; }
int sequenceCurrentStep() { return seq_current; }
int sequenceLength() { return seq_length; }
