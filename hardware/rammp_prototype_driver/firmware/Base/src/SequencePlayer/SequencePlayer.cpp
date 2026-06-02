#include <Arduino.h>
#include "SequencePlayer.h"
#include "../MotorBase/MotorBase.h"
#include "../CommandParser/CommandParser.h"
#include "../CommandDispatch/CommandDispatch.h"

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
static bool seq_guard_triggered[SEQ_NUM_MOTORS];
static float seq_latch_pos[SEQ_NUM_MOTORS];

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------

// Compute the final target position for motor i in the current keyframe.
static inline float finalTarget(const Keyframe &kf, int i) {
  if (kf.relative[i])
    return seq_start_pos[i] + kf.targets[i]; // relative delta
  return kf.targets[i];                      // absolute position
}

// A "delta-zero" motor is active but has a relative target of 0 — meaning
// "don't move, just wait."  These motors should not enter position control
// and should not draw power; only their duration contributes to keyframe
// timing.
static inline bool isDeltaZero(const Keyframe &kf, int i) {
  return kf.active[i] && kf.relative[i] && kf.targets[i] == 0.0f;
}

// Begin interpolation toward the current keyframe.
static void beginInterp(MotorBase *motors[SEQ_NUM_MOTORS]) {
  const Keyframe &kf = seq_keyframes[seq_current];

  for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
    seq_start_pos[i] = motors[i]->current_pos;
    seq_guard_triggered[i] = false;

    if (isDeltaZero(kf, i)) {
      // Delta-zero motors: disable so they draw no power and don't fight
      // external forces.  Their duration still counts for keyframe timing.
      motors[i]->disable();
    } else if (kf.active[i]) {
      // Re-enable active motors that may have been disabled by a previous
      // delta-zero keyframe.  setMode resets PIDs on mode change.
      motors[i]->setMode(MotorBase::POSITION_CONTROL);
      motors[i]->setTargetPosition(motors[i]->current_pos);
    }
  }
  seq_interp_start = millis();
  seq_interpolating = true;
  seq_settling = false;

  Serial.print("SEQ_STATUS,");
  Serial.print(seq_current);
  Serial.print(",");
  Serial.print(seq_length);
  Serial.println(",1"); // 1 = interpolating
}

// ---------------------------------------------------------------------------
//  Payload parser (SEQ_NUM_MOTORS = 10 only)
// ---------------------------------------------------------------------------
bool parseKeyframePayload(const String &payload, Keyframe &kf) {
  const int MAX_VALS = SEQ_NUM_MOTORS * 7;
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

  // Guarded format: 70 values
  // targets, active, relative, durations, carriage_return, guard_threshold,
  // guard_condition
  if (count == SEQ_NUM_MOTORS * 7) {
    for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
      kf.targets[i] = vals[i];
      kf.active[i] = (vals[SEQ_NUM_MOTORS + i] > 0.5f);
      kf.relative[i] = (vals[SEQ_NUM_MOTORS * 2 + i] > 0.5f);
      kf.duration_ms[i] = (uint32_t)vals[SEQ_NUM_MOTORS * 3 + i];
      kf.carriage_return[i] = (int32_t)vals[SEQ_NUM_MOTORS * 4 + i];
      kf.guard_threshold[i] = vals[SEQ_NUM_MOTORS * 5 + i];
      kf.guard_condition[i] = (uint8_t)vals[SEQ_NUM_MOTORS * 6 + i];
    }
    return true;
  }

  // Standard format: 50 values
  // targets, active, relative, durations, carriage_return
  if (count == SEQ_NUM_MOTORS * 5) {
    for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
      kf.targets[i] = vals[i];
      kf.active[i] = (vals[SEQ_NUM_MOTORS + i] > 0.5f);
      kf.relative[i] = (vals[SEQ_NUM_MOTORS * 2 + i] > 0.5f);
      kf.duration_ms[i] = (uint32_t)vals[SEQ_NUM_MOTORS * 3 + i];
      kf.carriage_return[i] = (int32_t)vals[SEQ_NUM_MOTORS * 4 + i];
      kf.guard_threshold[i] = 0.0f;
      kf.guard_condition[i] = GUARD_NONE;
    }
    return true;
  }

  // Compact format: 31 values
  // targets, active, global_duration_ms, carriage_return
  if (count == SEQ_NUM_MOTORS * 3 + 1) {
    uint32_t global_dur = (uint32_t)vals[SEQ_NUM_MOTORS * 2];
    for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
      kf.targets[i] = vals[i];
      kf.active[i] = (vals[SEQ_NUM_MOTORS + i] > 0.5f);
      kf.relative[i] = false;
      kf.duration_ms[i] = global_dur;
      kf.carriage_return[i] = (int32_t)vals[SEQ_NUM_MOTORS * 2 + 1 + i];
      kf.guard_threshold[i] = 0.0f;
      kf.guard_condition[i] = GUARD_NONE;
    }
    return true;
  }

  return false; // unrecognised format
}

// ---------------------------------------------------------------------------
//  Enter / Exit
// ---------------------------------------------------------------------------
void sequenceEnter(MotorBase *motors[SEQ_NUM_MOTORS]) {
  seq_length = 0;
  seq_current = -1;
  seq_interpolating = false;
  seq_settling = false;
  seq_auto_run = false;

  // Zero synthetic drive-wheel positions (not ODrive axes — keep real encoder
  // state).  Prevents float drift on drive_fb / drive_lr virtual encoders.
  for (int i = SEQ_DRIVE_START; i < SEQ_ODRIVE_START; i++) {
    motors[i]->current_pos = 0.0f;
    motors[i]->prev_pos = 0.0f;
    motors[i]->current_vel = 0.0f;
    motors[i]->prev_vel = 0.0f;
    motors[i]->pos_pid.reset();
    motors[i]->vel_pid.reset();
  }

  // ALL motors — including drive wheels — run position control during
  // sequences.
  for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
    motors[i]->setMode(MotorBase::POSITION_CONTROL);
    motors[i]->setTargetPosition(motors[i]->current_pos);
    seq_start_pos[i] = motors[i]->current_pos;
  }
}

void sequenceExit(MotorBase *motors[SEQ_NUM_MOTORS]) {
  // Disable actuators before tearing down state (actuators off first).
  for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
    motors[i]->disable();
  }

  seq_auto_run = false;
  seq_settling = false;
  seq_interpolating = false;
}

// ---------------------------------------------------------------------------
//  Command handler
// ---------------------------------------------------------------------------
void sequenceHandleCommand(const RobotCommand &cmd,
                           MotorBase *motors[SEQ_NUM_MOTORS],
                           const String &payload) {
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
        Serial.print(idx);
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
    if (!seq_interpolating && !seq_settling && target_step >= 0 &&
        target_step < seq_length) {
      seq_current = target_step;
      beginInterp(motors);
    }
    return;
  }
}

// ---------------------------------------------------------------------------
//  Update (called every loop iteration while in AUTO_CURB_CLIMBING)
// ---------------------------------------------------------------------------
void sequenceUpdate(MotorBase *motors[SEQ_NUM_MOTORS]) {
  if (!seq_interpolating || seq_current < 0 || seq_current >= seq_length)
    return;

  Keyframe &kf = seq_keyframes[seq_current];

  // ==== Phase 1: Interpolation (lerp toward targets) ====================
  if (!seq_settling) {
    unsigned long elapsed = millis() - seq_interp_start;
    bool all_lerps_done = true;
    int blocking_motor = -1;
    uint32_t blocking_dur = 0;

    for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
      if (!kf.active[i])
        continue;

      // Delta-zero motors: no position control, but their duration still
      // contributes to keyframe timing so we track t_i below.
      bool delta_zero = isDeltaZero(kf, i);

      if (!delta_zero && kf.guard_condition[i] != GUARD_NONE &&
          !seq_guard_triggered[i]) {
        bool condition_met = false;
        float current_load = motors[i]->current_load;
        if (kf.guard_condition[i] == GUARD_GREATER_THAN) {
          condition_met = (current_load >= kf.guard_threshold[i]);
        } else if (kf.guard_condition[i] == GUARD_LESS_THAN) {
          condition_met = (current_load <= kf.guard_threshold[i]);
        }
        if (condition_met) {
          seq_guard_triggered[i] = true;
          seq_latch_pos[i] = motors[i]->current_pos;
          Serial.print("SEQ_GUARD_TRIG,m");
          Serial.print(i);
          Serial.print(",load=");
          Serial.println(current_load);
        }
      }

      if (!delta_zero && seq_guard_triggered[i]) {
        motors[i]->setTargetPosition(seq_latch_pos[i]);
      } else {
        float t_i = (kf.duration_ms[i] == 0)
                        ? 1.0f
                        : min(1.0f, (float)elapsed / (float)kf.duration_ms[i]);
        if (t_i < 1.0f) {
          all_lerps_done = false;
          blocking_motor = i;
          blocking_dur = kf.duration_ms[i];
        }
        if (!delta_zero) {
          float dest = finalTarget(kf, i);
          float pos = seq_start_pos[i] + t_i * (dest - seq_start_pos[i]);
          motors[i]->setTargetPosition(pos);
        }
      }
    }

    if (elapsed % 500 < 20 && !all_lerps_done) {
      Serial.print("SEQ_LERP,elapsed=");
      Serial.print(elapsed);
      Serial.print(",blocking=m");
      Serial.print(blocking_motor);
      Serial.print(",dur=");
      Serial.print(blocking_dur);
      Serial.print(",active=[");
      for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
        if (i > 0)
          Serial.print(",");
        Serial.print(kf.active[i] ? "1" : "0");
      }
      Serial.print("],durs=[");
      for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
        if (i > 0)
          Serial.print(",");
        Serial.print(kf.duration_ms[i]);
      }
      Serial.println("]");
    }

    if (all_lerps_done) {
      // Ensure every motor's PID is chasing the exact final target.
      // Delta-zero motors are disabled and have no target to snap to.
      for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
        if (kf.active[i] && !isDeltaZero(kf, i)) {
          float final_dest =
              seq_guard_triggered[i] ? seq_latch_pos[i] : finalTarget(kf, i);
          motors[i]->setTargetPosition(final_dest);
        }
      }
      seq_settling = true;
      seq_settle_start = millis();

      Serial.print("SEQ_STATUS,");
      Serial.print(seq_current);
      Serial.print(",");
      Serial.print(seq_length);
      Serial.println(",2"); // 2 = settling
    }
    return;
  }

  // ==== Phase 2: Settling (wait for motors to physically arrive) ========
  bool all_settled = true;
  unsigned long settle_elapsed = millis() - seq_settle_start;

  for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
    if (!kf.active[i] || isDeltaZero(kf, i))
      continue;

    float dest = seq_guard_triggered[i] ? seq_latch_pos[i] : finalTarget(kf, i);
    motors[i]->setTargetPosition(dest);
    float err = fabs(motors[i]->current_pos - dest);

    if (err > SEQ_COMPLETION_DEADZONE[i]) {
      all_settled = false;
      if (settle_elapsed % 500 < 20) {
        Serial.print("SEQ_SETTLE_WAIT,");
        Serial.print(i);
        Serial.print(",err=");
        Serial.print(err, 1);
        Serial.print(",pos=");
        Serial.print(motors[i]->current_pos, 1);
        Serial.print(",tgt=");
        Serial.print(dest, 1);
        Serial.print(",dz=");
        Serial.println(SEQ_COMPLETION_DEADZONE[i], 1);
      }
    }
  }

  bool timed_out = settle_elapsed > SEQ_COMPLETION_TIMEOUT_MS;

  if (all_settled || timed_out) {
    // ---- Keyframe complete ----
    seq_settling = false;

    if (timed_out)
      Serial.println("SEQ_TIMEOUT");

    // Auto-run: advance to next keyframe if available.
    if (seq_auto_run && seq_current < seq_length - 1) {
      seq_current++;
      beginInterp(motors); // sends SEQ_STATUS ...,1
    } else {
      seq_interpolating = false;
      Serial.print("SEQ_STATUS,");
      Serial.print(seq_current);
      Serial.print(",");
      Serial.print(seq_length);
      Serial.println(",0"); // 0 = idle / complete
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
