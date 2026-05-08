#include <Arduino.h>
#include "SequencePlayer.h"
#include "../Motor/Motor.h"
#include "../ODrive/ODrive.h"
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
static float seq_start_pos_odrives[SEQ_NUM_ODRIVES];

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

// ODrive note: add odrive final target function
static inline float finalTargetOdrive(const Keyframe &kf, int i) {
  if (kf.odrive_relative[0])
    return seq_start_pos_odrives[i] + kf.odrive_targets[0]; // relative delta
  return kf.odrive_targets[0];                              // absolute position
}

// A "delta-zero" motor is active but has a relative target of 0 — meaning
// "don't move, just wait."  These motors should not enter position control
// and should not draw power; only their duration contributes to keyframe
// timing.
static inline bool isDeltaZero(const Keyframe &kf, int i) {
  return kf.active[i] && kf.relative[i] && kf.targets[i] == 0.0f;
}

// Begin interpolation toward the current keyframe.
static void beginInterp(Motor *motors[SEQ_NUM_MOTORS],
                        ODrive *odrives[SEQ_NUM_ODRIVES]) {
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

      motors[i]->setMode(Motor::POSITION_CONTROL);
      motors[i]->setTargetPosition(motors[i]->current_pos);
    }
  }
  // ODrive note: put another loop for odrives, need to set mode and target
  // position for odrive
  if (kf.odrive_active[0]) {
    for (int i = 0; i < SEQ_NUM_ODRIVES; i++) {
      seq_start_pos_odrives[i] = odrives[i]->current_pos;
      odrives[i]->setMode(ODrive::POSITION_CONTROL);
      odrives[i]->setTargetPosition(odrives[i]->getCurrentPosition());
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
//  Payload parser (supports new 32-value and legacy 17-value formats)
// ---------------------------------------------------------------------------
// ODrive note: need to change to support parsing odrive payloads
bool parseKeyframePayload(const String &payload, Keyframe &kf) {
  // Maximum possible values: 8*6 = 48
  const int MAX_VALS = SEQ_NUM_MOTORS * 9; // ODrive note: add 3 for odrives
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

  // New guarded format: 48 values (targets, active, relative, durations,
  // guard_thresholds, guard_conditions)
  if (count == SEQ_NUM_MOTORS * 9) { // ODrive note: add 3 for odrives
    for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
      kf.targets[i] = vals[i];
      kf.active[i] = (vals[SEQ_NUM_MOTORS + i] > 0.5f);
      kf.relative[i] = (vals[SEQ_NUM_MOTORS * 2 + i] > 0.5f);
      kf.duration_ms[i] = (uint32_t)vals[SEQ_NUM_MOTORS * 3 + i];
      kf.guard_threshold[i] = vals[SEQ_NUM_MOTORS * 4 + i];
      kf.guard_condition[i] = (uint8_t)vals[SEQ_NUM_MOTORS * 5 + i];

      kf.odrive_active[i] = (vals[SEQ_NUM_MOTORS * 6 + i] > 0.5f);
      kf.odrive_relative[i] = (vals[SEQ_NUM_MOTORS * 7 + i] > 0.5f);
      kf.odrive_targets[i] = vals[SEQ_NUM_MOTORS * 8 + i];
    }

    Serial.println("ODrive target in parseKeyframePayload: ");
    Serial.println(kf.odrive_targets[0]);
    return true;
  }

  // New format: 32 values  (targets, active, relative, durations)
  if (count == SEQ_NUM_MOTORS * 7) { // ODrive note: add 3 for odrives
    for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
      kf.targets[i] = vals[i];
      kf.active[i] = (vals[SEQ_NUM_MOTORS + i] > 0.5f);
      kf.relative[i] = (vals[SEQ_NUM_MOTORS * 2 + i] > 0.5f);
      kf.duration_ms[i] = (uint32_t)vals[SEQ_NUM_MOTORS * 3 + i];
      kf.guard_threshold[i] = 0.0f;
      kf.guard_condition[i] = GUARD_NONE;

      kf.odrive_active[i] = (vals[SEQ_NUM_MOTORS * 4 + i] > 0.5f);
      kf.odrive_relative[i] = (vals[SEQ_NUM_MOTORS * 5 + i] > 0.5f);
      kf.odrive_targets[i] = vals[SEQ_NUM_MOTORS * 6 + i];
    }
    Serial.println("ODrive target in parseKeyframePayload: ");
    Serial.println(kf.odrive_targets[0]);
    return true;
  }

  // Legacy format: 17 values  (targets, active, one global duration)
  if (count == SEQ_NUM_MOTORS * 5 + 1) { // ODrive note: add 3 for odrives
    uint32_t global_dur = (uint32_t)vals[SEQ_NUM_MOTORS * 2];
    for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
      kf.targets[i] = vals[i];
      kf.active[i] = (vals[SEQ_NUM_MOTORS + i] > 0.5f);
      kf.relative[i] = false;
      kf.duration_ms[i] = global_dur;
      kf.guard_threshold[i] = 0.0f;
      kf.guard_condition[i] = GUARD_NONE;

      kf.odrive_active[i] = (vals[SEQ_NUM_MOTORS * 2 + i] > 0.5f);
      kf.odrive_relative[i] = (vals[SEQ_NUM_MOTORS * 3 + i] > 0.5f);
      kf.odrive_targets[i] = vals[SEQ_NUM_MOTORS * 4 + i];
    }
    Serial.println("ODrive target in parseKeyframePayload: ");
    Serial.println(kf.odrive_targets[0]);
    return true;
  }

  return false; // unrecognised format
}

// ---------------------------------------------------------------------------
//  Enter / Exit
// ---------------------------------------------------------------------------
// ODrive note: add odrive array to passed arguments
void sequenceEnter(Motor *motors[SEQ_NUM_MOTORS],
                   ODrive *odrives[SEQ_NUM_ODRIVES]) {
  // Serial.println("SEQ: Entered sequenceEnter");
  seq_length = 0;
  seq_current = -1;
  seq_interpolating = false;
  seq_settling = false;
  seq_auto_run = false;

  // Zero drive wheel positions so closed-loop control starts from a known
  // origin.  This prevents float precision issues from accumulating over long
  // drives and ensures repeatable sequence behavior.
  for (int i = SEQ_NUM_POS_MOTORS; i < SEQ_NUM_MOTORS; i++) {
    motors[i]->current_pos = 0.0f;
    motors[i]->prev_pos = 0.0f;
    motors[i]->current_vel = 0.0f;
    motors[i]->prev_vel = 0.0f;
    motors[i]->pos_pid.reset();
    motors[i]->vel_pid.reset();
  }
  // Serial.println("SEQ: Set motor positions to 0");
  // ALL motors — including drive wheels — run position control during
  // sequences.
  // ODrive note: need to loop over odrives to set mode and target position and
  // get current position
  for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
    motors[i]->setMode(Motor::POSITION_CONTROL);
    motors[i]->setTargetPosition(motors[i]->current_pos);
    seq_start_pos[i] = motors[i]->current_pos;
  }
  // Serial.println("SEQ: Set odrive positions to 0");
  for (int i = 0; i < SEQ_NUM_ODRIVES; i++) {
    odrives[i]->setMode(ODrive::POSITION_CONTROL);
    // Serial.println("ODrivesetMode complete");
    odrives[i]->setTargetPosition(odrives[i]->getCurrentPosition());
    // Serial.println("ODrive setTargetPosition complete");
    seq_start_pos_odrives[i] = odrives[i]->getCurrentPosition();
    // Serial.println("ODrive getCurrentPosition complete");
  }
  // Serial.println("SEQ: Set odrive mode to POSITION_CONTROL");

  // Serial.println("SEQ: Exited sequenceEnter");
}

void sequenceExit(Motor *motors[SEQ_NUM_MOTORS],
                  ODrive *odrives[SEQ_NUM_ODRIVES]) {
  // Disable actuators before tearing down state (actuators off first).
  // ODrive note: need to loop over odrives to disable
  float pos = odrives[0]->getCurrentPosition();
  Serial.print("ODrive current position in sequenceExit: ");
  Serial.println(pos);
  for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
    motors[i]->disable();
  }

  for (int i = 0; i < SEQ_NUM_ODRIVES; i++) {
    odrives[i]->disable();
  }

  seq_auto_run = false;
  seq_settling = false;
  seq_interpolating = false;
}

// ---------------------------------------------------------------------------
//  Command handler
// ---------------------------------------------------------------------------
// ODrive note: add odrive array to passed arguments
void sequenceHandleCommand(const RobotCommand &cmd,
                           Motor *motors[SEQ_NUM_MOTORS],
                           ODrive *odrives[SEQ_NUM_ODRIVES],
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
        Serial.println(idx);
      }
    }
    return;
  }

  // ---- Step forward ----
  if (cmd.type == CMD_SEQ_STEP_FWD) {
    if (!seq_interpolating && !seq_settling && seq_current < seq_length - 1) {
      seq_current++;
      beginInterp(motors, odrives);
    }
    return;
  }

  // ---- Step backward ----
  if (cmd.type == CMD_SEQ_STEP_BWD) {
    if (!seq_interpolating && !seq_settling && seq_current > 0) {
      seq_current--;
      beginInterp(motors, odrives);
    }
    return;
  }

  // ---- Go to index ----
  if (cmd.type == CMD_SEQ_GOTO) {
    int target_step = cmd.actuator_id;
    if (!seq_interpolating && !seq_settling && target_step >= 0 &&
        target_step < seq_length) {
      seq_current = target_step;
      beginInterp(motors, odrives);
    }
    return;
  }
}

// ---------------------------------------------------------------------------
//  Update (called every loop iteration while in AUTO_CURB_CLIMBING)
// ---------------------------------------------------------------------------
// ODrive note: add odrive array to passed arguments
void sequenceUpdate(Motor *motors[SEQ_NUM_MOTORS],
                    ODrive *odrives[SEQ_NUM_ODRIVES]) {
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

    // ODrive note: add odrive loop here
    for (int i = 0; i < SEQ_NUM_ODRIVES; i++) {
      float t_i = (kf.duration_ms[i] == 0)
                      ? 1.0f
                      : min(1.0f, (float)elapsed / (float)kf.duration_ms[0]);

      if (!kf.odrive_active[0])
        continue;
      float dest = finalTargetOdrive(kf, i);
      float pos =
          seq_start_pos_odrives[i] + t_i * (dest - seq_start_pos_odrives[i]);
      Serial.println("ODrive target position in sequenceUpdate: ");
      Serial.println(pos);
      odrives[i]->setTargetPosition(pos);
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
      // ODrive note: maybe add odrive loop here?
      for (int i = 0; i < SEQ_NUM_ODRIVES; i++) {
        if (!kf.odrive_active[0])
          continue;
        float dest = finalTargetOdrive(kf, i);
        odrives[i]->setTargetPosition(dest);
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

  // ODrive note: add odrive loop here
  for (int i = 0; i < SEQ_NUM_ODRIVES; i++) {
    if (!kf.odrive_active[0])
      continue;
    float dest = finalTargetOdrive(kf, i);
    float err = fabs(odrives[i]->getCurrentPosition() - dest);
    Serial.print("ODrive error in sequenceUpdate: ");
    Serial.println(err);
    if (err > 1.0f) {
      all_settled = false;
    }
  }

  bool timed_out = settle_elapsed > SEQ_COMPLETION_TIMEOUT_MS;

  if (all_settled) { // ODrive note: removed || timeout check for testing odrive
    // ---- Keyframe complete ----
    seq_settling = false;

    if (timed_out)
      Serial.println("SEQ_TIMEOUT");

    // Auto-run: advance to next keyframe if available.
    if (seq_auto_run && seq_current < seq_length - 1) {
      seq_current++;
      beginInterp(motors, odrives); // sends SEQ_STATUS ...,1
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
