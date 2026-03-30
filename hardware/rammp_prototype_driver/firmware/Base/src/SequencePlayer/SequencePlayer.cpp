#include <Arduino.h>
#include "SequencePlayer.h"
#include "../Motor/Motor.h"
#include "../CommandParser/CommandParser.h"

// --- Module-scope state (moved from Base.ino globals) ---
static Keyframe seq_keyframes[MAX_SEQ_KEYFRAMES];
static int seq_length = 0;
static int seq_current = -1;
static bool seq_interpolating = false;
static unsigned long seq_interp_start = 0;
static float seq_start_pos[SEQ_NUM_MOTORS];

bool parseKeyframePayload(const String &payload, Keyframe &kf) {
  float vals[13];
  int count = 0;
  int start = 0;
  for (int i = 0; i <= (int)payload.length() && count < 13; i++) {
    char c = (i < (int)payload.length()) ? payload.charAt(i) : ',';
    if (c == ',') {
      vals[count++] = payload.substring(start, i).toFloat();
      start = i + 1;
    }
  }
  if (count < 13)
    return false;
  for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
    kf.targets[i] = vals[i];
    kf.active[i] = (vals[6 + i] > 0.5f);
  }
  kf.duration_ms = (uint32_t)vals[12];
  return true;
}

void sequenceEnter(Motor* motors[SEQ_NUM_MOTORS]) {
  seq_length = 0;
  seq_current = -1;
  seq_interpolating = false;
  for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
    motors[i]->setMode(Motor::POSITION_CONTROL);
    motors[i]->setTargetPosition(motors[i]->current_pos);
    seq_start_pos[i] = motors[i]->current_pos;
  }
}

void sequenceExit() {
  // No special cleanup needed — state resets on next enter
}

void sequenceHandleCommand(const RobotCommand& cmd, Motor* motors[SEQ_NUM_MOTORS],
                           const String& payload) {
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
  } else if (cmd.type == CMD_SEQ_STEP_FWD) {
    if (!seq_interpolating && seq_current < seq_length - 1) {
      seq_current++;
      for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
        seq_start_pos[i] = motors[i]->current_pos;
      }
      seq_interp_start = millis();
      seq_interpolating = true;
      Serial.print("SEQ_STATUS,");
      Serial.print(seq_current);
      Serial.print(",");
      Serial.print(seq_length);
      Serial.println(",1");
    }
  } else if (cmd.type == CMD_SEQ_STEP_BWD) {
    if (!seq_interpolating && seq_current > 0) {
      seq_current--;
      for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
        seq_start_pos[i] = motors[i]->current_pos;
      }
      seq_interp_start = millis();
      seq_interpolating = true;
      Serial.print("SEQ_STATUS,");
      Serial.print(seq_current);
      Serial.print(",");
      Serial.print(seq_length);
      Serial.println(",1");
    }
  } else if (cmd.type == CMD_SEQ_GOTO) {
    int target_step = cmd.actuator_id;
    if (!seq_interpolating && target_step >= 0 && target_step < seq_length) {
      seq_current = target_step;
      for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
        seq_start_pos[i] = motors[i]->current_pos;
      }
      seq_interp_start = millis();
      seq_interpolating = true;
      Serial.print("SEQ_STATUS,");
      Serial.print(seq_current);
      Serial.print(",");
      Serial.print(seq_length);
      Serial.println(",1");
    }
  }
}

void sequenceUpdate(Motor* motors[SEQ_NUM_MOTORS]) {
  if (seq_interpolating && seq_current >= 0 && seq_current < seq_length) {
    Keyframe &kf = seq_keyframes[seq_current];
    unsigned long elapsed = millis() - seq_interp_start;
    float t = (kf.duration_ms == 0)
                  ? 1.0f
                  : min(1.0f, (float)elapsed / (float)kf.duration_ms);
    for (int i = 0; i < SEQ_NUM_MOTORS; i++) {
      if (kf.active[i]) {
        float target =
            seq_start_pos[i] + t * (kf.targets[i] - seq_start_pos[i]);
        motors[i]->setTargetPosition(target);
      }
    }
    if (t >= 1.0f) {
      seq_interpolating = false;
      Serial.print("SEQ_STATUS,");
      Serial.print(seq_current);
      Serial.print(",");
      Serial.print(seq_length);
      Serial.println(",0");
    }
  }
}

bool sequenceIsInterpolating() { return seq_interpolating; }
int sequenceCurrentStep() { return seq_current; }
int sequenceLength() { return seq_length; }
