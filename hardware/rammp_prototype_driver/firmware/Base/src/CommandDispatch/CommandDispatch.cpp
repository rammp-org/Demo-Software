#include <Arduino.h>
#include "CommandDispatch.h"
#include "../CommandParser/CommandParser.h"
#include "../MotorMap/MotorMap.h"
#include "../MotorBase/MotorBase.h"
#include "../EncoderContainer/EncoderContainer.h"
#include "../ConfigStorage/ConfigStorage.h"

// Extern declarations for functions/globals defined in Base.ino
extern void saveMotorConfig(int motor_id, MotorBase *m);
extern void saveAllMotorConfigs();
extern MotorBase drive_fb, drive_lr;
extern int8_t ml_enc_dir, mr_enc_dir;

// DEBUG_MODE is defined in Base.ino — replicate here for debug prints
#ifndef DEBUG_MODE
#define DEBUG_MODE 1
#endif

// --- Handler implementations (migrated from Base.ino TUNER_MODE switch) ---

void handleSetMode(CommandContext &ctx) {
  if (ctx.value == 0)
    ctx.motor->setMode(MotorBase::OPEN_LOOP);
  else if (ctx.value == 1)
    ctx.motor->setMode(MotorBase::VELOCITY_CONTROL);
  else if (ctx.value == 2)
    ctx.motor->setMode(MotorBase::POSITION_CONTROL);
  if (DEBUG_MODE) {
    Serial.print("DEBUG: Set Mode to ");
    Serial.println(ctx.value);
  }
}

void handleSetTarget(CommandContext &ctx) {
  if (ctx.motor->mode == MotorBase::OPEN_LOOP)
    ctx.motor->setTargetPWM(ctx.value);
  else if (ctx.motor->mode == MotorBase::VELOCITY_CONTROL)
    ctx.motor->setTargetVelocity(ctx.value);
  else if (ctx.motor->mode == MotorBase::POSITION_CONTROL)
    ctx.motor->setTargetPosition(ctx.value);
  if (DEBUG_MODE) {
    Serial.print("DEBUG: Set Target to ");
    Serial.println(ctx.value, 4);
  }
}

void handlePosPGain(CommandContext &ctx) {
  ctx.motor->pos_pid.kp = ctx.value;
  saveMotorConfig(ctx.actuator_id, ctx.motor);
  if (DEBUG_MODE)
    Serial.println("DEBUG: Set Pos P");
}

void handlePosIGain(CommandContext &ctx) {
  ctx.motor->pos_pid.ki = ctx.value;
  saveMotorConfig(ctx.actuator_id, ctx.motor);
  if (DEBUG_MODE)
    Serial.println("DEBUG: Set Pos I");
}

void handlePosDGain(CommandContext &ctx) {
  ctx.motor->pos_pid.kd = ctx.value;
  saveMotorConfig(ctx.actuator_id, ctx.motor);
  if (DEBUG_MODE)
    Serial.println("DEBUG: Set Pos D");
}

void handlePosFeedForward(CommandContext &ctx) {
  ctx.motor->pos_pid.setFeedForward(ctx.value);
  saveMotorConfig(ctx.actuator_id, ctx.motor);
  if (DEBUG_MODE)
    Serial.println("DEBUG: Set Pos FF");
}

void handleVelPGain(CommandContext &ctx) {
  ctx.motor->vel_pid.kp = ctx.value;
  saveMotorConfig(ctx.actuator_id, ctx.motor);
  if (DEBUG_MODE)
    Serial.println("DEBUG: Set Vel P");
}

void handleVelIGain(CommandContext &ctx) {
  ctx.motor->vel_pid.ki = ctx.value;
  saveMotorConfig(ctx.actuator_id, ctx.motor);
  if (DEBUG_MODE)
    Serial.println("DEBUG: Set Vel I");
}

void handleVelDGain(CommandContext &ctx) {
  ctx.motor->vel_pid.kd = ctx.value;
  saveMotorConfig(ctx.actuator_id, ctx.motor);
  if (DEBUG_MODE)
    Serial.println("DEBUG: Set Vel D");
}

void handleVelFeedForward(CommandContext &ctx) {
  // TODO: /10000 scaling is inconsistent with other commands — preserved
  // for protocol compatibility. All other FF/gain commands pass raw values.
  ctx.motor->vel_pid.setFeedForward(ctx.value / 10000);
  saveMotorConfig(ctx.actuator_id, ctx.motor);
  if (DEBUG_MODE)
    Serial.println("DEBUG: Set Vel FF");
}

void handleInputLpf(CommandContext &ctx) {
  ctx.motor->setInputLpfAlpha(ctx.value);
  saveMotorConfig(ctx.actuator_id, ctx.motor);
  if (DEBUG_MODE)
    Serial.println("DEBUG: Set Input LPF");
}

void handlePosLpf(CommandContext &ctx) {
  ctx.motor->pos_pid.setLpfAlpha(ctx.value);
  saveMotorConfig(ctx.actuator_id, ctx.motor);
  if (DEBUG_MODE)
    Serial.println("DEBUG: Set Pos LPF");
}

void handleVelLpf(CommandContext &ctx) {
  ctx.motor->vel_pid.setLpfAlpha(ctx.value);
  saveMotorConfig(ctx.actuator_id, ctx.motor);
  if (DEBUG_MODE)
    Serial.println("DEBUG: Set Vel LPF");
}

void handlePosRamp(CommandContext &ctx) {
  ctx.motor->pos_pid.setRampRate(ctx.value);
  saveMotorConfig(ctx.actuator_id, ctx.motor);
  if (DEBUG_MODE)
    Serial.println("DEBUG: Set Pos max ramp rate");
}

void handleVelRamp(CommandContext &ctx) {
  ctx.motor->vel_pid.setRampRate(ctx.value);
  saveMotorConfig(ctx.actuator_id, ctx.motor);
  if (DEBUG_MODE)
    Serial.println("DEBUG: Set Vel max ramp rate");
}

void handleResetPID(CommandContext &ctx) {
  ctx.motor->pos_pid.reset();
  ctx.motor->vel_pid.reset();
  if (DEBUG_MODE)
    Serial.println("DEBUG: Reset PID state (cleared integrator)");
}

void handleHome(CommandContext &ctx) {
  if (ctx.actuator_id == 7 || ctx.actuator_id == 8) {
    ctx.encoders.zeroEncoder(9);
    ctx.encoders.zeroEncoder(10);
    drive_fb.pos_pid.reset();
    drive_fb.vel_pid.reset();
    drive_fb.target_pos = 0;
    drive_lr.pos_pid.reset();
    drive_lr.vel_pid.reset();
    drive_lr.target_pos = 0;
  } else {
    int enc_idx = motor_map[ctx.actuator_id - 1].encoder_index;
    ctx.encoders.zeroEncoder(enc_idx);
    ctx.motor->pos_pid.reset();
    ctx.motor->vel_pid.reset();
    ctx.motor->target_pos = 0;
  }
  if (DEBUG_MODE) {
    Serial.print("DEBUG: Homed encoder for joint ");
    Serial.println(ctx.actuator_id);
  }
}

void handleOffset(CommandContext &ctx) {
  const MotorEntry *oentry = getMotorEntry(ctx.actuator_id);
  int enc_idx = oentry ? oentry->encoder_index : 0;

  if (enc_idx > 0 && oentry->supports_offset) {
    float raw_pos = (float)ctx.encoders.getRawReading(enc_idx);
    float encoder_dir = ctx.motor->getEncoderDirection();
    signed long new_offset = (signed long)(raw_pos - (ctx.value / encoder_dir));

    ctx.encoders.setOffset(enc_idx, new_offset);

    ctx.motor->pos_pid.reset();
    ctx.motor->vel_pid.reset();
    ctx.motor->target_pos = ctx.value;
    ctx.motor->current_pos = ctx.value;
    ctx.motor->prev_pos = ctx.value;

    if (DEBUG_MODE) {
      Serial.print("DEBUG: Set offset J");
      Serial.print(ctx.actuator_id);
      Serial.print(": new logical pos=");
      Serial.println(ctx.value);
    }

    ConfigStorage::save_position(ctx.actuator_id, ctx.value);
  }
}

void handleToggleDir(CommandContext &ctx) {
  ctx.motor->toggleDirection();
  MotorConfig conf = ConfigStorage::loadMotorConfig(ctx.actuator_id);
  conf.motor_dir = ctx.motor->getDirection();
  ConfigStorage::saveMotorConfig(ctx.actuator_id, conf);
  if (DEBUG_MODE) {
    Serial.print("DEBUG: Toggled direction for motor ");
    Serial.print(ctx.actuator_id);
    Serial.print(" to ");
    Serial.println(ctx.motor->getDirection());
  }
}

void handleToggleEncDir(CommandContext &ctx) {
  if (ctx.actuator_id == 7 || ctx.actuator_id == 8) {
    int8_t &enc_dir = (ctx.actuator_id == 7) ? ml_enc_dir : mr_enc_dir;
    enc_dir = (enc_dir >= 0) ? -1 : 1;
    ctx.motor->setEncoderDirection(enc_dir);
    saveMotorConfig(ctx.actuator_id, ctx.motor);
    ctx.motor->setEncoderDirection(1);
  } else {
    ctx.motor->toggleEncoderDirection();
    saveMotorConfig(ctx.actuator_id, ctx.motor);
  }
  if (DEBUG_MODE) {
    Serial.print("DEBUG: Toggled enc direction for motor ");
    Serial.print(ctx.actuator_id);
    Serial.print(" to ");
    if (ctx.actuator_id == 7)
      Serial.println(ml_enc_dir);
    else if (ctx.actuator_id == 8)
      Serial.println(mr_enc_dir);
    else
      Serial.println(ctx.motor->getEncoderDirection());
  }
}

void handleSetEncDir(CommandContext &ctx) {
  if (ctx.actuator_id == 7 || ctx.actuator_id == 8) {
    int8_t &enc_dir = (ctx.actuator_id == 7) ? ml_enc_dir : mr_enc_dir;
    enc_dir = (ctx.value >= 0) ? 1 : -1;
    ctx.motor->setEncoderDirection(enc_dir);
    saveMotorConfig(ctx.actuator_id, ctx.motor);
    ctx.motor->setEncoderDirection(1);
  } else {
    ctx.motor->setEncoderDirection((int8_t)ctx.value);
    saveMotorConfig(ctx.actuator_id, ctx.motor);
  }
  if (DEBUG_MODE) {
    Serial.print("DEBUG: Set enc direction for motor ");
    Serial.print(ctx.actuator_id);
    Serial.print(" to ");
    if (ctx.actuator_id == 7)
      Serial.println(ml_enc_dir);
    else if (ctx.actuator_id == 8)
      Serial.println(mr_enc_dir);
    else
      Serial.println(ctx.motor->getEncoderDirection());
  }
}

void handleSaveConfig(CommandContext &ctx) {
  saveMotorConfig(ctx.actuator_id, ctx.motor);
  if (DEBUG_MODE) {
    Serial.print("DEBUG: Saved config for motor ");
    Serial.println(ctx.actuator_id);
  }
}

void handlePosMin(CommandContext &ctx) {
  ctx.motor->updateLimits(ctx.value, ctx.motor->pos_limit_max);
  saveMotorConfig(ctx.actuator_id, ctx.motor);
  if (DEBUG_MODE) {
    Serial.print("DEBUG: Set min limit to ");
    Serial.println(ctx.value);
  }
}

void handlePosMax(CommandContext &ctx) {
  ctx.motor->updateLimits(ctx.motor->pos_limit_min, ctx.value);
  saveMotorConfig(ctx.actuator_id, ctx.motor);
  if (DEBUG_MODE) {
    Serial.print("DEBUG: Set max limit to ");
    Serial.println(ctx.value);
  }
}

void handleGetConfig(CommandContext &ctx) {
  Serial.print("CONFIG,");
  Serial.print(ctx.actuator_id);
  Serial.print(",");
  Serial.print(ctx.motor->pos_pid.kp, 4);
  Serial.print(",");
  Serial.print(ctx.motor->pos_pid.ki, 4);
  Serial.print(",");
  Serial.print(ctx.motor->pos_pid.kd, 4);
  Serial.print(",");
  Serial.print(ctx.motor->pos_pid.kff, 4);
  Serial.print(",");
  Serial.print(ctx.motor->vel_pid.kp, 4);
  Serial.print(",");
  Serial.print(ctx.motor->vel_pid.ki, 4);
  Serial.print(",");
  Serial.print(ctx.motor->vel_pid.kd, 4);
  Serial.print(",");
  Serial.print(ctx.motor->vel_pid.kff, 4);
  Serial.print(",");
  Serial.print(ctx.motor->pos_pid.getLpfAlpha(), 4);
  Serial.print(",");
  Serial.print(ctx.motor->vel_pid.getLpfAlpha(), 4);
  Serial.print(",");
  Serial.print(ctx.motor->lpf_input_alpha, 4);
  Serial.print(",");
  Serial.print(ctx.motor->pos_limit_min);
  Serial.print(",");
  Serial.print(ctx.motor->pos_limit_max);
  Serial.print(",");
  Serial.print(ctx.motor->pos_pid.max_ramp_rate, 4);
  Serial.print(",");
  Serial.print(ctx.motor->vel_pid.max_ramp_rate, 4);
  Serial.print(",");
  Serial.print(ctx.motor->getDirection());
  Serial.print(",");
  Serial.println(ctx.motor->getEncoderDirection());
}

// --- Dispatch table: 24 TUNER_MODE commands ---
// State commands (CMD_Z, CMD_C, CMD_LEVEL_*, CMD_SEQ_MODE) stay in Base.ino.
// Sequence commands (CMD_SEQ_KEYFRAME, CMD_SEQ_STEP_*, CMD_SEQ_GOTO) move to
// SequencePlayer.
const CommandDispatchEntry COMMAND_TABLE[] = {
    {CMD_T, handleSetTarget},          {CMD_M, handleSetMode},
    {CMD_POS_P, handlePosPGain},       {CMD_POS_I, handlePosIGain},
    {CMD_POS_D, handlePosDGain},       {CMD_POS_FF, handlePosFeedForward},
    {CMD_VEL_P, handleVelPGain},       {CMD_VEL_I, handleVelIGain},
    {CMD_VEL_D, handleVelDGain},       {CMD_VEL_FF, handleVelFeedForward},
    {CMD_R, handleResetPID},           {CMD_HOME, handleHome},
    {CMD_OFFSET, handleOffset},        {CMD_DIR, handleToggleDir},
    {CMD_ENC_DIR, handleToggleEncDir}, {CMD_SAVE_CONFIG, handleSaveConfig},
    {CMD_GET_CONFIG, handleGetConfig}, {CMD_POS_MIN, handlePosMin},
    {CMD_POS_MAX, handlePosMax},       {CMD_INPUT_LPF, handleInputLpf},
    {CMD_POS_LPF, handlePosLpf},       {CMD_VEL_LPF, handleVelLpf},
    {CMD_POS_RAMP, handlePosRamp},     {CMD_VEL_RAMP, handleVelRamp},
};
const int COMMAND_TABLE_SIZE = sizeof(COMMAND_TABLE) / sizeof(COMMAND_TABLE[0]);

void dispatchCommand(const RobotCommand &cmd, CommandContext &ctx) {
  for (int i = 0; i < COMMAND_TABLE_SIZE; i++) {
    if (COMMAND_TABLE[i].type == cmd.type) {
      COMMAND_TABLE[i].handler(ctx);
      return;
    }
  }
}
