#ifndef COMMAND_DISPATCH_H
#define COMMAND_DISPATCH_H

#include <Arduino.h>
#include "../CommandParser/CommandParser.h"

class Motor;
class RoboClaw;
class EncoderContainer;
struct MotorEntry;

// Context struct providing all data a command handler needs.
// NOTE: has const String& member, so no default constructor — always
// construct with all fields populated.
struct CommandContext {
  Motor *motor;          // Target motor (nullptr for global commands like K0)
  uint8_t actuator_id;   // 1-indexed actuator ID from protocol
  float value;           // Parsed float value from command
  const String &payload; // Raw payload string (for CMD_SEQ_KEYFRAME etc.)
  EncoderContainer &encoders; // For encoder operations (CMD_HOME, CMD_OFFSET)
  const MotorEntry *mapping;  // Motor map entry for this actuator
};

// Handler function pointer type
typedef void (*CommandHandler)(CommandContext &ctx);

// Dispatch table entry
struct CommandDispatchEntry {
  CommandType type;
  CommandHandler handler;
};

// Dispatch a TUNER_MODE command through the lookup table.
// NOTE: CMD_SAVE_CONFIG with actuator_id==0 ("save all") must be handled
// by the caller BEFORE calling this function — the dispatch table only
// handles per-motor commands.
void dispatchCommand(const RobotCommand &cmd, CommandContext &ctx);

extern const CommandDispatchEntry COMMAND_TABLE[];
extern const int COMMAND_TABLE_SIZE;

#endif // COMMAND_DISPATCH_H
