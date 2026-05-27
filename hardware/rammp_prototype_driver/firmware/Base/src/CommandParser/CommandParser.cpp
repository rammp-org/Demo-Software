#include "CommandParser.h"

CommandParser::CommandParser(unsigned long timeout_ms)
    : last_heartbeat(millis()), timeout_ms(timeout_ms), buffer("") {}

void CommandParser::feedWatchdog() { last_heartbeat = millis(); }

bool CommandParser::isTimedOut() {
  return (millis() - last_heartbeat) > timeout_ms;
}

RobotCommand CommandParser::parse(Stream &serial) {
  RobotCommand cmd = {CMD_NONE, -1, 0.0f};

  while (serial.available() > 0) {
    char c = serial.read();

    // Command execution occurs on newline
    if (c == '\n' || c == '\r') {
      if (buffer.length() == 0)
        continue;

      feedWatchdog(); // Any valid command feeds the watchdog

      if (buffer.charAt(0) == 'z') {
        cmd.type = CMD_Z;
      } else if (buffer.charAt(0) == 'c') {
        cmd.type = CMD_C;
      } else if (buffer.charAt(0) == '>') {
        cmd.type = CMD_SEQ_STEP_FWD;
      } else if (buffer.charAt(0) == '<') {
        cmd.type = CMD_SEQ_STEP_BWD;
      } else if (buffer.charAt(0) == '@' && buffer.length() >= 2) {
        cmd.type = CMD_SEQ_GOTO;
        cmd.actuator_id = buffer.substring(1).toInt();
      } else if (buffer.length() >= 2) { // Need at least R<id> or X<id>:<val>
        char type_char = buffer.charAt(0);
        int colon_idx = buffer.indexOf(':');

        // Commands without value (e.g., R1 for reset, H1 for home, V1 for
        // direction)
        if (colon_idx == -1) {
          cmd.actuator_id = buffer.substring(1).toInt();
          cmd.value = 0.0f;
          switch (type_char) {
          case 'R':
            cmd.type = CMD_R;
            break;
          case 'H':
            cmd.type = CMD_HOME;
            break;
          case 'V':
            cmd.type = CMD_DIR;
            break;
          case 'E':
            cmd.type = CMD_ENC_DIR;
            break;
          case 'K':
            cmd.type = CMD_SAVE_CONFIG;
            break;
          case 'G':
            cmd.type = CMD_GET_CONFIG;
            break;
          case 'M':
            cmd.type = CMD_MANUAL_CONTROL;
            break;
          default:
            cmd.type = CMD_UNKNOWN;
            break;
          }
        }
        // Global ODrive velocity: s:<vel> (no actuator id)
        else if (colon_idx == 1) {
          last_payload = buffer.substring(colon_idx + 1);
          cmd.value = last_payload.toFloat();
          cmd.actuator_id = 0;
          if (type_char == 's')
            cmd.type = CMD_ODRIVE_VEL;
          else
            cmd.type = CMD_UNKNOWN;
        }
        // Commands with value (e.g., T1:0.5)
        else if (colon_idx > 1) {
          cmd.actuator_id = buffer.substring(1, colon_idx).toInt();
          last_payload =
              buffer.substring(colon_idx + 1); // full raw payload after ':'
          cmd.value = last_payload.toFloat();

          switch (type_char) {
          case 'T':
            cmd.type = CMD_T;
            break;
          case 'M':
            cmd.type = CMD_M;
            break;
          case 'P':
            cmd.type = CMD_POS_P;
            break;
          case 'I':
            cmd.type = CMD_POS_I;
            break;
          case 'D':
            cmd.type = CMD_POS_D;
            break;
          case 'F':
            cmd.type = CMD_POS_FF;
            break;
          case 'p':
            cmd.type = CMD_VEL_P;
            break;
          case 'i':
            cmd.type = CMD_VEL_I;
            break;
          case 'd':
            cmd.type = CMD_VEL_D;
            break;
          case 'f':
            cmd.type = CMD_VEL_FF;
            break;
          case 'l':
            cmd.type = CMD_INPUT_LPF;
            break;
          case 'Q':
            cmd.type = CMD_POS_LPF;
            break;
          case 'q':
            cmd.type = CMD_VEL_LPF;
            break;
          case 'U':
            cmd.type = CMD_POS_RAMP;
            break;
          case 'u':
            cmd.type = CMD_VEL_RAMP;
            break;
          case 'n':
            cmd.type = CMD_POS_MIN;
            break;
          case 'x':
            cmd.type = CMD_POS_MAX;
            break;
          case 'O':
            cmd.type = CMD_OFFSET;
            break;
          case 'L':
            cmd.type = CMD_LEVEL_MODE;
            break;
          case 'B':
            cmd.type = CMD_SEQ_MODE;
            break;
          case 'J':
            cmd.type = CMD_SEQ_KEYFRAME;
            break;
          case 'W':
            cmd.type = CMD_CALIBRATE;
            break;
          case 'A':
            if (cmd.actuator_id == 1)
              cmd.type = CMD_LEVEL_PITCH;
            else if (cmd.actuator_id == 2)
              cmd.type = CMD_LEVEL_ROLL;
            else
              cmd.type = CMD_UNKNOWN;
            break;
          default:
            cmd.type = CMD_UNKNOWN;
            break;
          }
        } else {
          cmd.type = CMD_UNKNOWN;
        }
      } else {
        cmd.type = CMD_UNKNOWN;
      }

      buffer = ""; // Reset buffer after parsing
      return cmd;  // Return the parsed command immediately
    } else {
      buffer += c;
    }
  }

  return cmd; // Returns CMD_NONE if no complete command was parsed
}
