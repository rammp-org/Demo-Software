#include "CommandParser.h"

CommandParser::CommandParser(unsigned long timeout_ms) 
    : last_heartbeat(millis()), timeout_ms(timeout_ms), buffer("") {}

void CommandParser::feedWatchdog() {
    last_heartbeat = millis();
}

bool CommandParser::isTimedOut() {
    return (millis() - last_heartbeat) > timeout_ms;
}

RobotCommand CommandParser::parse(Stream& serial) {
    RobotCommand cmd = {CMD_NONE, -1, 0.0f};

    while (serial.available() > 0) {
        char c = serial.read();
        
        // Command execution occurs on newline
        if (c == '\n' || c == '\r') {
            if (buffer.length() == 0) continue;
            
            feedWatchdog(); // Any valid command feeds the watchdog

            if (buffer.charAt(0) == 'z') {
                cmd.type = CMD_Z;
            } else if (buffer.charAt(0) == 'c') {
                cmd.type = CMD_C;
            } else if (buffer.length() >= 2) { // Need at least R<id> or X<id>:<val>
                char type_char = buffer.charAt(0);
                int colon_idx = buffer.indexOf(':');
                
                // Commands without value (e.g., R1 for reset, H1 for home, V1 for direction)
                if (colon_idx == -1) {
                    cmd.actuator_id = buffer.substring(1).toInt();
                    cmd.value = 0.0f;
                    switch(type_char) {
                        case 'R': cmd.type = CMD_R; break;
                        case 'H': cmd.type = CMD_HOME; break;
                        case 'V': cmd.type = CMD_DIR; break;
                        default: cmd.type = CMD_UNKNOWN; break;
                    }
                }
                // Commands with value (e.g., T1:0.5)
                else if (colon_idx > 1) {
                    cmd.actuator_id = buffer.substring(1, colon_idx).toInt();
                    cmd.value = buffer.substring(colon_idx + 1).toFloat();
                    
                    switch(type_char) {
                        case 'T': cmd.type = CMD_T; break;
                        case 'M': cmd.type = CMD_M; break;
                        case 'P': cmd.type = CMD_POS_P; break;
                        case 'I': cmd.type = CMD_POS_I; break;
                        case 'D': cmd.type = CMD_POS_D; break;
                        case 'F': cmd.type = CMD_POS_FF; break;
                        case 'p': cmd.type = CMD_VEL_P; break;
                        case 'i': cmd.type = CMD_VEL_I; break;
                        case 'd': cmd.type = CMD_VEL_D; break;
                        case 'f': cmd.type = CMD_VEL_FF; break;
                        default: cmd.type = CMD_UNKNOWN; break;
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
