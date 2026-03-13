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
            } else if (buffer.length() > 2) { // Need at least Xid:val
                char type_char = buffer.charAt(0);
                int colon_idx = buffer.indexOf(':');
                
                if (colon_idx > 1) {
                    cmd.actuator_id = buffer.substring(1, colon_idx).toInt();
                    cmd.value = buffer.substring(colon_idx + 1).toFloat();
                    
                    if (type_char == 'T') {
                        cmd.type = CMD_T;
                    } else if (type_char == 'S') {
                        cmd.type = CMD_S;
                    } else {
                        cmd.type = CMD_UNKNOWN;
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
