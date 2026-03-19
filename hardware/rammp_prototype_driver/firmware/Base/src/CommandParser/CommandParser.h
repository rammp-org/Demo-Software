#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <Arduino.h>

enum CommandType {
    CMD_T,       // e.g. T<id>:<val> (Target)
    CMD_M,       // e.g. M<id>:<val> (Mode)
    CMD_POS_P,   // e.g. P<id>:<val> (Position Kp)
    CMD_POS_I,   // e.g. I<id>:<val> (Position Ki)
    CMD_POS_D,   // e.g. D<id>:<val> (Position Kd)
    CMD_POS_FF,  // e.g. F<id>:<val> (Position Feed-Forward)
    CMD_VEL_P,   // e.g. p<id>:<val> (Velocity Kp)
    CMD_VEL_I,   // e.g. i<id>:<val> (Velocity Ki)
    CMD_VEL_D,   // e.g. d<id>:<val> (Velocity Kd)
    CMD_VEL_FF,  // e.g. f<id>:<val> (Velocity Feed-Forward)
    CMD_R,       // e.g. R<id> (Reset PID state - clear integrator windup)
    CMD_HOME,    // e.g. H<id> (Home/zero encoder position)
    CMD_DIR,     // e.g. V<id> (Toggle motor direction - V for inVert)
    CMD_ENC_DIR, // e.g. E<id> (Toggle encoder direction)
    CMD_SAVE_CONFIG, // e.g. K<id> (Save config to EEPROM for joint)
    CMD_GET_CONFIG,  // e.g. G<id> (Get config from EEPROM for joint)
    CMD_INPUT_LPF,   // e.g. l<id>:<val> (Motor input LPF alpha)
    CMD_POS_LPF,     // e.g. Q<id>:<val> (Position PID output LPF alpha)
    CMD_VEL_LPF,     // e.g. q<id>:<val> (Velocity PID output LPF alpha)
    CMD_Z,       // e.g. z (ESTOP)
    CMD_C,       // e.g. c (Clear ESTOP)
    CMD_LEVEL_MODE, // e.g. L1:1 (On), L1:0 (Off)
    CMD_LEVEL_PITCH, // e.g. A1:<pitch>
    CMD_LEVEL_ROLL,  // e.g. A2:<roll>
    CMD_UNKNOWN,
    CMD_NONE
};

struct RobotCommand {
    CommandType type;
    int actuator_id;
    float value;
};

class CommandParser {
public:
    CommandParser(unsigned long timeout_ms = 500);

    // Read available characters from Serial and parse
    RobotCommand parse(Stream& serial);

    // Check if watchdog timeout has occurred
    bool isTimedOut();

    // Reset watchdog timer manually if needed
    void feedWatchdog();

private:
    unsigned long last_heartbeat;
    unsigned long timeout_ms;
    String buffer;
};

#endif
