#ifndef COMMAND_PARSER_H
#define COMMAND_PARSER_H

#include <Arduino.h>

enum CommandType {
    CMD_T,       // e.g. T<id>:<val>
    CMD_S,       // e.g. S<id>:<val>
    CMD_Z,       // e.g. z
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
