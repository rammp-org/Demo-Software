# Command Reference & Parser

The `CommandParser` class (`src/CommandParser/CommandParser.cpp`) acts as a non-blocking serial ingestion state machine. It prevents the Teensy from hanging if it receives a malformed or incomplete packet from the PC.

```
src/CommandParser/
├── CommandParser.h   (74 lines)  — CommandType enum, RobotCommand struct, class declaration
└── CommandParser.cpp             — parse(), feedWatchdog(), isTimedOut()

src/CommandDispatch/
├── CommandDispatch.h             — CommandContext struct, dispatch table declaration
└── CommandDispatch.cpp           — Table-driven command handlers for all TUNER_MODE commands
```

______________________________________________________________________

## Parsing Logic (Lines 14–91)

Instead of using `Serial.readStringUntil('\n')` (which can block the tight 5ms control loop if the PC stops sending mid-string), the parser accumulates bytes into a `String buffer` on every `loop()` cycle. The buffer is a class member so it persists across calls.

```mermaid
flowchart TD
    Start([parse called]) --> Avail{serial.available?}
    Avail -- No --> ReturnNone[Return CMD_NONE]
    Avail -- Yes --> ReadChar[Read char]

    ReadChar --> IsNewline{char == newline\nor carriage return?}
    IsNewline -- No --> Append[Append to buffer]
    Append --> Avail

    IsNewline -- Yes --> EmptyBuf{buffer empty?}
    EmptyBuf -- Yes --> Avail
    EmptyBuf -- No --> FeedWD[Feed Watchdog]

    FeedWD --> IsZ{buffer[0] == 'z'?}
    IsZ -- Yes --> CmdZ[CMD_Z]
    IsZ -- No --> IsC{buffer[0] == 'c'?}
    IsC -- Yes --> CmdC[CMD_C]
    IsC -- No --> HasColon{Contains ':'?}

    HasColon -- No --> NoValCmd[Parse ID only\nR/H/V/E/K/G]
    HasColon -- Yes --> WithValCmd[Parse ID + float value\nfrom buffer]

    NoValCmd --> ClearBuf[Clear buffer]
    WithValCmd --> ClearBuf
    CmdZ --> ClearBuf
    CmdC --> ClearBuf
    ClearBuf --> ReturnCmd[Return RobotCommand]
```

### Parsing Steps on Newline

1. Feed the watchdog — any successfully terminated line counts as a heartbeat.
1. Check if buffer starts with `z` or `c` (single-character commands, no ID or value).
1. Slice the first character as the `CommandType` discriminator.
1. Scan for `:`. If absent → parse only `buffer[1..]` as integer `actuator_id`, `value = 0.0`.
1. If `:` is present → parse `buffer[1..colon]` as `actuator_id`, `buffer[colon+1..]` as `float value`.
1. Clear `buffer` and return the populated `RobotCommand`.

If no complete command is ready, returns `{CMD_NONE, -1, 0.0f}`.

______________________________________________________________________

## `RobotCommand` Struct

```cpp
struct RobotCommand {
    CommandType type;   // Enum value identifying the command
    int actuator_id;    // 1-6 for joint-specific; 0 for global; -1 for commands without ID
    float value;        // Numeric argument; 0.0f for no-value commands
};
```

______________________________________________________________________

## `CommandType` Enum — Full Reference

All command types are defined in `CommandParser.h:6-43` and dispatched via the table-driven `CommandDispatch` module (`src/CommandDispatch/CommandDispatch.cpp`).

| `CommandType`       | Char | Format            | Value Meaning                                      | Notes                                                                     |
| ------------------- | ---- | ----------------- | -------------------------------------------------- | ------------------------------------------------------------------------- |
| `CMD_T`             | `T`  | `T<id>:<val>`     | PWM / velocity / position target (depends on mode) | Dispatches to `setTargetPWM`, `setTargetVelocity`, or `setTargetPosition` |
| `CMD_M`             | `M`  | `M<id>:<val>`     | `0`=Open Loop, `1`=Velocity, `2`=Position          | Sets `Motor::ControlMode`                                                 |
| `CMD_POS_P`         | `P`  | `P<id>:<val>`     | Position Kp                                        |                                                                           |
| `CMD_POS_I`         | `I`  | `I<id>:<val>`     | Position Ki                                        |                                                                           |
| `CMD_POS_D`         | `D`  | `D<id>:<val>`     | Position Kd                                        |                                                                           |
| `CMD_POS_FF`        | `F`  | `F<id>:<val>`     | Position Feed-Forward                              |                                                                           |
| `CMD_VEL_P`         | `p`  | `p<id>:<val>`     | Velocity Kp                                        | Lowercase                                                                 |
| `CMD_VEL_I`         | `i`  | `i<id>:<val>`     | Velocity Ki                                        | Lowercase                                                                 |
| `CMD_VEL_D`         | `d`  | `d<id>:<val>`     | Velocity Kd                                        | Lowercase                                                                 |
| `CMD_VEL_FF`        | `f`  | `f<id>:<val>`     | Velocity Feed-Forward                              | **Divided by 10000 before applying** — see note below                     |
| `CMD_INPUT_LPF`     | `l`  | `l<id>:<val>`     | Input LPF alpha (0–1)                              | Lowercase                                                                 |
| `CMD_POS_LPF`       | `Q`  | `Q<id>:<val>`     | Position PID output LPF alpha                      | Uppercase                                                                 |
| `CMD_VEL_LPF`       | `q`  | `q<id>:<val>`     | Velocity PID output LPF alpha                      | Lowercase                                                                 |
| `CMD_POS_RAMP`      | `U`  | `U<id>:<val>`     | Position PID output max ramp rate                  | Uppercase                                                                 |
| `CMD_VEL_RAMP`      | `u`  | `u<id>:<val>`     | Velocity PID output max ramp rate                  | Lowercase                                                                 |
| `CMD_POS_MIN`       | `n`  | `n<id>:<val>`     | Minimum position limit in ticks                    | Lowercase                                                                 |
| `CMD_POS_MAX`       | `x`  | `x<id>:<val>`     | Maximum position limit in ticks                    |                                                                           |
| `CMD_R`             | `R`  | `R<id>`           | Reset PID integrators and filter state             | No value                                                                  |
| `CMD_HOME`          | `H`  | `H<id>`           | Zero encoder for this joint                        | No value                                                                  |
| `CMD_OFFSET`        | `O`  | `O<id>:<val>`     | Set encoder position offset to arbitrary value     | Sets the logical position to `val` without moving the joint               |
| `CMD_DIR`           | `V`  | `V<id>`           | Toggle motor direction                             | `V` = in**V**ert                                                          |
| `CMD_ENC_DIR`       | `E`  | `E<id>`           | Toggle encoder direction                           | No value                                                                  |
| `CMD_SAVE_CONFIG`   | `K`  | `K<id>` or `K0`   | Save config to EEPROM. `K0` saves all 8 joints     | No value                                                                  |
| `CMD_GET_CONFIG`    | `G`  | `G<id>`           | Request CONFIG response for this joint             | No value. Safe during any state including ESTOP.                          |
| `CMD_Z`             | `z`  | `z`               | ESTOP — disable all motors immediately             | No ID, no value                                                           |
| `CMD_C`             | `c`  | `c`               | Clear ESTOP, return to IDLE                        | No ID, no value                                                           |
| `CMD_LEVEL_MODE`    | `L`  | `L1:1` / `L1:0`   | Enable (1) or disable (0) Self-Leveling mode       | ID always `1`                                                             |
| `CMD_LEVEL_PITCH`   | `A`  | `A1:<deg>`        | Set self-leveling target pitch in degrees          | ID = `1` — see note below                                                 |
| `CMD_LEVEL_ROLL`    | `A`  | `A2:<deg>`        | Set self-leveling target roll in degrees           | ID = `2` — see note below                                                 |
| `CMD_SEQ_MODE`      | `B`  | `B1:1` / `B1:0`   | Enter (1) or exit (0) AUTO_CURB_CLIMBING mode      | `B2:1`/`B2:0` toggles auto-run — see note below                          |
| `CMD_SEQ_KEYFRAME`  | `J`  | `J<idx>:<payload>` | Upload keyframe data at index                      | Payload is CSV; 32-value or 17-value format                               |
| `CMD_SEQ_STEP_FWD`  | `>`  | `>`               | Step forward to next keyframe                      | No ID, no value                                                           |
| `CMD_SEQ_STEP_BWD`  | `<`  | `<`               | Step backward to previous keyframe                 | No ID, no value                                                           |
| `CMD_SEQ_GOTO`      | `@`  | `@<idx>`          | Jump directly to keyframe index                    | No value after index                                                      |
| `CMD_UNKNOWN`       | —    | —                 | Unrecognized command                               | Parser returns this for syntax errors                                     |
| `CMD_NONE`          | —    | —                 | No complete command yet this cycle                 | Default return when buffer has no newline                                 |

### Velocity Feed-Forward Scaling Note

`CMD_VEL_FF` receives the value from the GUI and divides it by 10,000 before applying to the PID:

```cpp
case CMD_VEL_FF:
    m->vel_pid.setFeedForward(cmd.value / 10000);
```

This is because the `vel_pid` uses a `scaling = 10000` divisor internally (see [PID Controller](PID_CONTROLLER.md)). The GUI transmits the FF gain in the "raw" user-facing units, and the divide-by-10000 here normalizes it to match the internal scaling — so the FF term operates consistently with the P, I, D terms.

### `B` Command Sub-Discriminator

The sequence mode commands share the character `B`, differentiated by `actuator_id`:

- `B1:1` / `B1:0` — Enter or exit `AUTO_CURB_CLIMBING` mode (triggers `sequenceEnter()` / `sequenceExit()`)
- `B2:1` / `B2:0` — Enable or disable auto-run (automatically advance to next keyframe on completion)

### `A` Command Sub-Discriminator

The pitch and roll self-leveling targets share the same command character `A`, differentiated by the `actuator_id` field rather than a separate character:

```cpp
case 'A':
    if (cmd.actuator_id == 1) cmd.type = CMD_LEVEL_PITCH;  // A1:<pitch>
    else if (cmd.actuator_id == 2) cmd.type = CMD_LEVEL_ROLL;   // A2:<roll>
    else cmd.type = CMD_UNKNOWN;
    break;
```

This is the only place in the parser where `actuator_id` influences the `CommandType` itself rather than just indicating which motor to target. It means `A1` and `A2` are effectively distinct commands with different behaviors, not the same command applied to joints 1 and 2.

______________________________________________________________________

## Watchdog Timer

A critical safety feature. The `CommandParser` holds a `last_heartbeat` timestamp (Lines 57–58 in `.h`). Any complete command line (even `CMD_UNKNOWN`) resets this timer via `feedWatchdog()`.

The watchdog timeout is configured at **60,000ms** (60 seconds) in `Base.ino`:

```cpp
CommandParser parser(60000);
```

> **Note:** The `CommandParser.h` default constructor argument is `500ms`, but `Base.ino` overrides it with `60000ms`. The shorter value in older documentation refers to the default, not the actual deployed configuration.

If `parser.isTimedOut()` returns `true`, `Base.ino` forcibly transitions to `ESTOP` (Lines 536–544):

```cpp
if (parser.isTimedOut() && current_state != ESTOP) {
    current_state = ESTOP;
    Serial.println("WATCHDOG TIMEOUT -> ESTOP");
    if (was_connected) {
        saveAllMotorConfigs();
        Serial.println("AUTO-SAVE: All motor configs saved on disconnect.");
        was_connected = false;
    }
}
```

This ensures all motors cut power if serial communication is lost for over 60 seconds — for example, if the controlling PC crashes or the USB cable is disconnected. Additionally, if a serial connection had previously been established, all 8 motor configs (including current positions) are automatically saved to EEPROM before entering ESTOP. This preserves the last known state across power cycles.
