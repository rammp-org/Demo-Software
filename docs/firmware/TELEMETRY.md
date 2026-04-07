# Telemetry System

The Teensy needs to communicate roughly 75 discrete float/int values back to the PC to keep the UI in sync. This includes positions, velocities, PWM outputs, control modes, limit switch states, IMU data, quaternions, self-leveling debug values, strain gauge readings, and drive wheel telemetry. To prevent serial buffer overflow and maintain performance, Telemetry is strictly emitted at **10Hz** using a non-blocking millis() timer.

```cpp
// Base.ino snippet (Lines 740-744)
static unsigned long last_telem_time = 0;
if (millis() - last_telem_time >= 100) { // Fixed 10Hz telemetry
    last_telem_time = millis();
    sendTelemetry();
}
```

## Module Structure

The telemetry logic was extracted from `Base.ino` into a dedicated module:

```
src/Telemetry/
├── Telemetry.h   — SystemState enum, SystemTelemetry struct, function declarations
└── Telemetry.cpp — updateTelemetry(), sendTelemetry()
```

### `SystemState` Enum

Defined in `Telemetry.h`, the 7 firmware states are:

```cpp
enum SystemState {
  INIT, IDLE, TUNER_MODE, ESTOP,
  SELF_LEVELING, CONFIGURATION, AUTO_CURB_CLIMBING
};
```

### `SystemTelemetry` Struct

Groups all telemetry fields before serialization. Key arrays:

| Field                     | Size | Description                                   |
| ------------------------- | ---- | --------------------------------------------- |
| `positions[6]`            | 6    | Joint 1-6 current positions                   |
| `velocities[6]`           | 6    | Joint 1-6 current velocities                  |
| `pwms[6]`                 | 6    | Joint 1-6 target PWMs                         |
| `directions[6]`           | 6    | Motor direction (±1) for joints 1-6           |
| `enc_directions[6]`       | 6    | Encoder direction (±1) for joints 1-6         |
| `limit_switches[4]`       | 4    | Carriage limit switch states                  |
| `imu[6]`                  | 6    | pitch, roll, yaw, ax, ay, az                  |
| `quat[4]`                 | 4    | Swing quaternion w, x, y, z                   |
| `leveling[5]`             | 5    | pitch_err, roll_err, z_ml, z_rc, z_mr         |
| `sg[4]`                   | 4    | Strain gauge values (RC, FC, ML, MR)          |
| `modes[6]`                | 6    | Control mode per joint (0=Open, 1=Vel, 2=Pos) |
| `drive_positions[2]`      | 2    | Drive FB/LR positions                         |
| `drive_velocities[2]`     | 2    | Drive FB/LR velocities                        |
| `drive_pwms[2]`           | 2    | Drive FB/LR PWMs                              |
| `drive_modes[2]`          | 2    | Drive FB/LR control modes                     |
| `raw_enc_positions[2]`    | 2    | Raw ML/MR encoder positions                   |
| `raw_enc_velocities[2]`   | 2    | Raw ML/MR encoder velocities                  |
| `drive_directions[2]`     | 2    | Drive FB/LR motor directions                  |
| `drive_enc_directions[2]` | 2    | Drive FB/LR encoder directions                |

## `sendTelemetry()`

Located in `Telemetry.cpp`. This function builds the entire 75-value CSV line into a single `char buf[800]` using `snprintf()`, then emits it with a single `Serial.print(buf)` call.

Why not use multiple `Serial.print()` calls? The original approach with ~50 consecutive prints was replaced with a single buffer to reduce per-call overhead and ensure atomic writes to the UART.

Why not build a `String`? String concatenation in C++ (especially on microcontrollers) causes heap fragmentation. Using `snprintf()` into a fixed stack buffer is faster, safer, and uses less RAM.

For the exact field-by-field structure of the telemetry payload, see the [Serial Protocol Specification](../shared/SERIAL_PROTOCOL.md).
