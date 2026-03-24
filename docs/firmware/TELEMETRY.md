# Telemetry System

The Teensy needs to communicate roughly 50 discrete float/int values back to the PC to keep the UI in sync. This includes positions, velocities, limit switch states, IMU data, quaternions, and strain gauge readings. To prevent serial buffer overflow and maintain performance, Telemetry is strictly emitted at **10Hz** using a non-blocking millis() timer.

```cpp
// Base.ino snippet
static unsigned long last_telem_time = 0;
if (millis() - last_telem_time >= 100) { // Fixed 10Hz telemetry
    last_telem_time = millis();
    sendTelemetry();
}
```

## `sendTelemetry()`

Located in `Base.ino`, this function executes roughly 50 consecutive `Serial.print()` statements.

Why not build a single `String`?
String concatenation in C++ (especially on microcontrollers) causes heap fragmentation. Directly printing to the hardware UART buffer is faster, safer, and uses less RAM.

For the exact structure of the telemetry payload, see the [Serial Protocol Specification](../shared/SERIAL_PROTOCOL.md).
