# RAMMP / MEBot Firmware Documentation

This directory documents the C++ Teensy 4.1 Firmware found in `hardware/rammp_prototype_driver/firmware/Base/`.

The firmware provides a 100-200Hz hard real-time control loop for 6 independent robotic joints using a cascaded PID architecture (Position -> Velocity -> PWM), while continuously managing safety bounds and self-leveling kinematics.

## Navigation

### High-Level

- [**Architecture**](ARCHITECTURE.md): The overall flow of `Base.ino`, boot sequence, `src/` module table, and loop timing.
- [**State Machine**](STATE_MACHINE.md): All 7 `SystemState` values — behaviors, transition triggers, and implementation locations in `Base.ino`.

### Control & Actuation

- [**Motor Control & PID**](MOTOR_CONTROL.md): `Motor` class — control modes, cascaded PID fallthrough, `disable()` safety, software limits, direction multipliers, and RoboClaw dispatch.
- [**PID Controller**](PID_CONTROLLER.md): `PIDController` class — the complete algorithm: `scaling` divisor, conditional anti-windup, output LPF, and feed-forward.

### Sensors

- [**Encoder Layer**](ENCODER_LAYER.md): `EncoderContainer` — physical pin map, logical array index mapping, `K_sensors` filter, and offset/zeroing mechanism.
- [**IMU Layer**](IMU_LAYER.md): `IMU_Class` — BNO055 upside-down mount fix, quaternion→Euler conversion, roll +180° offset, `extractSwing()` yaw decomposition.

### Autonomous Behaviors

- [**Self-Leveling Kinematics**](SELF_LEVELING.md): `runSelfLeveling()` — quaternion error decomposition, rotation matrix construction, 4×4 chassis geometry transform, and tick dispatch.

### Communication

- [**Command Reference**](COMMAND_REFERENCE.md): `CommandParser` — full `CommandType` enum table, parsing state machine, watchdog timer, and the `A`-command sub-discriminator quirk.
- [**Telemetry**](TELEMETRY.md): How the Teensy builds and transmits the 10Hz telemetry string.
