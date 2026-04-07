# System Architecture

The RAMMP / MEBot system consists of two primary components communicating over a fast Serial link (460800 baud).

- **Python GUI Application (`pid_tuner`)**: A high-level control interface running on a host PC (or Jetson), providing a user interface for tuning PID parameters, overriding targets, monitoring telemetry, and issuing high-level mode commands.
- **Teensy 4.1 Firmware (`firmware/Base`)**: Hard real-time execution engine for running 16 independent PID loops across 8 motor controllers (6 actuated joints + 2 body-frame drive wheels). It handles low-level PWM outputs to RoboClaws, encoder reads, IMU polling, EEPROM configuration management, and automated curb-climbing sequences.

## High Level Flowchart

```mermaid
flowchart TB
    subgraph Host PC [Host PC / Jetson]
        direction TB
        UI[PyQt6 GUI]
        DataStore[(Data Store)]
        PythonSerial[Serial Handler Thread]
        
        UI <-->|Qt Signals| DataStore
        DataStore <-->|Qt Signals| PythonSerial
    end

    subgraph Hardware [Teensy 4.1 Board]
        direction TB
        CommandParser[Command Parser]
        StateMachine[State Machine]
        PID_Loop[8x Motor PID Loops]
        EEPROM[(ConfigStorage EEPROM)]
        
        Sensors[Encoders / IMU / Limits]
        RoboClaws[3x RoboClaws]
        
        CommandParser --> StateMachine
        CommandParser --> EEPROM
        StateMachine --> PID_Loop
        Sensors --> PID_Loop
        PID_Loop --> RoboClaws
    end

    PythonSerial <==>|UART 460800 Baud| CommandParser
    PythonSerial <..>|Telemetry Data| Sensors
    
    classDef hardware fill:#eef,stroke:#333,stroke-width:2px;
    classDef software fill:#efe,stroke:#333,stroke-width:2px;
    
    class Host PC software
    class Hardware hardware
```

## Communication Protocol Highlights

- **Downlink (PC -> Teensy):** Short ASCII commands. E.g., `T3:1000` (Set Target for Joint 3 to 1000 ticks), `P1:5.2` (Set Position P gain for Joint 1 to 5.2).
- **Uplink (Teensy -> PC):** Dense telemetry strings emitted at 10Hz. E.g., `TELEMETRY,<millis>,<state>,<pos1..6>,<vel1..6>,...`.
- **Heartbeat:** If the Python app disconnects or crashes, a watchdog timer (60 seconds) on the Teensy triggers an immediate ESTOP, cutting power to the RoboClaws and auto-saving all motor configs to EEPROM.

## Component Responsibilities

| Component          | Responsibilities                                                                            |
| ------------------ | ------------------------------------------------------------------------------------------- |
| **Python GUI**     | Visualizing telemetry (plots, 3D IMU), exposing tuning knobs, managing configurations.      |
| **Command Parser** | Translating ASCII bytes to structured `RobotCommand` structs, feeding the watchdog.         |
| **State Machine**  | Managing `IDLE`, `TUNER_MODE`, `ESTOP`, `SELF_LEVELING`, `AUTO_CURB_CLIMBING` modes safely. |
| **Motor Class**    | Cascaded PID loops (Position & Velocity), limit switch enforcement, direction abstraction.  |
| **Config Storage** | Abstracting EEPROM to save/load PID gains, limits, and offsets persistently.                |
