# RAMMP / MEBot Software Documentation

Welcome to the central documentation hub for the RAMMP / MEBot prototype software ecosystem.

This repository contains two main software components that communicate with each other over a high-speed serial interface (115200 baud by default, updated to 460800 baud on Jetson/Teensy hardware):

1. **[PID Tuner Application](pid_tuner/README.md)**: A Python GUI application built with PyQt6 and pyqtgraph for real-time monitoring, tuning, and control of the robot's joints.
1. **[Base Firmware](firmware/README.md)**: The C++ Arduino/Teensy firmware responsible for hard real-time motor control, PID execution, sensor integration (IMU, Encoders), and system safety.

## System Architecture

Start with the [System Architecture Guide](ARCHITECTURE.md) for a high-level flowchart of how the Python Application and the Teensy Firmware interact.

## Documentation Structure

### 🖥️ Python Application (`pid_tuner`)

The UI and application layer documentation can be found in `docs/pid_tuner/`.

- [Overview & Quick Start](pid_tuner/README.md)
- [Architecture & Flowchart](pid_tuner/ARCHITECTURE.md)
- [UI Components Layer](pid_tuner/UI_LAYER.md)
- [Data Storage & Processing Layer](pid_tuner/DATA_LAYER.md)
- [Serial Communication Layer](pid_tuner/SERIAL_LAYER.md)
- [Qt Signals Reference](pid_tuner/SIGNALS_REFERENCE.md)

### 🤖 Hardware Firmware (`firmware`)

The Teensy C++ driver and control logic documentation can be found in `docs/firmware/`.

- [Overview & Quick Start](firmware/README.md)
- [Firmware Architecture](firmware/ARCHITECTURE.md)
- [State Machine & Behavior](firmware/STATE_MACHINE.md)
- [Motor Control & PID](firmware/MOTOR_CONTROL.md)
- [PID Controller — Algorithm Deep Dive](firmware/PID_CONTROLLER.md)
- [Encoder Layer](firmware/ENCODER_LAYER.md)
- [IMU Layer](firmware/IMU_LAYER.md)
- [Self-Leveling Kinematics](firmware/SELF_LEVELING.md)
- [Command Parsing](firmware/COMMAND_REFERENCE.md)
- [Telemetry System](firmware/TELEMETRY.md)

### 🔗 Shared Interface (`shared`)

Documentation detailing the explicit agreements between the Python app and the Firmware can be found in `docs/shared/`.

- [Serial Protocol Specification](shared/SERIAL_PROTOCOL.md)
- [Joint Mapping & Definitions](shared/JOINT_MAPPING.md)
- [EEPROM Configuration Storage](shared/CONFIG_STORAGE.md)
