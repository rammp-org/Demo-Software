# Serial Layer Reference

The Python app must handle 460800 baud serial traffic without stuttering the GUI. This is done by separating the physical port reading onto a background `QThread`.

## 1. `SerialHandler` (`serial_driver/serial_handler.py`)
**Lines: 1-342**

### Core Responsibilities
- Managing the PySerial connection lifecycle.
- Running a tight `while True:` loop on a background thread.
- Exposing thread-safe methods (Slots) for the UI to send commands out.

### Key Logical Sections
- **Connection Logic (Lines 77-126):** `connect()` and `disconnect()`.
- **The Read Loop (Lines 128-193):** `_read_loop()` reads `\n` terminated strings. It uses `ProtocolParser.parse_line()` to convert strings into arrays, and then emits `data_received(list)` to the Main Thread. It also emits `raw_line_received(str)` for the Console widget.
- **Write Methods (Lines 195-342):** Functions like `set_target()`, `set_pid()`, `request_config()`. They use `ProtocolEncoder` to create ASCII strings and dump them to the port.

## 2. `ProtocolParser` & `ProtocolEncoder` (`serial_driver/protocol.py`)
**Lines: 1-408**

### Core Responsibilities
- Converting between python function arguments and the ASCII protocol described in [SERIAL_PROTOCOL.md](../shared/SERIAL_PROTOCOL.md).

### Key Logical Sections
- **Data Classes (Lines 16-64):** `TelemetryData` and `ConfigData` definitions.
- **`ProtocolParser` (Lines 66-189):** Translates incoming ASCII strings. Returns Enums identifying the packet type.
- **`ProtocolEncoder` (Lines 191-408):** Static methods to build output strings like `T1:5000\n`.