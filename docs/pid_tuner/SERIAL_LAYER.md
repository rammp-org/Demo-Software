# Serial Layer Reference

The Python app must handle 460800 baud serial traffic without stuttering the GUI. This is done by separating the physical port reading onto a background `QThread`.

## 1. `SerialHandler` (`serial_driver/serial_handler.py`)

**Lines: 1-406**

### Core Responsibilities

- Managing the PySerial connection lifecycle.
- Running a tight `while True:` loop on a background thread.
- Exposing thread-safe methods (Slots) for the UI to send commands out.
- Emitting 7 signals: `data_received`, `config_received`, `raw_lines_received`, `connection_changed`, `error_occurred`, `seq_ack_received`, and `seq_status_received`.

### Key Logical Sections

- **Connection Logic (Lines 85-136):** `connect()` and `disconnect_port()`.
- **The Read Loop (Lines 280-312):** `_read_loop()` reads `\n` terminated strings. It uses `ProtocolParser.parse_line()` to convert strings into data objects.
- **Batched Emission (Lines 335-381):** `_emit_batched_data()` processes buffered lines and emits signals at a controlled rate (20Hz) to prevent UI overload.
- **Write Methods (Lines 149-278):** Functions like `set_target()`, `set_pid()`, `request_config()`. They use `ProtocolEncoder` to create ASCII strings and dump them to the port.
- **Sequence Methods (Lines 383-406):** Methods for managing automated sequences, including `enter_sequence_mode()`, `send_keyframe()`, and `seq_step_forward()`.

## 2. `ProtocolParser` & `ProtocolEncoder` (`serial_driver/protocol.py`)

**Lines: 1-601**

### Core Responsibilities

- Converting between python function arguments and the ASCII protocol described in [SERIAL_PROTOCOL.md](../shared/SERIAL_PROTOCOL.md).

### Key Logical Sections

- **Data Classes (Lines 41-156):** `EncoderData`, `ConfigData`, `SeqAckData`, and `SeqStatusData` definitions.
- **`ProtocolParser` (Lines 158-357):** Translates incoming ASCII strings. Returns data objects identifying the packet type.
- **`ProtocolEncoder` (Lines 360-601):** Static methods to build output strings like `T1:5000\n` or `J0:10,20,30,40,50,60,70,80,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,1000,1000,1000,1000,1000,1000,1000,1000\n`.
