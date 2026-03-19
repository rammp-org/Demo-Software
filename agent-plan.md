# Plan: Persistent Position & Motor Limits Implementation

## Goal
Add a "Save Robot Position State" button to the GUI that sends a command to the firmware to save encoder locations to persistent memory, allowing motor control code to pull those saved memory on start up. Additionally, add motor limits to the GUI and firmware, treating them like limit switches, and scale the GUI encoder bars to these relative limits.

## 1. Firmware Updates (`firmware/base/src/ConfigStorage/ConfigStorage.h`)
- **Extend `MotorConfig` Struct:** Add `int32_t saved_position`, `int32_t pos_limit_min`, and `int32_t pos_limit_max` to the EEPROM configuration memory layout. 

## 2. Motor Logic & Limit Switches (`firmware/base/src/Motor/`)
- **Update `Motor.h` / `Motor.cpp`:** Add `pos_limit_min`, `pos_limit_max`, and a boolean flag `limits_enabled` (true if `min != max`).
- **Enforce Limits:** In `Motor::update()`, add logic to check if `limits_enabled` is true. If `current_pos <= pos_limit_min` and the `target_pwm < 0`, force `target_pwm = 0`. Apply the same logic for `pos_limit_max` and `target_pwm > 0`. This replicates the exact behavior of physical limit switches in software.

## 3. Command Processing & Startup (`firmware/base/base.ino`)
- **Startup Routine (`setup()`):**
  - When initializing the 6 `Motor` objects from `ConfigStorage`, load `pos_limit_min` and `pos_limit_max`.
  - Load `saved_position` and use it to offset the `EncoderContainer`: `EContr.encoder_offset[i] = EContr.getRawReading(i) - conf.saved_position;`. This ensures the motor inherently resumes its exact position from before it was powered off.
- **Save State Command (`CMD_SAVE_CONFIG` / `K`):** 
  - Enhance the existing `K` command to support an `actuator_id` of `0` (e.g., `K0`), which iterates over all 6 motors to update their individual `MotorConfig` values.
  - When saving, copy `m->current_pos` into `conf.saved_position`, alongside saving the existing limits and PIDs to EEPROM.
- **New Serial Commands:** 
  - Register `CMD_POS_MIN` (e.g., `n<id>:<val>`) and `CMD_POS_MAX` (e.g., `x<id>:<val>`) in `CommandParser` to allow the GUI to update limits over serial.
  - Append the `pos_limit_min` and `pos_limit_max` values to the outgoing `CONFIG,` telemetry string.

## 4. Python Protocol & Serial Updates (`pid_tuner/serial_driver/`)
- **Update `ConfigData` (`protocol.py`):** Append `pos_limit_min` and `pos_limit_max` fields to the dataclass.
- **Update Parser:** Modify `ProtocolParser.parse_line` to extract the new limit fields from the extended `CONFIG,` message.
- **Add Encoders:** Add `set_pos_limit_min(joint_id, val)` and `set_pos_limit_max(joint_id, val)` functions to `ProtocolEncoder` and expose them in `SerialHandler`.

## 5. GUI Control Panel (`pid_tuner/ui/control_panel.py`)
- **New Buttons:** Add a `QPushButton` labeled **"Save Robot Position State"** next to "Save to EEPROM". When clicked, this will transmit the command (`K0`) to save all 6 motor positions to persistent memory at once.
- **Limit Interfaces:** Add two new input spinboxes for "Min Limit" and "Max Limit" to the motor configuration section.
- **Data Binding:** Wire these new spinboxes to send the new serial commands upon changes, and have them auto-populate whenever new config data is received from the Teensy via `data_store.config_updated`.

## 6. GUI Encoder Overview Scaling (`pid_tuner/ui/encoder_overview.py`)
- **Dynamic Scaling:** Subscribe `EncoderOverview` to the `DataStore`'s `config_updated` signal.
- **Update Ranges:** When a motor's configuration is loaded from the Teensy, explicitly call `EncoderBar.set_range(config.pos_limit_min, config.pos_limit_max)` for the respective joint. This will automatically scale the bar's render width relative to its functional limits, preventing it from being "blown out" visually.
- **Fallback:** If `pos_limit_min` == `pos_limit_max` (e.g., 0,0), it gracefully falls back to the existing `-50` to `50` scale to avoid dividing by zero render bugs.
