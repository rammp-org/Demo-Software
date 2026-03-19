# Plan: LPF Modularization + EEPROM + GUI

## Overview

Three areas of work across 9 files. The goal is to:
1. Add output LPF to each `PIDController` object (per-PID, tunable live)
2. Keep and unify the existing motor-level input LPF (anti-jitter, per-motor, tunable live)
3. Persist all LPF values to EEPROM alongside existing motor config
4. Expose all LPF values in the GUI for live tuning

---

## Task 1 — Firmware: PIDController gets output LPF

**Files:** `src/PIDController/PIDController.h`, `src/PIDController/PIDController.cpp`

### PIDController.h changes
- Add `float lpf_alpha` member (default `1.0` = no filtering / passthrough)
- Add `float _filtered_output` private state for IIR accumulator
- Add `void setLpfAlpha(float alpha)` setter (clamps input to [0.0, 1.0])
- Add `float getLpfAlpha()` getter

### PIDController.cpp changes
- In `compute()`, after clamping output:
  ```cpp
  _filtered_output += lpf_alpha * (output - _filtered_output);
  return _filtered_output;
  ```
- In `reset()`: also zero `_filtered_output` to prevent stale-value spikes on re-activation

---

## Task 2 — Firmware: Motor input LPF cleanup + command wiring

**Files:** `src/Motor/Motor.h`, `src/Motor/Motor.cpp`, `src/CommandParser/CommandParser.h`, `src/CommandParser/CommandParser.cpp`, `Base.ino`

### Motor.h changes
- Unify `lpf_vel_alpha` and the currently-unused `lpf_pos_alpha` into a single `lpf_input_alpha` (default `0.5`)
- Add `void setInputLpfAlpha(float alpha)` setter (clamps to [0.0, 1.0])

### Motor.cpp changes
- In `updateSensorData()`, apply `lpf_input_alpha` IIR to both the position reading and the computed velocity:
  ```cpp
  // position input LPF
  current_pos += lpf_input_alpha * (raw_pos - current_pos);
  // velocity input LPF
  current_vel += lpf_input_alpha * (raw_vel - current_vel);
  ```

### CommandParser.h/.cpp changes
- Add 3 new `CommandType` entries:
  | Char | Type | Format | Description |
  |------|------|--------|-------------|
  | `l` | `CMD_INPUT_LPF` | `l<id>:<val>` | Motor input LPF alpha (anti-jitter) |
  | `Q` | `CMD_POS_LPF`   | `Q<id>:<val>` | Position PID output LPF alpha |
  | `q` | `CMD_VEL_LPF`   | `q<id>:<val>` | Velocity PID output LPF alpha |

### Base.ino changes
- In the `TUNER_MODE` dispatch block, add cases:
  - `CMD_INPUT_LPF` → `motor.setInputLpfAlpha(value)`
  - `CMD_POS_LPF`   → `motor.pos_pid.setLpfAlpha(value)`
  - `CMD_VEL_LPF`   → `motor.vel_pid.setLpfAlpha(value)`
- Update `CMD_GET_CONFIG` (`G`) Serial print to also send the 3 new LPF values (11 total fields instead of 8)

---

## Task 3 — Firmware: EEPROM storage update

**Files:** `src/ConfigStorage/ConfigStorage.h`, `src/ConfigStorage/ConfigStorage.cpp`

### MotorConfig struct — extended
```cpp
struct MotorConfig {
    int8_t motor_dir;        // 1 or -1
    int8_t encoder_dir;      // 1 or -1
    float lpf_input_alpha;   // NEW: motor-level sensor input LPF (anti-jitter)
    float pos_p, pos_i, pos_d, pos_ff;
    float pos_lpf_alpha;     // NEW: pos PID output LPF alpha
    float vel_p, vel_i, vel_d, vel_ff;
    float vel_lpf_alpha;     // NEW: vel PID output LPF alpha
};
```

### ConfigStorage.h/.cpp changes
- **Bump magic number** from `0xABCD` to `0xABCE` so devices with the old struct layout re-initialize cleanly rather than loading garbage alpha values into the new fields
- In `initializeDefaults()`: set all 3 new LPF fields to `1.0` (no filtering) except `lpf_input_alpha` which defaults to `0.5`
- `saveMotorConfig()` / `loadMotorConfig()`: no structural change needed — `EEPROM.put/get` writes/reads by `sizeof(MotorConfig)` automatically

### Base.ino load path changes
After `loadMotorConfig()`, also apply the new fields:
```cpp
motor.setInputLpfAlpha(cfg.lpf_input_alpha);
motor.pos_pid.setLpfAlpha(cfg.pos_lpf_alpha);
motor.vel_pid.setLpfAlpha(cfg.vel_lpf_alpha);
```

---

## Task 4 — GUI: Protocol updates

**File:** `pid_tuner/serial_driver/protocol.py`

### ConfigData dataclass additions
```python
pos_lpf_alpha: float = 1.0
vel_lpf_alpha: float = 1.0
input_lpf_alpha: float = 1.0
```

### ProtocolParser changes
- The `CONFIG` line currently parses 8 floats. Firmware will now send 11 (adding 3 LPF values).
- Handle backward compatibility: check field count before accessing new indices.

### ProtocolEncoder additions
```python
def set_pos_lpf(self, joint_id: int, alpha: float) -> bytes:
    return f"Q{joint_id}:{alpha:.4f}\n".encode()

def set_vel_lpf(self, joint_id: int, alpha: float) -> bytes:
    return f"q{joint_id}:{alpha:.4f}\n".encode()

def set_input_lpf(self, joint_id: int, alpha: float) -> bytes:
    return f"l{joint_id}:{alpha:.4f}\n".encode()
```

---

## Task 5 — GUI: Control panel LPF fields

**File:** `pid_tuner/ui/control_panel.py`

### In `_create_pid_group()` — for each PID group (pos and vel)
- Add a **"LPF α"** `QLineEdit` field alongside the existing P/I/D/FF inputs
- Add a "Set" button that sends `Q{id}:{val}` (pos) or `q{id}:{val}` (vel)
- Populate the field from `ConfigData.pos_lpf_alpha` / `vel_lpf_alpha` in `_on_config_updated()`

### In `_create_target_group()` — motor-level input LPF
- Add an **"Input LPF α"** `QLineEdit` + "Set" button in the motor config section
- Sends `l{id}:{val}` command
- Populated from `ConfigData.input_lpf_alpha` in `_on_config_updated()`

---

## Potential Issues / Notes

1. **`CMD_GET_CONFIG` response format** — The firmware `G` command handler must be updated to print all 11 values (8 original + 3 LPF). The `ProtocolParser` split count must match. Handle old 8-value responses gracefully in the GUI.

2. **`lpf_pos_alpha` currently unused in Motor** — The field exists in `Motor.h` but `updateSensorData()` never applies it. This plan resolves the inconsistency by unifying both into `lpf_input_alpha`.

3. **Magic number bump is required** — Changing `MotorConfig` struct size without bumping the magic number will load garbage into the new alpha fields on existing hardware. `0xABCD` → `0xABCE`.

4. **`_filtered_output` must be zeroed in `reset()`** — Prevents a stale filtered value from causing an output spike when a PID re-activates after a reset or mode change.

5. **Alpha bounds enforcement** — All `setLpfAlpha()` and `setInputLpfAlpha()` setters clamp input to `[0.0, 1.0]` to prevent invalid IIR states.

6. **Default behavior is unchanged** — All new LPF alphas default to `1.0` (output LPF) or `0.5` (input LPF, matching current behavior). Existing hardware with re-initialized EEPROM will behave the same as before.

---

## Files to be Modified (9 total)

| File | Change |
|------|--------|
| `hardware/.../src/PIDController/PIDController.h` | Add `lpf_alpha`, `_filtered_output`, setters/getters |
| `hardware/.../src/PIDController/PIDController.cpp` | Apply output LPF in `compute()`, zero in `reset()` |
| `hardware/.../src/Motor/Motor.h` | Unify to `lpf_input_alpha`, add setter |
| `hardware/.../src/Motor/Motor.cpp` | Apply input LPF to both pos and vel in `updateSensorData()` |
| `hardware/.../src/CommandParser/CommandParser.h` | Add `CMD_INPUT_LPF`, `CMD_POS_LPF`, `CMD_VEL_LPF` enum values |
| `hardware/.../src/CommandParser/CommandParser.cpp` | Parse `l`, `Q`, `q` command characters |
| `hardware/.../src/ConfigStorage/ConfigStorage.h` | Extend `MotorConfig` struct with 3 LPF floats |
| `hardware/.../src/ConfigStorage/ConfigStorage.cpp` | Bump magic number, update defaults, wire load/save |
| `hardware/.../Base.ino` | Wire new commands, extend `G` response, apply loaded LPF values |
| `pid_tuner/serial_driver/protocol.py` | Extend `ConfigData`, add encoder methods, update parser |
| `pid_tuner/ui/control_panel.py` | Add LPF fields to PID groups + motor input LPF field |
