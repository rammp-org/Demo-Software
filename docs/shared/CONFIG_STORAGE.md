# Configuration Storage

The Teensy 4.1's EEPROM is used to durably store the PID tuning parameters, motor inversions, and most importantly, the **saved position offsets**. This ensures that if the Teensy reboots, the joints don't lose their logical position reference.

## Memory Map

`hardware/rammp_prototype_driver/firmware/Base/src/ConfigStorage/ConfigStorage.h`

| Address         | Length (bytes) | Description                                                           |
| --------------- | -------------- | --------------------------------------------------------------------- |
| `0`             | 2              | Magic Number `0xABD1`. Used to detect if EEPROM has been initialized. |
| `10`            | ~68            | Motor 1 Config (`MotorConfig` struct)                                 |
| `10 + (1 × 68)` | ~68            | Motor 2 Config                                                        |
| `10 + (2 × 68)` | ~68            | Motor 3 Config                                                        |
| ...             | ...            | ...                                                                   |
| `10 + (7 × 68)` | ~68            | Motor 8 Config (Drive LR)                                             |

There are 8 motor config slots total (`NUM_MOTORS = 8`), matching the 8 joints defined in [Joint Mapping](JOINT_MAPPING.md).

> **Note:** The magic number was bumped from `0xABD0` → `0xABD1` when the system was expanded from 6 to 8 motors (adding the drive wheel controllers). On first boot after this change, the EEPROM is detected as uninitialized and defaults are written for all 8 slots.

## `MotorConfig` Struct

Each joint saves the following struct:

```cpp
struct MotorConfig {
    int8_t motor_dir;         // +1 or -1
    int8_t encoder_dir;       // +1 or -1
    float lpf_input_alpha;    // 0.0 to 1.0
    float pos_p, pos_i, pos_d, pos_ff;
    float pos_lpf_alpha;
    float pos_max_ramp_rate;  // Maximum output change per second
    float vel_p, vel_i, vel_d, vel_ff;
    float vel_lpf_alpha;
    float vel_max_ramp_rate;  // Maximum output change per second
    float saved_position;     // Float absolute position at last save
    int32_t pos_limit_min;    // Minimum encoder tick limit
    int32_t pos_limit_max;    // Maximum encoder tick limit
};
```

## Save Logic

- Triggered manually from the Python GUI via the "Save Config" button (sends `K<id>`).
- Saving `K0` iterates through and saves all 8 motor configs simultaneously.
- **Auto-save on disconnect:** If the Teensy has ever received a valid command (i.e., a serial connection was established), all 8 motor configs are automatically saved when the watchdog timeout fires. This ensures the last known position is preserved even if the host PC crashes.
- When `Base.ino` initializes, it checks the Magic Number. If invalid (first run), it writes defaults.
- On boot, the `saved_position` is read, and the `EncoderContainer` offset is adjusted so that the current physical position matches the `saved_position`.

## Validation

On load, `ConfigStorage` sanitizes each field:

- **Directions:** Validated to `±1` — reset to `+1` if corrupted.
- **Saved position:** `NaN`, `Inf`, or values outside `±10,000,000` are rejected and reset to `0.0`.
- **Float gains:** `NaN` and `Inf` values are sanitized to `0.0` by `Base.ino` during the boot restore sequence.

## Drive Wheel Encoder Direction

Motors 7 and 8 (drive wheels) have a special encoder direction handling: the firmware maintains global `ml_enc_dir` / `mr_enc_dir` variables that are the runtime source of truth for drive wheel encoder direction. These are saved to and restored from the motor 7/8 config slots, but the `Motor` objects themselves always use `encoder_dir = 1` (the direction correction is applied externally in the drive kinematics).
