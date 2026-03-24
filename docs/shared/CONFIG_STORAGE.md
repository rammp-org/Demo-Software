# Configuration Storage

The Teensy 4.1's EEPROM is used to durably store the PID tuning parameters, motor inversions, and most importantly, the **saved position offsets**. This ensures that if the Teensy reboots, the joints don't lose their logical position reference.

## Memory Map

`hardware/rammp_prototype_driver/firmware/Base/src/ConfigStorage/ConfigStorage.h`

| Address         | Length (bytes) | Description                                                           |
| --------------- | -------------- | --------------------------------------------------------------------- |
| `0`             | 2              | Magic Number `0xABD0`. Used to detect if EEPROM has been initialized. |
| `10`            | ~68            | Motor 1 Config (`MotorConfig` struct)                                 |
| `10 + (1 * 68)` | ~68            | Motor 2 Config                                                        |
| `10 + (2 * 68)` | ~68            | Motor 3 Config                                                        |
| ...             | ...            | ...                                                                   |

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
- Saving `K0` iterates through and saves all 6 joints simultaneously.
- When `Base.ino` initializes, it checks the Magic Number. If invalid (first run), it writes defaults.
- On boot, the `saved_position` is read, and the `EncoderContainer` offset is adjusted so that the current physical position matches the `saved_position`.
