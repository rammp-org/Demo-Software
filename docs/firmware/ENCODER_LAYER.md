# Encoder Layer

The `EncoderContainer` class (`src/EncoderContainer/EncoderContainer.cpp`) manages all 12 quadrature encoders on the Teensy 4.1. It handles raw hardware reads, applies per-encoder offset zeroing, and exposes a filtered position array to the rest of the firmware.

## Class Overview

```
src/EncoderContainer/
├── EncoderContainer.h   (35 lines)  — Member declarations, Encoder objects
└── EncoderContainer.cpp (72 lines)  — retrieve_readings(), zeroEncoder(), getRawReading()
```

______________________________________________________________________

## Physical Encoder Pin Mapping

The Teensy 4.1 has dedicated hardware quadrature decoder pins. Each `Encoder` object is constructed with two pins (A, B channels). There are 12 encoders instantiated, though only 6 are actively used by the current motor mapping.

| Object  | Pins (A, B) | Physical Location (per header comment) |
| ------- | ----------- | -------------------------------------- |
| `Enc1`  | 3, 2        | RC top                                 |
| `Enc2`  | 1, 0        | RC bottom                              |
| `Enc3`  | 5, 4        | FC top                                 |
| `Enc4`  | 7, 6        | FC bottom                              |
| `Enc5`  | 9, 8        | ML drive wheel                         |
| `Enc6`  | 11, 10      | ML front                               |
| `Enc7`  | 24, 12      | ML carriage                            |
| `Enc8`  | 26, 25      | MR drive wheel                         |
| `Enc9`  | 28, 27      | MR carriage                            |
| `Enc10` | 30, 29      | MR front                               |
| `Enc11` | 32, 31      | ML back                                |
| `Enc12` | 36, 37      | MR back                                |

> **Note:** There is a `// TODO: switch enc1 and enc2 once they are switched back` comment in the header, indicating `Enc1` and `Enc2` were physically swapped at some point and may not yet reflect the final wiring state.

______________________________________________________________________

## Logical Array Index Mapping (`encoderf[]`)

This is the most important — and most confusing — aspect of this class. The `Encoder` objects do **not** map sequentially to the `encoder[]` / `encoderf[]` arrays. The `retrieve_readings()` function (Lines 3–32) manually assigns each hardware encoder to a specific array index:

| Array Index (`encoderf[N]`) | Encoder Object | Physical Location | Used By Motor               |
| --------------------------- | -------------- | ----------------- | --------------------------- |
| `[1]`                       | `Enc2`         | RC top            | *(unused by active joints)* |
| `[2]`                       | `Enc4`         | FC bottom         | `fc` (Joint 2)              |
| `[3]`                       | `Enc1`         | RC bottom         | `rc` (Joint 1)              |
| `[4]`                       | `Enc3`         | FC top            | *(unused by active joints)* |
| `[5]`                       | `Enc12`        | MR back           | `mr` (Joint 4)              |
| `[6]`                       | `Enc6`         | ML front          | *(unused — commented out)*  |
| `[7]`                       | `Enc11`        | ML back           | `ml` (Joint 3)              |
| `[8]`                       | `Enc10`        | MR front          | *(unused — commented out)*  |
| `[9]`                       | `Enc5`         | ML drive wheel    | *(unused by active joints)* |
| `[10]`                      | `Enc8`         | MR drive wheel    | *(unused by active joints)* |
| `[11]`                      | `Enc7`         | ML carriage       | `ml_carriage` (Joint 5)     |
| `[12]`                      | `Enc9`         | MR carriage       | `mr_carriage` (Joint 6)     |

The Motor instances in `Base.ino` (Lines 395–400) use these specific indices:

```cpp
rc.updateSensorData(EContr.encoderf[3], dt);
fc.updateSensorData(EContr.encoderf[2], dt);
ml.updateSensorData(EContr.encoderf[7], dt);
mr.updateSensorData(EContr.encoderf[5], dt);
ml_carriage.updateSensorData(EContr.encoderf[11], dt);
mr_carriage.updateSensorData(EContr.encoderf[12], dt);
```

> **There is a verified TODO in `Base.ino:393`** — "verify map encoders to motor positions (I took a guess, but I'm unsure)" — this mapping should be validated on hardware before assuming it is final.

The same index mapping must be mirrored in the `setup()` function for the saved-position offset restore and in the `CMD_HOME` handler. See `Base.ino:362-370` and `Base.ino:594-601`.

______________________________________________________________________

## `retrieve_readings()` — The Filter (Lines 3–32)

After reading raw encoder values and subtracting offsets, `retrieve_readings()` applies a first-order IIR (Infinite Impulse Response) low-pass filter to produce the `encoderf[]` values:

```cpp
encoderf[N] = encoderf[N] + K_sensors * (encoder[N] - encoderf[N]);
```

This is the standard discrete-time form: `output = output + α × (input - output)`, where `K_sensors` is the filter coefficient `α`.

**`K_sensors` is currently `1.0`**, which means `encoderf[N]` equals `encoder[N]` exactly on every cycle — there is no filtering. The variable is a placeholder for future noise-reduction tuning. If encoder noise becomes an issue, lowering `K_sensors` (e.g. to `0.7`) would smooth the position signal without adding code complexity.

Note that `encoderf[6]` and `encoderf[8]` are explicitly commented out in `retrieve_readings()` — those encoder positions are not needed.

______________________________________________________________________

## Zeroing / Homing — `zeroEncoder(int index)` (Lines 34–54)

Triggered when the user sends an `H<id>` (Home) command. The implementation records the current **raw hardware reading** as an offset:

```cpp
encoder_offset[index] = Enc7.read(); // example for index 11
encoderf[index] = 0;                 // snap filtered value to zero immediately
```

On subsequent `retrieve_readings()` calls:

```cpp
encoder[11] = Enc7.read() - encoder_offset[11];
```

This means position `0` is defined as wherever the joint was when `H` was received. The offset is stored in RAM only — it resets to `0` on power cycle. To persist a zero reference across reboots, the position is instead saved to EEPROM via `ConfigStorage` (see `docs/shared/CONFIG_STORAGE.md`).

### Homing vs. EEPROM Position Restore

These are two different mechanisms:

| Mechanism                                       | Trigger                 | Persistence              | Purpose                                                                           |
| ----------------------------------------------- | ----------------------- | ------------------------ | --------------------------------------------------------------------------------- |
| `zeroEncoder(idx)`                              | `H<id>` command         | RAM only, lost on reboot | Set a new logical zero at the current physical position                           |
| `encoder_offset` restored from `saved_position` | `setup()` boot sequence | EEPROM, survives reboot  | Restore the last known logical position so motors resume from where they left off |

The boot restore logic in `Base.ino:376-379` computes the initial `encoder_offset` from the EEPROM-saved position:

```cpp
EContr.encoder_offset[enc_idx] = EContr.getRawReading(enc_idx) -
    (signed long)(conf.saved_position / (float)conf.encoder_dir);
```

This ensures `(raw_reading - offset)` will equal `saved_position` on the first `retrieve_readings()` call.

______________________________________________________________________

## `getRawReading(int index)` (Lines 56–72)

Returns the raw tick count from the hardware encoder **before** any offset is applied. Used exclusively during the boot-time position restore in `setup()` to compute the correct `encoder_offset`.

______________________________________________________________________

## Data Flow Summary

```mermaid
flowchart LR
    HW[Hardware Quadrature Encoders]
    HW -->|Enc.read()| RawRead["encoder[N] = Enc.read() − offset"]
    RawRead -->|IIR filter K_sensors| EF["encoderf[N]"]
    EF -->|encoderf[3,2,7,5,11,12]| Motors["Motor::updateSensorData()"]
    Motors -->|current_pos| PID["PID Loops"]
```
