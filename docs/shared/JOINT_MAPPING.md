# Joint Mapping

The system uses a unified 1-indexed Joint ID system to map physical components to GUI controls. There are 8 joints total: 6 position-controlled actuators and 2 body-frame drive wheel controllers.

## Actuated Joints (1–6)

| Joint ID | Short Name | Full Name           | Firmware Motor Instance | Encoder Array Index (`encoderf`) | RoboClaw Assignment     | Notes                                                 |
| -------- | ---------- | ------------------- | ----------------------- | -------------------------------- | ----------------------- | ----------------------------------------------------- |
| 1        | RC         | Rear Caster         | `rc`                    | 3                                | `roboclaw_casters` M1   | Legacy reference: "RC Bottom (0-850)"                 |
| 2        | FC         | Front Caster        | `fc`                    | 2                                | `roboclaw_casters` M2   |                                                       |
| 3        | ML         | Main Left           | `ml`                    | 7                                | `roboclaw_main` M1      | Left Drive Wheel                                      |
| 4        | MR         | Main Right          | `mr`                    | 5                                | `roboclaw_main` M2      | Right Drive Wheel                                     |
| 5        | ML_C       | Main Left Carriage  | `ml_carriage`           | 11                               | `roboclaw_carriages` M1 | Has associated Limit Switches (`CARRIAGE_SW1`, `SW2`) |
| 6        | MR_C       | Main Right Carriage | `mr_carriage`           | 12                               | `roboclaw_carriages` M2 | Has associated Limit Switches (`CARRIAGE_SW3`, `SW4`) |

## Drive Wheel Controllers (7–8)

These are virtual body-frame velocity controllers derived from the ML and MR encoder readings. They do **not** have dedicated RoboClaw connections — their PWM output is read by an external RNET joystick spoofer.

| Joint ID | Short Name | Full Name        | Firmware Motor Instance | Encoder Array Index (`encoderf`) | Notes                                                     |
| -------- | ---------- | ---------------- | ----------------------- | -------------------------------- | --------------------------------------------------------- |
| 7        | D_FB       | Drive Fwd/Back   | `drive_fb`              | 9                                | Average of ML + MR encoders. No EEPROM offset restore.    |
| 8        | D_LR       | Drive Left/Right | `drive_lr`              | 10                               | Difference of ML − MR encoders. No EEPROM offset restore. |

The differential drive kinematics in `Base.ino` compute:

```
drive_fb_pos = (ml_enc + mr_enc) / 2.0   // Forward/backward (average)
drive_lr_pos = (ml_enc - mr_enc)          // Left/right (difference)
```

A deadzone of `DRIVE_DEADZONE_TICKS = 300.0` is applied in `POSITION_CONTROL` mode — if the position error is within 300 ticks, the target is snapped to current position and both PIDs are reset to prevent joystick creep.

## Usage

- **Firmware:** The centralized `motor_map[]` array (defined in `Base.ino`, declared in `src/MotorMap/MotorMap.h`) maps each 1-indexed actuator ID to its `Motor*`, encoder index, RoboClaw controller, channel, and feature flags. Command dispatch uses `getMotor(actuator_id)` and `getEncoderIndex(actuator_id)` to resolve targets.
- **Python App:** The `pid_tuner/data/joint_config.py` defines `JOINTS` as a list of 8 `JointInfo` objects using this exact ID mapping to render the UI components.
