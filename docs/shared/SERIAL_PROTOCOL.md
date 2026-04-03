# Serial Protocol Specification

This document details the communication protocol used between the Host PC (running the PID Tuner or other control software) and the RAMMP / MEBot Teensy 4.1 firmware over a 460800 baud serial connection.

The protocol uses human-readable ASCII, delimited by newline (`\n`) characters.

______________________________________________________________________

## ⬇️ Host -> Teensy: Commands

The PC sends commands to instruct the Teensy to alter its state, change PID tuning parameters, or command motor targets.

### Command Format

`[CommandChar][ActuatorID]:[Value]\n`

- `CommandChar`: A single case-sensitive ASCII character indicating the command type.
- `ActuatorID`: An integer `1-8` specifying the joint (see [Joint Mapping](JOINT_MAPPING.md)). Use `0` for commands that apply globally (e.g., Save All configs).
- `Value`: A float or integer value. (Omitted for valueless commands).

### Command Reference

| Command Char | Name           | Example      | Description                                                   |
| ------------ | -------------- | ------------ | ------------------------------------------------------------- |
| `T`          | Target         | `T1:1000`    | Set motor target (PWM, Vel, or Pos depending on mode).        |
| `M`          | Mode           | `M3:2`       | Set motor control mode (0=Open Loop, 1=Velocity, 2=Position). |
| `P`          | Pos P Gain     | `P1:5.5`     | Set Position PID Proportional Gain.                           |
| `I`          | Pos I Gain     | `I1:0.1`     | Set Position PID Integral Gain.                               |
| `D`          | Pos D Gain     | `D1:0.01`    | Set Position PID Derivative Gain.                             |
| `F`          | Pos FF Gain    | `F1:1.0`     | Set Position PID Feed-Forward Gain.                           |
| `p`          | Vel P Gain     | `p1:1.0`     | Set Velocity PID Proportional Gain.                           |
| `i`          | Vel I Gain     | `i1:0.0`     | Set Velocity PID Integral Gain.                               |
| `d`          | Vel D Gain     | `d1:0.0`     | Set Velocity PID Derivative Gain.                             |
| `f`          | Vel FF Gain    | `f1:0.5`     | Set Velocity PID Feed-Forward Gain.                           |
| `Q`          | Pos LPF        | `Q1:0.8`     | Set Position PID output Low-Pass Filter Alpha (0-1).          |
| `q`          | Vel LPF        | `q1:0.8`     | Set Velocity PID output Low-Pass Filter Alpha (0-1).          |
| `U`          | Pos Ramp       | `U1:500.0`   | Set Position PID output max ramp rate (units/s).              |
| `u`          | Vel Ramp       | `u1:0.1`     | Set Velocity PID output max ramp rate (units/s).              |
| `l`          | Input LPF      | `l1:0.5`     | Set Motor velocity input Low-Pass Filter Alpha (0-1).         |
| `n`          | Pos Min Lim    | `n4:-5000`   | Set Joint minimum position limit in ticks.                    |
| `x`          | Pos Max Lim    | `x4:5000`    | Set Joint maximum position limit in ticks.                    |
| `R`          | Reset PID      | `R3`         | Reset PID integrators and internal state.                     |
| `H`          | Home Enc       | `H5`         | Set current encoder position to 0 (Home).                     |
| `O`          | Set Offset     | `O1:500`     | Set encoder position offset to an arbitrary value.            |
| `V`          | Invert Motor   | `V2`         | Toggle Motor PWM direction.                                   |
| `E`          | Invert Enc     | `E2`         | Toggle Encoder read direction.                                |
| `K`          | Save Config    | `K1` / `K0`  | Save joint config to EEPROM. `K0` saves all joints.           |
| `G`          | Get Config     | `G1`         | Request the Teensy to send the config for Joint 1 back.       |
| `z`          | ESTOP          | `z`          | Immediately disable all motors and enter ESTOP state.         |
| `c`          | Clear ESTOP    | `c`          | Clear ESTOP state and return to IDLE.                         |
| `L1`         | Self Level     | `L1:1`       | Enable (1) or disable (0) Auto Self-Leveling mode.            |
| `A1`         | Level Pitch    | `A1:5.0`     | Set Self-Leveling target pitch in degrees.                    |
| `A2`         | Level Roll     | `A2:-2.0`    | Set Self-Leveling target roll in degrees.                     |
| `B1`         | Sequence Mode  | `B1:1`       | Enter (1) or exit (0) Auto Curb Climbing sequence mode.       |
| `B2`         | Seq Auto-Run   | `B2:1`       | Enable (1) or disable (0) auto-advance on keyframe completion.|
| `J`          | Seq Keyframe   | `J0:<payload>`| Upload keyframe data at index. See Keyframe Format below.     |
| `>`          | Seq Step Fwd   | `>`          | Step forward to next keyframe.                                |
| `<`          | Seq Step Bwd   | `<`          | Step backward to previous keyframe.                           |
| `@`          | Seq Goto       | `@3`         | Jump directly to keyframe index 3.                            |

### Keyframe Payload Formats (`J` command)

The `J<idx>:<payload>` command uploads a keyframe. The payload after the colon is a CSV string. Two formats are supported:

**New Format (32 values):**
`t1,t2,t3,t4,t5,t6,t7,t8,a1,a2,a3,a4,a5,a6,a7,a8,r1,r2,r3,r4,r5,r6,r7,r8,d1,d2,d3,d4,d5,d6,d7,d8`

- `t1–t8`: Target positions for 8 motors
- `a1–a8`: Active flags (1=participating, 0=inactive)
- `r1–r8`: Relative flags (1=offset from start position, 0=absolute)
- `d1–d8`: Per-motor interpolation durations in milliseconds

**Legacy Format (17 values):**
`t1,t2,t3,t4,t5,t6,t7,t8,a1,a2,a3,a4,a5,a6,a7,a8,dur_ms`

- `t1–t8`: Target positions for 8 motors
- `a1–a8`: Active flags
- `dur_ms`: Shared interpolation duration for all motors (all targets absolute)

______________________________________________________________________

## ⬆️ Teensy -> Host: Telemetry & Response Data

The Teensy emits several types of data back to the host.

### 1. Telemetry Stream

Emitted continuously at 10Hz.

**Format:**
`TELEMETRY,<millis>,<state>,<pos1..6>,<vel1..6>,<pwm1..6>,<mdir1..6>,<edir1..6>,<lim1..4>,<pitch>,<roll>,<yaw>,<ax>,<ay>,<az>,<qw>,<qx>,<qy>,<qz>,<leveling×5>,<sg×4>,<modes1..6>,<drive_pos×2>,<drive_vel×2>,<drive_pwm×2>,<drive_modes×2>,<raw_enc_pos×2>,<raw_enc_vel×2>,<drive_dir×2>,<drive_enc_dir×2>\n`

**Fields (75 data values after header):**

| Index | Count | Description                                                           |
| ----- | ----- | --------------------------------------------------------------------- |
| 1     | 1     | `millis()` timestamp                                                  |
| 2     | 1     | `SystemState` (int 0–6)                                              |
| 3–8   | 6     | `current_pos` for joints 1–6 (float, 2dp)                           |
| 9–14  | 6     | `current_vel` for joints 1–6 (float, 2dp)                           |
| 15–20 | 6     | `target_pwm` for joints 1–6 (float, 2dp)                            |
| 21–26 | 6     | Motor direction `1` or `-1` for joints 1–6                          |
| 27–32 | 6     | Encoder direction `1` or `-1` for joints 1–6                        |
| 33–36 | 4     | Limit switches (1=pressed, 0=open): `ml_fwd`, `ml_bwd`, `mr_fwd`, `mr_bwd` |
| 37–39 | 3     | IMU Euler Angles (deg, 2dp): `pitch`, `roll`, `yaw`                 |
| 40–42 | 3     | IMU Accelerometer (m/s², 3dp): `ax`, `ay`, `az`                     |
| 43–46 | 4     | IMU Quaternion (4dp): `w`, `x`, `y`, `z`                            |
| 47–51 | 5     | Leveling Debug (4dp): `pitch_err`, `roll_err`, `z_ml`, `z_rc`, `z_mr` |
| 52–55 | 4     | Strain Gauge ADC readings (2dp): `sg_rc`, `sg_fc`, `sg_ml`, `sg_mr` |
| 56–61 | 6     | Control modes for joints 1–6 (0=Open Loop, 1=Velocity, 2=Position)  |
| 62–63 | 2     | Drive wheel positions (2dp): `drive_fb`, `drive_lr`                  |
| 64–65 | 2     | Drive wheel velocities (2dp): `drive_fb`, `drive_lr`                 |
| 66–67 | 2     | Drive wheel PWMs (2dp): `drive_fb`, `drive_lr`                       |
| 68–69 | 2     | Drive wheel control modes: `drive_fb`, `drive_lr`                    |
| 70–71 | 2     | Raw encoder positions (2dp): `raw_ml`, `raw_mr`                      |
| 72–73 | 2     | Raw encoder velocities (2dp): `raw_ml`, `raw_mr`                     |
| 74–75 | 2     | Drive motor directions: `drive_fb`, `drive_lr`                       |
| 76–77 | 2     | Drive encoder directions: `drive_fb`, `drive_lr`                     |

### 2. Config Dump

Emitted when requested via the `G<id>` command. Used to sync the GUI sliders with the EEPROM values saved on the Teensy.

**Format:**
`CONFIG,<id>,<pP>,<pI>,<pD>,<pFF>,<vP>,<vI>,<vD>,<vFF>,<pLPF>,<vLPF>,<inLPF>,<posMin>,<posMax>,<pRamp>,<vRamp>,<motorDir>,<encDir>\n`

**Fields (18 total values):**

1. String `"CONFIG"`
2. Joint ID (1–8)
3. Position PID: P
4. Position PID: I
5. Position PID: D
6. Position PID: FF
7. Velocity PID: P
8. Velocity PID: I
9. Velocity PID: D
10. Velocity PID: FF
11. Position LPF Alpha
12. Velocity LPF Alpha
13. Input LPF Alpha
14. Position Limit Min
15. Position Limit Max
16. Position Max Ramp Rate
17. Velocity Max Ramp Rate
18. Motor Direction (+1 or -1)
19. Encoder Direction (+1 or -1)

### 3. Sequence Status

Emitted during `AUTO_CURB_CLIMBING` mode to report sequence playback state.

**Format:**
`SEQ_STATUS,<current_step>,<total_steps>,<phase>\n`

- `current_step`: 0-indexed keyframe being executed
- `total_steps`: Total number of uploaded keyframes
- `phase`: `1` = interpolating toward targets, `2` = settling (waiting for motors to reach targets)

### 4. Sequence Acknowledgment

Emitted when a keyframe upload (`J` command) is successfully received.

**Format:**
`SEQ_ACK,<step_index>\n`

- `step_index`: The 0-indexed keyframe slot that was written
