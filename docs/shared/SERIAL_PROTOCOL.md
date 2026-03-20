# Serial Protocol Specification

This document details the communication protocol used between the Host PC (running the PID Tuner or other control software) and the RAMMP / MEBot Teensy 4.1 firmware over a 460800 baud serial connection.

The protocol uses human-readable ASCII, delimited by newline (`\n`) characters.

---

## ⬇️ Host -> Teensy: Commands

The PC sends commands to instruct the Teensy to alter its state, change PID tuning parameters, or command motor targets.

### Command Format
`[CommandChar][ActuatorID]:[Value]\n`

- `CommandChar`: A single case-sensitive ASCII character indicating the command type.
- `ActuatorID`: An integer `1-6` specifying the joint. Use `0` for commands that apply globally (e.g., Save All configs).
- `Value`: A float or integer value. (Omitted for valueless commands).

### Command Reference

| Command Char | Name | Example | Description |
|---|---|---|---|
| `T` | Target | `T1:1000` | Set motor target (PWM, Vel, or Pos depending on mode). |
| `M` | Mode | `M3:2` | Set motor control mode (0=Open Loop, 1=Velocity, 2=Position). |
| `P` | Pos P Gain | `P1:5.5` | Set Position PID Proportional Gain. |
| `I` | Pos I Gain | `I1:0.1` | Set Position PID Integral Gain. |
| `D` | Pos D Gain | `D1:0.01`| Set Position PID Derivative Gain. |
| `F` | Pos FF Gain| `F1:1.0` | Set Position PID Feed-Forward Gain. |
| `p` | Vel P Gain | `p1:1.0` | Set Velocity PID Proportional Gain. |
| `i` | Vel I Gain | `i1:0.0` | Set Velocity PID Integral Gain. |
| `d` | Vel D Gain | `d1:0.0` | Set Velocity PID Derivative Gain. |
| `f` | Vel FF Gain| `f1:0.5` | Set Velocity PID Feed-Forward Gain. |
| `Q` | Pos LPF | `Q1:0.8` | Set Position PID output Low-Pass Filter Alpha (0-1). |
| `q` | Vel LPF | `q1:0.8` | Set Velocity PID output Low-Pass Filter Alpha (0-1). |
| `l` | Input LPF | `l1:0.5` | Set Motor velocity input Low-Pass Filter Alpha (0-1). |
| `n` | Pos Min Lim| `n4:-5000`| Set Joint minimum position limit in ticks. |
| `x` | Pos Max Lim| `x4:5000` | Set Joint maximum position limit in ticks. |
| `R` | Reset PID | `R3` | Reset PID integrators and internal state. |
| `H` | Home Enc | `H5` | Set current encoder position to 0 (Home). |
| `V` | Invert Motor| `V2` | Toggle Motor PWM direction. |
| `E` | Invert Enc | `E2` | Toggle Encoder read direction. |
| `K` | Save Config| `K1` / `K0` | Save joint config to EEPROM. `K0` saves all joints. |
| `G` | Get Config | `G1` | Request the Teensy to send the config for Joint 1 back. |
| `z` | ESTOP | `z` | Immediately disable all motors and enter ESTOP state. |
| `c` | Clear ESTOP| `c` | Clear ESTOP state and return to IDLE. |
| `L1`| Self Level | `L1:1` | Enable (1) or disable (0) Auto Self-Leveling mode. |
| `A1`| Level Pitch| `A1:5.0` | Set Self-Leveling target pitch in degrees. |
| `A2`| Level Roll | `A2:-2.0`| Set Self-Leveling target roll in degrees. |

---

## ⬆️ Teensy -> Host: Telemetry & Config Data

The Teensy emits data back to the host.

### 1. Telemetry Stream
Emitted continuously at 10Hz.

**Format:**
`TELEMETRY,<millis>,<state>,<pos1..6>,<vel1..6>,<pwm1..6>,<mdir1..6>,<edir1..6>,<lim1..4>,<pitch>,<roll>,<yaw>,<ax>,<ay>,<az>,<qw>,<qx>,<qy>,<qz>,<leveling_pitch_err>,<leveling_roll_err>,<z_ml>,<z_rc>,<z_mr>\n`

**Fields (49 total values):**
1. String `"TELEMETRY"`
2. `millis()` timestamp
3. `SystemState` (int 0-6)
4-9. `current_pos` for joints 1-6 (float)
10-15. `current_vel` for joints 1-6 (float)
16-21. `target_pwm` for joints 1-6 (float)
22-27. Motor direction `1` or `-1` for joints 1-6
28-33. Encoder direction `1` or `-1` for joints 1-6
34-37. Limit switches (1=pressed, 0=open): `ml_fwd`, `ml_bwd`, `mr_fwd`, `mr_bwd`
38-40. IMU Euler Angles (deg): `pitch`, `roll`, `yaw`
41-43. IMU Accelerometer: `ax`, `ay`, `az`
44-47. IMU Quaternion: `w`, `x`, `y`, `z`
48-49. Leveling Debug: `pitch_err`, `roll_err` (float)
50-52. Leveling Target Z Heights (cm): `z_target_ml`, `z_target_rc`, `z_target_mr` (float)

### 2. Config Dump
Emitted when requested via the `G<id>` command. Used to sync the GUI sliders with the EEPROM values saved on the Teensy.

**Format:**
`CONFIG,<id>,<pP>,<pI>,<pD>,<pFF>,<vP>,<vI>,<vD>,<vFF>,<pLPF>,<vLPF>,<inLPF>,<posMin>,<posMax>\n`

**Fields:**
1. String `"CONFIG"`
2. Joint ID (1-6)
3-6. Position PID: P, I, D, FF
7-10. Velocity PID: P, I, D, FF
11-13. Low Pass Filters: Pos Alpha, Vel Alpha, Input Alpha
14-15. Limits: Min, Max
