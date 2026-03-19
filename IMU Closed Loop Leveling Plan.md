# IMU Closed Loop Leveling Plan

## Context

The robot is a powered wheelchair with three independent vertical actuators that control platform tilt and height:

- **ML** (Motor Left) at body-frame position `(-34, -31)` cm
- **MR** (Motor Right) at body-frame position `(-34, +31)` cm
- **RC** (Rear Caster) at body-frame position `(+34, 0)` cm

The IMU is an Adafruit BNO055, mounted upside-down on the platform. Yaw is stripped from the IMU quaternion using a swing-twist decomposition (`extractSwing()` in `IMU_Class.cpp`) before being used for self-leveling, so heading changes do not affect the leveling logic.

The goal is: when self-leveling is enabled, capture the current IMU orientation as a setpoint and hold that orientation indefinitely — correcting for any ground slope or disturbance that would tilt the platform away from that pose.

---

## The Geometric Problem

Three independent vertical actuators control a platform with exactly **3 degrees of freedom** in the vertical sense:

- Pitch tilt (fore/aft)
- Roll tilt (left/right)
- Average height (uniform raise/lower)

3 actuators = 3 DOF → the system is **fully determined**. There is exactly one set of actuator positions that produces any given (pitch, roll, height) combination. No iterative solving is needed — the relationship is analytically invertible via straightforward trigonometry.

---

## Why the Current Code Has a Conceptual Error

The current `runSelfLeveling()` builds a 4×4 homogeneous rotation matrix using `dpitchrd` and `drollrd`, which are **error angles** (the difference between current IMU reading and setpoint). It then applies that matrix to the chassis geometry to get Z-height targets for each actuator.

The problem: when the error is zero (robot is at setpoint), all three actuators are commanded to the hardcoded 9.5 cm baseline. When there is error, the actuators are commanded *away* from 9.5 cm in proportion to the error angle. This is an open-loop correction stacked on a hardcoded baseline — it is not a proper closed-loop that drives the platform to a defined pose.

**What you actually want:** at each timestep, compute "what actuator positions would produce my target IMU orientation at the current baseline height?" — then set those as the motor position targets. The motor PIDs close the joint-space loop, and the kinematic math provides the correct feedforward.

---

## The Correct Architecture

```
IMU setpoint (captured at enable)
        |
        v
Kinematic solver (geometry math)  -->  Target joint positions (ticks)  -->  Motor PIDs
                                                                               |
                                                                        Encoder feedback
```

The geometry provides the **analytically correct actuator positions** for the desired pose. The motor PIDs handle the joint-space feedback loop (encoder → position). This is kinematic feedforward with joint-space PID — the standard approach for a Stewart-platform-style mechanism.

An optional slow outer IMU trim loop can be added later to correct for calibration drift in `CM_TO_TICKS` or contact point deflection. This is a refinement, not the primary control signal.

---

## The Correct Kinematic Formula

For a platform where each leg is a vertical column, the height of each contact point relative to the platform center is:

```
z_i = h_base - x_i * sin(θ) + y_i * sin(φ) * cos(θ)
```

Where:
- `θ` = target pitch angle in radians (from captured setpoint)
- `φ` = target roll angle in radians (from captured setpoint)
- `x_i`, `y_i` = body-frame X/Y position of contact point `i` (in cm)
- `h_base` = baseline platform height (cm), captured at enable-time

Applied to each actuator:

```
z_ML = h_base + (-X_ML * sin(θ)  +  Y_ML * sin(φ) * cos(θ))
z_MR = h_base + (-X_MR * sin(θ)  +  Y_MR * sin(φ) * cos(θ))
z_RC = h_base + (-X_RC * sin(θ)  +  Y_RC * sin(φ) * cos(θ))
```

Using the body-frame coordinates from `mebot`:

| Actuator | X (cm) | Y (cm) |
|----------|--------|--------|
| ML       | -34    | -31    |
| MR       | -34    | +31    |
| RC       | +34    |   0    |

This is simpler and more direct than the 4×4 homogeneous matrix approach. The matrix approach is equivalent but is more error-prone when the input angles are error deltas vs. absolute targets — which is exactly the bug in the current code.

---

## Baseline Height Capture

When self-leveling is enabled, snapshot the current average actuator height:

```cpp
h_base_cm = (ml.current_pos  / ML_CM_TO_TICKS
           + mr.current_pos  / MR_CM_TO_TICKS
           + rc.current_pos  / RC_CM_TO_TICKS) / 3.0f;
```

This freezes the average height at enable-time. Tilt correction happens symmetrically around this average — one actuator goes up while another goes down, preserving the mean. If any actuator would exceed its stroke limit, the remaining error is accepted (the platform can't achieve the full target tilt at that height) — clamping/handling this is a refinement for later.

---

## Setpoint Capture

When `CMD_LEVEL_MODE` is received with `value > 0.5` (i.e. self-leveling is being enabled):

1. Read `IMU.current_quat` (already yaw-stripped by `extractSwing()`).
2. Store it as `q_setpoint`.
3. Extract `target_pitch` and `target_roll` in degrees from `q_setpoint` using the same Euler decomposition already used in `retrieve_readings()`.
4. Capture `h_base_cm` from the current motor positions as described above.

These replace the current manual `target_pitch` / `target_roll` global floats, which currently have no automatic capture mechanism and require separate GUI commands to set.

---

## Where the IMU Error Quaternion Fits

The IMU error quaternion `q_err = q_target * q_meas.conjugate()` tells you how well the geometric solution is working. Because the system is kinematically determined, you do **not** need an outer PID on the IMU as the primary control path. The motor PIDs handle the joint-space loop and the kinematics give the correct joint targets analytically.

However, if `CM_TO_TICKS` calibration is off, or contact points have mechanical deflection, the IMU will show residual error after the kinematic solution is applied. In that case, an optional **slow trim integrator** can be added:

```
imu_pitch_error  →  small additive correction to target_pitch  (slow integral)
imu_roll_error   →  small additive correction to target_roll   (slow integral)
```

This is not a full PID — it is a bias correction on top of the kinematic feedforward. It would wind slowly toward zero IMU error even in the presence of systematic calibration error. This should only be added after the base kinematic solution is working correctly.

---

## Implementation Plan

| Step | What | File | Details |
|------|------|------|---------|
| 1 | Add `q_setpoint`, `h_base_cm`, `target_pitch_rad`, `target_roll_rad` globals | `Base.ino` | Replace or supplement existing `target_pitch`, `target_roll` float globals |
| 2 | Capture setpoint and baseline height on `CMD_LEVEL_MODE` enable | `Base.ino` state machine | Extract Euler angles from `IMU.current_quat` at enable-time |
| 3 | Replace `runSelfLeveling()` kinematics | `Base.ino` | Use direct formula `z_i = h_base - x_i*sin(θ) + y_i*sin(φ)*cos(θ)` with absolute target angles; remove the error-angle rotation matrix |
| 4 | Remove / fix the hardcoded 9.5 cm baseline in `rotm[2][3]` | `Base.ino` | Replace with `h_base_cm` captured dynamically |
| 5 | Verify contact point coordinates in `mebot` | `Base.ino` | Confirm X/Y positions match physical robot measurements |
| 6 | Tune `CM_TO_TICKS` per actuator | `Constants.h` | Currently all three share `17.5 ticks/cm` (marked TODO) — may need independent values |
| 7 | (Optional) Add slow IMU trim integrator | `Base.ino` | Only after base kinematic solution is verified working |

---

## Open Questions Before Implementing

1. **Contact point coordinates**: Are the `mebot` matrix values (`ML: (-34,-31)`, `MR: (-34,+31)`, `RC: (+34,0)`) accurate physical measurements, or estimates? These directly determine the tilt-to-height mapping and need to be correct.

2. **`CM_TO_TICKS` per actuator**: All three actuators currently share `17.5 ticks/cm`. If ML, MR, and RC have different gear ratios or stroke lengths, they need independent conversion factors.

3. **`FC_MAX_TICKS = 0.0f`**: The front caster is commanded to position 0, described as "top of range." This seems like it may need updating — is 0 actually the correct top-of-range encoder value for FC?

4. **Carriage role at enable-time**: When self-leveling is enabled, carriages are currently held at `0.1f * CARRIAGE_CM_TO_TICKS`. Is this the correct neutral/home position, or should they also be captured at their current position?

5. **Stroke limits**: If the tilt correction at a given `h_base_cm` would require an actuator to go beyond its min/max limit, how should the system respond? Options: clamp and accept residual error, reduce `h_base_cm` to give more headroom, or emit a warning.
