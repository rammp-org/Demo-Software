# Plan: IMU Self-Leveling Quaternion Rewrite

## Goal
Replace the Gimbal-Lock-prone Euler angle error calculation in `runSelfLeveling` with a mathematically robust Quaternion-based error calculation. This completely eliminates the +/- 180 discontinuity jumps while ensuring the physical chassis kinematics (the 4x4 rotation matrix applied to the wheels) remain exactly the same.

## 1. Modify `IMU_Class.h`
- Add `imu::Quaternion current_quat;` as a public member variable to expose the raw, unfiltered 3D orientation from the BNO055 to `Base.ino`.

## 2. Modify `IMU_Class.cpp`
- In `retrieve_readings()`, right after `imu::Quaternion quat = bno_sensor.getQuat();`, save `quat` to the new `current_quat` member.
- *Crucially, leave all the existing Euler angle extraction and LPF code exactly as it is.* This ensures that the Pitch/Roll/Yaw telemetry strings sent to the Python GUI remain identical, preventing the GUI from breaking.

## 3. Modify `Base.ino` (`runSelfLeveling` function)
- Remove the old logic that reads `IMU.pitchf` / `IMU.rollf` and calculates `pitch_error` and `roll_error` via subtraction and `while (> 180)` wrapping loops.
- Retrieve the measured orientation: `imu::Quaternion q_meas = IMU.current_quat;`
- Construct a Target Quaternion (`q_target`) from the user's `target_pitch` and `target_roll` setpoints.
  - Since the IMU defines Pitch as rotation around the X-axis, and Roll as rotation around the Y-axis (upside down, so +180), we construct two temporary quaternions for the target axes and multiply them.
- Compute the Error Quaternion: `imu::Quaternion q_err = q_target * q_meas.conjugate();` (or `q_meas.conjugate() * q_target` depending on frame order) to find the exact 3D rotation needed to get from the current state to the target state.
- Extract `pitch_error` and `roll_error` directly from `q_err` using the standard aerospace Euler extraction formulas.
  - Because `q_err` represents only the *difference* between target and measured, the extracted angles will naturally center around `0` (e.g., `2.0` degrees), completely bypassing the +/- 180 gimbal lock boundary.
- Convert these raw error angles to radians (`dpitchrd` and `drollrd`) and feed them straight into the existing 4x4 rotation matrix.
- Leave the `rotm * mebot` matrix multiplication and the Z-height extraction exactly as they are. This ensures the physical geometry math is untouched.