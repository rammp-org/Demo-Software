# Cup Stabilizer

> **Summary**
> Cup stabilization is embedded directly inside `ArmDriverNode` (in `hardware/arm_driver`) rather than running as a standalone ROS node. This eliminates the round-trip latency of publishing `/arm/atdev/twist` over ROS — closed-loop control at 40 Hz requires IMU data and arm commands to be on the same call stack.
>
> When the arm enters `CUP_STABILIZE` mode, a 40 Hz internal timer reads cached IMU data (populated at 100 Hz by the existing `_publish_joint_states` timer from `BaseCyclicClient.RefreshFeedback()`) and sends base-frame twist commands that align the tool Y-axis with gravity using a PD controller. No external topics are involved in the control loop.

______________________________________________________________________

## Architecture

### Old (standalone ROS node — removed)

```
/camera/wrist/accel/sample  ─┐
/camera/wrist/gyro/sample   ─┤→ kortex_cup_stabilizer node → /arm/atdev/twist → arm_driver
/arm/joint_states           ─┘
```

### New (embedded in arm_driver)

```
ArmDriverNode
  ├─ _publish_joint_states (100 Hz)
  │    └─ KinovaArm.get_state()["imu"]  →  self._latest_imu_data  (cache)
  │
  ├─ /arm/calibrate  (action, on demand)
  │    └─ samples _latest_imu_data["gyro"] × 80 min → CupStabilizer.calibrate()
  │
  └─ CUP_STABILIZE mode
       ├─ _cup_stabilize_tick (40 Hz)
       │    ├─ reads self._latest_imu_data
       │    ├─ CupStabilizer.feed()          (pure PD algorithm)
       │    └─ KinovaArm.send_twist_base_frame()
```

No ROS messages are exchanged during the control loop. All hardware I/O goes through the Kinova kortex API via `KinovaArm.get_state()` and `KinovaArm.send_twist_base_frame()`.

______________________________________________________________________

## Activating Cup Stabilization

Cup stabilization is triggered through the standard `SetMode` service on the arm driver:

| Service         | Type                         | Field                       |
| --------------- | ---------------------------- | --------------------------- |
| `/arm/set_mode` | `arm_interfaces/srv/SetMode` | `mode = MODE_CUP_STABILIZE` |

To deactivate, call `SetMode` with any other valid mode (e.g. `MODE_IDLE`).

______________________________________________________________________

## Startup Sequence

**At arm connect:** no calibration is run automatically.

**To calibrate** (call before entering `CUP_STABILIZE`):

```bash
ros2 action send_goal /arm/calibrate arm_interfaces/action/Calibrate "{}"
```

The action blocks for up to `cup_stabilizer.calibration_s` seconds (default 5 s).
Keep the arm still during this window. The action requires at least 80 gyro samples
and aborts if the timeout expires first.

**When entering `CUP_STABILIZE`:**

1. The **40 Hz PD control timer** (`_cup_stabilize_timer`) is activated.
1. If `/arm/calibrate` has not been called, ticks are no-ops until calibration is run.

**When leaving `CUP_STABILIZE`:**

1. The control timer is cancelled. The gyro offset is **not** cleared — call `/arm/calibrate` again before the next session if needed.

______________________________________________________________________

## PD Controller

The controller aligns the **tool Y-axis** with the gravity vector (anti-gravity direction derived from the raw accelerometer). Raw (unfiltered) accelerometer data is intentional — it acts as a proxy for cup vibration, which the derivative term can damp.

| Parameter              | Value | ROS param                      |
| ---------------------- | ----- | ------------------------------ |
| Control rate           | 40 Hz | `cup_stabilizer.hz`            |
| Proportional gain (Kp) | 8.0   | `cup_stabilizer.kp`            |
| Derivative gain (Kd)   | 1.0   | `cup_stabilizer.kd`            |
| Calibration duration   | 5.0 s | `cup_stabilizer.calibration_s` |

Control law (base frame):

```
up       = -accel / |accel|          # gravity direction from accelerometer
tool_y   = R(ee_euler_deg)[:, 1]     # tool Y-axis in base frame
error    = tool_y - up

omega_x  =  Kp * error[1] + Kd * gyro_x_rads
omega_y  = -Kp * error[0] - Kd * gyro_y_rads
```

`omega_z` is always zero — roll about the tool axis is not controlled.

______________________________________________________________________

## Relevant Source Files

| File                                                 | Purpose                                                                      |
| ---------------------------------------------------- | ---------------------------------------------------------------------------- |
| `hardware/arm_driver/arm_driver/cup_stabilizer.py`   | `CupStabilizer` — pure PD algorithm class, no ROS/Kortex dependencies        |
| `hardware/arm_driver/arm_driver/arm_driver.py`       | `ArmDriverNode` — state machine, calibration thread, `_cup_stabilize_tick()` |
| `hardware/arm_driver/arm_driver/arm_interface.py`    | `KinovaArm.get_state()["imu"]`, `KinovaArm.send_twist_base_frame()`          |
| `hardware/arm_driver/test/test_cup_stabilizer.py`    | Unit tests for `CupStabilizer` (no hardware required)                        |
| `hardware/arm_driver/test/test_arm_driver_safety.py` | Integration tests for cup stabilizer state machine (`TestCupStabilizeState`) |
