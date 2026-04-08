# Cup Stabilizer

> **Summary**
> Cup stabilization is now embedded directly inside `ArmDriverNode` (in `hardware/arm_driver`) rather than running as a standalone ROS node. This change was made to eliminate the round-trip latency of publishing `/arm/atdev/twist` over ROS — closed-loop control at 40 Hz requires IMU data and arm commands to be on the same call stack.
>
> When the arm enters `CUP_STABILIZE` mode, a 40 Hz internal timer reads IMU data directly from the Kinova API (`BaseCyclicClient.RefreshFeedback()`) and sends base-frame twist commands that align the tool Y-axis with gravity using a PD controller. No external topics are involved in the control loop.

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
  └─ CUP_STABILIZE mode
       ├─ Kinova API: BaseCyclicClient.RefreshFeedback()  (accel, gyro, EE pose)
       └─ Kinova API: BaseClient.SendTwistCommand()       (base-frame twist)
```

No ROS messages are exchanged during the control loop. All I/O goes through the Kinova kortex API via `KinovaArm.get_imu_data()` and `KinovaArm.send_twist_base_frame()`.

______________________________________________________________________

## Activating Cup Stabilization

Cup stabilization is triggered through the standard `SetMode` service on the arm driver:

| Service         | Type                         | Field                       |
| --------------- | ---------------------------- | --------------------------- |
| `/arm/set_mode` | `arm_interfaces/srv/SetMode` | `mode = MODE_CUP_STABILIZE` |

To deactivate, call `SetMode` with any other valid mode (e.g. `MODE_IDLE`).

The `kortex_cup_stabilizer.py` script in this package (`atdev_coffee_stabilizer/`) remains as a **standalone diagnostic tool** for tuning and benchmarking the controller outside of the full ROS stack. It is not used at runtime.

______________________________________________________________________

## Startup Sequence

When transitioning into `CUP_STABILIZE`:

1. The active twist stream (`_twist_timer`) is stopped and its cache is cleared.
1. A background thread runs a **3-second gyro calibration** (`CUP_STABILIZE_CALIBRATION_S = 3.0 s`) — keep the arm still during this window.
1. After calibration, the **40 Hz PD control timer** (`_cup_stabilize_timer`) is activated.

When transitioning out of `CUP_STABILIZE`, the control timer is cancelled and the gyro offset is cleared immediately.

______________________________________________________________________

## PD Controller

The controller aligns the **tool Y-axis** with the gravity vector (anti-gravity direction derived from the raw accelerometer).

| Parameter              | Value | Constant                      |
| ---------------------- | ----- | ----------------------------- |
| Control rate           | 40 Hz | `CUP_STABILIZE_HZ`            |
| Proportional gain (Kp) | 8.0   | `CUP_STABILIZE_KP`            |
| Derivative gain (Kd)   | 1.0   | `CUP_STABILIZE_KD`            |
| Calibration duration   | 3.0 s | `CUP_STABILIZE_CALIBRATION_S` |

Control law (base frame):

```
up       = -accel / |accel|          # gravity direction from accelerometer
tool_y   = R(ee_euler_deg)[:, 1]     # tool Y-axis in base frame
error    = tool_y - up

omega_x  =  Kp * error[1] + Kd * gyro_x_rads
omega_y  = -(Kp * error[0] + Kd * gyro_y_rads)
```

`omega_z` is always zero — roll about the tool axis is not controlled.

______________________________________________________________________

## Relevant Source Files

| File                                                                                    | Purpose                                                                      |
| --------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------- |
| `hardware/arm_driver/arm_driver/arm_driver.py`                                          | `ArmDriverNode` — state machine, calibration thread, `_cup_stabilize_tick()` |
| `hardware/arm_driver/arm_driver/arm_interface.py`                                       | `KinovaArm.get_imu_data()`, `KinovaArm.send_twist_base_frame()`              |
| `demo_modules/atdev_coffee_stabilizer/atdev_coffee_stabilizer/kortex_cup_stabilizer.py` | Standalone diagnostic / tuning script (not used at runtime)                  |
| `demo_modules/atdev_coffee_stabilizer/atdev_coffee_stabilizer/evaluate_latency.py`      | Latency benchmarking tool                                                    |
| `demo_modules/atdev_coffee_stabilizer/atdev_coffee_stabilizer/kortex_sine_sweep.py`     | Frequency-response characterization tool                                     |
