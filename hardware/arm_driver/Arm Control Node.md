# Arm Control Node

> **Summary**
>
> Main node that controls the arm. This is the only node that communicates with the arm directly. It publishes `/arm/joint_states`, `/arm/ee/pose`, `/arm/ee/velocity`, `/arm/ee/force`, and `/arm/status` to other nodes. Based on the internal state machine, it gates incoming commands to a single authorized source per state.
>
> There are two types of command sources that are accepted, depending on the state:
>
> - **TWIST inputs** (`MANUAL`, `CUP_STABILIZE`): authorized source streams velocity commands on `/arm/{source}/twist` at up to 25 Hz.
> - **POSITION inputs** (`OPEN_DOOR`, `ORDER_DRINK`, `DRINKING`): authorized source sends target joint position or Cartesian pose commands, or submits a trajectory via the `/arm/{source}/execute_trajectory` action.

> **TODO:**
>
> - [ ] Configure all needed position presets
> - [ ] Define how conflicting services and messages interact

______________________________________________________________________

## State Machine

| State              | Authorized Source | Command Mode | Description                                                                                 |
| ------------------ | ----------------- | ------------ | ------------------------------------------------------------------------------------------- |
| `IDLE`             | —                 | None         | Default standby. Arm connected, no commands accepted.                                       |
| `PRESET_IN_MOTION` | —                 | None         | Arm moving to a preset via `/arm/reach_preset`. No commands accepted.                       |
| `OPEN_DOOR`        | `cmu`             | Position     | CMU controls arm to open a door.                                                            |
| `ORDER_DRINK`      | `cornell`         | Position     | Cornell controls arm to order a drink.                                                      |
| `DRINKING`         | `cornell`         | Position     | Cornell controls arm to assist with drinking.                                               |
| `CUP_STABILIZE`    | `atdev`           | Twist        | ATDev streams velocity corrections to stabilize a cup.                                      |
| `MANUAL`           | `xbox`            | Twist        | Xbox controller streams velocity commands for manual teleoperation.                         |
| `ERROR`            | —                 | None         | Entered on any safety event. No commands accepted. See [Clearing Errors](#clearing-errors). |

State transitions are requested via the `/arm/set_mode` service or triggered automatically by `/arm/reach_preset` actions, the `/estop` topic, or safety system events. **`ERROR` is the only state that cannot be exited via `/arm/set_mode`** — it requires an explicit `/arm/clear_error` call.

______________________________________________________________________

## Safety Systems

Five independent safety layers run continuously. All result in a transition to `ERROR` (except the twist watchdog, which is recoverable).

### 1. E-stop (`/estop`)

Publishing `True` to `/estop` immediately calls `arm.stop()` and transitions to `ERROR`, regardless of current state. The arm will not accept any commands until `/arm/clear_error` is called.

### 2. Stale-Twist Watchdog (10 Hz)

In twist-controlled states (`MANUAL`, `CUP_STABILIZE`), if no twist message arrives within **0.5 s** (`TWIST_TIMEOUT_S`) the arm is stopped via `arm.stop()`. The state remains `MANUAL`/`CUP_STABILIZE` — this is **recoverable** by resuming the publisher. This prevents the arm from executing the last velocity command indefinitely if the publishing node dies.

### 3. Collision Detection (100 Hz, Pinocchio RNEA)

On every feedback cycle, measured joint torques are compared against torques predicted by the rigid-body dynamics model (gravity + Coriolis, zero acceleration assumed). The **max per-joint residual** is compared against the threshold for the current state. If exceeded, the arm is stopped and transitions to `ERROR`.

Max per-joint residual is used rather than sum: a contact force loads joints near the contact point, so max is more sensitive to localised contacts and its threshold has a direct physical meaning (Nm on a single joint). Validated on the real arm: ~2 Nm noise floor, ~30–42 Nm under firm hand contact.

| Parameter                               | Default   | Description                                                             |
| --------------------------------------- | --------- | ----------------------------------------------------------------------- |
| `collision_checker.threshold_default`   | **30 Nm** | All states except `OPEN_DOOR`                                           |
| `collision_checker.threshold_open_door` | **50 Nm** | `OPEN_DOOR` requires intentional contact to press accessibility buttons |

Override at launch: `--ros-args -p collision_checker.threshold_default:=25.0`

### 4. Arm Communication Watchdog (100 Hz)

If `RefreshFeedback` raises `TimeoutError` for more than **0.5 s** (`COMMS_TIMEOUT_S`) consecutively, the driver transitions to `ERROR` with reason `"Arm communication timeout"`.

### 5. Kortex Hardware Fault Detection (1 Hz)

The Kortex base is polled once per second via `GetArmState()`. If `ARMSTATE_IN_FAULT` is detected (e.g. following error, overcurrent, physical e-stop button on the base), the driver transitions to `ERROR`. The Kortex state name is published in `/arm/status`.

______________________________________________________________________

## Clearing Errors

Call the `/arm/clear_error` service to exit `ERROR` and return to `IDLE`:

```bash
ros2 service call /arm/clear_error std_srvs/srv/Trigger {}
```

This service:

1. Calls `arm.clear_faults()` on the Kortex base, polling until `ARMSTATE_SERVOING_READY` (up to 5 s). Returns `success: False` if the hardware fault cannot be cleared — resolve the physical cause before retrying.
1. Clears the error reason and transitions to `IDLE`.

> **Note:** `/arm/set_mode` is rejected while in `ERROR`. `clear_error` is the only valid exit.

______________________________________________________________________

## Publishers

| Topic                | Type                                   | Rate   | Notes                                                                                                                        |
| -------------------- | -------------------------------------- | ------ | ---------------------------------------------------------------------------------------------------------------------------- |
| `/arm/joint_states`  | `sensor_msgs/msg/JointState`           | 100 Hz | Gripper position appended as last element (normalized 0–1); gripper velocity and effort are always 0                         |
| `/arm/ee/pose`       | `geometry_msgs/msg/PoseStamped`        | 100 Hz | End-effector position (x, y, z) and orientation as quaternion (x, y, z, w)                                                   |
| `/arm/ee/velocity`   | `geometry_msgs/msg/TwistStamped`       | 100 Hz | End-effector linear velocity (x, y, z) and angular velocity (x, y, z) in tool frame                                          |
| `/arm/ee/force`      | `geometry_msgs/msg/Vector3Stamped`     | 100 Hz | External wrench force at end-effector                                                                                        |
| `/arm/status`        | `diagnostic_msgs/msg/DiagnosticStatus` | 1 Hz   | `message`: current `ArmState` name. `values`: `kortex_arm_state` (Kortex hardware state), `error_reason` (set when in ERROR) |
| `/robot_description` | `std_msgs/msg/String`                  | —      |                                                                                                                              |

## Subscribers

| Topic                         | Type                            | Authorized State                         |
| ----------------------------- | ------------------------------- | ---------------------------------------- |
| `/arm/atdev/twist`            | `geometry_msgs/msg/Twist`       | `CUP_STABILIZE`                          |
| `/arm/xbox/twist`             | `geometry_msgs/msg/Twist`       | `MANUAL`                                 |
| `/arm/cornell/joint_position` | `sensor_msgs/msg/JointState`    | `ORDER_DRINK`, `DRINKING`                |
| `/arm/cornell/cartesian_pose` | `geometry_msgs/msg/PoseStamped` | `ORDER_DRINK`, `DRINKING`                |
| `/arm/cmu/joint_position`     | `sensor_msgs/msg/JointState`    | `OPEN_DOOR`                              |
| `/arm/cmu/cartesian_pose`     | `geometry_msgs/msg/PoseStamped` | `OPEN_DOOR`                              |
| `/estop`                      | `std_msgs/msg/Bool`             | Any — immediately transitions to `ERROR` |

## Service Servers

| Service                     | Type                                    | Notes                                                                                                            |
| --------------------------- | --------------------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| `/arm/set_mode`             | `arm_interfaces/srv/SetMode`            | Rejected when in `ERROR`                                                                                         |
| `/arm/clear_error`          | `std_srvs/srv/Trigger`                  | Only valid exit from `ERROR`; clears Kortex hardware faults                                                      |
| `/arm/set_speed_preset`     | `arm_interfaces/srv/SetSpeedPreset`     |                                                                                                                  |
| `/arm/get_speed_preset`     | `arm_interfaces/srv/GetSpeedPreset`     |                                                                                                                  |
| `/arm/open_gripper`         | `std_srvs/srv/Trigger`                  |                                                                                                                  |
| `/arm/close_gripper`        | `std_srvs/srv/Trigger`                  |                                                                                                                  |
| `/arm/set_gripper_position` | `arm_interfaces/srv/SetGripperPosition` | Partial gripper position; `position` in \[0, 1\] (0.0 = open, 1.0 = closed, clamped). Rejected in `IDLE`/`ERROR` |
| `/arm/check_reachability`   | `arm_interfaces/srv/CheckReachability`  |                                                                                                                  |

## Action Servers

| Topic                             | Type                                      | Authorized State          |
| --------------------------------- | ----------------------------------------- | ------------------------- |
| `/arm/reach_preset`               | `arm_interfaces/action/ReachPreset`       | Any non-`ERROR` state     |
| `/arm/cornell/execute_trajectory` | `arm_interfaces/action/ExecuteTrajectory` | `ORDER_DRINK`, `DRINKING` |
| `/arm/cmu/execute_trajectory`     | `arm_interfaces/action/ExecuteTrajectory` | `OPEN_DOOR`               |

### `ReachPreset` presets

| Constant               | Value | Description                    |
| ---------------------- | ----- | ------------------------------ |
| `PRESET_HOME`          | 0     | Move to home position          |
| `PRESET_RETRACT`       | 1     | Move to retract position       |
| `PRESET_ZERO`          | 2     | Move to zero position          |
| `PRESET_CUP_STABILIZE` | 3     | Move to cup stabilize position |

All presets transition through `PRESET_IN_MOTION` while executing, stream `JointState` feedback, and return to `IDLE` on completion.
