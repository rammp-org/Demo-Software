# Arm Control Node

> **Summary**
> Main node that controls the Kinova arm. This is the only node that communicates with the arm directly. It publishes `/arm/joint_states` and `/arm/status` to other nodes. Based on the internal state machine, it gates incoming commands to a single authorised source per state.
>
> Command sources use two paradigms depending on the state:
>
> - **TWIST states** (`MANUAL`, `CUP_STABILIZE`): authorised source streams velocity commands on `/arm/{source}/twist` at up to 25 Hz. A 200 ms watchdog stops the arm if the stream goes silent.
> - **POSITION states** (`OPEN_DOOR`, `ORDER_DRINK`, `DRINKING`): authorised source sends fire-and-forget joint position or Cartesian pose commands, or submits a trajectory via the `/arm/{source}/execute_trajectory` action.

> **TODO:**
>
> - \[ \] Determine how to recover from ERROR state

______________________________________________________________________

## State Machine

| State              | Authorised Source | Command Mode | Description                                                           |
| ------------------ | ----------------- | ------------ | --------------------------------------------------------------------- |
| `RETRACTED`        | —                 | None         | Default standby. Arm in safe retracted position. MEBot can drive.     |
| `HOME`             | —                 | None         | Arm at home position.                                                 |
| `PRESET_IN_MOTION` | —                 | None         | Arm moving to a preset via `/arm/reach_preset`. No commands accepted. |
| `OPEN_DOOR`        | `cmu`             | Position     | CMU controls arm to open a door.                                      |
| `ORDER_DRINK`      | `cornell`         | Position     | Cornell controls arm to order a drink.                                |
| `DRINKING`         | `cornell`         | Position     | Cornell controls arm to assist with drinking.                         |
| `CUP_STABILIZE`    | `atdev`           | Twist        | ATDev streams velocity corrections to stabilize a cup.                |
| `MANUAL`           | `xbox`            | Twist        | Xbox controller streams velocity commands for manual teleoperation.   |
| `ERROR`            | —                 | None         | Entered on fault or e-stop. No commands accepted.                     |

State transitions are requested via the `/arm/set_mode` service or triggered automatically by `/arm/reach_preset` actions and the `/estop` topic.

______________________________________________________________________

## Publishers

| Topic                | Type                                   | Rate   |
| -------------------- | -------------------------------------- | ------ |
| `/arm/joint_states`  | `sensor_msgs/msg/JointState`           | 100 Hz |
| `/arm/imu`           | `sensor_msgs/msg/Imu`                  | —      |
| `/arm/status`        | `diagnostic_msgs/msg/DiagnosticStatus` | 1 Hz   |
| `/robot_description` | `std_msgs/msg/String`                  | —      |

## Subscribers

| Topic                         | Type                            | Authorised State                         |
| ----------------------------- | ------------------------------- | ---------------------------------------- |
| `/arm/atdev/twist`            | `geometry_msgs/msg/Twist`       | `CUP_STABILIZE`                          |
| `/arm/xbox/twist`             | `geometry_msgs/msg/Twist`       | `MANUAL`                                 |
| `/arm/cornell/joint_position` | `sensor_msgs/msg/JointState`    | `ORDER_DRINK`, `DRINKING`                |
| `/arm/cornell/cartesian_pose` | `geometry_msgs/msg/PoseStamped` | `ORDER_DRINK`, `DRINKING`                |
| `/arm/cmu/joint_position`     | `sensor_msgs/msg/JointState`    | `OPEN_DOOR`                              |
| `/arm/cmu/cartesian_pose`     | `geometry_msgs/msg/PoseStamped` | `OPEN_DOOR`                              |
| `/estop`                      | `std_msgs/msg/Bool`             | Any — immediately transitions to `ERROR` |

## Service Servers

| Topic           | Type                         |
| --------------- | ---------------------------- |
| `/arm/set_mode` | `arm_interfaces/srv/SetMode` |

## Action Servers

| Topic                             | Type                                      | Authorised State          |
| --------------------------------- | ----------------------------------------- | ------------------------- |
| `/arm/reach_preset`               | `arm_interfaces/action/ReachPreset`       | Any (ungated)             |
| `/arm/cornell/execute_trajectory` | `arm_interfaces/action/ExecuteTrajectory` | `ORDER_DRINK`, `DRINKING` |
| `/arm/cmu/execute_trajectory`     | `arm_interfaces/action/ExecuteTrajectory` | `OPEN_DOOR`               |

### `ReachPreset` presets

| Constant               | Value | Final State     |
| ---------------------- | ----- | --------------- |
| `PRESET_HOME`          | 0     | `HOME`          |
| `PRESET_RETRACT`       | 1     | `RETRACTED`     |
| `PRESET_ZERO`          | 2     | `RETRACTED`     |
| `PRESET_CUP_STABILIZE` | 3     | `CUP_STABILIZE` |

All presets transition through `PRESET_IN_MOTION` while executing and stream `JointState` feedback.
