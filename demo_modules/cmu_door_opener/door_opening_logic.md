# Door Opening Logic — `cmu_door_opener`

## Overview

The door opening pipeline runs on two ROS 2 nodes (`button_detector` and `button_push_controller`) that work sequentially: detect the button, present it to the user, and push on confirmation.

______________________________________________________________________

## Flow

### 1. Initialization (Enter Door-Open Mode)

Both nodes launch and begin subscribing to topics (wrist camera feeds from AT-Dev, arm force/velocity, etc.), but the YOLO model is **not** loaded into GPU — preserving GPU memory until needed.

### 2. Enable Detection

A `/arm/door/detection/enable` service call loads the YOLO segmentation model onto GPU and starts the detection loop at 5 Hz.

### 3. Detection Pipeline (each cycle)

| Step                      | Description                                                                                                                                                                |
| ------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| YOLO segmentation         | Run on the latest RGB frame. Produces a segmentation mask and bounding box in **image frame**.                                                                             |
| 3D projection             | Project depth pixels near the bounding box center into 3D using depth intrinsics + depth-to-color extrinsics. Compute the median centroid in the **camera optical frame**. |
| TF transform              | Transform the centroid from camera frame to **`base_link`** via TF.                                                                                                        |
| Surface normal estimation | Estimate the button surface normal from the 3D point cloud (Open3D), transform it to `base_link`. This determines the push direction.                                      |
| EMA filtering             | Apply exponential moving average on xyz and rpy to stabilize the pose across frames.                                                                                       |
| Reachability check        | Flag `is_pressable = true` if the button is within the arm's 2D reach from `base_link`.                                                                                    |

The output is a `ButtonInfo` message containing:

- Button pose (xyz, rpy) in `base_link`
- Confidence score
- Segmentation mask + bounding box (image frame)
- `is_pressable` flag

If any step fails, a partial or empty `ButtonInfo` is published so the UI still reflects the current state.

### 4. User Confirmation

All detection results are displayed on the user's UI — the camera feed, bounding box overlay, and the target push point. The user visually verifies the detection and decides whether to confirm the push.

### 5. Push Execution (on user confirmation)

The `/arm/door/open` action is called. Before executing, the controller checks:

- Arm must be in `OPEN_DOOR` mode (set via `/arm/set_mode`)
- A valid `ButtonInfo` must exist with `is_pressable = true`

If either condition fails, the action **aborts with an error message** and the arm does not move.

**Phase 1 — Approach:** The arm moves to the button's surface pose. During this phase, end-effector force is monitored. If force exceeds **30 N** (e.g. unexpected obstacle), the arm **stops immediately and aborts** — no retract, no further motion.

**Phase 2 — Press:** The arm pushes **10 cm along the button's surface normal**. Force is continuously monitored — once **30 N** is reached, the arm stops (button pressed). If the phase times out without hitting 30 N, it still proceeds to retract.

### 6. Retract

After the push, the arm retracts to a preset position via the `/arm/reach_preset` action.

### 7. Result

| Outcome                                                       | Result                                                  | Arm State                     |
| ------------------------------------------------------------- | ------------------------------------------------------- | ----------------------------- |
| Button pressed (force threshold reached) and retract succeeds | `success = true`, "Button push complete, arm retracted" | Retracted to preset           |
| Push timed out (no force contact) but retract succeeds        | `success = true`, "Button push complete, arm retracted" | Retracted to preset           |
| Force exceeded during approach (Phase 1)                      | `success = false`, force exceeded error                 | Stopped in place (no retract) |
| No valid detection / not pressable                            | `success = false`, detection error                      | Did not move                  |
| Arm not in `OPEN_DOOR` mode                                   | `success = false`, wrong mode error                     | Did not move                  |
| Retract service unavailable or rejected                       | `success = false`, retract error                        | Stopped at push position      |

### 8. Disable Detection

When door-open mode ends, a `/arm/door/detection/enable = false` call unloads the YOLO model and frees GPU memory.
