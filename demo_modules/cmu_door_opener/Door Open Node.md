# CMU Door Opener — Unified Documentation

---

## Package Overview

The `cmu_door_opener` package contains two ROS 2 nodes that work together to detect and press a door-opener button. The `cmu_door_opener_interfaces` package defines the custom message and action types they use.

**Nodes:**
1. `button_detector` — perception only (YOLO + depth + filtering)
2. `button_push_controller` — motion execution (push + force monitor + retract)

**Launch:** `ros2 launch cmu_door_opener cmu_door_opener.launch.py`

---

## File Inventory

| File | Purpose |
|---|---|
| `cmu_door_opener/button_detector.py` | Detector node source |
| `cmu_door_opener/button_push_controller.py` | Push controller node source |
| `cmu_door_opener/button_yolo_weights.pt` | YOLO segmentation model (6.5 MB) |
| `launch/cmu_door_opener.launch.py` | Launch file for both nodes |
| `package.xml` | ROS 2 package manifest |
| `setup.py` | Python package build config & entry points |
| *interfaces:* `msg/ButtonInfo.msg` | Custom message for detected button data |
| *interfaces:* `action/DoorOpen.action` | Custom action for the push sequence |

---

## Custom Interfaces (`cmu_door_opener_interfaces`)

### `ButtonInfo.msg`

| Field | Type | Description |
|---|---|---|
| `id` | `uint8` | Button identifier (currently always 0) |
| `segmentation_mask` | `sensor_msgs/Image` | Single-channel (mono8) mask, 255 = button pixel |
| `bounding_box` | `int32[4]` | Pixel coords `[x_min, y_min, x_max, y_max]` from YOLO |
| `confidence` | `float32` | YOLO detection confidence `[0.0 - 1.0]` |
| `pose_xyzrpy` | `float64[6]` | `[x, y, z, roll, pitch, yaw]` in `base_link` frame. Position in meters, orientation in radians (xyz euler). |
| `is_pressable` | `bool` | Whether the arm can reach this button (IK solvable). **TODO: currently hardcoded `true` on success — Yucheng to provide Swap the exact IK function.** |

### ButtonInfo States

The message is published every detection cycle (~5 Hz). It can be in one of three states:

| State | When | `confidence` | `segmentation_mask` | `bounding_box` | `pose_xyzrpy` | `is_pressable` |
|---|---|---|---|---|---|---|
| **1. No button found** | YOLO finds nothing, camera not ready | `0.0` | 1x1 black | `[0,0,0,0]` | all `-1.0` | `false` |
| **2. Detected, not pressable** | YOLO found button but depth/TF/filter failed, too far, IK unsolvable | real `> 0` | real mask | real bbox | all `-1.0` | `false` |
| **3. Detected and pressable** | Full pipeline succeeded, filtered pose ready | real `> 0` | real mask | real bbox | real values | `true` |

**How consumers distinguish states:**
- `confidence == 0.0` → state 1 (no button)
- `confidence > 0.0` and `pose_xyzrpy` all `-1.0` → state 2 (detected, not pressable)
- `confidence > 0.0` and `pose_xyzrpy` valid and `is_pressable == true` → state 3 (ready to push)

### `DoorOpen.action`

| Section | Field | Type | Description |
|---|---|---|---|
| **Goal** | *(empty)* | | Triggers the push sequence, no parameters |
| **Result** | `success` | `bool` | Whether the full sequence completed |
| | `message` | `string` | Human-readable status/error |
| **Feedback** | `distance_to_button` | `float32` | Approximate meters remaining to button contact (published during push phase) |

---

## Node 1: `button_detector`

**ROS node name:** `button_press_vision_node`
**Entry point:** `button_detector = cmu_door_opener.button_detector:main`
**Executor:** single-threaded (`rclpy.spin`)

### Start / Stop / Enable / Disable Lifecycle

```
NODE LAUNCH (ros2 run / launch file)
  |
  v
__init__():
  1. Declare all ROS parameters
  2. Initialize all state (CvBridge, TF buffer, pose filter, viz counters)
  3. self._detection_enabled = False
  4. Create all publishers
  5. Create /arm/door/detection/enable service server (SetBool)
  6. Create all subscribers (camera topics — always active, data is buffered)
  7. self.yolo = None   *** YOLO IS NOT LOADED — GPU memory preserved ***
  8. Optionally open OpenCV visualization window
  9. Create timer at process_rate_hz (default 5 Hz) → process_once()
  |
  v
rclpy.spin(node)   ← node alive, timer fires every 200ms
  |                   process_once() returns immediately (detection disabled)
  |                   draw_viz() still runs if OpenCV window is enabled
  v
  ...waiting for /arm/door/detection/enable service call...
```

```
/arm/door/detection/enable called with data=True
  |
  v
_srv_detection_enable():
  1. _load_yolo()  ← YOLO model loaded to GPU/CPU NOW
  2. _pose_filter.reset()
  3. self._detection_enabled = True
  |
  v
process_once() now runs the full pipeline each tick (~5 Hz)
  - ALWAYS publishes a ButtonInfo every cycle
  - If any step fails → publishes failure ButtonInfo (pose = all -1)
  - If pipeline succeeds and filter is stable → publishes real filtered data
```

```
/arm/door/detection/enable called with data=False
  |
  v
_srv_detection_enable():
  1. self._detection_enabled = False
  2. _pose_filter.reset()
  3. _unload_yolo()  ← YOLO model deleted, torch.cuda.empty_cache(), GPU freed
  |
  v
process_once() returns immediately again (detection disabled)
```

### YOLO Model Load / Unload

| Event | Action |
|---|---|
| Node launch (`__init__`) | `self.yolo = None`. Model is **not loaded**. GPU memory is free. |
| `enable(True)` | `_load_yolo()`: Loads `button_yolo_weights.pt` via `YOLO(path)`, moves to `cuda:0`. Falls back to CPU if CUDA fails. |
| `enable(False)` | `_unload_yolo()`: `del self.yolo`, `self.yolo = None`, `torch.cuda.empty_cache()`. GPU memory freed. |
| Re-enable after disable | `_load_yolo()` loads the model again from disk. |

### Subscribers (always active, even when detection is disabled)

| Topic | Type | QoS | Callback | What it stores |
|---|---|---|---|---|
| `/camera/wrist/color/image_raw` | `sensor_msgs/Image` | `SENSOR_DATA` (best-effort) | `cb_rgb` | Converts to BGR8 via CvBridge → `self.latest_rgb`. Records timestamp. |
| `/camera/wrist/depth/image_rect_raw` | `sensor_msgs/Image` | `SENSOR_DATA` (best-effort) | `cb_depth` | Converts via CvBridge → `depth_to_meters()` → `self.latest_depth_m`. Records timestamp. |
| `/camera/wrist/color/camera_info` | `sensor_msgs/CameraInfo` | Reliable, depth=10 | `cb_color_info` | Stores as `self.color_info`. Extracts `header.frame_id` → `self.color_frame_id`. |
| `/camera/wrist/depth/camera_info` | `sensor_msgs/CameraInfo` | Reliable, depth=10 | `cb_depth_info` | Stores as `self.depth_info` (depth intrinsics). |
| `/camera/wrist/extrinsics/depth_to_color` | `realsense2_camera_msgs/Extrinsics` | Reliable, TRANSIENT_LOCAL, depth=1 | `cb_extrinsics` | Stores rotation (3x3) + translation (3x1) → `self.depth_to_color_extr`. |

These are always subscribed so that data is buffered and ready the instant detection is enabled — no startup delay.

### Service Servers

| Topic | Type | Callback | Behavior |
|---|---|---|---|
| `/arm/door/detection/enable` | `std_srvs/SetBool` | `_srv_detection_enable` | `True`: load YOLO, reset filter, start pipeline. `False`: stop pipeline, reset filter, unload YOLO, free GPU. Returns `success=False` only if YOLO load fails. |

### Publishers

| Topic | Type | Rate | Condition | What it publishes |
|---|---|---|---|---|
| `/arm/door/button_info` | `ButtonInfo` | ~5 Hz | **Every cycle when detection enabled** — real data OR failure defaults | ButtonInfo with filtered pose (or -1 on failure) |
| `/button/debug_point_camera` | `geometry_msgs/PointStamped` | ~5 Hz | Only on successful 3D computation | Raw (unfiltered) button centroid in camera optical frame |
| `/button/debug_point_base` | `geometry_msgs/PointStamped` | ~5 Hz | Only on successful TF transform | Raw (unfiltered) button centroid in `base_link` frame |
| `/button/normal_marker` | `visualization_msgs/Marker` | ~5 Hz | Only on successful normal estimation | Red arrow from centroid along surface normal, 0.1m, in `base_link`. For RViz. |

### Processing Pipeline (each timer tick, when detection enabled)

Every cycle **always** publishes a `ButtonInfo` in one of three states.

```
process_once()
  |
  +-- draw_viz()              # OpenCV window (always, even when disabled)
  |
  +-- [guard] _detection_enabled == False? → return (no publish)
  |
  |   *** FROM HERE: every exit path publishes a ButtonInfo ***
  |
  +-- [check] camera data missing?
  |     → publish STATE 1 (no button), return
  |
  +-- Step 1: YOLO inference
  |     run_yolo(latest_rgb)
  |       - Runs YOLO segmentation on the latest BGR8 image
  |       - Selects highest-confidence detection (optionally filtered by target_class)
  |       - CUDA fallback: if CUDA fails mid-run, switches to CPU permanently
  |       - Returns: mask (bool numpy array) or None
  |     mask is None? → publish STATE 1 (no button), return
  |
  |   === Button detected from here (mask, bbox, confidence are valid) ===
  |
  +-- Step 2: 3D centroid from depth
  |     compute_3d_from_bbox_center_depth_extrinsics(bbox, depth)
  |       - Iterates depth image at stride intervals
  |       - Collects 3D points near bbox center, takes median centroid
  |     Failed? → publish STATE 2 (detected, not pressable), return
  |
  +-- Step 3: TF transform to base_link
  |     transform_point_to_base(centroid_cam)
  |     Failed? → publish STATE 2 (detected, not pressable), return
  |
  +-- Step 4: Surface normal estimation
  |     estimate_surface_normal(points_cam)  → Open3D normals
  |     transform_vector_to_base(normal_cam) → rotate to base frame
  |     Build orientation: tool Z = -normal → rotation matrix → euler xyz → rpy
  |     (if normal fails: rpy = [0, 0, 0])
  |
  +-- Step 5: Low-pass filter (PoseFilter)
  |     _pose_filter.update(centroid_base, rpy)
  |       EMA: new = alpha * raw + (1 - alpha) * prev
  |     Filter not stable yet (count < min_samples)?
  |       → publish STATE 2 (detected, not pressable), return
  |
  +-- Step 6: Publish STATE 3 (detected and pressable)
        _publish_detected_pressable(filtered_xyz, filtered_rpy, mask, bbox, confidence)
        - pose_xyzrpy = filtered values (valid)
        - is_pressable = True (TODO: IK check)
        - mask, bbox, confidence = raw values from this frame's YOLO
```

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `rgb_topic` | str | `/camera/wrist/color/image_raw` | RGB image topic |
| `color_info_topic` | str | `/camera/wrist/color/camera_info` | RGB camera info topic |
| `depth_topic` | str | `/camera/wrist/depth/image_rect_raw` | Depth image topic |
| `depth_info_topic` | str | `/camera/wrist/depth/camera_info` | Depth camera info topic |
| `extrinsics_topic` | str | `/camera/wrist/extrinsics/depth_to_color` | RealSense depth→color extrinsics |
| `color_optical_frame` | str | `camera_color_optical_frame` | Fallback TF frame (overridden by CameraInfo header) |
| `base_frame` | str | `base_link` | Target TF frame for all output |
| `yolo_model` | str | `<package>/button_yolo_weights.pt` | Path to YOLO .pt weights |
| `detection_confidence` | float | `0.5` | Minimum YOLO confidence to accept |
| `target_class` | str | `""` | Filter by class name (empty = any) |
| `yolo_use_fp16` | bool | `True` | FP16 on CUDA (ignored on CPU) |
| `yolo_imgsz` | int | `640` | YOLO input size |
| `depth_stride` | int | `4` | Sample every Nth depth pixel |
| `min_depth_m` | float | `0.10` | Minimum valid depth (m) |
| `max_depth_m` | float | `3.00` | Maximum valid depth (m) |
| `min_projected_points` | int | `30` | Min 3D points for valid centroid |
| `mask_core_min_dist_px` | float | `6.0` | Distance transform threshold for mask interior |
| `mask_core_min_points` | int | `40` | Min interior points for mask core |
| `bbox_center_radius_px` | int | `18` | Radius around bbox center for depth collection |
| `normal_radius` | float | `0.05` | Open3D normal search radius (m) |
| `normal_max_nn` | int | `30` | Max neighbors for normal search |
| `press_offset` | float | `0.0` | Offset along normal before press (m) |
| `fixed_pose_quat` | float[4] | `[0.5, 0.5, 0.5, 0.5]` | Fallback quaternion [x,y,z,w] |
| `filter_alpha` | float | `0.3` | EMA responsiveness (0=frozen, 1=no smoothing) |
| `filter_min_samples` | int | `3` | Samples before filter is "stable" |
| `show_opencv_windows` | bool | `True` | Show debug visualization window |
| `window_scale` | float | `1.0` | Visualization window scale factor |
| `process_rate_hz` | float | `5.0` | Timer frequency |
| `tf_timeout_s` | float | `0.5` | TF2 lookup timeout (s) |

### OpenCV Visualization Window

Shows side-by-side view (`button_viz`):
- **Left:** Raw RGB feed with detection status (`Det=ON/OFF`), YOLO load status, FPS, image age.
- **Right:** YOLO overlay — bbox, mask, class+confidence. Centroid pixel marker. Camera and base frame xyz.

Press `q` to shut down the node.

### Internal Classes

**`PoseFilter`** — EMA over `xyz` (float64[3]) and `rpy` (float64[3]).
- `update(xyz, rpy)`: First call seeds. Subsequent: `new = alpha * raw + (1-alpha) * prev`.
- `is_stable`: True when `count >= min_samples`.
- `reset()`: Called on enable/disable. Zeroes state and count.

---

## Node 2: `button_push_controller`

**ROS node name:** `button_push_controller`
**Entry point:** `button_push_controller = cmu_door_opener.button_push_controller:main`
**Executor:** `MultiThreadedExecutor` (required because the action server callback blocks)

### Start / Stop Logic

```
NODE LAUNCH (ros2 run / launch file)
  |
  v
__init__():
  1. Create ReentrantCallbackGroup
  2. Subscribe to /arm/door/button_info (ButtonInfo) → stores latest
  3. Subscribe to /arm/ee_force (Vector3) → stores latest force vector
  4. Create publisher: /arm/cmu/cartesian_pose (PoseStamped)
  5. Create publisher: /arm/cmu/twist (Twist)
  6. Create action client: /arm/reach_preset (ReachPreset)
  7. Create action server: /arm/door/open (DoorOpen)
  |
  v
MultiThreadedExecutor.spin()   ← passive, waiting for action goals
  |                              continuously buffers latest ButtonInfo + force
  v
SHUTDOWN: KeyboardInterrupt → destroy node
```

This node is passive. It just buffers the latest `ButtonInfo` from the detector. When `/arm/door/open` is called (by behavior tree, after user confirmation, etc.), it grabs the latest message and acts.

### Subscribers

| Topic | Type | QoS | Callback | What it stores |
|---|---|---|---|---|
| `/arm/door/button_info` | `ButtonInfo` | Reliable, depth=10 | `_cb_button_info` | Stores entire msg as `self.latest_button_info`. Overwrites every message. |
| `/arm/ee_force` | `geometry_msgs/Vector3` | Reliable, depth=10 | `_cb_ee_force` | Converts to `np.array([x, y, z])` → `self.latest_ee_force`. |

### Publishers

| Topic | Type | When | What |
|---|---|---|---|
| `/arm/cmu/cartesian_pose` | `geometry_msgs/PoseStamped` | Once per action (step 2) | Push pose = button pose + PUSH_EXTRA overshoot. Frame: `base_link`. |
| `/arm/cmu/twist` | `geometry_msgs/Twist` | On contact or timeout (step 3) | All-zero Twist to stop arm. |

### Action Servers

| Topic | Type | Callback |
|---|---|---|
| `/arm/door/open` | `DoorOpen` | `_execute_open_door` |

### Action Clients

| Topic | Type | When used |
|---|---|---|
| `/arm/reach_preset` | `arm_interfaces/ReachPreset` | Step 4 (retract). Sends `PRESET_RETRACT` (value 1). |

### Action Execution: `_execute_open_door`

```
/arm/door/open goal received
  |
  +-- Step 1: Grab latest ButtonInfo
  |     Read self.latest_button_info IMMEDIATELY (no waiting)
  |     ABORT if: None (no messages received at all)
  |     ABORT if: confidence==0 + pose all -1 (state 1: no button detected)
  |     ABORT if: confidence>0 + pose all -1 (state 2: detected but not pressable)
  |     ABORT if: is_pressable == False (state with valid pose but IK fails)
  |     Convert: pose_xyzrpy → PoseStamped
  |       position = xyz directly
  |       orientation = Rotation.from_euler('xyz', [r,p,y]) → quaternion
  |       frame_id = 'base_link'
  |
  +-- Step 2: Command push
  |     push_pose = target + PUSH_EXTRA (0.02m) on x-axis
  |     Publish → /arm/cmu/cartesian_pose
  |
  +-- Step 3: Monitor force (blocking loop, 50 Hz, up to 10s)
  |     Each iteration:
  |       Read force magnitude (L2 norm of latest_ee_force)
  |       Publish feedback: distance_to_button
  |       If force > 20 N → stop_arm(), break
  |     Timeout → stop_arm() anyway
  |
  +-- Step 4: Retract arm
  |     /arm/reach_preset → PRESET_RETRACT
  |     Wait for acceptance (5s) → wait for completion (30s)
  |     ABORT if: server unavailable or goal rejected
  |
  +-- Step 5: Return result
        goal_handle.succeed()
        success=True, message='Button push complete, arm retracted'
```

### Constants

| Name | Value | Description |
|---|---|---|
| `PUSH_EXTRA` | `0.02` m | Overshoot past button surface |
| `FORCE_THRESHOLD` | `20.0` N | Contact detection threshold |
| `PUSH_TIMEOUT` | `10.0` s | Max wait for contact |
| `FORCE_CHECK_RATE` | `0.02` s | Force check interval (50 Hz) |

### Abort Conditions

1. No ButtonInfo received (`latest_button_info is None`) — detector not running
2. **State 1**: `confidence == 0.0` and pose all -1 — no button detected
3. **State 2**: `confidence > 0.0` but pose all -1 — button detected but not pressable (depth/TF/filter failed, too far)
4. `is_pressable == False` — button detected, pose valid, but IK not solvable
5. `/arm/reach_preset` server unavailable (5s timeout)
6. Retract goal rejected

Push timeout does NOT abort — proceeds to retract and returns success.

---

## Launch File

`launch/cmu_door_opener.launch.py` starts both nodes:

| Node | Parameters overridden |
|---|---|
| `button_detector` | `show_opencv_windows: False`, `process_rate_hz: 5.0`, `filter_alpha: 0.3`, `filter_min_samples: 3` |
| `button_push_controller` | *(none)* |

---

## End-to-End Sequence

```
                 External System              button_detector            button_push_controller      arm_driver
                      │                            │                            │                       │
  1. Launch both nodes│                            │                            │                       │
                      │                            │ (node alive, YOLO NOT      │ (node alive,          │
                      │                            │  loaded, subs buffering    │  buffering ButtonInfo  │
                      │                            │  camera data)              │  + force data)         │
                      │                            │                            │                       │
  2. Enable detection │                            │                            │                       │
     ros2 service call│                            │                            │                       │
     /arm/door/       │                            │                            │                       │
     detection/enable │                            │                            │                       │
     data: true       │                            │                            │                       │
                      │───────────────────────────▶│                            │                       │
                      │                            │ _load_yolo() → GPU         │                       │
                      │                            │ _detection_enabled = True   │                       │
                      │                            │                            │                       │
  3. Detection runs   │                            │ process_once() every 200ms │                       │
                      │                            │                            │                       │
     Cycle N (fail):  │                            │──pub ButtonInfo(pose=-1)──▶│ stores latest          │
     Cycle N+1 (fail):│                            │──pub ButtonInfo(pose=-1)──▶│ stores latest          │
     Cycle N+2 (fail):│                            │──pub ButtonInfo(pose=-1)──▶│ stores latest          │
     ... filter warmup│                            │                            │                       │
     Cycle N+3 (ok):  │                            │──pub ButtonInfo(pose=OK)──▶│ stores latest          │
     Cycle N+4 (ok):  │                            │──pub ButtonInfo(pose=OK)──▶│ stores latest          │
                      │                            │  ... continuous ~5 Hz ...  │                       │
                      │                            │                            │                       │
  4. User confirms    │                            │                            │                       │
     (middle layer    │                            │                            │                       │
      handles this)   │                            │                            │                       │
                      │                            │                            │                       │
  5. Send open cmd    │                            │                            │                       │
     /arm/door/open   │───────────────────────────────────────────────────────▶│                       │
                      │                            │                            │ grab latest ButtonInfo │
                      │                            │                            │ (the one with pose=OK) │
                      │                            │                            │                       │
                      │                            │                            │ pub cartesian_pose ──▶│ move arm
                      │                            │                            │◀── ee_force ──────────│
                      │◀─────────────── feedback: distance_to_button ──────────│                       │
                      │                            │                            │ force > 20N            │
                      │                            │                            │ pub zero twist ───────▶│ stop
                      │                            │                            │ call reach_preset ────▶│ retract
                      │◀─────────────── result: success ───────────────────────│                       │
                      │                            │                            │                       │
  6. Disable detection│                            │                            │                       │
     /arm/door/       │                            │                            │                       │
     detection/enable │                            │                            │                       │
     data: false      │                            │                            │                       │
                      │───────────────────────────▶│                            │                       │
                      │                            │ _detection_enabled = False  │                       │
                      │                            │ _unload_yolo() → GPU freed │                       │
```

---

## Full Topic / Service / Action Reference

### Topics Published

| Topic | Type | Publisher | Rate | Description |
|---|---|---|---|---|
| `/arm/door/button_info` | `ButtonInfo` | button_detector | ~5 Hz (every cycle when enabled) | Detection result — real data or failure (-1) |
| `/button/debug_point_camera` | `PointStamped` | button_detector | ~5 Hz | Raw centroid in camera frame (debug) |
| `/button/debug_point_base` | `PointStamped` | button_detector | ~5 Hz | Raw centroid in base_link (debug) |
| `/button/normal_marker` | `Marker` | button_detector | ~5 Hz | Surface normal arrow for RViz (debug) |
| `/arm/cmu/cartesian_pose` | `PoseStamped` | button_push_controller | Once per action | Push pose command to arm_driver |
| `/arm/cmu/twist` | `Twist` | button_push_controller | Once per action | Zero twist to stop arm |

### Topics Subscribed

| Topic | Type | Subscriber | QoS | Description |
|---|---|---|---|---|
| `/camera/wrist/color/image_raw` | `Image` | button_detector | SENSOR_DATA | RGB from wrist camera |
| `/camera/wrist/depth/image_rect_raw` | `Image` | button_detector | SENSOR_DATA | Depth from wrist camera |
| `/camera/wrist/color/camera_info` | `CameraInfo` | button_detector | Reliable | RGB intrinsics |
| `/camera/wrist/depth/camera_info` | `CameraInfo` | button_detector | Reliable | Depth intrinsics |
| `/camera/wrist/extrinsics/depth_to_color` | `Extrinsics` | button_detector | Reliable+TRANSIENT_LOCAL | Depth→color registration |
| `/arm/door/button_info` | `ButtonInfo` | button_push_controller | Reliable | From detector |
| `/arm/ee_force` | `Vector3` | button_push_controller | Reliable | From arm_driver |

### Services

| Topic | Type | Server | Description |
|---|---|---|---|
| `/arm/door/detection/enable` | `std_srvs/SetBool` | button_detector | `True`=load YOLO + start. `False`=stop + unload YOLO + free GPU. |

### Actions

| Topic | Type | Server | Client | Description |
|---|---|---|---|---|
| `/arm/door/open` | `DoorOpen` | button_push_controller | External (behavior tree) | Full push sequence |
| `/arm/reach_preset` | `ReachPreset` | arm_driver | button_push_controller | Retract arm (PRESET_RETRACT=1) |

---

## TF Frames Used

| Frame | Used by | Purpose |
|---|---|---|
| Camera optical frame (from CameraInfo header, fallback: `camera_color_optical_frame` param) | button_detector | Source frame for 3D computations |
| `base_link` | button_detector, button_push_controller | Target frame for all poses, arm's reference frame |

---

## TODO

- [ ] **Yucheng → Swap:** Provide exact IK solver function for `is_pressable` field. Currently hardcoded `True` on success in `button_detector.py:745`.
- [ ] Refine `distance_to_button` feedback — currently static `PUSH_EXTRA`. Could use live EE position + FK.
- [ ] Push overshoot is hardcoded +x. Should follow surface normal approach direction.
