# TF2 Camera Extrinsics for GUI Bridge

## TL;DR

> **Quick Summary**: Replace hardcoded camera extrinsics in `Gui_bridge.py` with live TF2 lookups so UE's projective grid mapping uses actual camera poses from the robot's TF tree.
> 
> **Deliverables**:
> - TF2 Buffer + Listener integrated into `GuiBridge` node
> - Pure conversion function `_tf_to_ue_extrinsics()` with unit test
> - Wrist, nav1, nav2 camera extrinsics sourced from TF2
> - Fallback behavior when TF tree is unavailable
> - Configurable TF frame names as ROS parameters
> 
> **Estimated Effort**: Medium
> **Parallel Execution**: YES — 2 waves
> **Critical Path**: Task 1 → Task 2 → Task 3 → Task 4

---

## Context

### Original Request
Wrist camera depth data displays as a wall when arm faces the floor. Root cause: extrinsics are hardcoded to `(0,0,0)` and never updated from the arm's pose. User expanded scope to replace ALL hardcoded camera extrinsics (wrist, nav1, nav2) with TF2 lookups for consistency.

### Interview Summary
**Key Discussions**:
- Wrist camera extrinsics always `(0,0,0)`, subscriber was commented out and pointed at wrong topic
- Nav camera extrinsics hardcoded — user wants TF2 for all cameras
- Branch `feature/106-drive-base-publisher` already has full TF infrastructure
- User confirmed: TF2 lookup approach, `wrist_color_frame` (or `wrist_wrist_camera_link`) as wrist TF frame
- All 3 cameras must use identical lookup+convert pattern

**Research Findings**:
- TF tree confirmed in `description.launch.py`: `mebot` → `base_link` → `{nav1_link, nav2_link, ...→end_effector_link→wrist_wrist_camera_link}`
- Nav1 subscribes to `image_rotated` topics (pre-rotated for 90° camera mount) while nav2 uses `image_raw`
- `scipy.spatial.transform.Rotation` already imported in `Gui_bridge.py`
- UE metadata format: `{x, y, z, pitch, roll, yaw}` with `transform_space: "relative"`
- `SingleThreadedExecutor` — TF2 lookups must use zero timeout

### Metis Review
**Identified Gaps** (addressed):
- ROS→UE coordinate conversion needs empirical validation (Y-axis negation, Euler convention) → added debug logging task + conversion test
- Nav1 image rotation compensation (90° roll in TF should not go to UE) → explicit handling in conversion function
- Source frame should be `mebot` (user-specified), conversion function handles the mapping
- SingleThreadedExecutor constraint → zero-timeout lookups with exception fallback
- Fallback behavior when TF unavailable → last-known-good with throttled warnings
- `tf2_ros` dependency missing from `package.xml` → added to Task 1

---

## Work Objectives

### Core Objective
Replace hardcoded camera extrinsics in `Gui_bridge.py` with live TF2 lookups from the robot's transform tree, so UE's projective grid mapping receives accurate camera poses.

### Concrete Deliverables
- Modified `core/rammp_prototype_gui/rammp_prototype_gui/Gui_bridge.py` with TF2 integration
- Modified `core/rammp_prototype_gui/package.xml` with `tf2_ros` dependency
- Modified `core/rammp_prototype_gui/launch/Gui_bridge.launch.py` with TF frame name parameters
- Unit test for the ROS→UE coordinate conversion function

### Definition of Done
- [ ] All 3 cameras (wrist, nav1, nav2) get extrinsics from TF2
- [ ] Wrist camera extrinsics change when arm moves
- [ ] Node starts cleanly even when TF tree is not yet available
- [ ] Debug logging shows converted TF2 values for verification

### Must Have
- TF2 Buffer + TransformListener in GuiBridge
- Zero-timeout lookups (SingleThreadedExecutor compatible)
- Fallback to last-known-good extrinsics on TF2 failure
- Configurable TF frame names as ROS parameters
- Consistent style across all 3 cameras
- Debug logging that prints the TF2-derived UE values for validation

### Must NOT Have (Guardrails)
- DO NOT change the executor to `MultiThreadedExecutor` — risk is too high, affects all 30+ callbacks
- DO NOT modify `send_image()`, `send_depth()`, or the `meta["transform"]` dict structure
- DO NOT modify the `Extrinsics` or `Vector` dataclass definitions
- DO NOT touch the rear camera — excluded, no TF frame exists
- DO NOT modify `description.launch.py` — TF frames already defined and working
- DO NOT change existing callback signatures
- DO NOT over-engineer a generic ROS→UE conversion utility module — keep it as a private helper in GuiBridge
- DO NOT use blocking timeouts in `lookup_transform()` — zero timeout only
- DO NOT spam logs on TF failure — throttle warnings to once per 5 seconds

---

## Verification Strategy

> **ZERO HUMAN INTERVENTION** — ALL verification is agent-executed. No exceptions.

### Test Decision
- **Infrastructure exists**: YES (colcon test, only lint tests currently in this package)
- **Automated tests**: YES — Tests-after for the conversion function
- **Framework**: `pytest` via colcon test

### QA Policy
Every task includes agent-executed QA scenarios.
Evidence saved to `.sisyphus/evidence/task-{N}-{scenario-slug}.{ext}`.

- **ROS Node**: Use Bash — `colcon build`, `colcon test`, log inspection
- **Conversion Logic**: Use Bash — pytest with specific assertions

---

## Execution Strategy

### Parallel Execution Waves

```
Wave 1 (Start Immediately — foundation):
├── Task 1: Add tf2_ros dependency + imports + Buffer/Listener init [quick]
├── Task 2: Conversion function + unit test [quick]
└── Task 3: Add TF frame name ROS parameters + launch file params [quick]

Wave 2 (After Wave 1 — wire up cameras):
└── Task 4: Wire all 3 cameras to TF2 + fallback + cleanup [deep]

Wave FINAL (After ALL tasks):
├── Task F1: Plan compliance audit (oracle)
├── Task F2: Code quality review (unspecified-high)
├── Task F3: Real manual QA (unspecified-high)
└── Task F4: Scope fidelity check (deep)
-> Present results -> Get explicit user okay
```

### Dependency Matrix

| Task | Depends On | Blocks | Wave |
|------|-----------|--------|------|
| 1    | —         | 4      | 1    |
| 2    | —         | 4      | 1    |
| 3    | —         | 4      | 1    |
| 4    | 1, 2, 3   | F1-F4  | 2    |

### Agent Dispatch Summary

- **Wave 1**: **3 tasks** — T1 → `quick`, T2 → `quick`, T3 → `quick`
- **Wave 2**: **1 task** — T4 → `deep`
- **FINAL**: **4 tasks** — F1 → `oracle`, F2 → `unspecified-high`, F3 → `unspecified-high`, F4 → `deep`

---

## TODOs

- [ ] 1. Add tf2_ros dependency, imports, and TF2 Buffer/Listener initialization

  **What to do**:
  - Add `<depend>tf2_ros</depend>` and `<depend>tf2_geometry_msgs</depend>` to `core/rammp_prototype_gui/package.xml`
  - Add imports to `Gui_bridge.py`: `import tf2_ros` and `from rclpy.duration import Duration`
  - In `GuiBridge.__init__()`, after existing init, create TF2 infrastructure:
    ```python
    self.tf_buffer = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
    ```
  - Add instance variable for last-known-good extrinsics cache:
    ```python
    self._last_extrinsics: dict[str, Extrinsics] = {}
    ```

  **Must NOT do**:
  - DO NOT change the executor model
  - DO NOT modify any existing init code — only append new TF2 initialization after existing setup

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Small, well-defined changes — add dependency, add imports, add 3 lines of init
  - **Skills**: []
  - **Skills Evaluated but Omitted**:
    - None relevant for a dependency/init task

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 2, 3)
  - **Blocks**: Task 4
  - **Blocked By**: None

  **References**:

  **Pattern References**:
  - `core/rammp_prototype_gui/rammp_prototype_gui/Gui_bridge.py:130-226` — `GuiBridge.__init__()` where TF2 init should be added (after line 226, after `self.init_service()`)
  - `demo_modules/cmu_door_opener/cmu_door_opener/button_detector.py` — Has existing TF2 Buffer+Listener pattern in this codebase (search for `tf2_ros.Buffer`)

  **API/Type References**:
  - `core/rammp_prototype_gui/package.xml` — Add `<depend>` entries alongside existing dependencies

  **WHY Each Reference Matters**:
  - `Gui_bridge.py:130-226` — Shows exactly where in `__init__` to place the TF2 init (after all subscribers/publishers are set up)
  - `button_detector.py` — Shows the codebase's existing convention for TF2 initialization

  **Acceptance Criteria**:

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: Build succeeds with new dependency
    Tool: Bash
    Preconditions: ROS 2 workspace sourced
    Steps:
      1. Run `colcon build --packages-select rammp_prototype_gui`
      2. Check exit code is 0
    Expected Result: Build completes with 0 errors
    Failure Indicators: Missing dependency error for tf2_ros
    Evidence: .sisyphus/evidence/task-1-build-success.txt

  Scenario: Node starts without crash
    Tool: Bash
    Preconditions: ROS 2 workspace sourced, `colcon build` passed
    Steps:
      1. Run `timeout 5 ros2 run rammp_prototype_gui GuiBridge 2>&1 || true`
      2. Check output does NOT contain "ImportError" or "ModuleNotFoundError"
      3. Check output contains "Gui_bridge node has been started"
    Expected Result: Node initializes (may warn about UE connection, that's fine)
    Failure Indicators: ImportError for tf2_ros, crash on init
    Evidence: .sisyphus/evidence/task-1-node-start.txt
  ```

  **Commit**: YES
  - Message: `feat(gui): add tf2_ros dependency and TF2 buffer initialization`
  - Files: `core/rammp_prototype_gui/package.xml`, `core/rammp_prototype_gui/rammp_prototype_gui/Gui_bridge.py`
  - Pre-commit: `colcon build --packages-select rammp_prototype_gui`

- [ ] 2. Create pure conversion function `_tf_to_ue_extrinsics()` with unit test

  **What to do**:
  - Add a private method or module-level function `_tf_to_ue_extrinsics(transform_stamped) -> Extrinsics` in `Gui_bridge.py` that:
    1. Extracts translation (x, y, z) from `geometry_msgs.msg.TransformStamped`
    2. Converts meters → centimeters (multiply by 100)
    3. Extracts rotation quaternion, converts to Euler angles (roll, pitch, yaw) in degrees using `scipy.spatial.transform.Rotation` (already imported as `R`)
    4. The Euler decomposition should use `R.from_quat([qx, qy, qz, qw]).as_euler('xyz', degrees=True)` — this returns `[roll, pitch, yaw]`
    5. Returns an `Extrinsics` dataclass with `location=Vector(x_cm, y_cm, z_cm)`, `rotation=Vector(roll_deg, pitch_deg, yaw_deg)`, `scale=Vector(1,1,1)`
  - **CRITICAL**: Add debug logging that prints the input quaternion + output Euler angles + output position. This is essential for the user to verify the coordinate mapping is correct when they run it on the real robot.
  - Write a unit test in `core/rammp_prototype_gui/test/test_tf_conversion.py`:
    - Test identity transform → Extrinsics with all zeros
    - Test known translation (1.0, 2.0, 3.0)m → (100.0, 200.0, 300.0)cm
    - Test known quaternion (e.g. 90° pitch) → correct Euler output
    - Test that `scale` is always `(1, 1, 1)`

  **IMPORTANT NOTE on coordinate conventions**:
  The exact ROS→UE coordinate mapping (whether Y needs negation, whether pitch needs sign flip) is uncertain. The function should start with a **direct** conversion (no axis flips). The debug logging will allow the user to compare TF2-derived values against what UE expects, and adjust if needed. Include a clearly-marked comment block where axis adjustments can be added:
  ```python
  # ROS→UE coordinate adjustment (verify empirically):
  # If UE Y-axis is flipped from ROS, negate y_cm here.
  # If UE pitch convention differs, negate pitch_deg here.
  ```

  **Must NOT do**:
  - DO NOT guess at coordinate flips — start with direct conversion, let debug logs guide
  - DO NOT modify the Extrinsics or Vector dataclass definitions
  - DO NOT import anything not already available (scipy.spatial.transform.Rotation is already imported as `R`)

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: One pure function + one test file, well-defined math
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 3)
  - **Blocks**: Task 4
  - **Blocked By**: None

  **References**:

  **Pattern References**:
  - `core/rammp_prototype_gui/rammp_prototype_gui/Gui_bridge.py:27-53` — `Vector`, `Quaternion`, `Extrinsics` dataclass definitions (the output type)
  - `core/rammp_prototype_gui/rammp_prototype_gui/Gui_bridge.py:24` — `from scipy.spatial.transform import Rotation as R` (already imported, use this)
  - `core/rammp_prototype_gui/rammp_prototype_gui/Gui_bridge.py:1002-1011` — Example of using `R.from_euler(...).as_quat()` in `update_button_info()` — the REVERSE direction of what we need

  **API/Type References**:
  - `geometry_msgs.msg.TransformStamped` — The input type from `tf2_ros.Buffer.lookup_transform()`
  - `TransformStamped.transform.translation.{x, y, z}` — position in meters
  - `TransformStamped.transform.rotation.{x, y, z, w}` — quaternion

  **External References**:
  - scipy `Rotation.as_euler('xyz', degrees=True)` — returns `[roll, pitch, yaw]` in degrees

  **WHY Each Reference Matters**:
  - Lines 27-53 define the exact output structure the function must produce
  - Line 24 shows scipy Rotation is already available — no new imports needed
  - Lines 1002-1011 show the codebase already does quat↔euler conversion (reverse direction)

  **Acceptance Criteria**:
  - [ ] Unit test file exists and passes: `test/test_tf_conversion.py`
  - [ ] `colcon test --packages-select rammp_prototype_gui` passes

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: Unit tests pass for conversion function
    Tool: Bash
    Preconditions: colcon build passed
    Steps:
      1. Run `colcon test --packages-select rammp_prototype_gui --pytest-args test/test_tf_conversion.py`
      2. Check all tests pass
    Expected Result: All conversion tests pass (identity, translation, rotation, scale)
    Failure Indicators: Any assertion failure
    Evidence: .sisyphus/evidence/task-2-test-results.txt

  Scenario: Identity transform produces zero extrinsics
    Tool: Bash
    Preconditions: Test file exists
    Steps:
      1. Run the identity test case specifically
      2. Verify output: location=(0,0,0), rotation=(0,0,0), scale=(1,1,1)
    Expected Result: All values are 0.0 for identity transform
    Failure Indicators: Non-zero values for identity input
    Evidence: .sisyphus/evidence/task-2-identity-test.txt
  ```

  **Commit**: YES
  - Message: `feat(gui): add ROS→UE extrinsics conversion with unit test`
  - Files: `Gui_bridge.py`, `test/test_tf_conversion.py`
  - Pre-commit: `colcon test --packages-select rammp_prototype_gui`

- [ ] 3. Add TF frame name ROS parameters and launch file updates

  **What to do**:
  - In `Gui_bridge.py` `__init__()`, declare 3 new ROS parameters for camera TF frame names:
    ```python
    self.declare_parameter("wrist_camera_tf_frame", "wrist_wrist_camera_link")
    self.declare_parameter("nav_camera_1_tf_frame", "nav1_link")
    self.declare_parameter("nav_camera_2_tf_frame", "nav2_link")
    ```
  - Also declare the base frame parameter:
    ```python
    self.declare_parameter("tf_base_frame", "mebot")
    ```
  - Read these parameters into instance variables (following the existing pattern for other params like `wrist_camera_namespace`)
  - In `Gui_bridge.launch.py`, add corresponding `DeclareLaunchArgument` entries and pass them to the node's `parameters` dict (follow the existing pattern for `wrist_camera_namespace` etc.)

  **Must NOT do**:
  - DO NOT change existing parameter names or defaults
  - DO NOT modify `description.launch.py`

  **Recommended Agent Profile**:
  - **Category**: `quick`
    - Reason: Boilerplate parameter declaration following an existing pattern
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: YES
  - **Parallel Group**: Wave 1 (with Tasks 1, 2)
  - **Blocks**: Task 4
  - **Blocked By**: None

  **References**:

  **Pattern References**:
  - `core/rammp_prototype_gui/rammp_prototype_gui/Gui_bridge.py:140-197` — Existing parameter declaration pattern (`declare_parameter` + `get_parameter`) — follow this exactly
  - `core/rammp_prototype_gui/launch/Gui_bridge.launch.py:10-91` — Existing launch argument + parameter forwarding pattern — follow this exactly

  **API/Type References**:
  - `core/rammp_prototype_description/launch/description.launch.py:32-36` — The `camera_frame` launch arg with default `wrist_wrist_camera_link` (this is the frame name to match)
  - `core/rammp_prototype_description/launch/description.launch.py:81` — `nav1_link` frame name
  - `core/rammp_prototype_description/launch/description.launch.py:104` — `nav2_link` frame name

  **WHY Each Reference Matters**:
  - Lines 140-197 show the exact code style for parameter declaration — executor must match this pattern
  - Launch file lines 10-91 show the existing argument→parameter forwarding pattern
  - description.launch.py confirms the exact TF frame name strings to use as defaults

  **Acceptance Criteria**:
  - [ ] `colcon build --packages-select rammp_prototype_gui` succeeds

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: Build succeeds with new parameters
    Tool: Bash
    Preconditions: ROS 2 workspace sourced
    Steps:
      1. Run `colcon build --packages-select rammp_prototype_gui`
      2. Check exit code is 0
    Expected Result: Build succeeds
    Failure Indicators: Syntax errors in parameter declarations
    Evidence: .sisyphus/evidence/task-3-build-success.txt

  Scenario: Parameters appear in node description
    Tool: Bash
    Preconditions: Build passed
    Steps:
      1. Run `timeout 5 ros2 run rammp_prototype_gui GuiBridge --ros-args -p wrist_camera_tf_frame:=test_frame 2>&1 || true`
      2. Check output does NOT contain "parameter not declared" errors
    Expected Result: Node accepts the parameter without error
    Failure Indicators: "has not been declared" error
    Evidence: .sisyphus/evidence/task-3-param-check.txt
  ```

  **Commit**: YES
  - Message: `feat(gui): add TF frame name parameters to launch and node`
  - Files: `Gui_bridge.py`, `Gui_bridge.launch.py`
  - Pre-commit: `colcon build --packages-select rammp_prototype_gui`

- [ ] 4. Wire all 3 cameras to TF2 extrinsics with fallback and cleanup

  **What to do**:
  This is the main integration task. Wire the conversion function (Task 2) and TF2 infrastructure (Task 1) into the camera callbacks using the frame name parameters (Task 3).

  **Step A — Add a private lookup helper method**:
  Create `_lookup_camera_extrinsics(self, camera_tf_frame: str) -> Extrinsics` that:
  1. Calls `self.tf_buffer.lookup_transform(self.tf_base_frame, camera_tf_frame, rclpy.time.Time(), timeout=Duration(seconds=0.0))`
  2. Passes the result to `_tf_to_ue_extrinsics()` (from Task 2)
  3. On success: caches the result in `self._last_extrinsics[camera_tf_frame]` and returns it
  4. On `tf2_ros.LookupException`, `ConnectivityException`, or `ExtrapolationException`:
     - Returns cached value from `self._last_extrinsics[camera_tf_frame]` if available
     - Otherwise returns the zero-default `Extrinsics(Vector(0,0,0), Vector(0,0,0), Vector(1,1,1))`
     - Logs warning at **throttled rate** (not every frame) using `self.get_logger().warn(..., throttle_duration_sec=5.0)`
  5. Includes debug logging (throttled to ~1Hz) that prints the camera name + derived UE values:
     `self.get_logger().debug(f"TF2 extrinsics [{camera_tf_frame}]: pos=({x:.1f}, {y:.1f}, {z:.1f})cm rot=({roll:.1f}, {pitch:.1f}, {yaw:.1f})deg", throttle_duration_sec=1.0)`

  **Step B — Wire wrist camera**:
  - In `wrist_camera_image_callback` and `wrist_camera_depth_callback`, BEFORE calling `send_wrist_camera_image()`/`send_wrist_camera_depth()`, update:
    ```python
    self.wrist_camera_extrinsics = self._lookup_camera_extrinsics(self.wrist_camera_tf_frame)
    ```
  - Remove the hardcoded `(0,0,0)` initialization for `self.wrist_camera_extrinsics` at line 313-315 — replace with the zero-default (same value, but now clearly a "no TF yet" default)

  **Step C — Wire nav camera 1**:
  - Same pattern: in `nav_camera_1_image_callback` and `nav_camera_1_depth_callback`, update extrinsics before sending
  - Remove the hardcoded `Vector(3.634, -26.4223, 38.361)` / `Vector(0.0, -20.0, 0)` initialization at lines 320-324

  **Step D — Wire nav camera 2**:
  - Same pattern as nav1
  - Remove hardcoded initialization at lines 329-333

  **Step E — Cleanup**:
  - Remove ALL commented-out extrinsics subscriber code blocks (lines 370-376 wrist, 407-413 nav1, 444-450 nav2, 480-486 rear)
  - Remove the unused callback methods: `wrist_camera_extrinsics_callback`, `nav_camera_1_extrinsics_callback`, `nav_camera_2_extrinsics_callback`, `rear_camera_extrinsics_callback` (lines 562-614 area)

  **IMPORTANT NOTES**:
  - All 3 cameras MUST use the identical `_lookup_camera_extrinsics()` helper — no per-camera special logic
  - The `send_wrist_camera_image()` / `send_nav_camera_1_image()` etc. methods do NOT need to change — they already read `self.*_extrinsics` and pass to `send_image()`. We only change HOW the extrinsics are populated.
  - Nav1's `image_rotated` vs nav2's `image_raw` difference is handled by the TF tree itself (the static transforms in description.launch.py already account for physical mount angles). The UE projective grid mapping receives the TF-derived transform and handles projection accordingly. Do NOT try to manually subtract rotation from nav1.

  **Must NOT do**:
  - DO NOT add per-camera conversion logic — all cameras go through the same function
  - DO NOT modify `send_image()`, `send_depth()`, or the metadata format
  - DO NOT touch rear camera extrinsics or callbacks
  - DO NOT use blocking timeout in lookup_transform
  - DO NOT change the executor

  **Recommended Agent Profile**:
  - **Category**: `deep`
    - Reason: Integration task touching multiple sections of a large file, needs careful wiring without breaking existing functionality
  - **Skills**: []

  **Parallelization**:
  - **Can Run In Parallel**: NO — Sequential
  - **Parallel Group**: Wave 2 (after Tasks 1, 2, 3)
  - **Blocks**: F1-F4
  - **Blocked By**: Tasks 1, 2, 3

  **References**:

  **Pattern References**:
  - `core/rammp_prototype_gui/rammp_prototype_gui/Gui_bridge.py:554-560` — `wrist_camera_image_callback` / `wrist_camera_depth_callback` — where to add extrinsics update
  - `core/rammp_prototype_gui/rammp_prototype_gui/Gui_bridge.py:571-577` — `nav_camera_1_image_callback` / `nav_camera_1_depth_callback`
  - `core/rammp_prototype_gui/rammp_prototype_gui/Gui_bridge.py:588-594` — `nav_camera_2_image_callback` / `nav_camera_2_depth_callback`
  - `core/rammp_prototype_gui/rammp_prototype_gui/Gui_bridge.py:616-666` — `send_image()` method showing how extrinsics are consumed (lines 648-657) — this must NOT change
  - `core/rammp_prototype_gui/rammp_prototype_gui/Gui_bridge.py:313-333` — Hardcoded extrinsics to REMOVE
  - `core/rammp_prototype_gui/rammp_prototype_gui/Gui_bridge.py:370-376, 407-413, 444-450, 480-486` — Commented-out subscribers to REMOVE
  - `core/rammp_prototype_gui/rammp_prototype_gui/Gui_bridge.py:562-614` — Unused extrinsics callbacks to REMOVE

  **API/Type References**:
  - `tf2_ros.Buffer.lookup_transform(target_frame, source_frame, time, timeout)` — Returns `geometry_msgs.msg.TransformStamped`
  - `rclpy.time.Time()` — Latest available transform (no specific timestamp)
  - `rclpy.duration.Duration(seconds=0.0)` — Zero timeout for non-blocking lookup

  **External References**:
  - tf2_ros Python API: `lookup_transform` raises `LookupException`, `ConnectivityException`, `ExtrapolationException`

  **WHY Each Reference Matters**:
  - Lines 554-594 are the exact callback functions where 1 line of extrinsics update must be added
  - Lines 616-666 show the downstream consumer — must NOT be modified, only verify it still works
  - Lines 313-333 are the hardcoded values being replaced
  - Lines 370-486 are dead code to clean up

  **Acceptance Criteria**:
  - [ ] `colcon build --packages-select rammp_prototype_gui` succeeds
  - [ ] `colcon test --packages-select rammp_prototype_gui` passes
  - [ ] No hardcoded extrinsics remain for wrist, nav1, nav2
  - [ ] All commented-out extrinsics subscribers removed
  - [ ] Unused extrinsics callback methods removed

  **QA Scenarios (MANDATORY):**

  ```
  Scenario: Build succeeds after wiring
    Tool: Bash
    Preconditions: Tasks 1-3 completed
    Steps:
      1. Run `colcon build --packages-select rammp_prototype_gui`
      2. Check exit code is 0
    Expected Result: Build succeeds
    Failure Indicators: Import errors, syntax errors, type mismatches
    Evidence: .sisyphus/evidence/task-4-build-success.txt

  Scenario: All tests still pass
    Tool: Bash
    Preconditions: Build passed
    Steps:
      1. Run `colcon test --packages-select rammp_prototype_gui`
      2. Check all tests pass (including Task 2's conversion tests)
    Expected Result: 0 failures
    Failure Indicators: Any test regression
    Evidence: .sisyphus/evidence/task-4-test-results.txt

  Scenario: No hardcoded extrinsics remain
    Tool: Bash (grep)
    Preconditions: Code changes applied
    Steps:
      1. Search Gui_bridge.py for `Vector(3.634` (old nav1 hardcoded X)
      2. Search for `Vector(0, 0, 0), rotation=Vector(0, 0, 0)` outside of fallback/default context
      3. Search for `Vector(3.634, 26.4223` (old nav2 hardcoded)
    Expected Result: No hardcoded extrinsics values found (except in fallback defaults)
    Failure Indicators: Old hardcoded values still present
    Evidence: .sisyphus/evidence/task-4-no-hardcoded.txt

  Scenario: Commented-out subscriber code removed
    Tool: Bash (grep)
    Preconditions: Code changes applied
    Steps:
      1. Search Gui_bridge.py for `extrinsics/depth_to_color` (old subscriber topic)
      2. Search for `wrist_camera_extrinsics_callback` definition
    Expected Result: No matches — all dead code removed
    Failure Indicators: Commented-out code still present
    Evidence: .sisyphus/evidence/task-4-cleanup.txt

  Scenario: Node starts without TF tree (fallback test)
    Tool: Bash
    Preconditions: Build passed, NO description.launch.py running
    Steps:
      1. Run `timeout 5 ros2 run rammp_prototype_gui GuiBridge 2>&1 || true`
      2. Check output for TF2 warning messages (throttled, not spamming)
      3. Verify node does NOT crash
    Expected Result: Node starts, logs throttled TF2 warnings, doesn't crash
    Failure Indicators: Node crash, unhandled exception, log spam (>1 warning/sec)
    Evidence: .sisyphus/evidence/task-4-fallback.txt
  ```

  **Commit**: YES
  - Message: `feat(gui): wire all cameras to TF2 extrinsics with fallback`
  - Files: `core/rammp_prototype_gui/rammp_prototype_gui/Gui_bridge.py`
  - Pre-commit: `colcon build && colcon test --packages-select rammp_prototype_gui`

---

## Final Verification Wave (MANDATORY — after ALL implementation tasks)

> 4 review agents run in PARALLEL. ALL must APPROVE. Present consolidated results to user and get explicit "okay" before completing.
>
> **Do NOT auto-proceed after verification. Wait for user's explicit approval before marking work complete.**

- [ ] F1. **Plan Compliance Audit** — `oracle`
  Read the plan end-to-end. For each "Must Have": verify implementation exists (read file, check code). For each "Must NOT Have": search codebase for forbidden patterns — reject with file:line if found. Check evidence files exist in .sisyphus/evidence/. Compare deliverables against plan.
  Output: `Must Have [N/N] | Must NOT Have [N/N] | Tasks [N/N] | VERDICT: APPROVE/REJECT`

- [ ] F2. **Code Quality Review** — `unspecified-high`
  Run `colcon build --packages-select rammp_prototype_gui` + `colcon test --packages-select rammp_prototype_gui`. Review all changed files for: empty catches, unused imports, inconsistent style between the 3 camera lookups. Check AI slop: excessive comments, over-abstraction.
  Output: `Build [PASS/FAIL] | Test [PASS/FAIL] | Files [N clean/N issues] | VERDICT`

- [ ] F3. **Real Manual QA** — `unspecified-high`
  Execute EVERY QA scenario from EVERY task — follow exact steps, capture evidence. Test cross-task integration. Save to `.sisyphus/evidence/final-qa/`.
  Output: `Scenarios [N/N pass] | Integration [N/N] | VERDICT`

- [ ] F4. **Scope Fidelity Check** — `deep`
  For each task: read "What to do", read actual diff (git log/diff). Verify 1:1 — everything in spec was built (no missing), nothing beyond spec was built (no creep). Check "Must NOT do" compliance. Detect cross-task contamination. Flag unaccounted changes.
  Output: `Tasks [N/N compliant] | Contamination [CLEAN/N issues] | VERDICT`

---

## Commit Strategy

| # | Message | Files | Pre-commit |
|---|---------|-------|------------|
| 1 | `feat(gui): add tf2_ros dependency and TF2 buffer initialization` | `package.xml`, `Gui_bridge.py` | `colcon build --packages-select rammp_prototype_gui` |
| 2 | `feat(gui): add ROS→UE extrinsics conversion with unit test` | `Gui_bridge.py`, test file | `colcon test --packages-select rammp_prototype_gui` |
| 3 | `feat(gui): add TF frame name parameters to launch and node` | `Gui_bridge.py`, `Gui_bridge.launch.py` | `colcon build --packages-select rammp_prototype_gui` |
| 4 | `feat(gui): wire all cameras to TF2 extrinsics with fallback` | `Gui_bridge.py` | `colcon build && colcon test --packages-select rammp_prototype_gui` |

---

## Success Criteria

### Verification Commands
```bash
colcon build --packages-select rammp_prototype_gui  # Expected: SUCCESS
colcon test --packages-select rammp_prototype_gui   # Expected: all tests pass
```

### Final Checklist
- [ ] All 3 cameras use TF2 (no hardcoded extrinsics remain for wrist/nav1/nav2)
- [ ] Wrist extrinsics update dynamically with arm pose
- [ ] Node survives TF tree being unavailable at startup
- [ ] Rear camera completely untouched
- [ ] No MultiThreadedExecutor changes
- [ ] No blocking TF2 lookups
- [ ] Debug logging present for value verification
- [ ] `package.xml` includes `tf2_ros` dependency
- [ ] TF frame names configurable as ROS parameters
