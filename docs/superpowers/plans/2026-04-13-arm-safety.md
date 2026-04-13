# Arm Safety Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add three safety layers to `arm_driver`: fix the stubbed e-stop, detect arm communication loss, kill stale twist commands, and detect collisions via Pinocchio RNEA — all inside the existing `ArmDriverNode`.

**Architecture:** `CollisionChecker` is a plain class instantiated inside `ArmDriverNode` and called on every feedback cycle. Comms health and twist timeout are tracked with wall-clock timestamps directly in the driver node. No new ROS nodes are introduced.

**Tech Stack:** Python, ROS 2 (rclpy), Pinocchio (`pin`), NumPy, pytest + unittest.mock

______________________________________________________________________

## File Map

| File                                                          | Action        | Responsibility                                                    |
| ------------------------------------------------------------- | ------------- | ----------------------------------------------------------------- |
| `hardware/arm_driver/arm_driver/arm_driver.py`                | Modify        | E-stop fix, comms health, twist watchdog, CollisionChecker wiring |
| `hardware/arm_driver/arm_driver/collision_checker.py`         | Create        | Pinocchio RNEA, per-state threshold logic                         |
| `hardware/arm_driver/arm_driver/urdf/gen3_robotiq_2f_85.urdf` | Create (copy) | Gen3 dynamics model for Pinocchio                                 |
| `hardware/arm_driver/setup.py`                                | Modify        | Install URDF into ament share                                     |
| `hardware/arm_driver/test/test_arm_driver_safety.py`          | Create        | Unit tests for e-stop, comms health, twist watchdog               |
| `hardware/arm_driver/test/test_collision_checker.py`          | Create        | Unit tests for CollisionChecker                                   |

______________________________________________________________________

## Background: Pinocchio q Encoding for the Gen3

The Gen3 URDF has alternating joint types:

- `joint_1, 3, 5, 7` → `continuous` (unbounded revolute) → Pinocchio encodes as `[cos(θ), sin(θ)]`
- `joint_2, 4, 6` → `revolute` (bounded) → Pinocchio encodes as `[θ]`

So for joint angles `q[0..6]`, the Pinocchio configuration vector `q_pin` has 11 elements:

```
q_pin = [cos(q[0]), sin(q[0]),  q[1],
         cos(q[2]), sin(q[2]),  q[3],
         cos(q[4]), sin(q[4]),  q[5],
         cos(q[6]), sin(q[6])]
```

This is used in `CollisionChecker.check()` and is the only non-obvious part of the implementation.

______________________________________________________________________

## Task 1: Fix the E-Stop Stub

**Files:**

- Modify: `hardware/arm_driver/arm_driver/arm_driver.py:354-366`

- Create: `hardware/arm_driver/test/test_arm_driver_safety.py`

- [ ] **Step 1: Write the failing test**

```python
# hardware/arm_driver/test/test_arm_driver_safety.py
import rclpy
import pytest
from unittest.mock import MagicMock, patch
from std_msgs.msg import Bool

# We patch KinovaArm before importing arm_driver so __init__ doesn't try to
# ping the real hardware.
@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node():
    """Create ArmDriverNode with KinovaArm fully mocked."""
    with patch("arm_driver.arm_driver.KinovaArm") as MockArm:
        mock_arm = MagicMock()
        MockArm.return_value = mock_arm
        from arm_driver.arm_driver import ArmDriverNode, ArmState
        node = ArmDriverNode()
        return node, mock_arm, ArmState


def test_estop_calls_arm_stop():
    node, mock_arm, ArmState = _make_node()
    try:
        msg = Bool()
        msg.data = True
        node._on_estop(msg)
        mock_arm.stop.assert_called_once()
        assert node._state == ArmState.ERROR
    finally:
        node.destroy_node()


def test_estop_false_does_nothing():
    node, mock_arm, ArmState = _make_node()
    try:
        msg = Bool()
        msg.data = False
        node._on_estop(msg)
        mock_arm.stop.assert_not_called()
        assert node._state == ArmState.IDLE
    finally:
        node.destroy_node()
```

- [ ] **Step 2: Run to confirm failure**

```bash
cd /path/to/Demo-Software
pytest hardware/arm_driver/test/test_arm_driver_safety.py::test_estop_calls_arm_stop -v
```

Expected: `FAILED` — `stop` was not called (it's currently stubbed out).

- [ ] **Step 3: Fix the stub in `_on_estop`**

In `hardware/arm_driver/arm_driver/arm_driver.py`, replace:

```python
    def _on_estop(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("E-stop received.")
            # STUB: self._arm.stop()
            self._error_reason = "E-stop triggered"
            self._transition_to(ArmState.ERROR)
```

With:

```python
    def _on_estop(self, msg: Bool):
        if msg.data:
            self.get_logger().warn("E-stop received.")
            if self._arm:
                try:
                    self._arm.stop()
                except Exception as e:
                    self.get_logger().warn(f"stop() failed during e-stop ({e!r})")
            self._error_reason = "E-stop triggered"
            self._transition_to(ArmState.ERROR)
```

- [ ] **Step 4: Run tests to confirm pass**

```bash
pytest hardware/arm_driver/test/test_arm_driver_safety.py -v
```

Expected: both tests PASS.

- [ ] **Step 5: Commit**

```bash
git add hardware/arm_driver/arm_driver/arm_driver.py hardware/arm_driver/test/test_arm_driver_safety.py
git commit -m "fix: actually call arm.stop() when e-stop is triggered"
```

______________________________________________________________________

## Task 2: Arm Communication Health

Tracks wall-clock time of the last successful `get_state()`. If the arm goes silent for more than `COMMS_TIMEOUT_S` (0.5 s), the node transitions to `ERROR`. At 100 Hz feedback this catches ~50 consecutive dropped cycles.

**Files:**

- Modify: `hardware/arm_driver/arm_driver/arm_driver.py`

- Modify: `hardware/arm_driver/test/test_arm_driver_safety.py`

- [ ] **Step 1: Write the failing test**

Add to `test_arm_driver_safety.py`:

```python
import time
from unittest.mock import MagicMock, patch, PropertyMock


def test_comms_timeout_triggers_error():
    node, mock_arm, ArmState = _make_node()
    try:
        # Simulate arm going silent: get_state raises TimeoutError every call
        mock_arm.get_state.side_effect = TimeoutError("no response")

        # Rewind _last_feedback_time far enough to exceed the threshold
        from arm_driver.arm_driver import COMMS_TIMEOUT_S
        node._last_feedback_time = time.monotonic() - COMMS_TIMEOUT_S - 0.1

        # Trigger one feedback cycle
        node._publish_joint_states()

        assert node._state == ArmState.ERROR
        assert "communication" in node._error_reason.lower()
    finally:
        node.destroy_node()


def test_comms_timeout_resets_on_success():
    node, mock_arm, ArmState = _make_node()
    try:
        from arm_driver.arm_driver import COMMS_TIMEOUT_S
        # Prime a near-expired timestamp
        node._last_feedback_time = time.monotonic() - COMMS_TIMEOUT_S + 0.2

        # get_state succeeds — should reset the clock
        mock_arm.get_state.return_value = {
            "position": __import__("numpy").zeros(7),
            "velocity": __import__("numpy").zeros(7),
            "effort": __import__("numpy").zeros(7),
            "ee_pos": __import__("numpy").zeros(7),
            "ee_vel": __import__("numpy").zeros(6),
            "ee_force": __import__("numpy").zeros(3),
            "gripper_pos": 0.0,
        }
        node._publish_joint_states()

        # Timestamp should now be fresh — no ERROR
        assert node._state == ArmState.IDLE
    finally:
        node.destroy_node()
```

- [ ] **Step 2: Run to confirm failure**

```bash
pytest hardware/arm_driver/test/test_arm_driver_safety.py::test_comms_timeout_triggers_error -v
```

Expected: `AttributeError` — `_last_feedback_time` not yet defined.

- [ ] **Step 3: Add comms-health tracking to `ArmDriverNode`**

Add the constant near the top of `arm_driver.py` (after the existing `FEEDBACK_RATE` line):

```python
COMMS_TIMEOUT_S = 0.5  # seconds without arm feedback before ERROR
```

In `ArmDriverNode.__init__`, add after `self._error_reason: str = ""`:

```python
        self._last_feedback_time: float = time.monotonic()
```

Replace the `_publish_joint_states` method's `if self._arm:` block. Currently it has:

```python
        if self._arm:
            try:
                state = self._arm.get_state()
            except (TimeoutError, concurrent.futures.TimeoutError):
                self.get_logger().warn(
                    "RefreshFeedback timed out — skipping publish cycle",
                    throttle_duration_sec=1.0,
                )
                return
```

Replace with:

```python
        if self._arm:
            try:
                state = self._arm.get_state()
                self._last_feedback_time = time.monotonic()
            except (TimeoutError, concurrent.futures.TimeoutError):
                self.get_logger().warn(
                    "RefreshFeedback timed out — skipping publish cycle",
                    throttle_duration_sec=1.0,
                )
                if (
                    self._state != ArmState.ERROR
                    and time.monotonic() - self._last_feedback_time > COMMS_TIMEOUT_S
                ):
                    self.get_logger().error(
                        "Arm communication lost — no feedback for "
                        f"{COMMS_TIMEOUT_S}s"
                    )
                    self._error_reason = "Arm communication timeout"
                    self._transition_to(ArmState.ERROR)
                return
```

Also add `import time` at the top of `arm_driver.py` if not already present (it is — `arm_interface.py` has it, but check `arm_driver.py` imports).

- [ ] **Step 4: Run tests**

```bash
pytest hardware/arm_driver/test/test_arm_driver_safety.py -v
```

Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add hardware/arm_driver/arm_driver/arm_driver.py hardware/arm_driver/test/test_arm_driver_safety.py
git commit -m "feat: transition to ERROR on arm communication timeout"
```

______________________________________________________________________

## Task 3: Stale Twist Command Watchdog

In twist states (`MANUAL`, `CUP_STABILIZE`), the Kortex arm runs the last `SendTwistCommand` indefinitely (`duration=0`). If the publisher dies, the arm keeps moving. This watchdog calls `arm.stop()` when no twist has arrived within `TWIST_TIMEOUT_S`.

Note: this intentionally does **not** transition to `ERROR` — a momentary publisher hiccup should be recoverable. The arm just stops until the next command arrives.

**Files:**

- Modify: `hardware/arm_driver/arm_driver/arm_driver.py`

- Modify: `hardware/arm_driver/test/test_arm_driver_safety.py`

- [ ] **Step 1: Write the failing test**

Add to `test_arm_driver_safety.py`:

```python
def test_twist_watchdog_stops_arm_when_stale():
    node, mock_arm, ArmState = _make_node()
    try:
        from arm_driver.arm_driver import TWIST_TIMEOUT_S, ArmState as AS
        node._state = AS.MANUAL

        # Simulate a twist command received in the past, now stale
        node._last_twist_time = time.monotonic() - TWIST_TIMEOUT_S - 0.1

        node._check_twist_timeout()

        mock_arm.stop.assert_called_once()
        # Should NOT go to ERROR — just stop
        assert node._state == AS.MANUAL
    finally:
        node.destroy_node()


def test_twist_watchdog_does_not_stop_when_fresh():
    node, mock_arm, ArmState = _make_node()
    try:
        from arm_driver.arm_driver import TWIST_TIMEOUT_S, ArmState as AS
        node._state = AS.MANUAL

        # Recent twist command
        node._last_twist_time = time.monotonic() - 0.1

        node._check_twist_timeout()

        mock_arm.stop.assert_not_called()
    finally:
        node.destroy_node()


def test_twist_watchdog_ignores_non_twist_states():
    node, mock_arm, ArmState = _make_node()
    try:
        from arm_driver.arm_driver import TWIST_TIMEOUT_S, ArmState as AS
        node._state = AS.IDLE

        node._last_twist_time = time.monotonic() - TWIST_TIMEOUT_S - 1.0

        node._check_twist_timeout()

        mock_arm.stop.assert_not_called()
    finally:
        node.destroy_node()
```

- [ ] **Step 2: Run to confirm failure**

```bash
pytest hardware/arm_driver/test/test_arm_driver_safety.py::test_twist_watchdog_stops_arm_when_stale -v
```

Expected: `AttributeError` — `TWIST_TIMEOUT_S` / `_last_twist_time` not defined.

- [ ] **Step 3: Implement the twist watchdog**

Add constant after `COMMS_TIMEOUT_S`:

```python
TWIST_TIMEOUT_S = 0.5  # seconds since last twist before stopping the arm
```

The set of states that accept twist commands (derive from existing `COMMAND_MODE` dict — do not hardcode):

```python
_TWIST_STATES = frozenset(
    state for state, mode in COMMAND_MODE.items() if mode == CommandMode.TWIST
)
```

In `ArmDriverNode.__init__`, add after `self._last_feedback_time`:

```python
        self._last_twist_time: float | None = None
```

In `_handle_twist`, add at the **top** of the method (before calling `self._arm.send_twist`):

```python
        self._last_twist_time = time.monotonic()
```

In `_init_timers`, add a 10 Hz watchdog timer:

```python
        self.create_timer(0.1, self._check_twist_timeout)   # 10 Hz twist watchdog
```

Add the new method to `ArmDriverNode`:

```python
    def _check_twist_timeout(self):
        """Stop the arm if no twist command has arrived recently in a twist state.

        Prevents runaway motion when a twist publisher dies mid-stream.
        Does not transition to ERROR — the publisher may recover.
        """
        if self._state not in _TWIST_STATES:
            return
        if self._last_twist_time is None:
            return
        if time.monotonic() - self._last_twist_time > TWIST_TIMEOUT_S:
            if self._arm:
                try:
                    self._arm.stop()
                except Exception as e:
                    self.get_logger().warn(
                        f"stop() failed in twist watchdog ({e!r})",
                        throttle_duration_sec=1.0,
                    )
            self._last_twist_time = None  # reset so we only stop once per stale window
```

- [ ] **Step 4: Run tests**

```bash
pytest hardware/arm_driver/test/test_arm_driver_safety.py -v
```

Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add hardware/arm_driver/arm_driver/arm_driver.py hardware/arm_driver/test/test_arm_driver_safety.py
git commit -m "feat: stop arm on stale twist command to prevent runaway motion"
```

______________________________________________________________________

## Task 4: CollisionChecker Class

A plain Python class — no ROS dependency — that wraps Pinocchio RNEA. Takes the URDF path and a per-state threshold dict at construction. Returns `True` if a collision is detected.

**Files:**

- Create: `hardware/arm_driver/arm_driver/collision_checker.py`

- Create: `hardware/arm_driver/test/test_collision_checker.py`

- [ ] **Step 1: Write the failing tests**

```python
# hardware/arm_driver/test/test_collision_checker.py
import math
import numpy as np
import pytest
from unittest.mock import patch, MagicMock


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_checker(thresholds=None):
    """Build a CollisionChecker with Pinocchio fully mocked."""
    if thresholds is None:
        thresholds = {"DEFAULT": 100.0, "OPEN_DOOR": 500.0}

    with patch("arm_driver.collision_checker.pin") as mock_pin:
        mock_model = MagicMock()
        mock_model.nq = 11
        mock_model.nv = 7
        mock_pin.buildModelFromUrdf.return_value = mock_model
        mock_pin.neutral.return_value = np.zeros(11)
        mock_model.createData.return_value = MagicMock()

        from arm_driver.collision_checker import CollisionChecker
        checker = CollisionChecker("/fake/path.urdf", thresholds)
        checker._mock_pin = mock_pin   # keep reference for per-test patching
        return checker


def _q():
    return np.zeros(7)


def _dq():
    return np.zeros(7)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_no_collision_when_torques_match():
    checker = _make_checker()
    tau_measured = np.array([1.0, 2.0, 1.5, 0.5, 0.3, 0.2, 0.1])

    # RNEA returns exactly the measured torques → residual = 0
    with patch("arm_driver.collision_checker.pin") as mock_pin:
        checker._pin = mock_pin
        mock_pin.rnea.return_value = None
        checker._model_data.tau = tau_measured.copy()

        result = checker.check(_q(), _dq(), tau_measured, "IDLE")

    assert result is False


def test_collision_when_residual_exceeds_default_threshold():
    checker = _make_checker()
    tau_measured = np.array([200.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])  # 200 Nm residual

    with patch("arm_driver.collision_checker.pin") as mock_pin:
        checker._pin = mock_pin
        mock_pin.rnea.return_value = None
        checker._model_data.tau = np.zeros(7)  # model says 0

        result = checker.check(_q(), _dq(), tau_measured, "IDLE")

    assert result is True


def test_open_door_uses_higher_threshold():
    checker = _make_checker(thresholds={"DEFAULT": 100.0, "OPEN_DOOR": 500.0})
    # 200 Nm residual — exceeds DEFAULT (100) but not OPEN_DOOR (500)
    tau_measured = np.array([200.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    with patch("arm_driver.collision_checker.pin") as mock_pin:
        checker._pin = mock_pin
        mock_pin.rnea.return_value = None
        checker._model_data.tau = np.zeros(7)

        result_normal = checker.check(_q(), _dq(), tau_measured, "IDLE")
        result_door = checker.check(_q(), _dq(), tau_measured, "OPEN_DOOR")

    assert result_normal is True
    assert result_door is False


def test_unknown_state_falls_back_to_default():
    checker = _make_checker(thresholds={"DEFAULT": 100.0})
    tau_measured = np.array([200.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    with patch("arm_driver.collision_checker.pin") as mock_pin:
        checker._pin = mock_pin
        mock_pin.rnea.return_value = None
        checker._model_data.tau = np.zeros(7)

        result = checker.check(_q(), _dq(), tau_measured, "SOME_FUTURE_STATE")

    assert result is True  # falls back to DEFAULT threshold of 100


def test_q_pin_encoding_for_gen3():
    """Verify the cos/sin encoding for continuous joints."""
    from arm_driver.collision_checker import CollisionChecker

    q = np.array([math.pi / 4, 0.5, math.pi / 3, 1.0, math.pi / 6, 0.8, math.pi / 2])
    expected = np.array([
        math.cos(q[0]), math.sin(q[0]),   # joint_1 continuous
        q[1],                              # joint_2 revolute
        math.cos(q[2]), math.sin(q[2]),   # joint_3 continuous
        q[3],                              # joint_4 revolute
        math.cos(q[4]), math.sin(q[4]),   # joint_5 continuous
        q[5],                              # joint_6 revolute
        math.cos(q[6]), math.sin(q[6]),   # joint_7 continuous
    ])

    result = CollisionChecker._to_q_pin(q)
    np.testing.assert_array_almost_equal(result, expected)
```

- [ ] **Step 2: Run to confirm failure**

```bash
pytest hardware/arm_driver/test/test_collision_checker.py -v
```

Expected: `ModuleNotFoundError` — `collision_checker.py` does not exist yet.

- [ ] **Step 3: Implement `collision_checker.py`**

```python
# hardware/arm_driver/arm_driver/collision_checker.py
import math

import numpy as np
import pinocchio as pin


class CollisionChecker:
    """Detects arm collisions by comparing measured joint torques against a
    Pinocchio RNEA model prediction.

    The residual is the max absolute difference across all joints between
    measured torque and model-predicted torque.  If the residual exceeds the
    threshold for the current arm state, a collision is reported.

    Args:
        urdf_path: Absolute path to the Gen3 URDF file.
        thresholds: Mapping of ArmState name (e.g. ``"OPEN_DOOR"``) to
            residual threshold in Nm.  Any state not listed falls back to
            ``thresholds["DEFAULT"]``.
    """

    def __init__(self, urdf_path: str, thresholds: dict[str, float]) -> None:
        self._model = pin.buildModelFromUrdf(urdf_path)
        self._model_data = self._model.createData()
        self._thresholds = thresholds

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def check(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        tau: np.ndarray,
        state_name: str,
    ) -> bool:
        """Return True if a collision is detected.

        Args:
            q: Joint positions (7,) in radians.
            dq: Joint velocities (7,) in rad/s.
            tau: Measured joint torques (7,) in Nm.
            state_name: Current ArmState name, used to look up threshold.
        """
        q_pin = self._to_q_pin(q)
        pin.rnea(self._model, self._model_data, q_pin, dq, np.zeros(7))
        tau_model = self._model_data.tau.copy()

        residual = np.max(np.abs(tau - tau_model))
        threshold = self._thresholds.get(state_name, self._thresholds["DEFAULT"])
        return bool(residual > threshold)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _to_q_pin(q: np.ndarray) -> np.ndarray:
        """Convert 7-element joint angle vector to Pinocchio configuration.

        The Gen3 URDF has alternating continuous / revolute joints:
          joints 1,3,5,7 (indices 0,2,4,6) are ``continuous``
            → Pinocchio encodes as [cos(θ), sin(θ)]
          joints 2,4,6   (indices 1,3,5)   are ``revolute``
            → Pinocchio encodes as [θ]

        Resulting vector has 11 elements.
        """
        return np.array([
            math.cos(q[0]), math.sin(q[0]),   # joint_1
            q[1],                              # joint_2
            math.cos(q[2]), math.sin(q[2]),   # joint_3
            q[3],                              # joint_4
            math.cos(q[4]), math.sin(q[4]),   # joint_5
            q[5],                              # joint_6
            math.cos(q[6]), math.sin(q[6]),   # joint_7
        ])
```

- [ ] **Step 4: Run tests**

```bash
pytest hardware/arm_driver/test/test_collision_checker.py -v
```

Expected: all 5 tests PASS.

- [ ] **Step 5: Commit**

```bash
git add hardware/arm_driver/arm_driver/collision_checker.py hardware/arm_driver/test/test_collision_checker.py
git commit -m "feat: add CollisionChecker with Pinocchio RNEA and per-state thresholds"
```

______________________________________________________________________

## Task 5: Add Gen3 URDF to the arm_driver Package

The URDF is needed at runtime for Pinocchio. It lives in the feeding-deployment repo and must be committed into ours and installed into the ament share directory.

**Files:**

- Create: `hardware/arm_driver/arm_driver/urdf/gen3_robotiq_2f_85.urdf`

- Modify: `hardware/arm_driver/setup.py`

- [ ] **Step 1: Copy the URDF**

```bash
mkdir -p hardware/arm_driver/arm_driver/urdf
cp ../feeding-deployment/src/feeding_deployment/control/robot_controller/urdfs/gen3_robotiq_2f_85.urdf \
   hardware/arm_driver/arm_driver/urdf/gen3_robotiq_2f_85.urdf
```

- [ ] **Step 2: Register the URDF as an installed data file**

In `hardware/arm_driver/setup.py`, replace the `data_files` list:

```python
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/urdf", ["arm_driver/urdf/gen3_robotiq_2f_85.urdf"]),
    ],
```

- [ ] **Step 3: Verify the URDF installs correctly**

```bash
cd hardware/arm_driver
pip install -e .
python3 -c "
from ament_index_python.packages import get_package_share_directory
import os
p = os.path.join(get_package_share_directory('arm_driver'), 'urdf', 'gen3_robotiq_2f_85.urdf')
print('URDF found:', os.path.exists(p), p)
"
```

Expected output: `URDF found: True /path/to/share/arm_driver/urdf/gen3_robotiq_2f_85.urdf`

- [ ] **Step 4: Commit**

```bash
git add hardware/arm_driver/arm_driver/urdf/gen3_robotiq_2f_85.urdf hardware/arm_driver/setup.py
git commit -m "feat: add Gen3 URDF to arm_driver package for Pinocchio collision detection"
```

______________________________________________________________________

## Task 6: Integrate CollisionChecker into ArmDriverNode

Wire `CollisionChecker` into the existing 100 Hz feedback loop. Thresholds and URDF path come from ROS 2 parameters so they are tunable without recompiling.

**Files:**

- Modify: `hardware/arm_driver/arm_driver/arm_driver.py`

- Modify: `hardware/arm_driver/test/test_arm_driver_safety.py`

- [ ] **Step 1: Write the failing test**

Add to `test_arm_driver_safety.py`:

```python
def test_collision_triggers_error():
    node, mock_arm, ArmState = _make_node()
    try:
        import numpy as np
        from unittest.mock import patch

        # Inject a CollisionChecker that always reports collision
        mock_checker = MagicMock()
        mock_checker.check.return_value = True
        node._collision_checker = mock_checker

        # Provide a valid get_state response
        mock_arm.get_state.return_value = {
            "position": np.zeros(7),
            "velocity": np.zeros(7),
            "effort": np.zeros(7),
            "ee_pos": np.zeros(7),
            "ee_vel": np.zeros(6),
            "ee_force": np.zeros(3),
            "gripper_pos": 0.0,
        }
        node._last_feedback_time = __import__("time").monotonic()

        node._publish_joint_states()

        assert node._state == ArmState.ERROR
        assert "collision" in node._error_reason.lower()
    finally:
        node.destroy_node()


def test_no_collision_does_not_trigger_error():
    node, mock_arm, ArmState = _make_node()
    try:
        import numpy as np

        mock_checker = MagicMock()
        mock_checker.check.return_value = False
        node._collision_checker = mock_checker

        mock_arm.get_state.return_value = {
            "position": np.zeros(7),
            "velocity": np.zeros(7),
            "effort": np.zeros(7),
            "ee_pos": np.zeros(7),
            "ee_vel": np.zeros(6),
            "ee_force": np.zeros(3),
            "gripper_pos": 0.0,
        }
        node._last_feedback_time = __import__("time").monotonic()

        node._publish_joint_states()

        assert node._state == ArmState.IDLE
    finally:
        node.destroy_node()
```

- [ ] **Step 2: Run to confirm failure**

```bash
pytest hardware/arm_driver/test/test_arm_driver_safety.py::test_collision_triggers_error -v
```

Expected: `AttributeError` — `_collision_checker` not set on the node.

- [ ] **Step 3: Declare ROS 2 parameters for collision thresholds**

Add the import at the top of `arm_driver.py`:

```python
from ament_index_python.packages import get_package_share_directory
from arm_driver.collision_checker import CollisionChecker
import os
```

In `ArmDriverNode.__init__`, before `self._init_publishers()`, declare parameters:

```python
        # Collision checker parameters
        self.declare_parameter(
            "collision_checker.urdf_path",
            os.path.join(
                get_package_share_directory("arm_driver"),
                "urdf",
                "gen3_robotiq_2f_85.urdf",
            ),
        )
        self.declare_parameter("collision_checker.threshold_default", 100.0)
        self.declare_parameter("collision_checker.threshold_open_door", 500.0)
```

- [ ] **Step 4: Instantiate `CollisionChecker` in `_init_arm`**

In `_init_arm`, after the arm is connected successfully, add:

```python
        try:
            urdf_path = self.get_parameter("collision_checker.urdf_path").value
            thresholds = {
                "DEFAULT": self.get_parameter(
                    "collision_checker.threshold_default"
                ).value,
                "OPEN_DOOR": self.get_parameter(
                    "collision_checker.threshold_open_door"
                ).value,
            }
            self._collision_checker = CollisionChecker(urdf_path, thresholds)
            self.get_logger().info("CollisionChecker initialised.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialise CollisionChecker: {e}")
            self._collision_checker = None
```

Also add `self._collision_checker = None` in `__init__` alongside `self._arm: KinovaArm | None = None`.

- [ ] **Step 5: Call `check()` in `_publish_joint_states`**

After `self._last_feedback_time = time.monotonic()` and after the `state` dict is populated, add:

```python
            if (
                self._collision_checker is not None
                and self._state != ArmState.ERROR
            ):
                if self._collision_checker.check(
                    state["position"],
                    state["velocity"],
                    state["effort"],
                    self._state.name,
                ):
                    self.get_logger().error("Collision detected — stopping arm.")
                    self._error_reason = "Collision detected"
                    self._transition_to(ArmState.ERROR)
                    return
```

Place this block **before** the publish calls so we don't publish state data from a collided arm.

- [ ] **Step 6: Run all tests**

```bash
pytest hardware/arm_driver/test/test_arm_driver_safety.py hardware/arm_driver/test/test_collision_checker.py -v
```

Expected: all tests PASS.

- [ ] **Step 7: Commit**

```bash
git add hardware/arm_driver/arm_driver/arm_driver.py hardware/arm_driver/test/test_arm_driver_safety.py
git commit -m "feat: integrate CollisionChecker into arm_driver feedback loop"
```

______________________________________________________________________

## Self-Review

### Spec coverage

| Requirement                             | Task                             |
| --------------------------------------- | -------------------------------- |
| Fix e-stop stub                         | Task 1                           |
| Arm comms health                        | Task 2                           |
| Stale twist watchdog                    | Task 3                           |
| CollisionChecker with Pinocchio RNEA    | Task 4                           |
| Per-state thresholds (OPEN_DOOR higher) | Task 4 (class) + Task 6 (params) |
| Thresholds tunable without recompile    | Task 6 (ROS 2 params)            |
| URDF available at runtime               | Task 5                           |
| Collision escalates to ERROR            | Task 6                           |

### Placeholder scan

None found — all steps contain complete code.

### Type consistency

- `CollisionChecker.check()` signature `(q, dq, tau, state_name) -> bool` is used identically in Task 4 (impl), Task 4 (tests), and Task 6 (integration).
- `_collision_checker: CollisionChecker | None` initialised in `__init__` before `_init_arm` sets it.
- `COMMS_TIMEOUT_S`, `TWIST_TIMEOUT_S` defined as module-level constants and imported in tests via `from arm_driver.arm_driver import ...`.

______________________________________________________________________
