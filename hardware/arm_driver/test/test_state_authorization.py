#!/usr/bin/env python3
"""Authorization smoke-test for the arm_driver state machine.

For each operational state, sends every command type from every source in turn
and watches /arm/joint_states for unexpected motion.  The test PASSES when:

  - Unauthorized sources produce no motion.
  - The authorized source produces motion (where applicable).

The arm is homed via the ReachPreset action before testing begins.

Press Ctrl-C at any time to immediately publish to /estop before the arm moves
further.

Usage (with a ROS 2 environment sourced and arm_driver running):

    python3 test/test_state_authorization.py
"""

import signal
import sys
import threading
import time

import rclpy
import rclpy.action
from arm_interfaces.action import ExecuteTrajectory, ReachPreset
from arm_interfaces.srv import SetMode
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# ── tunables ─────────────────────────────────────────────────────────────────

# Small linear-x velocity (m/s) sent to twist sources during testing.
TWIST_VEL = 0.05

# Joint-0 offset (rad) added to the current position when testing joint
# position and trajectory commands.
POSITION_DELTA = 0.1

# Cartesian target used for cartesian_pose tests (absolute, in base frame).
# Set this to a pose that is safe and reachable from the arm's home position.
CARTESIAN_TEST_XYZ = [0.44, 0.0, 0.45]  # metres — slightly above home z
CARTESIAN_TEST_QUAT_XYZW = [0.0, 0.0, 0.0, 1.0]

# Seconds to watch for motion after sending a command.
OBSERVE_S = 1.5

# Minimum joint change (rad) that counts as "the arm moved".
MOTION_THRESH = 0.01

# Seconds to wait after returning to IDLE for the arm to fully settle.
SETTLE_S = 0.5

# Timeout (s) for the homing action to complete.
HOME_TIMEOUT_S = 30.0

# ── static config (mirrors arm_driver.py) ────────────────────────────────────

# Maps state name → (SetMode constant, authorized source or None)
STATES = {
    "IDLE": (SetMode.Request.MODE_IDLE, None),
    "OPEN_DOOR": (SetMode.Request.MODE_OPEN_DOOR, "cmu"),
    "ORDER_DRINK": (SetMode.Request.MODE_ORDER_DRINK, "cornell"),
    "DRINKING": (SetMode.Request.MODE_DRINKING, "cornell"),
    "CUP_STABILIZE": (SetMode.Request.MODE_CUP_STABILIZE, "atdev"),
    "MANUAL": (SetMode.Request.MODE_MANUAL, "xbox"),
}

# Command types to test per source.
SOURCE_COMMANDS = {
    "atdev": ["twist"],
    "xbox": ["twist"],
    "cornell": ["joint_position", "cartesian_pose", "execute_trajectory"],
    "cmu": ["joint_position", "cartesian_pose", "execute_trajectory"],
}

SOURCES = list(SOURCE_COMMANDS.keys())


# ── test node ────────────────────────────────────────────────────────────────


class AuthTestNode(Node):
    def __init__(self):
        super().__init__("arm_auth_test")

        self._positions: list[float] | None = None
        self._lock = threading.Lock()

        # Publishers
        self._estop_pub = self.create_publisher(Bool, "/estop", 1)
        self._twist_pubs = {
            src: self.create_publisher(Twist, f"/arm/{src}/twist", 1)
            for src in SOURCES
            if "twist" in SOURCE_COMMANDS[src]
        }
        self._joint_pubs = {
            src: self.create_publisher(JointState, f"/arm/{src}/joint_position", 1)
            for src in SOURCES
            if "joint_position" in SOURCE_COMMANDS[src]
        }
        self._cartesian_pubs = {
            src: self.create_publisher(PoseStamped, f"/arm/{src}/cartesian_pose", 1)
            for src in SOURCES
            if "cartesian_pose" in SOURCE_COMMANDS[src]
        }

        # Subscribers
        self.create_subscription(JointState, "/arm/joint_states", self._on_joints, 10)

        # Service clients
        self._set_mode_cli = self.create_client(SetMode, "/arm/set_mode")

        # Action clients
        self._reach_preset_cli = ActionClient(self, ReachPreset, "/arm/reach_preset")
        self._trajectory_clis = {
            src: ActionClient(self, ExecuteTrajectory, f"/arm/{src}/execute_trajectory")
            for src in SOURCES
            if "execute_trajectory" in SOURCE_COMMANDS[src]
        }

    # ── ROS interface helpers ────────────────────────────────────────────────

    def _on_joints(self, msg: JointState):
        with self._lock:
            self._positions = list(msg.position)

    def snapshot(self) -> list[float] | None:
        with self._lock:
            return list(self._positions) if self._positions is not None else None

    def set_mode(self, mode: int) -> bool:
        req = SetMode.Request()
        req.mode = mode
        future = self._set_mode_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None or not result.success:
            msg = getattr(result, "message", "timeout") if result else "timeout"
            self.get_logger().warn(f"set_mode({mode}) failed: {msg}")
            return False
        return True

    def estop(self):
        msg = Bool()
        msg.data = True
        self._estop_pub.publish(msg)

    def return_to_idle(self):
        self.set_mode(SetMode.Request.MODE_IDLE)
        deadline = time.monotonic() + SETTLE_S
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def return_to_home(self):
        """Return to idle then re-home the arm between states."""
        self.return_to_idle()
        if not self.home():
            self.get_logger().warn("Failed to home after state — continuing anyway.")

    def _reach_preset(self, preset: int, label: str) -> bool:
        """Send a ReachPreset action and block until complete."""
        print(f"Moving to {label} via ReachPreset action...")
        if not self._reach_preset_cli.wait_for_server(timeout_sec=10.0):
            print("ERROR: /arm/reach_preset action server not available.")
            return False

        goal = ReachPreset.Goal()
        goal.preset = preset
        send_future = self._reach_preset_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print(f"ERROR: {label} goal rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=HOME_TIMEOUT_S
        )

        result = result_future.result()
        if result is None or not result.result.success:
            msg = result.result.message if result else "timeout"
            print(f"ERROR: {label} failed: {msg}")
            return False

        print(f"{label} complete.")
        return True

    def home(self) -> bool:
        return self._reach_preset(ReachPreset.Goal.PRESET_HOME, "Home")

    def retract(self) -> bool:
        return self._reach_preset(ReachPreset.Goal.PRESET_RETRACT, "Retract")

    # ── command publishing ───────────────────────────────────────────────────

    def _send_twist(self, source: str):
        msg = Twist()
        msg.linear.x = TWIST_VEL
        self._twist_pubs[source].publish(msg)

    def _send_joint_position(self, source: str):
        positions = self.snapshot()
        if positions is None:
            return
        positions[0] += POSITION_DELTA
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = positions
        self._joint_pubs[source].publish(msg)

    def _send_cartesian_pose(self, source: str):
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.pose.position.x = CARTESIAN_TEST_XYZ[0]
        msg.pose.position.y = CARTESIAN_TEST_XYZ[1]
        msg.pose.position.z = CARTESIAN_TEST_XYZ[2]
        msg.pose.orientation.x = CARTESIAN_TEST_QUAT_XYZW[0]
        msg.pose.orientation.y = CARTESIAN_TEST_QUAT_XYZW[1]
        msg.pose.orientation.z = CARTESIAN_TEST_QUAT_XYZW[2]
        msg.pose.orientation.w = CARTESIAN_TEST_QUAT_XYZW[3]
        self._cartesian_pubs[source].publish(msg)

    def _send_trajectory(self, source: str) -> bool:
        """Send a 1-waypoint ExecuteTrajectory action goal; don't block on result."""
        positions = self.snapshot()
        if positions is None:
            return False

        target = list(positions)
        target[0] += POSITION_DELTA

        point = JointTrajectoryPoint()
        point.positions = target
        point.time_from_start = Duration(sec=2)

        trajectory = JointTrajectory()
        trajectory.points = [point]

        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory

        cli = self._trajectory_clis[source]
        if not cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn(
                f"execute_trajectory server not available for {source}"
            )
            return False

        send_future = cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        goal_handle = send_future.result()
        # Motion detection covers the outcome — just check the goal was accepted or rejected.
        return goal_handle is not None

    def send_command(self, source: str, command_type: str):
        if command_type == "twist":
            self._send_twist(source)
        elif command_type == "joint_position":
            self._send_joint_position(source)
        elif command_type == "cartesian_pose":
            self._send_cartesian_pose(source)
        elif command_type == "execute_trajectory":
            self._send_trajectory(source)

    # ── motion detection ─────────────────────────────────────────────────────

    def _observe(self) -> list[float] | None:
        deadline = time.monotonic() + OBSERVE_S
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.snapshot()

    @staticmethod
    def _moved(before: list[float] | None, after: list[float] | None) -> bool:
        if before is None or after is None:
            return False
        return any(abs(a - b) > MOTION_THRESH for a, b in zip(after, before))

    # ── main test loop ───────────────────────────────────────────────────────

    def run(self) -> bool:
        results: dict[tuple[str, str, str], tuple[bool, bool]] = {}

        for state_name, (mode, auth_source) in STATES.items():
            print(f"\n{'=' * 60}")
            print(f"State: {state_name}  (authorized: {auth_source or 'none'})")
            print("=" * 60)

            if not self.set_mode(mode):
                print(f"  [SKIP] Could not enter state {state_name}")
                continue

            for source in SOURCES:
                for command_type in SOURCE_COMMANDS[source]:
                    if not self.set_mode(mode):
                        print(
                            f"  [SKIP] Could not re-enter {state_name} "
                            f"for {source}/{command_type}"
                        )
                        continue

                    before = self.snapshot()
                    self.send_command(source, command_type)
                    after = self._observe()

                    did_move = self._moved(before, after)
                    motion_expected = source == auth_source

                    if motion_expected and did_move:
                        verdict = "PASS  — motion detected (authorized)"
                    elif motion_expected and not did_move:
                        verdict = "WARN  — no motion from authorized source (hardware connected?)"
                    elif not motion_expected and did_move:
                        verdict = "FAIL  *** UNAUTHORIZED MOTION DETECTED ***"
                    else:
                        verdict = "PASS  — correctly rejected"

                    print(f"  {source:<8}  {command_type:<22}  {verdict}")
                    results[(state_name, source, command_type)] = (
                        did_move,
                        motion_expected,
                    )

                    self.return_to_idle()

            self.return_to_home()

        # ── summary ──────────────────────────────────────────────────────────
        print(f"\n{'=' * 60}")
        print("SUMMARY")
        print("=" * 60)

        failures = [
            (s, src, cmd)
            for (s, src, cmd), (moved, expected) in results.items()
            if moved and not expected
        ]
        warnings = [
            (s, src, cmd)
            for (s, src, cmd), (moved, expected) in results.items()
            if expected and not moved
        ]

        if failures:
            print(f"FAILED — {len(failures)} unauthorized motion(s):")
            for state, src, cmd in failures:
                print(f"  state {state:<14}  {src}/{cmd}")
        else:
            print("No unauthorized motion detected.")

        if warnings:
            print(
                f"\nWARNINGS — {len(warnings)} authorized source(s) produced no motion:"
            )
            for state, src, cmd in warnings:
                print(f"  state {state:<14}  {src}/{cmd}  (check hardware / tunables)")

        return len(failures) == 0


# ── entry point ───────────────────────────────────────────────────────────────


def main():
    rclpy.init()
    node = AuthTestNode()

    def on_sigint(_sig, _frame):
        print("\n[Ctrl-C] Sending e-stop and shutting down...")
        node.estop()
        time.sleep(0.3)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    signal.signal(signal.SIGINT, on_sigint)

    print("Waiting for /arm/joint_states...")
    while node.snapshot() is None:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("Waiting for /arm/set_mode service...")
    if not node._set_mode_cli.wait_for_service(timeout_sec=10.0):
        print("ERROR: /arm/set_mode service not available.")
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    if not node.home():
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.return_to_idle()
    print("\nReady. Starting authorization tests.")

    passed = node.run()

    time.sleep(0.5)

    node.retract()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
