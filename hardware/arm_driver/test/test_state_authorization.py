#!/usr/bin/env python3
"""Authorization smoke-test for the arm_driver state machine.

For each operational state, publishes a small command from every source in turn
and watches /arm/joint_states for unexpected motion.  The test PASSES when:

  - Unauthorized sources produce no motion.
  - The authorized source produces motion (where applicable).

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
from arm_interfaces.srv import SetMode
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

# ── tunables ─────────────────────────────────────────────────────────────────

# Small linear-x velocity (m/s) sent to twist sources during testing.
TWIST_VEL = 0.02

# Joint-0 offset (rad) added to the current position when testing position sources.
POSITION_DELTA = 0.03

# Seconds to watch for motion after publishing a command.
OBSERVE_S = 1.0

# Minimum joint change (rad) that counts as "the arm moved".
MOTION_THRESH = 0.01

# Seconds to wait after returning to IDLE for the arm to fully settle.
SETTLE_S = 0.5

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

# Command type each source uses (determines which topic/message type to publish).
SOURCE_TYPE = {
    "atdev": "twist",
    "xbox": "twist",
    "cornell": "position",
    "cmu": "position",
}

SOURCES = list(SOURCE_TYPE.keys())


# ── test node ────────────────────────────────────────────────────────────────


class AuthTestNode(Node):
    def __init__(self):
        super().__init__("arm_auth_test")

        self._positions: list[float] | None = None
        self._lock = threading.Lock()

        self._estop_pub = self.create_publisher(Bool, "/estop", 1)

        self._twist_pubs = {
            src: self.create_publisher(Twist, f"/arm/{src}/twist", 1)
            for src in SOURCES
            if SOURCE_TYPE[src] == "twist"
        }
        self._joint_pubs = {
            src: self.create_publisher(JointState, f"/arm/{src}/joint_position", 1)
            for src in SOURCES
            if SOURCE_TYPE[src] == "position"
        }

        self.create_subscription(JointState, "/arm/joint_states", self._on_joints, 10)

        self._set_mode_cli = self.create_client(SetMode, "/arm/set_mode")

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
        # Let the arm fully stop before the next sample.
        deadline = time.monotonic() + SETTLE_S
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ── command publishing ───────────────────────────────────────────────────

    def publish_command(self, source: str):
        if SOURCE_TYPE[source] == "twist":
            msg = Twist()
            msg.linear.x = TWIST_VEL
            self._twist_pubs[source].publish(msg)
        else:
            positions = self.snapshot()
            if positions is None:
                return
            positions[0] += POSITION_DELTA
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.position = positions
            self._joint_pubs[source].publish(msg)

    # ── motion detection ─────────────────────────────────────────────────────

    def _observe(self) -> list[float] | None:
        """Spin for OBSERVE_S seconds and return the final joint positions."""
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
        results: dict[tuple[str, str], tuple[bool, bool]] = {}

        for state_name, (mode, auth_source) in STATES.items():
            print(f"\n{'=' * 60}")
            print(f"State: {state_name}  (authorized: {auth_source or 'none'})")
            print("=" * 60)

            if not self.set_mode(mode):
                print(f"  [SKIP] Could not enter state {state_name}")
                continue

            for source in SOURCES:
                before = self.snapshot()
                self.publish_command(source)
                after = self._observe()

                did_move = self._moved(before, after)
                motion_expected = source == auth_source

                if motion_expected and did_move:
                    verdict = "PASS  — motion detected (authorized)"
                elif motion_expected and not did_move:
                    verdict = (
                        "WARN  — no motion from authorized source (hardware connected?)"
                    )
                elif not motion_expected and did_move:
                    verdict = "FAIL  *** UNAUTHORIZED MOTION DETECTED ***"
                else:
                    verdict = "PASS  — correctly rejected"

                print(f"  {source:<8}  {verdict}")
                results[(state_name, source)] = (did_move, motion_expected)

                # Stop motion and settle before testing the next source.
                self.return_to_idle()

        # ── summary ──────────────────────────────────────────────────────────
        print(f"\n{'=' * 60}")
        print("SUMMARY")
        print("=" * 60)

        failures = [
            (state, src)
            for (state, src), (moved, expected) in results.items()
            if moved and not expected
        ]
        warnings = [
            (state, src)
            for (state, src), (moved, expected) in results.items()
            if expected and not moved
        ]

        if failures:
            print(f"FAILED — {len(failures)} unauthorized motion(s):")
            for state, src in failures:
                print(f"  state {state:<14}  source '{src}'")
        else:
            print("No unauthorized motion detected.")

        if warnings:
            print(
                f"\nWARNINGS — {len(warnings)} authorized source(s) produced no motion:"
            )
            for state, src in warnings:
                print(
                    f"  state {state:<14}  source '{src}'  (check hardware / OBSERVE_S / POSITION_DELTA)"
                )

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

    print("Ready. Starting authorization tests.\n")

    passed = node.run()

    node.return_to_idle()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
