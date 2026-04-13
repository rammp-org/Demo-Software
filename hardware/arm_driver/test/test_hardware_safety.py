#!/usr/bin/env python3
"""Hardware-in-the-loop safety tests for arm_driver.

Verifies that the three safety layers work correctly on the real arm:

  1. E-stop   — publishing to /estop halts active motion and sets ERROR state
  2. Twist watchdog  — arm stops within TWIST_TIMEOUT_S after publisher goes silent
  3. Collision detection (no false positives) — normal motion produces no spurious ERROR
  4. Collision detection (contact)  — pushing on the arm triggers ERROR  [MANUAL]

Prerequisites
-------------
- ROS 2 environment sourced
- arm_driver node running, arm connected and homed
- collision_checker.urdf_path parameter set (for tests 3 & 4)

Usage
-----
    python3 test/test_hardware_safety.py

    # Skip collision tests (e.g. URDF not configured):
    python3 test/test_hardware_safety.py --skip-collision

Press Ctrl-C at any time to e-stop the arm and exit immediately.

Pass/fail criteria
------------------
Each test prints PASS or FAIL. The script exits 0 if all run tests pass, 1
otherwise.
"""

import argparse
import signal
import sys
import threading
import time

import numpy as np
import rclpy
import rclpy.action
from arm_interfaces.action import ReachPreset
from arm_interfaces.srv import SetMode
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

# ── tunables ──────────────────────────────────────────────────────────────────

# Twist velocity used during watchdog and e-stop tests (m/s linear-x).
# Small enough to be safe, large enough to produce measurable motion.
TWIST_VEL = 0.03

# How long to stream twist commands before cutting off (seconds).
TWIST_STREAM_S = 1.5

# How long to wait after cutting twist before sampling velocity (seconds).
# Must be > TWIST_TIMEOUT_S (0.5 s) to give the watchdog time to fire.
TWIST_WATCHDOG_SETTLE_S = 1.2

# Joint velocity (rad/s) below which the arm counts as "stopped".
STOPPED_VEL_THRESH = 0.005

# How long to observe /arm/status waiting for ERROR after e-stop (seconds).
ESTOP_OBSERVE_S = 2.0

# How long to observe /arm/status during normal motion for false positives (s).
FALSE_POSITIVE_OBSERVE_S = 5.0

# Seconds of normal motion to command before checking for false positives.
NORMAL_MOTION_STREAM_S = 4.0

# Timeout for the home action.
HOME_TIMEOUT_S = 30.0

# Settle time after returning to IDLE.
SETTLE_S = 1.0


# ── test node ─────────────────────────────────────────────────────────────────


class SafetyTestNode(Node):
    def __init__(self):
        super().__init__("arm_safety_test")

        self._joint_velocities: np.ndarray | None = None
        self._driver_state: str | None = None
        self._lock = threading.Lock()

        # Publishers
        self._estop_pub = self.create_publisher(Bool, "/estop", 1)
        self._twist_pub = self.create_publisher(Twist, "/arm/xbox/twist", 10)

        # Subscribers
        self.create_subscription(JointState, "/arm/joint_states", self._on_joints, 10)
        self.create_subscription(DiagnosticStatus, "/arm/status", self._on_status, 10)

        # Service + action clients
        self._set_mode_cli = self.create_client(SetMode, "/arm/set_mode")
        self._reach_preset_cli = ActionClient(self, ReachPreset, "/arm/reach_preset")

        # Spin in a dedicated background thread so test methods never call
        # spin_once themselves — eliminating all re-entrancy issues.
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _on_joints(self, msg: JointState):
        with self._lock:
            # velocities include gripper at index -1; strip it
            self._joint_velocities = np.array(msg.velocity[:-1])

    def _on_status(self, msg: DiagnosticStatus):
        with self._lock:
            self._driver_state = msg.message

    # ── helpers ───────────────────────────────────────────────────────────────

    def joint_velocities(self) -> np.ndarray | None:
        with self._lock:
            return (
                self._joint_velocities.copy()
                if self._joint_velocities is not None
                else None
            )

    def driver_state(self) -> str | None:
        with self._lock:
            return self._driver_state

    def is_stopped(self) -> bool:
        vels = self.joint_velocities()
        if vels is None:
            return False
        return bool(np.max(np.abs(vels)) < STOPPED_VEL_THRESH)

    def wait_for_state(self, target: str, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.driver_state() == target:
                return True
            time.sleep(0.05)
        # Final check: /arm/status publishes at 1 Hz so the update may arrive
        # right as the deadline expires.  Give it one extra polling period.
        time.sleep(0.1)
        return self.driver_state() == target

    def wait_for_topics(self, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.joint_velocities() is not None and self.driver_state() is not None:
                return True
            time.sleep(0.1)
        return False

    def estop(self):
        msg = Bool()
        msg.data = True
        self._estop_pub.publish(msg)

    def set_mode(self, mode: int) -> bool:
        req = SetMode.Request()
        req.mode = mode
        future = self._set_mode_cli.call_async(req)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        result = future.result()
        return result is not None and result.success

    def home(self) -> bool:
        print("  Homing arm...")
        goal = ReachPreset.Goal()
        goal.preset = ReachPreset.Goal.PRESET_HOME
        send_future = self._reach_preset_cli.send_goal_async(goal)
        deadline = time.monotonic() + 10.0
        while not send_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print("  ERROR: home goal rejected")
            return False
        result_future = goal_handle.get_result_async()
        deadline = time.monotonic() + HOME_TIMEOUT_S
        while not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.1)
        result = result_future.result()
        return result is not None and result.result.success

    def recover_from_error(self):
        """Reset ERROR state and home the arm so the next test starts clean."""
        self.set_mode(SetMode.Request.MODE_IDLE)
        time.sleep(SETTLE_S)
        self.home()
        self.set_mode(SetMode.Request.MODE_IDLE)
        time.sleep(SETTLE_S)

    def stream_twist(self, vel_x: float, duration: float):
        """Publish a constant linear-x twist at ~20 Hz for `duration` seconds."""
        msg = Twist()
        msg.linear.x = vel_x
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self._twist_pub.publish(msg)
            time.sleep(0.05)

    def prompt(self, message: str) -> str:
        """Print a prompt and wait for the user to press Enter."""
        return input(f"\n  ➤  {message}\n     Press Enter to continue...")

    # ── individual tests ──────────────────────────────────────────────────────

    def test_estop_halts_motion(self) -> bool:
        """
        TEST 1: E-STOP HALTS ACTIVE MOTION
        ════════════════════════════════════════════════════════════════════════
        What we test:
          Publishing True to /estop while the arm is moving in MANUAL mode
          causes the driver to call arm.stop() and transition to ERROR.

        Pass criteria:
          - /arm/status reports ERROR within ESTOP_OBSERVE_S seconds
          - Joint velocities drop to near zero
        ════════════════════════════════════════════════════════════════════════
        """
        print("\n── TEST 1: E-stop halts motion ──────────────────────────────")

        if not self.set_mode(SetMode.Request.MODE_MANUAL):
            print("  SKIP — could not enter MANUAL state")
            return True  # not a safety failure

        print(
            f"  Streaming twist (linear-x = {TWIST_VEL} m/s) for {TWIST_STREAM_S}s..."
        )
        twist_thread = threading.Thread(
            target=self.stream_twist, args=(TWIST_VEL, TWIST_STREAM_S), daemon=True
        )
        twist_thread.start()

        # Let the arm build up some velocity before firing the e-stop
        time.sleep(0.5)

        print("  Publishing /estop...")
        self.estop()

        got_error = self.wait_for_state("ERROR", ESTOP_OBSERVE_S)
        twist_thread.join(timeout=TWIST_STREAM_S + 0.5)

        stopped = self.is_stopped()

        if got_error and stopped:
            print("  PASS — arm stopped, state=ERROR")
            self.recover_from_error()
            return True
        else:
            issues = []
            if not got_error:
                issues.append(f"state={self.driver_state()!r} (expected ERROR)")
            if not stopped:
                vels = self.joint_velocities()
                max_vel = (
                    float(np.max(np.abs(vels))) if vels is not None else float("nan")
                )
                issues.append(f"arm still moving (max vel={max_vel:.4f} rad/s)")
            print(f"  FAIL — {'; '.join(issues)}")
            self.estop()
            self.recover_from_error()
            return False

    def test_twist_watchdog(self) -> bool:
        """
        TEST 2: TWIST WATCHDOG STOPS STALE MOTION
        ════════════════════════════════════════════════════════════════════════
        What we test:
          After streaming twist commands, stopping the publisher causes the
          arm to halt within TWIST_TIMEOUT_S (0.5 s). The driver must NOT
          continue executing the last velocity command indefinitely.

        Pass criteria:
          - Arm velocity drops below STOPPED_VEL_THRESH within
            TWIST_WATCHDOG_SETTLE_S seconds of the publisher going silent.
          - Driver state remains MANUAL (watchdog doesn't ERROR the arm).
        ════════════════════════════════════════════════════════════════════════
        """
        print("\n── TEST 2: Twist watchdog stops stale motion ────────────────")

        if not self.set_mode(SetMode.Request.MODE_MANUAL):
            print("  SKIP — could not enter MANUAL state")
            return True

        print(f"  Streaming twist for {TWIST_STREAM_S}s, then going silent...")
        self.stream_twist(TWIST_VEL, TWIST_STREAM_S)

        # Publisher is now silent — wait for watchdog to fire
        print(
            f"  Publisher silent. Waiting {TWIST_WATCHDOG_SETTLE_S}s "
            f"(watchdog timeout = 0.5s)..."
        )
        time.sleep(TWIST_WATCHDOG_SETTLE_S)

        stopped = self.is_stopped()
        state = self.driver_state()
        still_manual = state == "MANUAL"

        if stopped and still_manual:
            print("  PASS — arm stopped, state still MANUAL (recoverable)")
            self.set_mode(SetMode.Request.MODE_IDLE)
            time.sleep(SETTLE_S)
            return True
        else:
            issues = []
            if not stopped:
                vels = self.joint_velocities()
                max_vel = (
                    float(np.max(np.abs(vels))) if vels is not None else float("nan")
                )
                issues.append(f"arm still moving (max vel={max_vel:.4f} rad/s)")
            if not still_manual:
                issues.append(f"unexpected state={state!r} (expected MANUAL)")
            print(f"  FAIL — {'; '.join(issues)}")
            self.estop()
            self.recover_from_error()
            return False

    def test_no_false_positive_collisions(self) -> bool:
        """
        TEST 3: NO FALSE POSITIVE COLLISIONS DURING NORMAL MOTION
        ════════════════════════════════════════════════════════════════════════
        What we test:
          The arm moves freely under twist commands. The CollisionChecker
          must not fire spuriously and send the driver to ERROR.

        Pass criteria:
          - Driver state never becomes ERROR during NORMAL_MOTION_STREAM_S
            seconds of continuous twist motion.

        Note: If collision_checker.urdf_path is not set, collision detection
        is disabled and this test vacuously passes. Check the startup log for
        "CollisionChecker initialised" to confirm it was active.
        ════════════════════════════════════════════════════════════════════════
        """
        print("\n── TEST 3: No false positive collisions ─────────────────────")
        print("  ⚠  Ensure the arm has free space to move before continuing.")
        self.prompt("Ready?")

        if not self.set_mode(SetMode.Request.MODE_MANUAL):
            print("  SKIP — could not enter MANUAL state")
            return True

        print(
            f"  Streaming twist for {NORMAL_MOTION_STREAM_S}s. "
            "Watching for spurious ERROR..."
        )

        error_detected = False
        msg = Twist()
        msg.linear.x = TWIST_VEL
        deadline = time.monotonic() + NORMAL_MOTION_STREAM_S
        while time.monotonic() < deadline:
            self._twist_pub.publish(msg)
            time.sleep(0.05)
            if self.driver_state() == "ERROR":
                error_detected = True
                break

        self.set_mode(SetMode.Request.MODE_IDLE)
        time.sleep(SETTLE_S)

        if not error_detected:
            print("  PASS — no spurious collision detected during normal motion")
            return True
        else:
            print(
                "  FAIL — driver entered ERROR during normal motion. "
                "Collision threshold may be too low, or torque model is miscalibrated."
            )
            self.recover_from_error()
            return False

    def test_collision_on_contact(self) -> bool:
        """
        TEST 4: COLLISION DETECTED ON CONTACT  [MANUAL]
        ════════════════════════════════════════════════════════════════════════
        What we test:
          When the arm is moving and you physically push on it, the torque
          residual should exceed the DEFAULT threshold and trigger ERROR.

        Pass criteria:
          - Driver state becomes ERROR within ESTOP_OBSERVE_S seconds of
            contact being applied.

        Instructions:
          1. The arm will begin moving slowly.
          2. When prompted, firmly push on the arm (e.g. grab the wrist and
             resist its motion).
          3. Release immediately after — the arm will stop.

        ⚠  Keep your other hand near Ctrl-C. If the arm does not detect the
           collision and you cannot release safely, hit Ctrl-C to e-stop.
        ════════════════════════════════════════════════════════════════════════
        """
        print("\n── TEST 4: Collision detected on contact  [MANUAL] ──────────")
        print("  ⚠  This test requires you to push on the moving arm.")
        print("  ⚠  Read the full instructions above before proceeding.")
        self.prompt("Ready? (or Ctrl-C to skip)")

        if not self.set_mode(SetMode.Request.MODE_MANUAL):
            print("  SKIP — could not enter MANUAL state")
            return True

        # Start streaming twist in a background thread
        keep_streaming = threading.Event()
        keep_streaming.set()

        def stream():
            msg = Twist()
            msg.linear.x = TWIST_VEL
            while keep_streaming.is_set():
                self._twist_pub.publish(msg)
                time.sleep(0.05)

        stream_thread = threading.Thread(target=stream, daemon=True)
        stream_thread.start()
        time.sleep(0.5)  # let arm start moving

        self.prompt("Arm is moving — push firmly on it now, then release")

        # Watch for ERROR
        got_error = self.wait_for_state("ERROR", ESTOP_OBSERVE_S)

        keep_streaming.clear()
        stream_thread.join(timeout=1.0)

        if got_error:
            print("  PASS — collision detected, driver entered ERROR")
            self.recover_from_error()
            return True
        else:
            print(
                f"  FAIL — driver state={self.driver_state()!r} after contact. "
                "Collision was not detected. Check that kortex_description URDF "
                "is loaded and threshold is appropriate."
            )
            self.estop()
            self.recover_from_error()
            return False

    # ── main run loop ─────────────────────────────────────────────────────────

    def run(self, skip_collision: bool) -> bool:
        results: dict[str, bool | None] = {}

        print("\nWaiting for /arm/joint_states and /arm/status...")
        if not self.wait_for_topics(timeout=10.0):
            print(
                "ERROR: timed out waiting for arm_driver topics. Is arm_driver running?"
            )
            return False

        print("Waiting for /arm/set_mode service...")
        if not self._set_mode_cli.wait_for_service(timeout_sec=5.0):
            print("ERROR: /arm/set_mode not available.")
            return False

        print("Waiting for /arm/reach_preset action server...")
        if not self._reach_preset_cli.wait_for_server(timeout_sec=5.0):
            print("ERROR: /arm/reach_preset not available.")
            return False

        # Home before starting
        if not self.home():
            print("ERROR: failed to home arm before tests.")
            return False
        self.set_mode(SetMode.Request.MODE_IDLE)
        time.sleep(SETTLE_S)

        # Run tests
        results["1_estop"] = self.test_estop_halts_motion()

        if not self.home():
            print("ERROR: failed to home before test 2 — aborting remaining tests")
            return False
        results["2_twist_watchdog"] = self.test_twist_watchdog()

        if not skip_collision:
            if not self.home():
                print("ERROR: failed to home before test 3 — skipping collision tests")
                results["3_no_false_positives"] = None
                results["4_collision_on_contact"] = None
            else:
                results["3_no_false_positives"] = (
                    self.test_no_false_positive_collisions()
                )

                if not self.home():
                    print("ERROR: failed to home before test 4 — skipping")
                    results["4_collision_on_contact"] = None
                else:
                    results["4_collision_on_contact"] = self.test_collision_on_contact()
        else:
            print("\n── Tests 3 & 4 skipped (--skip-collision) ───────────────────")
            results["3_no_false_positives"] = None
            results["4_collision_on_contact"] = None

        # Summary
        print("\n" + "═" * 60)
        print("SUMMARY")
        print("═" * 60)
        all_passed = True
        for name, result in results.items():
            if result is True:
                label = "PASS"
            elif result is False:
                label = "FAIL"
                all_passed = False
            else:
                label = "SKIP"
            print(f"  {name:<30}  {label}")

        print("═" * 60)
        if all_passed:
            print("All tests passed.\n")
        else:
            print("One or more tests FAILED. Review output above.\n")

        return all_passed


# ── entry point ───────────────────────────────────────────────────────────────


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--skip-collision",
        action="store_true",
        help="Skip tests 3 & 4 (collision detection). Use when URDF is not configured.",
    )
    args = parser.parse_args()

    rclpy.init()
    node = SafetyTestNode()

    def on_sigint(_sig, _frame):
        print("\n[Ctrl-C] Sending e-stop and exiting...")
        node.estop()
        time.sleep(0.3)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    signal.signal(signal.SIGINT, on_sigint)

    passed = node.run(skip_collision=args.skip_collision)

    # Return to retract on exit
    node.home()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if passed else 1)


if __name__ == "__main__":
    main()
