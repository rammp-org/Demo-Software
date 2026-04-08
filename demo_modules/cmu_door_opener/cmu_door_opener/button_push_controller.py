#!/usr/bin/env python3
"""
Button push controller for CMU door opener.

Exposes the /arm/door/open action server (DoorOpen).
When called, it takes the **latest** ButtonInfo from the detector and:
  1. Commands the arm to push the button via /arm/cmu/cartesian_pose
  2. Monitors /arm/ee_force — stops on contact via zero twist
  3. Retracts the arm via /arm/reach_preset

The behavior tree sets the arm to OPEN_DOOR mode before calling this action.
"""

import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped, TwistStamped
from diagnostic_msgs.msg import DiagnosticStatus
from scipy.spatial.transform import Rotation

from cmu_door_opener_interfaces.action import DoorOpen
from cmu_door_opener_interfaces.msg import ButtonInfo
from arm_interfaces.action import ReachPreset


# Button push parameters
APPROACH_OFFSET = 0.03  # meters — stop this far in front of the button first
PUSH_STEP = 0.01  # meters — incremental push distance per step (1cm)
PUSH_MAX = 0.10  # meters — max total push distance past approach point
FORCE_THRESHOLD = 30.0  # Newtons — contact detection threshold
PUSH_TIMEOUT = 10.0  # seconds — max time for each phase
FORCE_CHECK_RATE = 0.02  # seconds between force checks


class ButtonPushController(Node):
    def __init__(self):
        super().__init__("button_push_controller")

        self._cb_group = ReentrantCallbackGroup()

        # Latest ButtonInfo from detector
        self.latest_button_info = None
        self._latest_button_time = None
        self.create_subscription(
            ButtonInfo, "/arm/door/button_info", self._cb_button_info, 10
        )

        # Arm status (mode)
        self.latest_arm_status = None
        self.create_subscription(
            DiagnosticStatus, "/arm/status", self._cb_arm_status, 10
        )

        # End-effector force from arm_driver
        self.latest_ee_force = None
        self.create_subscription(Vector3Stamped, "/arm/ee/force", self._cb_ee_force, 10)

        # End-effector velocity from arm_driver
        self.latest_ee_velocity = None
        self.create_subscription(
            TwistStamped, "/arm/ee/velocity", self._cb_ee_velocity, 10
        )

        # Publisher to command arm cartesian pose
        self.pose_pub = self.create_publisher(
            PoseStamped, "/arm/cmu/cartesian_pose", 10
        )

        # Publisher to stop the arm (zero twist)
        self.twist_pub = self.create_publisher(Twist, "/arm/cmu/twist", 10)

        # Action client to retract the arm after push
        self._reach_preset_client = ActionClient(
            self, ReachPreset, "/arm/reach_preset", callback_group=self._cb_group
        )

        # Action server: /arm/door/open
        self._action_server = ActionServer(
            self,
            DoorOpen,
            "/arm/door/open",
            self._execute_open_door,
            callback_group=self._cb_group,
            cancel_callback=self._cancel_callback,
        )

        self.get_logger().info(
            "ButtonPushController ready — waiting for /arm/door/open"
        )

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("Door open action canceled — stopping arm")
        return CancelResponse.ACCEPT

    def _cb_button_info(self, msg: ButtonInfo):
        self.latest_button_info = msg
        self._latest_button_time = self.get_clock().now()

    def _cb_arm_status(self, msg: DiagnosticStatus):
        self.latest_arm_status = msg.message  # e.g. "OPEN_DOOR", "IDLE"

    def _cb_ee_force(self, msg: Vector3Stamped):
        self.latest_ee_force = np.array([msg.vector.x, msg.vector.y, msg.vector.z])

    def _cb_ee_velocity(self, msg: TwistStamped):
        v = msg.twist.linear
        self.latest_ee_velocity = np.array([v.x, v.y, v.z])

    def get_ee_force_magnitude(self):
        if self.latest_ee_force is None:
            return 0.0
        return float(np.linalg.norm(self.latest_ee_force))

    def stop_arm(self):
        zero_twist = Twist()
        self.twist_pub.publish(zero_twist)

    def get_ee_velocity_magnitude(self):
        """Get EE velocity magnitude from /arm/ee/velocity."""
        if self.latest_ee_velocity is None:
            return None
        return float(np.linalg.norm(self.latest_ee_velocity))

    def _button_info_to_pose(self, info: ButtonInfo) -> PoseStamped:
        """Convert ButtonInfo pose_xyzrpy to a PoseStamped for arm command."""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"

        x, y, z = info.pose_xyzrpy[0], info.pose_xyzrpy[1], info.pose_xyzrpy[2]
        roll, pitch, yaw = info.pose_xyzrpy[3], info.pose_xyzrpy[4], info.pose_xyzrpy[5]

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        rot = Rotation.from_euler("xyz", [roll, pitch, yaw])
        quat = rot.as_quat()  # [x, y, z, w]
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])

        return pose

    def _execute_open_door(self, goal_handle):
        """Action callback — runs the full button push sequence."""
        result = DoorOpen.Result()

        self.get_logger().info("=== /arm/door/open action received ===")

        # Check arm is in OPEN_DOOR mode
        self.get_logger().info(f"[STEP 0] Arm status: {self.latest_arm_status}")
        # Note From Guo:
        # the system control node set arm mode to OPEN_DOOR before sending the goal to this action server
        # But it may takes time for the `self.latest_arm_status` to be updated after the mode is set,
        # so we may receive the goal before we get the arm status update.
        # suggest: remove this check or add a short wait here to allow arm status to update before checking.
        if self.latest_arm_status != "OPEN_DOOR":
            result.success = False
            result.message = (
                f"Arm is not in OPEN_DOOR mode (current: {self.latest_arm_status}). "
                "Set mode first via /arm/set_mode."
            )
            self.get_logger().error(f"[STEP 0] ABORT: {result.message}")
            goal_handle.abort()
            return result

        # --- 1. Grab the latest ButtonInfo ---
        info = self.latest_button_info
        self.get_logger().info(
            f"[STEP 1] Checking ButtonInfo: received={info is not None}"
        )
        if info is not None:
            self.get_logger().info(
                f"[STEP 1] ButtonInfo: confidence={info.confidence:.2f}, "
                f"pose_xyzrpy={[f'{v:.4f}' for v in info.pose_xyzrpy]}, "
                f"is_pressable={info.is_pressable}"
            )

        if info is None:
            result.success = False
            result.message = "No ButtonInfo received — is the detector running?"
            self.get_logger().error(f"[STEP 1] ABORT: {result.message}")
            goal_handle.abort()
            return result

        # Age validation — reject stale button data (>10s)
        BUTTON_INFO_MAX_AGE_S = 10.0
        if self._latest_button_time is not None:
            age_s = (
                self.get_clock().now() - self._latest_button_time
            ).nanoseconds / 1e9
            if age_s > BUTTON_INFO_MAX_AGE_S:
                result.success = False
                result.message = (
                    f"ButtonInfo is stale ({age_s:.1f}s old, max {BUTTON_INFO_MAX_AGE_S}s) "
                    "— is the detector still running?"
                )
                self.get_logger().error(f"[STEP 1] ABORT: {result.message}")
                goal_handle.abort()
                return result

        pose_arr = np.array(info.pose_xyzrpy)
        pose_invalid = np.allclose(pose_arr, -1.0)

        if info.confidence == 0.0 and pose_invalid:
            result.success = False
            result.message = "No button detected by the detector"
            self.get_logger().error(f"[STEP 1] ABORT: {result.message}")
            goal_handle.abort()
            return result

        if pose_invalid:
            result.success = False
            result.message = (
                f"Button detected (confidence={info.confidence:.2f}) "
                "but pose is invalid — button may be too far or depth/TF failed"
            )
            self.get_logger().error(f"[STEP 1] ABORT: {result.message}")
            goal_handle.abort()
            return result

        # Log distance from base for debugging
        btn_xyz = np.array(info.pose_xyzrpy[:3])
        dist_from_base = float(np.linalg.norm(btn_xyz))
        self.get_logger().info(
            f"[STEP 1] Button distance from base: {dist_from_base:.4f}m, "
            f"xyz=[{btn_xyz[0]:.4f}, {btn_xyz[1]:.4f}, {btn_xyz[2]:.4f}], "
            f"is_pressable={info.is_pressable}"
        )

        if not info.is_pressable:
            result.success = False
            result.message = (
                f"Button detected (confidence={info.confidence:.2f}) "
                f"but out of reach (dist={dist_from_base:.3f}m)"
            )
            self.get_logger().error(f"[STEP 1] ABORT: {result.message}")
            goal_handle.abort()
            return result

        self.get_logger().info("[STEP 1] ButtonInfo valid and pressable")

        target_pose = self._button_info_to_pose(info)
        target_xyz = np.array(
            [
                target_pose.pose.position.x,
                target_pose.pose.position.y,
                target_pose.pose.position.z,
            ]
        )
        q = target_pose.pose.orientation
        self.get_logger().info(
            f"[STEP 1] Target pose: xyz=[{target_xyz[0]:.4f}, {target_xyz[1]:.4f}, {target_xyz[2]:.4f}] "
            f"quat=[{q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f}]"
        )

        # --- 2a. Move to target (button surface) — 5s timeout ---
        import copy

        PHASE_TIMEOUT = 5.0
        MIN_MOVE_TIME = 1.0  # wait at least 1s before checking velocity
        SPEED_THRESHOLD = 0.005  # m/s — consider stopped below this

        self.get_logger().info(
            f"[STEP 2a] Moving to target: x={target_xyz[0]:.4f} (timeout={PHASE_TIMEOUT}s)"
        )
        self.pose_pub.publish(target_pose)

        arrived = False
        start_t = time.time()
        while rclpy.ok() and (time.time() - start_t) < PHASE_TIMEOUT:
            time.sleep(FORCE_CHECK_RATE)
            force_mag = self.get_ee_force_magnitude()
            velocity = self.get_ee_velocity_magnitude()
            elapsed = time.time() - start_t

            velocity_str = f"{velocity:.4f}" if velocity is not None else "n/a"
            self.get_logger().debug(
                f"[MOVE] {elapsed:5.2f}s  force={force_mag:6.1f} N  velocity={velocity_str} m/s"
            )

            # handle cancel goal during move
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.stop_arm()
                result.success = False
                result.message = "Door open action canceled during move — stopping arm"
                return result

            if force_mag > FORCE_THRESHOLD:
                self.get_logger().warn(
                    f"[STEP 2a] Force exceeded during move ({force_mag:.1f} N) — stopping"
                )
                self.stop_arm()
                goal_handle.abort()
                result.success = False
                result.message = f"Force exceeded moving to target: {force_mag:.1f} N"
                return result

            # After min move time, if velocity near zero → arm has stopped → arrived
            if (
                elapsed > MIN_MOVE_TIME
                and velocity is not None
                and velocity < SPEED_THRESHOLD
            ):
                self.get_logger().info(
                    f"[STEP 2a] Arm stopped (velocity={velocity:.4f} m/s) — arrived ({elapsed:.2f}s)"
                )
                arrived = True
                break

        if not arrived:
            self.get_logger().warn(
                "[STEP 2a] Move timed out — proceeding to push anyway"
            )

        # --- 2b. Push along approach direction (EE z-axis = into button) ---
        push_pose = copy.deepcopy(target_pose)

        # Extract the approach direction from the target orientation
        # The tool z-axis points into the button surface
        q = target_pose.pose.orientation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        approach_dir = rot.as_matrix()[:, 2]  # z-axis column
        push_offset = approach_dir * PUSH_MAX

        push_pose.pose.position.x += float(push_offset[0])
        push_pose.pose.position.y += float(push_offset[1])
        push_pose.pose.position.z += float(push_offset[2])
        push_pose.header.stamp = self.get_clock().now().to_msg()

        self.get_logger().info(
            f"[STEP 2b] Pushing along approach dir=[{approach_dir[0]:.3f},{approach_dir[1]:.3f},{approach_dir[2]:.3f}]: "
            f"xyz=[{push_pose.pose.position.x:.4f},{push_pose.pose.position.y:.4f},{push_pose.pose.position.z:.4f}] "
            f"(+{PUSH_MAX * 100:.0f}cm overshoot), force_limit={FORCE_THRESHOLD}N"
        )
        self.pose_pub.publish(push_pose)

        contact = False
        start_t = time.time()
        while rclpy.ok() and (time.time() - start_t) < PHASE_TIMEOUT:
            time.sleep(FORCE_CHECK_RATE)
            force_mag = self.get_ee_force_magnitude()
            elapsed = time.time() - start_t
            self.get_logger().debug(
                f"[PUSH] {elapsed:5.2f}s  force={force_mag:6.1f} N  (threshold={FORCE_THRESHOLD})"
            )

            if force_mag > FORCE_THRESHOLD:
                self.get_logger().info(
                    f"[STEP 2b] Contact! Force: {force_mag:.1f} N after {elapsed:.2f}s — stopping arm"
                )
                self.stop_arm()
                contact = True
                break
            # handle cancel goal during move
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Door open action canceled during move — stopping arm"
                return result

        if not contact:
            elapsed = time.time() - start_t
            self.get_logger().warn(
                f"[STEP 2b] Timed out after {elapsed:.2f}s (no force > {FORCE_THRESHOLD}N) — stopping arm"
            )
            self.stop_arm()

        # --- 4. Retract ---
        self.get_logger().info("[STEP 4] Retracting arm via /arm/reach_preset...")

        if not self._reach_preset_client.wait_for_server(timeout_sec=5.0):
            result.success = False
            result.message = "Push done but /arm/reach_preset not available for retract"
            self.get_logger().error(f"[STEP 4] ABORT: {result.message}")
            goal_handle.abort()
            return result

        retract_goal = ReachPreset.Goal()
        retract_goal.preset = ReachPreset.Goal.PRESET_RETRACT
        self.get_logger().info("[STEP 4] Sending retract goal...")
        retract_future = self._reach_preset_client.send_goal_async(retract_goal)

        # Wait for goal acceptance without rclpy.spin_until_future_complete
        deadline = time.time() + 5.0
        while not retract_future.done() and time.time() < deadline:
            time.sleep(0.05)
        retract_handle = retract_future.result()
        if retract_handle is None or not retract_handle.accepted:
            result.success = False
            result.message = "Retract goal rejected"
            self.get_logger().error(f"[STEP 4] ABORT: {result.message}")
            goal_handle.abort()
            return result

        self.get_logger().info(
            "[STEP 4] Retract goal accepted — waiting for completion..."
        )
        result_future = retract_handle.get_result_async()

        # Wait for retract to finish
        deadline = time.time() + 30.0
        while not result_future.done() and time.time() < deadline:
            # handle cancel goal during move
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "Door open action canceled during move — stopping arm"
                return result
            time.sleep(0.05)
        self.get_logger().info("[STEP 4] Retract complete")

        # --- Done ---
        goal_handle.succeed()
        result.success = True
        result.message = "Button push complete, arm retracted"
        self.get_logger().info(f"=== ACTION SUCCEEDED: {result.message} ===")
        return result


def main():
    rclpy.init()
    node = ButtonPushController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
