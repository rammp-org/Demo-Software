#!/usr/bin/env python3
"""
Button push controller for Demo-Software.

Exposes the /arm/door/open action server. When the action is called
(by the behavior tree), it:
  1. Waits for a stable target pose from button_detector
  2. Commands the arm to push the button via /arm/cmu/cartesian_pose
  3. Monitors /arm/ee_force — stops on contact via zero twist
  4. Retracts the arm via /arm/reach_preset

The behavior tree sets the arm to OPEN_DOOR mode before calling this action.
"""
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_group import ReentrantCallbackGroup
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from sensor_msgs.msg import JointState
from cmu_door_opener_interfaces.action import OpenDoor
from arm_interfaces.action import ReachPreset


# Button push parameters
POSE_STABILITY_THRESHOLD = 0.01  # meters — pose must settle within this
PUSH_EXTRA = 0.02                # meters to overshoot past the button surface
POSE_SETTLE_TIMEOUT = 10.0       # seconds to wait for a stable pose
FORCE_THRESHOLD = 20.0           # Newtons — contact detection threshold
PUSH_TIMEOUT = 10.0              # seconds — max time for push motion
FORCE_CHECK_RATE = 0.02          # seconds between force checks


class ButtonPushController(Node):
    def __init__(self):
        super().__init__('button_push_controller')

        self._cb_group = ReentrantCallbackGroup()

        # Latest target pose from vision node
        self.latest_pose = None
        self.create_subscription(
            PoseStamped, '/button/target_pose', self._cb_target_pose, 10
        )

        # End-effector force from arm_driver
        self.latest_ee_force = None
        self.create_subscription(
            Vector3, '/arm/ee_force', self._cb_ee_force, 10
        )

        # Publisher to command arm cartesian pose
        self.pose_pub = self.create_publisher(
            PoseStamped, '/arm/cmu/cartesian_pose', 10
        )

        # Publisher to stop the arm (zero twist)
        self.twist_pub = self.create_publisher(
            Twist, '/arm/cmu/twist', 10
        )

        # Action client to retract the arm after push
        self._reach_preset_client = ActionClient(
            self, ReachPreset, '/arm/reach_preset',
            callback_group=self._cb_group
        )

        # Action server: /arm/door/open
        self._action_server = ActionServer(
            self, OpenDoor, '/arm/door/open',
            self._execute_open_door,
            callback_group=self._cb_group
        )

        self.get_logger().info('ButtonPushController ready — waiting for /arm/door/open')

    def _cb_target_pose(self, msg: PoseStamped):
        self.latest_pose = msg

    def _cb_ee_force(self, msg: Vector3):
        self.latest_ee_force = np.array([msg.x, msg.y, msg.z])

    def get_ee_force_magnitude(self):
        if self.latest_ee_force is None:
            return 0.0
        return float(np.linalg.norm(self.latest_ee_force))

    def stop_arm(self):
        zero_twist = Twist()
        self.twist_pub.publish(zero_twist)

    def _execute_open_door(self, goal_handle):
        """Action callback — runs the full button push sequence."""
        feedback = OpenDoor.Feedback()
        result = OpenDoor.Result()

        # --- 1. Wait for stable pose ---
        feedback.status = 'Waiting for stable button pose...'
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(feedback.status)

        prev_xyz = None
        stable_pose = None
        start_t = time.time()
        while rclpy.ok() and (time.time() - start_t) < POSE_SETTLE_TIMEOUT:
            if self.latest_pose is not None:
                p = self.latest_pose.pose.position
                xyz = np.array([p.x, p.y, p.z])
                if prev_xyz is not None and np.linalg.norm(xyz - prev_xyz) < POSE_STABILITY_THRESHOLD:
                    stable_pose = self.latest_pose
                    break
                prev_xyz = xyz
            time.sleep(0.1)

        if stable_pose is None:
            result.success = False
            result.message = 'Timed out waiting for stable /button/target_pose'
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        p = stable_pose.pose.position
        self.get_logger().info(f'Stable pose: [{p.x:.4f}, {p.y:.4f}, {p.z:.4f}]')

        # --- 2. Push the button ---
        feedback.status = 'Pushing button...'
        goal_handle.publish_feedback(feedback)

        push_pose = PoseStamped()
        push_pose.header = stable_pose.header
        push_pose.pose = stable_pose.pose
        push_pose.pose.position.x += PUSH_EXTRA

        self.pose_pub.publish(push_pose)

        # --- 3. Monitor force — stop on contact ---
        contact = False
        start_t = time.time()
        while rclpy.ok() and (time.time() - start_t) < PUSH_TIMEOUT:
            force_mag = self.get_ee_force_magnitude()
            if force_mag > FORCE_THRESHOLD:
                self.get_logger().info(f'Contact! Force: {force_mag:.1f} N')
                self.stop_arm()
                contact = True
                break
            time.sleep(FORCE_CHECK_RATE)

        if not contact:
            self.get_logger().warn('Push timed out — stopping arm')
            self.stop_arm()

        # --- 4. Retract ---
        feedback.status = 'Retracting arm...'
        goal_handle.publish_feedback(feedback)
        self.get_logger().info(feedback.status)

        if not self._reach_preset_client.wait_for_server(timeout_sec=5.0):
            result.success = False
            result.message = 'Push done but /arm/reach_preset not available for retract'
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        retract_goal = ReachPreset.Goal()
        retract_goal.preset = ReachPreset.Goal.PRESET_RETRACT
        retract_future = self._reach_preset_client.send_goal_async(retract_goal)

        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self, retract_future, timeout_sec=5.0)
        retract_handle = retract_future.result()
        if retract_handle is None or not retract_handle.accepted:
            result.success = False
            result.message = 'Retract goal rejected'
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        # Wait for retract to finish
        result_future = retract_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

        # --- Done ---
        goal_handle.succeed()
        result.success = True
        result.message = 'Button push complete, arm retracted'
        self.get_logger().info(result.message)
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


if __name__ == '__main__':
    main()
