#!/usr/bin/env python3
"""
Button push controller for CMU door opener.

Exposes the /arm/door/open action server (DoorOpen).
When called, it takes the **latest** ButtonInfo from the detector and:
  1. Commands the arm to push the button via /arm/cmu/cartesian_pose
  2. Monitors /arm/ee_force — stops on contact via zero twist
  3. Publishes feedback with distance_to_button
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
from scipy.spatial.transform import Rotation

from cmu_door_opener_interfaces.action import DoorOpen
from cmu_door_opener_interfaces.msg import ButtonInfo
from arm_interfaces.action import ReachPreset


# Button push parameters
PUSH_EXTRA = 0.02                # meters to overshoot past the button surface
FORCE_THRESHOLD = 20.0           # Newtons — contact detection threshold
PUSH_TIMEOUT = 10.0              # seconds — max time for push motion
FORCE_CHECK_RATE = 0.02          # seconds between force checks


class ButtonPushController(Node):
    def __init__(self):
        super().__init__('button_push_controller')

        self._cb_group = ReentrantCallbackGroup()

        # Latest ButtonInfo from detector
        self.latest_button_info = None
        self.create_subscription(
            ButtonInfo, '/arm/door/button_info', self._cb_button_info, 10
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
            self, DoorOpen, '/arm/door/open',
            self._execute_open_door,
            callback_group=self._cb_group
        )

        self.get_logger().info('ButtonPushController ready — waiting for /arm/door/open')

    def _cb_button_info(self, msg: ButtonInfo):
        self.latest_button_info = msg

    def _cb_ee_force(self, msg: Vector3):
        self.latest_ee_force = np.array([msg.x, msg.y, msg.z])

    def get_ee_force_magnitude(self):
        if self.latest_ee_force is None:
            return 0.0
        return float(np.linalg.norm(self.latest_ee_force))

    def stop_arm(self):
        zero_twist = Twist()
        self.twist_pub.publish(zero_twist)

    def _button_info_to_pose(self, info: ButtonInfo) -> PoseStamped:
        """Convert ButtonInfo pose_xyzrpy to a PoseStamped for arm command."""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'base_link'

        x, y, z = info.pose_xyzrpy[0], info.pose_xyzrpy[1], info.pose_xyzrpy[2]
        roll, pitch, yaw = info.pose_xyzrpy[3], info.pose_xyzrpy[4], info.pose_xyzrpy[5]

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        rot = Rotation.from_euler('xyz', [roll, pitch, yaw])
        quat = rot.as_quat()  # [x, y, z, w]
        pose.pose.orientation.x = float(quat[0])
        pose.pose.orientation.y = float(quat[1])
        pose.pose.orientation.z = float(quat[2])
        pose.pose.orientation.w = float(quat[3])

        return pose

    def _execute_open_door(self, goal_handle):
        """Action callback — runs the full button push sequence."""
        feedback = DoorOpen.Feedback()
        result = DoorOpen.Result()

        # --- 1. Grab the latest ButtonInfo ---
        info = self.latest_button_info
        if info is None:
            result.success = False
            result.message = 'No ButtonInfo received — is the detector running?'
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        # Check which state the ButtonInfo is in:
        #   State 1: no button found (confidence == 0, pose all -1)
        #   State 2: button detected but not pressable (confidence > 0, pose all -1)
        #   State 3: button detected and pressable (confidence > 0, valid pose)
        pose_arr = np.array(info.pose_xyzrpy)
        pose_invalid = np.allclose(pose_arr, -1.0)

        if info.confidence == 0.0 and pose_invalid:
            result.success = False
            result.message = 'No button detected by the detector'
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        if pose_invalid:
            result.success = False
            result.message = (
                f'Button detected (confidence={info.confidence:.2f}) '
                'but pose is invalid — button may be too far or depth/TF failed'
            )
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        if not info.is_pressable:
            result.success = False
            result.message = (
                f'Button detected (confidence={info.confidence:.2f}) '
                'but is not pressable (IK not solvable)'
            )
            self.get_logger().error(result.message)
            goal_handle.abort()
            return result

        target_pose = self._button_info_to_pose(info)
        target_xyz = np.array([
            target_pose.pose.position.x,
            target_pose.pose.position.y,
            target_pose.pose.position.z,
        ])

        self.get_logger().info(
            f'Target pose: [{target_xyz[0]:.4f}, {target_xyz[1]:.4f}, {target_xyz[2]:.4f}]'
        )

        # --- 2. Push the button ---
        push_pose = PoseStamped()
        push_pose.header = target_pose.header
        push_pose.pose = target_pose.pose
        # Overshoot along the approach direction (x for now, same as before)
        push_pose.pose.position.x += PUSH_EXTRA

        self.pose_pub.publish(push_pose)

        # --- 3. Monitor force — stop on contact, publish distance feedback ---
        contact = False
        start_t = time.time()
        while rclpy.ok() and (time.time() - start_t) < PUSH_TIMEOUT:
            force_mag = self.get_ee_force_magnitude()

            # Publish distance feedback (approximate — from target, not live EE pos)
            feedback.distance_to_button = float(PUSH_EXTRA)  # simplified; refine with live EE
            goal_handle.publish_feedback(feedback)

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
        self.get_logger().info('Retracting arm...')

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
