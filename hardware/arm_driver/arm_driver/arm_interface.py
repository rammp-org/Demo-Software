# Author: Jimmy Wu, Rajat Kumar Jenamani

# Python 3.10 removed these aliases from collections; kortex_api 2.6.0 still references them
import collections
import collections.abc
import enum
import math
import os
import subprocess
import threading
import time

import numpy as np
from scipy.spatial.transform import Rotation as R

collections.MutableMapping = collections.abc.MutableMapping
collections.MutableSequence = collections.abc.MutableSequence
collections.MutableSet = collections.abc.MutableSet
collections.Mapping = collections.abc.Mapping
collections.Sequence = collections.abc.Sequence
collections.Callable = collections.abc.Callable

try:
    from kortex_api.autogen.client_stubs.ActuatorConfigClientRpc import (
        ActuatorConfigClient,
    )
    from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
    from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient
    from kortex_api.autogen.client_stubs.ControlConfigClientRpc import (
        ControlConfigClient,
    )
    from kortex_api.autogen.client_stubs.DeviceConfigClientRpc import DeviceConfigClient
    from kortex_api.autogen.client_stubs.DeviceManagerClientRpc import (
        DeviceManagerClient,
    )
    from kortex_api.autogen.messages import (
        ActuatorConfig_pb2,
        Base_pb2,
        BaseCyclic_pb2,
        Common_pb2,
        ControlConfig_pb2,
        DeviceConfig_pb2,
        Session_pb2,
    )
    from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
    from kortex_api.SessionManager import SessionManager
    from kortex_api.TCPTransport import TCPTransport
    from kortex_api.UDPTransport import UDPTransport
except ModuleNotFoundError:
    pass


class SpeedPreset(enum.IntEnum):
    DEFAULT = -1  # sentinel for hardware defaults (no soft limits applied)
    LOW = 0
    MEDIUM = 1
    HIGH = 2
    MAX = 3  # sentinel for maximum possible limits (equal to hard limits)


class DeviceConnection:
    IP_ADDRESS = "192.168.1.10"
    TCP_PORT = 10000
    UDP_PORT = 10001

    @staticmethod
    def createTcpConnection():
        return DeviceConnection(port=DeviceConnection.TCP_PORT)

    @staticmethod
    def createUdpConnection():
        return DeviceConnection(port=DeviceConnection.UDP_PORT)

    def __init__(
        self, ip_address=IP_ADDRESS, port=TCP_PORT, credentials=("admin", "admin")
    ):
        self.ip_address = ip_address
        self.port = port
        self.credentials = credentials
        self.session_manager = None
        self.transport = (
            TCPTransport() if port == DeviceConnection.TCP_PORT else UDPTransport()
        )
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

    def __enter__(self):
        self.transport.connect(self.ip_address, self.port)
        if self.credentials[0] != "":
            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.credentials[0]
            session_info.password = self.credentials[1]
            session_info.session_inactivity_timeout = 10000  # (milliseconds)
            session_info.connection_inactivity_timeout = 2000  # (milliseconds)
            self.session_manager = SessionManager(self.router)
            print("Logging as", self.credentials[0], "on device", self.ip_address)
            self.session_manager.CreateSession(session_info)
        return self.router

    def __exit__(self, *_):
        if self.session_manager is not None:
            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000
            self.session_manager.CloseSession(router_options)
        self.transport.disconnect()


class KinovaArm:
    ACTION_TIMEOUT_DURATION = 20

    def __init__(self):
        # Check whether arm is connected
        try:
            subprocess.run(
                ["ping", "-c", "1", "192.168.1.10"],
                check=True,
                timeout=1,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        except subprocess.TimeoutExpired as e:
            raise Exception("Could not communicate with arm") from e

        # Lock file to enforce single instance
        self.lock_file = "/tmp/kinova.lock"
        if os.path.exists(self.lock_file):
            with open(self.lock_file, "r") as f:
                pid = int(f.read().strip())
            try:
                os.kill(pid, 0)
            except OSError:
                print(f"Removing stale lock file (PID {pid})")
                os.remove(self.lock_file)
            else:
                raise Exception(
                    f"Another instance of the arm is already running (PID {pid})"
                )
        with open(self.lock_file, "w") as f:
            f.write(str(os.getpid()))

        # General Kortex setup
        self.tcp_connection = DeviceConnection.createTcpConnection()
        self.udp_connection = DeviceConnection.createUdpConnection()
        self.base = BaseClient(self.tcp_connection.__enter__())
        self.base_cyclic = BaseCyclicClient(self.udp_connection.__enter__())

        self.device_config = DeviceConfigClient(self.base.router)
        self.actuator_config = ActuatorConfigClient(self.base.router)
        self.actuator_count = self.base.GetActuatorCount().count
        self.control_config = ControlConfigClient(self.base.router)
        device_manager = DeviceManagerClient(self.base.router)
        device_handles = device_manager.ReadAllDevices()
        self.actuator_device_ids = [
            handle.device_identifier
            for handle in device_handles.device_handle
            if handle.device_type
            in [Common_pb2.BIG_ACTUATOR, Common_pb2.SMALL_ACTUATOR]
        ]
        self.send_options = RouterClientSendOptions()
        self.send_options.timeout_ms = 50
        self.control_send_options = RouterClientSendOptions()
        self.control_send_options.timeout_ms = 200

        # clear faults
        self.clear_faults()

        # Command and feedback setup
        self.base_command = BaseCyclic_pb2.Command()
        for _ in range(self.actuator_count):
            self.base_command.actuators.add()
        self.motor_cmd = self.base_command.interconnect.gripper_command.motor_cmd.add()
        self.base_feedback = BaseCyclic_pb2.Feedback()

        # Make sure actuators are in position mode
        control_mode_message = ActuatorConfig_pb2.ControlModeInformation()
        control_mode_message.control_mode = ActuatorConfig_pb2.ControlMode.Value(
            "POSITION"
        )
        for device_id in self.actuator_device_ids:
            self.actuator_config.SetControlMode(control_mode_message, device_id)

        # Make sure arm is in high-level servoing mode
        self.set_arm_servoing_mode("high")

        print("Setting joint following error threshold to 10 degrees")
        # increase joint following error threshold to 10 degrees (upper hard limit)
        joint_following_safety_threshold = DeviceConfig_pb2.SafetyThreshold()
        joint_following_safety_threshold.handle.identifier = (
            ActuatorConfig_pb2.SafetyIdentifierBankA.Value("FOLLOWING_ERROR")
        )
        joint_following_safety_threshold.value = 10
        for device_id in self.actuator_device_ids:
            self.device_config.SetSafetyErrorThreshold(
                joint_following_safety_threshold, device_id
            )

        # Tracks speed presets
        self.speed_preset = SpeedPreset.MEDIUM
        self.choose_from_speed_presets(self.speed_preset)

        # Action topic notifications
        self.end_or_abort_event = threading.Event()
        self.end_or_abort_event.set()

        def check_for_end_or_abort(e):
            def check(notification, e=e):
                # print("EVENT : " + Base_pb2.ActionEvent.Name(notification.action_event))
                if notification.action_event in (
                    Base_pb2.ACTION_END,
                    Base_pb2.ACTION_ABORT,
                ):
                    e.set()

            return check

        self.notification_handle = self.base.OnNotificationActionTopic(
            check_for_end_or_abort(self.end_or_abort_event),
            Base_pb2.NotificationOptions(),
        )

    def set_tool(self, tool):
        print("Does not affect current controller, but setting tool to", tool)

    def disconnect(self):
        self.base.Unsubscribe(
            self.notification_handle
        )  # Rajat ToDo: Check if this is necessary before switching to low-level servoing mode
        self.tcp_connection.__exit__()
        self.udp_connection.__exit__()
        os.remove(self.lock_file)

    def ready(self):
        return self.end_or_abort_event.is_set()

    def wait_ready(self):
        return self.end_or_abort_event.wait(KinovaArm.ACTION_TIMEOUT_DURATION)

    def set_arm_servoing_mode(self, mode):
        if mode == "high":
            base_servo_mode = Base_pb2.ServoingModeInformation()
            base_servo_mode.servoing_mode = Base_pb2.SINGLE_LEVEL_SERVOING
            self.base.SetServoingMode(base_servo_mode)
        else:
            raise ValueError(
                "Invalid servoing mode (low level servoing mode removed from this version)"
            )

    def _execute_reference_action(self, action_name, blocking=True):
        # Retrieve reference action
        opts = self.control_send_options
        action_type = Base_pb2.RequestedActionType()
        action_type.action_type = Base_pb2.REACH_JOINT_ANGLES
        action_list = self.base.ReadAllActions(action_type, options=opts)
        action_handle = None
        for action in action_list.action_list:
            if action.name == action_name:
                action_handle = action.handle
        if action_handle is None:
            return

        # Execute action
        self.end_or_abort_event.clear()
        self.base.ExecuteActionFromReference(action_handle, options=opts)
        if blocking:
            self.end_or_abort_event.wait(KinovaArm.ACTION_TIMEOUT_DURATION)

    def home(self, blocking=True):
        self._execute_reference_action("Home", blocking=blocking)

    def retract(self, blocking=True):
        self._execute_reference_action("Retract", blocking=blocking)

    def zero(self, blocking=True):
        self._execute_reference_action("Zero", blocking=blocking)

    def cup_stabilize(self, blocking=True):
        self._execute_reference_action("Home", blocking=blocking)

    def send_twist(self, linear_xyz, angular_xyz):
        """Send a Cartesian twist velocity command (SINGLE_LEVEL_SERVOING).

        Uses CARTESIAN_REFERENCE_FRAME_MIXED: linear velocity in the base frame,
        angular velocity in the tool frame — standard for joystick teleoperation.

        Args:
            linear_xyz: Linear velocity [vx, vy, vz] in m/s.
            angular_xyz: Angular velocity [wx, wy, wz] in rad/s.
        """
        command = Base_pb2.TwistCommand()
        command.reference_frame = Base_pb2.CARTESIAN_REFERENCE_FRAME_MIXED
        command.duration = 0  # 0 = run until next command
        command.twist.linear_x = linear_xyz[0]
        command.twist.linear_y = linear_xyz[1]
        command.twist.linear_z = linear_xyz[2]
        command.twist.angular_x = math.degrees(angular_xyz[0])
        command.twist.angular_y = math.degrees(angular_xyz[1])
        command.twist.angular_z = math.degrees(angular_xyz[2])
        self.base.SendTwistCommand(command, options=self.control_send_options)

    def set_intermediate_zero_config(self):
        intermediate_zero_config = [0.0, 0.0, -3.12, 0.0, 0.0, 0.0, 0.0]
        self.move_angular(intermediate_zero_config)

    def get_ee_force(self):
        base_feedback = self.base_cyclic.RefreshFeedback(options=self.send_options)
        ee_force = np.array(
            [
                base_feedback.base.tool_external_wrench_force_x,
                base_feedback.base.tool_external_wrench_force_y,
                base_feedback.base.tool_external_wrench_force_z,
            ]
        )
        return ee_force

    def get_state(self):
        base_feedback = self.base_cyclic.RefreshFeedback(options=self.send_options)

        q, dq, tau = (
            np.zeros(self.actuator_count),
            np.zeros(self.actuator_count),
            np.zeros(self.actuator_count),
        )

        ee_pos, ee_vel, ee_force = (
            np.zeros(7),
            np.zeros(6),
            np.zeros(
                3
            ),  # We are only using force from the end effector wrench, not torque, so this is 3D not 6D
        )

        # Robot joint state
        for i in range(self.actuator_count):
            q[i] = math.radians(base_feedback.actuators[i].position)
            if q[i] > np.pi:
                q[i] -= 2 * np.pi
            dq[i] = math.radians(base_feedback.actuators[i].velocity)
            tau[i] = -base_feedback.actuators[i].torque

        # Robot cartesian state
        ee_pos[:3] = (
            base_feedback.base.tool_pose_x,
            base_feedback.base.tool_pose_y,
            base_feedback.base.tool_pose_z,
        )
        tool_rot = np.array(
            [
                base_feedback.base.tool_pose_theta_x,
                base_feedback.base.tool_pose_theta_y,
                base_feedback.base.tool_pose_theta_z,
            ]
        )
        ee_pos[3:] = R.from_euler("xyz", np.deg2rad(tool_rot)).as_quat()

        ee_vel[:3] = (
            base_feedback.base.tool_twist_linear_x,
            base_feedback.base.tool_twist_linear_y,
            base_feedback.base.tool_twist_linear_z,
        )
        # Kortex reports angular velocity in the base frame (deg/s).
        # Rotate into tool frame to match SendTwistCommand's
        # CARTESIAN_REFERENCE_FRAME_MIXED convention (angular in tool frame).
        angular_vel_base_degs = np.array(
            [
                base_feedback.base.tool_twist_angular_x,
                base_feedback.base.tool_twist_angular_y,
                base_feedback.base.tool_twist_angular_z,
            ]
        )
        R_tool_in_base = R.from_euler("xyz", np.deg2rad(tool_rot))
        ee_vel[3:] = R_tool_in_base.inv().apply(angular_vel_base_degs)

        ee_force[:3] = (
            base_feedback.base.tool_external_wrench_force_x,
            base_feedback.base.tool_external_wrench_force_y,
            base_feedback.base.tool_external_wrench_force_z,
        )

        # print("End Effector Force: ", ee_force)

        gripper_pos = (
            base_feedback.interconnect.gripper_feedback.motor[0].position / 100.0
        )

        return {
            "position": q,
            "velocity": dq,
            "effort": tau,
            "ee_pos": ee_pos,
            "ee_vel": ee_vel,
            "ee_force": ee_force,
            "gripper_pos": gripper_pos,
        }

    def move_angular_trajectory(self, trajectory_joint_angles, blocking=True):
        opts = self.control_send_options
        assert len(trajectory_joint_angles) > 0, "Invalid trajectory"
        assert (
            len(trajectory_joint_angles[0]) == self.actuator_count
        ), "Invalid number of joint angles"

        jointPoses = [
            [math.degrees(angle) for angle in jointPose]
            for jointPose in trajectory_joint_angles
        ]

        waypoints = Base_pb2.WaypointList()
        waypoints.duration = 0.0
        waypoints.use_optimal_blending = False

        index = 0
        for jointPose in jointPoses:
            waypoint = waypoints.waypoints.add()
            waypoint.name = "waypoint_" + str(index)
            waypoint.angular_waypoint.angles.extend(jointPose)
            waypoint.angular_waypoint.duration = 0.5
            index = index + 1

        result = self.base.ValidateWaypointList(waypoints, options=opts)
        if len(result.trajectory_error_report.trajectory_error_elements) == 0:
            print("Reaching angular pose trajectory...")

            self.end_or_abort_event.clear()
            self.base.ExecuteWaypointTrajectory(waypoints, options=opts)

            if blocking:
                print("Waiting for trajectory to finish ...")
                finished = self.end_or_abort_event.wait(
                    KinovaArm.ACTION_TIMEOUT_DURATION
                )
                if finished:
                    print("Angular movement completed")
                else:
                    print("Timeout on action notification wait")
        else:
            print("Error found in trajectory")
            print(result.trajectory_error_report)

    def move_angular(self, joint_angles, blocking=True):
        assert (
            len(joint_angles) == self.actuator_count
        ), "Invalid number of joint angles"

        # Create action
        action = Base_pb2.Action()
        for i in range(self.actuator_count):
            joint_angle = action.reach_joint_angles.joint_angles.joint_angles.add()
            joint_angle.joint_identifier = i
            joint_angle.value = math.degrees(joint_angles[i])
        self.end_or_abort_event.clear()
        self.base.ExecuteAction(action, options=self.control_send_options)
        if blocking:
            self.end_or_abort_event.wait(KinovaArm.ACTION_TIMEOUT_DURATION)
            # read states and check if the arm actually reached the desired position
            current_state = self.get_state()
            q = current_state["position"]
            # find error while wrapping angles
            error = np.degrees(q - joint_angles)
            while np.any(error > 180) or np.any(error < -180):
                error = np.where(error > 180, error - 360, error)
                error = np.where(error < -180, error + 360, error)

            if np.any(np.abs(error) > 5):  # 5 degrees
                self.stop()
                raise RuntimeError(
                    f"Arm did not reach desired angular position; "
                    f"max error {np.max(np.abs(error)):.1f} deg exceeds 5 deg threshold"
                )

    def move_cartesian(self, xyz, xyz_quat, blocking=True):
        theta_xyz = R.from_quat(xyz_quat).as_euler("xyz")

        # Create action
        action = Base_pb2.Action()
        cartesian_pose = action.reach_pose.target_pose
        cartesian_pose.x = xyz[0]
        cartesian_pose.y = xyz[1]
        cartesian_pose.z = xyz[2]
        cartesian_pose.theta_x = math.degrees(theta_xyz[0])
        cartesian_pose.theta_y = math.degrees(theta_xyz[1])
        cartesian_pose.theta_z = math.degrees(theta_xyz[2])
        self.end_or_abort_event.clear()
        self.base.ExecuteAction(action, options=self.control_send_options)
        if blocking:
            self.end_or_abort_event.wait(KinovaArm.ACTION_TIMEOUT_DURATION)
            # read states and check if the arm actually reached the desired position
            current_state = self.get_state()
            x = current_state["ee_pos"]
            if not np.allclose(x[:3], xyz, atol=0.01):  # 1 cm
                self.stop()
                raise RuntimeError(
                    f"Arm did not reach desired Cartesian position; "
                    f"actual={x[:3].tolist()}, target={xyz}"
                )

    def _gripper_position_command(self, value, blocking=True, timeout=1.0):
        b = self.base
        opts = self.control_send_options
        # Send gripper command
        gripper_command = Base_pb2.GripperCommand()
        gripper_command.mode = Base_pb2.GRIPPER_POSITION
        finger = gripper_command.gripper.finger.add()
        finger.value = value
        b.SendGripperCommand(gripper_command, options=opts)

        if blocking:
            # Wait for reported position to match value
            gripper_request = Base_pb2.GripperRequest()
            gripper_request.mode = Base_pb2.GRIPPER_POSITION
            start_time = time.perf_counter()
            while time.perf_counter() - start_time < timeout:
                gripper_measure = b.GetMeasuredGripperMovement(
                    gripper_request, options=opts
                )
                if abs(value - gripper_measure.finger[0].value) < 0.01:
                    break
                time.sleep(0.01)

    def open_gripper(self, blocking=True):
        self._gripper_position_command(0, blocking)

    def close_gripper(self, blocking=True):
        self._gripper_position_command(1, blocking)

    def set_joint_limits(
        self,
        speed_limits=(60, 60, 60, 60, 60, 60, 60),
        acceleration_limits=(80, 80, 80, 80, 80, 80, 80),
        cartesian=False,
    ):
        cc = self.control_config
        opts = self.control_send_options
        if cartesian:
            joint_speed_soft_limits = ControlConfig_pb2.JointSpeedSoftLimits()
            joint_speed_soft_limits.control_mode = (
                ControlConfig_pb2.CARTESIAN_TRAJECTORY
            )
            joint_speed_soft_limits.joint_speed_soft_limits.extend(speed_limits)
            cc.SetJointSpeedSoftLimits(joint_speed_soft_limits, options=opts)
        else:
            joint_speed_soft_limits = ControlConfig_pb2.JointSpeedSoftLimits()
            joint_speed_soft_limits.control_mode = ControlConfig_pb2.ANGULAR_TRAJECTORY
            joint_speed_soft_limits.joint_speed_soft_limits.extend(speed_limits)
            cc.SetJointSpeedSoftLimits(joint_speed_soft_limits, options=opts)
            joint_acceleration_soft_limits = (
                ControlConfig_pb2.JointAccelerationSoftLimits()
            )
            joint_acceleration_soft_limits.control_mode = (
                ControlConfig_pb2.ANGULAR_TRAJECTORY
            )
            joint_acceleration_soft_limits.joint_acceleration_soft_limits.extend(
                acceleration_limits
            )
            cc.SetJointAccelerationSoftLimits(
                joint_acceleration_soft_limits, options=opts
            )

    def choose_from_speed_presets(self, speed_preset: SpeedPreset):
        cc = self.control_config
        opts = self.control_send_options
        if not isinstance(speed_preset, SpeedPreset) or speed_preset in [
            SpeedPreset.DEFAULT,
            SpeedPreset.MAX,
        ]:
            raise ValueError(
                "speed_preset must be SpeedPreset type and not DEFAULT or MAX"
            )

        if speed_preset == SpeedPreset.LOW:
            speed_limits = [12.5, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5]
            acceleration_limits = [25, 25, 25, 25, 25, 25, 25]
        elif speed_preset == SpeedPreset.MEDIUM:
            speed_limits = [25, 25, 25, 25, 25, 25, 25]
            acceleration_limits = [50, 50, 50, 50, 50, 50, 50]
        elif speed_preset == SpeedPreset.HIGH:
            speed_limits = [50, 50, 50, 50, 50, 50, 50]
            acceleration_limits = [100, 100, 100, 100, 100, 100, 100]
        else:
            raise ValueError("Invalid speed preset")

        self.speed_preset = speed_preset

        self.set_joint_limits(speed_limits, acceleration_limits)

        # Also apply to CARTESIAN_JOYSTICK, which governs SendTwistCommand
        # (ANGULAR_TRAJECTORY limits set above do not affect Twist velocity control)
        cartesian_joystick_limits = ControlConfig_pb2.JointSpeedSoftLimits()
        cartesian_joystick_limits.control_mode = ControlConfig_pb2.CARTESIAN_JOYSTICK
        cartesian_joystick_limits.joint_speed_soft_limits.extend(speed_limits)
        cc.SetJointSpeedSoftLimits(cartesian_joystick_limits, options=opts)

    def get_speed_preset(self):
        return self.speed_preset

    def set_max_joint_limits(self):
        cc = self.control_config
        opts = self.control_send_options
        self.speed_preset = SpeedPreset.MAX
        speed_limits = cc.GetKinematicHardLimits(options=opts).joint_speed_limits
        acceleration_limits = cc.GetKinematicHardLimits(
            options=opts
        ).joint_acceleration_limits
        self.set_joint_limits(speed_limits, acceleration_limits)

        cartesian_joystick_limits = ControlConfig_pb2.JointSpeedSoftLimits()
        cartesian_joystick_limits.control_mode = ControlConfig_pb2.CARTESIAN_JOYSTICK
        cartesian_joystick_limits.joint_speed_soft_limits.extend(speed_limits)
        cc.SetJointSpeedSoftLimits(cartesian_joystick_limits, options=opts)

    def get_joint_limits(self):
        cc = self.control_config
        opts = self.control_send_options
        joint_limits = []
        control_mode_information = ControlConfig_pb2.ControlModeInformation()
        for control_mode in [
            ControlConfig_pb2.ANGULAR_JOYSTICK,
            ControlConfig_pb2.CARTESIAN_JOYSTICK,
            ControlConfig_pb2.ANGULAR_TRAJECTORY,
            ControlConfig_pb2.CARTESIAN_TRAJECTORY,
            ControlConfig_pb2.CARTESIAN_WAYPOINT_TRAJECTORY,
        ]:
            control_mode_information.control_mode = control_mode
            joint_limits.append(
                cc.GetKinematicSoftLimits(control_mode_information, options=opts)
            )
        return joint_limits

    def reset_joint_limits(self):
        cc = self.control_config
        opts = self.control_send_options
        self.speed_preset = SpeedPreset.DEFAULT
        control_mode_information = ControlConfig_pb2.ControlModeInformation()
        for control_mode in [
            ControlConfig_pb2.ANGULAR_JOYSTICK,
            ControlConfig_pb2.CARTESIAN_JOYSTICK,
            ControlConfig_pb2.ANGULAR_TRAJECTORY,
            ControlConfig_pb2.CARTESIAN_TRAJECTORY,
            ControlConfig_pb2.CARTESIAN_WAYPOINT_TRAJECTORY,
        ]:
            control_mode_information.control_mode = control_mode
            cc.ResetJointSpeedSoftLimits(control_mode_information, options=opts)
        for control_mode in [
            ControlConfig_pb2.ANGULAR_JOYSTICK,
            ControlConfig_pb2.ANGULAR_TRAJECTORY,
        ]:
            control_mode_information.control_mode = control_mode
            cc.ResetJointAccelerationSoftLimits(control_mode_information, options=opts)

    def set_twist_linear_limit(self, limit):
        cc = self.control_config
        opts = self.control_send_options
        twist_linear_soft_limit = ControlConfig_pb2.TwistLinearSoftLimit()
        twist_linear_soft_limit.control_mode = ControlConfig_pb2.CARTESIAN_TRAJECTORY
        twist_linear_soft_limit.twist_linear_soft_limit = limit
        cc.SetTwistLinearSoftLimit(twist_linear_soft_limit, options=opts)

    def set_max_twist_linear_limit(self):
        cc = self.control_config
        opts = self.control_send_options
        limit = cc.GetKinematicHardLimits(options=opts).twist_linear  # 0.5
        self.set_twist_linear_limit(limit)

    def reset_twist_linear_limit(self):
        cc = self.control_config
        opts = self.control_send_options
        control_mode_information = ControlConfig_pb2.ControlModeInformation()
        control_mode_information.control_mode = ControlConfig_pb2.CARTESIAN_TRAJECTORY
        cc.ResetTwistLinearSoftLimit(control_mode_information, options=opts)

    # Rajat ToDo: Check how the following work:
    def pause_action(self):
        self.base.PauseAction(options=self.control_send_options)

    def resume_action(self):
        self.base.ResumeAction(options=self.control_send_options)

    def stop_action(self):
        self.base.StopAction(options=self.control_send_options)

    def stop(self):
        self.base.Stop(options=self.control_send_options)

    # Not using this as we haven't tested it
    # def apply_emergency_stop(self):
    #     self.base.ApplyEmergencyStop()

    def clear_faults(self, timeout=5.0):
        if (
            self.base.GetArmState(options=self.control_send_options).active_state
            == Base_pb2.ARMSTATE_IN_FAULT
        ):
            self.base.ClearFaults(options=self.control_send_options)
            deadline = time.perf_counter() + timeout
            while (
                self.base.GetArmState(options=self.control_send_options).active_state
                != Base_pb2.ARMSTATE_SERVOING_READY
            ):
                if time.perf_counter() >= deadline:
                    raise TimeoutError(
                        f"clear_faults: arm did not reach SERVOING_READY within {timeout}s"
                    )
                time.sleep(0.1)


def main():
    arm = KinovaArm()
    try:
        arm.retract()

        # def cycle_arm(arm):
        #     input("Press Enter to move to home configuration")
        #     arm.home()
        #     input("Press Enter to move to retract configuration")
        #     arm.retract()

        # arm.choose_from_speed_presets("low")
        # cycle_arm(arm)

        # arm.choose_from_speed_presets("medium")
        # cycle_arm(arm)

        # arm.choose_from_speed_presets("high")
        # cycle_arm(arm)

    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
