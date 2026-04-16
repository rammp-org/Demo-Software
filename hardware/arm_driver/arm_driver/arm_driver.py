import concurrent.futures
import enum
import os
import subprocess
import time

import numpy as np
import rclpy
import rclpy.action
import rclpy.node
from ament_index_python.packages import get_package_share_directory
from arm_interfaces.action import Calibrate, ExecuteTrajectory, ReachPreset
from arm_interfaces.srv import (
    CheckReachability,
    GetSpeedPreset,
    SetMode,
    SetSpeedPreset,
)
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped, Vector3Stamped
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from arm_driver.arm_interface import KinovaArm, SpeedPreset
from arm_driver.collision_checker import CollisionChecker
from arm_driver.cup_stabilizer import CupStabilizer

FEEDBACK_RATE = 0.1  # seconds between feedback publishes during action execution
COMMS_TIMEOUT_S = 0.5  # seconds without arm feedback before transitioning to ERROR
TWIST_TIMEOUT_S = 0.5  # seconds since last twist command before stopping the arm
CALIBRATION_MIN_SAMPLES = 80


# Build enum matching service definition
class ArmState(enum.IntEnum):
    IDLE = SetMode.Request.MODE_IDLE
    OPEN_DOOR = SetMode.Request.MODE_OPEN_DOOR
    ORDER_DRINK = SetMode.Request.MODE_ORDER_DRINK
    DRINKING = SetMode.Request.MODE_DRINKING
    CUP_STABILIZE = SetMode.Request.MODE_CUP_STABILIZE
    MANUAL = SetMode.Request.MODE_MANUAL
    PRESET_IN_MOTION = SetMode.Request.MODE_PRESET_IN_MOTION
    ERROR = SetMode.Request.MODE_ERROR


# Maps each state to the source authorised to send commands (None = no commands accepted)
AUTHORIZED_SOURCE = {
    ArmState.OPEN_DOOR: "cmu",
    ArmState.ORDER_DRINK: "cornell",
    ArmState.DRINKING: "cornell",
    ArmState.CUP_STABILIZE: "atdev",
    ArmState.MANUAL: "xbox",
    ArmState.IDLE: None,
    ArmState.PRESET_IN_MOTION: None,
    ArmState.ERROR: None,
}

SOURCES = ["atdev", "xbox", "cornell", "cmu"]


class CommandMode(enum.Enum):
    POSITION = "position"  # fire-and-forget via ExecuteAction
    TWIST = "twist"  # continuous streaming via SendTwistCommand
    NONE = None  # no commands accepted


COMMAND_MODE = {
    ArmState.OPEN_DOOR: CommandMode.POSITION,
    ArmState.ORDER_DRINK: CommandMode.POSITION,
    ArmState.DRINKING: CommandMode.POSITION,
    ArmState.CUP_STABILIZE: CommandMode.TWIST,
    ArmState.MANUAL: CommandMode.TWIST,
    ArmState.IDLE: CommandMode.NONE,
    ArmState.PRESET_IN_MOTION: CommandMode.NONE,
    ArmState.ERROR: CommandMode.NONE,
}

# States that accept continuous twist commands (arm runs last command until next arrives)
_TWIST_STATES = frozenset(
    state for state, mode in COMMAND_MODE.items() if mode == CommandMode.TWIST
)

# Derived: command mode expected from each source (each source must map to a single mode)
_SOURCE_MODE = {
    source: COMMAND_MODE[state]
    for state, source in AUTHORIZED_SOURCE.items()
    if source is not None
}


class ArmDriverNode(rclpy.node.Node):
    """ROS 2 node that owns the Kinova arm connection and exposes its control interface.

    Implements a state machine that gates incoming commands to a single authorised
    source per state. Each source can publish twist, joint position, joint trajectory,
    and Cartesian pose commands on its namespaced topics.
    """

    def __init__(self):
        """Initialise the node, all pub/sub/service/action interfaces, and the arm connection."""
        super().__init__("arm_driver_node")

        self._state = ArmState.IDLE
        self._error_reason: str = ""
        self._arm: KinovaArm | None = None
        self._collision_checker = None  # set in _init_arm after CollisionChecker lands
        self._last_feedback_time: float = time.monotonic()
        self._last_twist_time: float | None = None

        # Last known arm state from Kortex, for fault handling
        self._kortex_arm_state: str = ""
        self._cup_stabilizer: CupStabilizer | None = None

        # ReentrantCallbackGroup allows action/service callbacks to run concurrently
        # with the rest of the node (timers, subscribers) on the MultiThreadedExecutor,
        # so that blocking service/action handlers don't starve the joint_states timer.
        self._action_group = ReentrantCallbackGroup()
        self._service_group = ReentrantCallbackGroup()

        # Collision checker threshold parameters (Nm)
        self.declare_parameter("collision_checker.threshold_default", 30.0)
        self.declare_parameter("collision_checker.threshold_open_door", 50.0)

        # Cup stabilizer parameters
        self.declare_parameter("cup_stabilizer.hz", 40.0)
        self.declare_parameter("cup_stabilizer.kp", 8.0)
        self.declare_parameter("cup_stabilizer.kd", 1.0)
        self.declare_parameter("cup_stabilizer.calibration_s", 5.0)

        self._cup_stabilizer = CupStabilizer(
            hz=self.get_parameter("cup_stabilizer.hz").value,
            kp=self.get_parameter("cup_stabilizer.kp").value,
            kd=self.get_parameter("cup_stabilizer.kd").value,
        )
        self._latest_imu_data: dict | None = None

        self._init_publishers()
        self._init_subscribers()
        self._init_services()
        self._init_actions()
        self._init_timers()

        # Arm connection is attempted on a 2 Hz timer so the node starts cleanly
        # even when the arm is powered on after the node.  The timer cancels itself
        # once connection succeeds.
        self._connect_timer = self.create_timer(0.5, self._try_connect_arm)

    # -------------------------------------------------------------------------
    # Initialization
    # -------------------------------------------------------------------------

    def _init_publishers(self):
        """Create all ROS publishers."""
        self._joint_state_pub = self.create_publisher(
            JointState, "/arm/joint_states", 10
        )
        self._status_pub = self.create_publisher(DiagnosticStatus, "/arm/status", 10)
        self._ee_force_pub = self.create_publisher(Vector3Stamped, "/arm/ee/force", 10)
        self._ee_pos_pub = self.create_publisher(PoseStamped, "/arm/ee/pose", 10)
        self._ee_vel_pub = self.create_publisher(TwistStamped, "/arm/ee/velocity", 10)
        self._robot_description_pub = self.create_publisher(
            String, "/robot_description", 10
        )
        # /tf is published by tf2_ros.TransformBroadcaster, initialised below
        # TODO: self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def _init_subscribers(self):
        """Create all ROS subscribers.

        Subscribes only to the command type relevant to each source, derived from
        ``_SOURCE_MODE``: twist sources get a single ``/arm/{source}/twist`` topic;
        position sources get ``joint_position``, ``joint_trajectory``, and
        ``cartesian_pose`` topics. All callbacks are routed through
        :meth:`_on_command` for authorisation gating.
        """
        for source in SOURCES:
            if _SOURCE_MODE[source] == CommandMode.TWIST:
                self.create_subscription(
                    Twist,
                    f"/arm/{source}/twist",
                    lambda msg, s=source: self._on_command(s, self._handle_twist, msg),
                    10,
                )
            else:  # CommandMode.POSITION
                self.create_subscription(
                    JointState,
                    f"/arm/{source}/joint_position",
                    lambda msg, s=source: self._on_command(
                        s, self._handle_joint_position, msg
                    ),
                    10,
                )
                self.create_subscription(
                    PoseStamped,
                    f"/arm/{source}/cartesian_pose",
                    lambda msg, s=source: self._on_command(
                        s, self._handle_cartesian_pose, msg
                    ),
                    10,
                )

        self.create_subscription(Bool, "/estop", self._on_estop, 10)

    def _init_services(self):
        """Create all ROS service servers."""
        self._set_mode_srv = self.create_service(
            SetMode,
            "/arm/set_mode",
            self._on_set_mode,
            callback_group=self._service_group,
        )
        self._set_speed_preset_srv = self.create_service(
            SetSpeedPreset,
            "/arm/set_speed_preset",
            self._on_set_speed_preset,
            callback_group=self._service_group,
        )
        self._get_speed_preset_srv = self.create_service(
            GetSpeedPreset,
            "/arm/get_speed_preset",
            self._on_get_speed_preset,
            callback_group=self._service_group,
        )
        self._open_gripper_srv = self.create_service(
            Trigger,
            "/arm/open_gripper",
            self._on_open_gripper,
            callback_group=self._service_group,
        )
        self._close_gripper_srv = self.create_service(
            Trigger,
            "/arm/close_gripper",
            self._on_close_gripper,
            callback_group=self._service_group,
        )
        self._check_reachability_srv = self.create_service(
            CheckReachability,
            "/arm/check_reachability",
            self._on_check_reachability,
            callback_group=self._service_group,
        )
        self._clear_error_srv = self.create_service(
            Trigger,
            "/arm/clear_error",
            self._on_clear_error,
            callback_group=self._service_group,
        )

    def _init_actions(self):
        """Create all ROS action servers."""
        self._reach_preset_action = ActionServer(
            self,
            ReachPreset,
            "/arm/reach_preset",
            self._on_reach_preset,
            callback_group=self._action_group,
        )
        self._execute_trajectory_actions = [
            ActionServer(
                self,
                ExecuteTrajectory,
                f"/arm/{source}/execute_trajectory",
                lambda goal_handle, s=source: self._on_execute_trajectory(
                    s, goal_handle
                ),
                callback_group=self._action_group,
            )
            for source in SOURCES
            if _SOURCE_MODE[source] == CommandMode.POSITION
        ]
        self._calibrate_action = ActionServer(
            self,
            Calibrate,
            "/arm/calibrate",
            self._on_calibrate,
            callback_group=self._action_group,
        )

    def _on_calibrate(self, goal_handle):
        """Collect gyro samples and calibrate CupStabilizer. Blocks until done."""
        result = Calibrate.Result()

        if not self._arm:
            result.success = False
            result.message = "Arm not connected"
            goal_handle.abort()
            return result

        calibration_s = self.get_parameter("cup_stabilizer.calibration_s").value
        deadline = time.monotonic() + calibration_s
        samples: list = []

        self.get_logger().info(
            f"Calibrating gyro — keep arm still "
            f"(need {CALIBRATION_MIN_SAMPLES} samples, timeout {calibration_s}s)"
        )

        while len(samples) < CALIBRATION_MIN_SAMPLES:
            if time.monotonic() > deadline:
                result.success = False
                result.message = (
                    f"Calibration timed out — "
                    f"only {len(samples)}/{CALIBRATION_MIN_SAMPLES} samples collected"
                )
                goal_handle.abort()
                return result
            imu = self._latest_imu_data
            if imu is not None:
                samples.append(imu["gyro"].copy())
            time.sleep(0.05)  # 20 Hz — every sample is fresh from 100 Hz cache

        self._cup_stabilizer.calibrate(samples)
        self.get_logger().info(
            f"Gyro calibrated from {len(samples)} samples "
            f"(offset: {self._cup_stabilizer.gyro_offset})"
        )
        result.success = True
        result.message = f"Calibrated from {len(samples)} samples"
        goal_handle.succeed()
        return result

    def _init_timers(self):
        """Create periodic timers for state publishing."""
        self.create_timer(0.01, self._publish_joint_states)  # 100 Hz
        self.create_timer(0.1, self._check_twist_timeout)  # 10 Hz twist watchdog
        self.create_timer(1.0, self._check_hardware_fault)  # 1 Hz Kortex fault poll
        self.create_timer(1.0, self._publish_status)  # 1 Hz

        cup_hz = self.get_parameter("cup_stabilizer.hz").value
        self._cup_stabilize_timer = self.create_timer(
            1.0 / cup_hz, self._cup_stabilize_tick
        )
        self._cup_stabilize_timer.cancel()

    def _try_connect_arm(self):
        """Attempt to connect to the Kinova arm at 2 Hz until successful.

        On success, initialises the CollisionChecker and cancels this timer.
        While disconnected, _publish_status surfaces an ERROR-level diagnostic
        so the outside world can observe the waiting state.
        """
        try:
            self._arm = KinovaArm()
            self.get_logger().info("Arm connected successfully.")
        except Exception as e:
            self.get_logger().warn(
                f"Arm not available, retrying: {e}",
                throttle_duration_sec=5.0,
            )
            return

        self._connect_timer.cancel()
        self._init_collision_checker()

    def _init_collision_checker(self):
        """Initialise the CollisionChecker from the kortex_description URDF.

        Called once, immediately after arm connection succeeds.  Collision
        detection is disabled (self._collision_checker remains None) if this
        fails, but the node continues operating.
        """
        try:
            xacro_file = os.path.join(
                get_package_share_directory("kortex_description"),
                "robots",
                "gen3.xacro",
            )
            result = subprocess.run(
                ["xacro", xacro_file, "dof:=7", "gripper:=robotiq_2f_85"],
                capture_output=True,
                text=True,
                check=True,
            )
            thresholds = {
                "DEFAULT": self.get_parameter(
                    "collision_checker.threshold_default"
                ).value,
                "OPEN_DOOR": self.get_parameter(
                    "collision_checker.threshold_open_door"
                ).value,
            }
            self._collision_checker = CollisionChecker(result.stdout, thresholds)
            self.get_logger().info(
                "CollisionChecker initialised from kortex_description"
            )
        except subprocess.CalledProcessError as e:
            self.get_logger().error(
                f"Failed to initialise CollisionChecker — xacro failed.\n"
                f"stderr: {e.stderr.strip()}"
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to initialise CollisionChecker: {e} — collision detection disabled"
            )

    def _cup_stabilize_tick(self) -> None:
        """Compute twist command from cached IMU data and send it. Runs at cup_stabilizer.hz."""
        if self._state != ArmState.CUP_STABILIZE:
            return
        if not self._arm:
            return

        imu = self._latest_imu_data
        if imu is None:
            return

        result = self._cup_stabilizer.feed(imu)
        if result is None:
            return  # still calibrating

        linear_xyz, angular_xyz = result
        try:
            self._arm.send_twist_base_frame(linear_xyz, angular_xyz)
            self._last_twist_time = time.monotonic()  # only update on success
        except Exception as e:
            self.get_logger().warn(
                f"Cup stabilizer: send_twist_base_frame failed ({e})",
                throttle_duration_sec=1.0,
            )
            try:
                self._arm.stop()
            except Exception as stop_exc:
                self.get_logger().warn(
                    f"Cup stabilizer: stop() also failed ({stop_exc})",
                    throttle_duration_sec=1.0,
                )

    # -------------------------------------------------------------------------
    # State machine
    # -------------------------------------------------------------------------

    def _transition_to(self, new_state: ArmState):
        """Transition the node to a new state, logging the change.

        Args:
            new_state: The state to transition to.
        """
        self.get_logger().info(f"State: {self._state.name} -> {new_state.name}")

        if self._arm:
            try:
                self._arm.stop()
            except Exception as e:
                self.get_logger().warn(
                    f"stop() failed during state transition — continuing: {e}",
                    throttle_duration_sec=1.0,
                )

        old_state = self._state
        self._state = new_state

        if old_state == ArmState.CUP_STABILIZE:
            self._cup_stabilize_timer.cancel()

        if new_state == ArmState.CUP_STABILIZE:
            self._cup_stabilize_timer.reset()

    # -------------------------------------------------------------------------
    # Command routing
    # -------------------------------------------------------------------------

    def _on_command(self, source: str, handler, msg):
        """Gate all incoming commands through source authorization.

        Drops the message silently if ``source`` is not the authorized source
        for the current state, as defined in ``AUTHORIZED_SOURCE``.

        Args:
            source: The source identifier (e.g. ``"atdev"``, ``"xbox"``).
            handler: The command handler to invoke if authorized.
            msg: The incoming ROS message.
        """
        if AUTHORIZED_SOURCE.get(self._state) != source:
            return
        handler(msg)

    def _handle_twist(self, msg: Twist):
        """Send a Cartesian twist command directly to the arm.

        Bypasses the streaming timer — commands are forwarded to the hardware
        immediately at whatever rate the publisher sends them.

        Args:
            msg: Desired end-effector linear and angular velocity.
        """
        self._last_twist_time = time.monotonic()
        if self._arm:
            try:
                self._arm.send_twist(
                    [msg.linear.x, msg.linear.y, msg.linear.z],
                    [msg.angular.x, msg.angular.y, msg.angular.z],
                )
            except Exception as e:
                self.get_logger().warn(
                    f"send_twist failed ({e!r}) — stopping arm to prevent runaway motion",
                    throttle_duration_sec=1.0,
                )
                try:
                    self._arm.stop()
                except Exception as stop_exc:
                    self.get_logger().warn(
                        f"stop() also failed after send_twist error ({stop_exc!r})",
                        throttle_duration_sec=1.0,
                    )

    def _handle_joint_position(self, msg: JointState):
        """Command the arm to a target joint position (HIGH_LEVEL).

        Args:
            msg: Desired joint positions in ``msg.position`` (radians).
        """
        if not self._arm:
            return
        try:
            self._arm.move_angular(list(msg.position), blocking=False)
        except Exception as e:
            self.get_logger().warn(
                f"move_angular failed ({e!r}) — skipping command",
                throttle_duration_sec=1.0,
            )

    def _handle_cartesian_pose(self, msg: PoseStamped):
        """Command the arm to a target end-effector Cartesian pose (HIGH_LEVEL).

        Args:
            msg: Desired end-effector pose in Cartesian space.
        """
        if not self._arm:
            return
        p = msg.pose.position
        q = msg.pose.orientation
        try:
            self._arm.move_cartesian(
                [p.x, p.y, p.z], [q.x, q.y, q.z, q.w], blocking=False
            )
        except Exception as e:
            self.get_logger().warn(
                f"move_cartesian failed ({e!r}) — skipping command",
                throttle_duration_sec=1.0,
            )

    # -------------------------------------------------------------------------
    # Subscribers
    # -------------------------------------------------------------------------

    def _on_estop(self, msg: Bool):
        """Handle an emergency stop signal.

        Immediately stops the arm and transitions to ERROR regardless of current state.

        Args:
            msg: ``True`` to trigger the e-stop.
        """
        if msg.data:
            self.get_logger().warn("E-stop received.")
            self._error_reason = "E-stop triggered"
            self._transition_to(ArmState.ERROR)

    # -------------------------------------------------------------------------
    # Service handlers
    # -------------------------------------------------------------------------

    def _on_set_mode(self, request, response):
        """Handle a /set_mode service request, transitioning the arm to the requested state.

        Rejects the request if the mode is unrecognised or if the node is in ERROR state.

        Args:
            request: Service request containing ``request.mode`` (uint8).
            response: Service response with ``response.success`` (bool).

        Returns:
            The populated service response.
        """
        try:
            new_state = ArmState(request.mode)
        except ValueError:
            self.get_logger().error(f"Unknown mode: {request.mode}")
            response.success = False
            response.message = f"Unknown mode: {request.mode}"
            return response

        if self._state == ArmState.ERROR:
            self.get_logger().warn("Cannot change mode while in ERROR state.")
            response.success = False
            response.message = "Cannot change mode while in ERROR state"
            return response

        self._transition_to(new_state)
        response.success = True
        return response

    def _on_set_speed_preset(self, request, response):
        """Handle a /arm/set_speed_preset service request.

        Args:
            request: Service request containing ``request.preset`` (uint8).
            response: Service response with ``response.success`` (bool).

        Returns:
            The populated service response.
        """
        try:
            preset = SpeedPreset(request.preset)
        except ValueError:
            response.success = False
            response.message = f"Unknown preset '{request.preset}'"
            return response

        if preset in (SpeedPreset.DEFAULT, SpeedPreset.MAX):
            response.success = False
            response.message = (
                "Cannot set DEFAULT or MAX preset directly. Choose LOW, MEDIUM, HIGH."
            )
            return response

        if not self._arm:
            response.success = False
            response.message = "Arm not connected"
            return response

        try:
            self._arm.choose_from_speed_presets(preset)
        except Exception as e:
            response.success = False
            response.message = f"Failed to set speed preset: {e}"
            return response
        self.get_logger().info(f"Speed preset set to '{preset.name}'.")
        response.success = True
        response.message = f"Speed preset set to '{preset.name}'"
        return response

    def _on_get_speed_preset(self, request, response):
        """Handle a /arm/get_speed_preset service request.

        Args:
            request: Empty request.
            response: Service response with ``response.preset`` (uint8).

        Returns:
            The populated service response.
        """
        if not self._arm:
            response.success = False
            response.message = "Arm not connected"
            return response

        response.success = True
        response.preset = int(self._arm.get_speed_preset())
        return response

    def _on_open_gripper(self, request, response):
        """Handle a /arm/open_gripper service request.

        Args:
            request: Empty Trigger request.
            response: Service response with ``response.success`` (bool).

        Returns:
            The populated service response.
        """
        if not self._arm:
            response.success = False
            response.message = "Arm not connected"
            return response

        if self._state in (ArmState.IDLE, ArmState.ERROR):
            response.success = False
            response.message = (
                f"Gripper commands not allowed in state {self._state.name}"
            )
            return response

        try:
            self._arm.open_gripper()
            response.success = True
            response.message = "Gripper opened"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _on_close_gripper(self, request, response):
        """Handle a /arm/close_gripper service request.

        Args:
            request: Empty Trigger request.
            response: Service response with ``response.success`` (bool).

        Returns:
            The populated service response.
        """
        if not self._arm:
            response.success = False
            response.message = "Arm not connected"
            return response

        if self._state in (ArmState.IDLE, ArmState.ERROR):
            response.success = False
            response.message = (
                f"Gripper commands not allowed in state {self._state.name}"
            )
            return response

        try:
            self._arm.close_gripper()
            response.success = True
            response.message = "Gripper closed"
        except Exception as e:
            response.success = False
            response.message = str(e)
        return response

    def _on_clear_error(self, request, response):
        """Handle a /arm/clear_error service request.

        Transitions the node from ERROR back to IDLE, allowing normal operation
        to resume.  Rejected if the node is not currently in ERROR state.

        Args:
            request: Empty Trigger request.
            response: Trigger response with ``success`` (bool) and ``message`` (str).

        Returns:
            The populated service response.
        """
        if self._state != ArmState.ERROR:
            response.success = False
            response.message = f"Not in ERROR state (current state: {self._state.name})"
            return response

        if not self._arm:
            response.success = False
            response.message = "Arm not connected — reconnect before clearing error"
            return response

        try:
            self._arm.clear_faults()
        except Exception as e:
            response.success = False
            response.message = f"Kortex fault could not be cleared: {e}"
            return response

        self._error_reason = ""
        self._kortex_arm_state = ""
        self._transition_to(ArmState.IDLE)
        response.success = True
        response.message = "Error cleared — arm is IDLE"
        return response

    def _on_check_reachability(self, request, response):
        """Handle a /arm/check_reachability service request.

        Calls Kortex's ComputeInverseKinematics with the target pose and the
        arm's current joint angles as the IK seed.  Returns reachable=True if
        the solver finds a valid joint configuration, False otherwise.

        Args:
            request: Service request containing ``target_pose`` (geometry_msgs/Pose).
            response: Service response with ``reachable`` (bool) and ``message`` (string).

        Returns:
            The populated service response.
        """
        if not self._arm:
            response.reachable = False
            response.message = "Arm not connected"
            return response

        p = request.target_pose.position
        q = request.target_pose.orientation
        try:
            ik_result = self._arm.compute_ik([p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
            response.reachable = True
            response.message = "IK solution found"
            import math

            response.joint_angles = [
                math.radians(j.value) for j in ik_result.joint_angles
            ]
        except Exception as e:
            response.reachable = False
            response.message = f"IK failed: {e}"
            response.joint_angles = []
        return response

    # -------------------------------------------------------------------------
    # Action handlers
    # -------------------------------------------------------------------------

    def _run_reference_action(self, goal_handle, arm_fn):
        """Common execution loop for reference actions (home, retract, zero, cup_stabilize).

        Starts the arm action non-blocking, then streams joint state feedback until
        the arm signals completion, then transitions to IDLE.

        Args:
            goal_handle: The action goal handle.
            arm_fn: Callable that starts the arm motion (e.g. ``self._arm.retract``).

        Returns:
            The populated action result.
        """
        self._transition_to(ArmState.PRESET_IN_MOTION)

        try:
            arm_fn(blocking=False)
            feedback = ReachPreset.Feedback()
            deadline = time.monotonic() + KinovaArm.ACTION_TIMEOUT_DURATION
            while not self._arm.ready():
                if self._state == ArmState.ERROR:
                    goal_handle.abort()
                    result = ReachPreset.Result()
                    result.success = False
                    result.message = self._error_reason
                    return result
                if time.monotonic() >= deadline:
                    self.get_logger().error(
                        "Reference action timed out waiting for arm ready signal"
                    )
                    self._error_reason = "Reference action timed out"
                    self._transition_to(ArmState.ERROR)
                    goal_handle.abort()
                    result = ReachPreset.Result()
                    result.success = False
                    result.message = self._error_reason
                    return result
                try:
                    state = self._arm.get_state()
                except Exception as e:
                    self.get_logger().warn(
                        f"get_state() failed in feedback loop ({e!r}) — skipping cycle",
                        throttle_duration_sec=1.0,
                    )
                    time.sleep(FEEDBACK_RATE)
                    continue
                feedback.joint_states.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_states.position = state["position"].tolist()
                feedback.joint_states.velocity = state["velocity"].tolist()
                feedback.joint_states.effort = state["effort"].tolist()
                goal_handle.publish_feedback(feedback)
                time.sleep(FEEDBACK_RATE)

            # Guard against estop firing at the exact moment the arm became ready.
            if self._state == ArmState.ERROR:
                goal_handle.abort()
                result = ReachPreset.Result()
                result.success = False
                result.message = "Action aborted: e-stop triggered during execution"
                return result

            self._transition_to(ArmState.IDLE)
            goal_handle.succeed()
            result = ReachPreset.Result()
            result.success = True
        except Exception as e:
            self.get_logger().error(f"Action failed: {e}")
            self._error_reason = f"Action execution failed: {e}"
            self._transition_to(ArmState.ERROR)
            goal_handle.abort()
            result = ReachPreset.Result()
            result.success = False
            result.message = self._error_reason

        return result

    def _on_reach_preset(self, goal_handle):
        """Handle a /arm/reach_preset action goal, moving the arm to a named preset position.

        Dispatches to the appropriate arm motion based on ``goal.preset``, streams joint
        state feedback while executing, then transitions to the corresponding final state.

        Args:
            goal_handle: The action goal handle containing ``preset`` (uint8).

        Returns:
            ReachPreset result with success flag and optional message.
        """
        if self._state == ArmState.ERROR:
            result = ReachPreset.Result()
            result.success = False
            result.message = (
                f"Cannot execute preset while in ERROR state: {self._error_reason}"
            )
            goal_handle.abort()
            return result

        dispatch = {
            ReachPreset.Goal.PRESET_HOME: self._arm.home,
            ReachPreset.Goal.PRESET_RETRACT: self._arm.retract,
            ReachPreset.Goal.PRESET_ZERO: self._arm.zero,
            ReachPreset.Goal.PRESET_CUP_STABILIZE: self._arm.move_to_stabilize_pose,
        }

        arm_fn = dispatch.get(goal_handle.request.preset)
        if arm_fn is None:
            result = ReachPreset.Result()
            result.success = False
            result.message = f"Unknown preset: {goal_handle.request.preset}"
            goal_handle.abort()
            return result

        return self._run_reference_action(goal_handle, arm_fn=arm_fn)

    def _on_execute_trajectory(self, source: str, goal_handle):
        """Handle a /arm/{source}/execute_trajectory action goal.

        Rejects the goal if ``source`` is not authorised for the current state.
        Otherwise executes the trajectory non-blocking and streams joint state
        feedback until completion.

        Args:
            source: The source identifier captured from the action endpoint name.
            goal_handle: Action goal handle containing ``trajectory``.

        Returns:
            ExecuteTrajectory result with success flag and optional message.
        """
        if self._state == ArmState.ERROR:
            result = ExecuteTrajectory.Result()
            result.success = False
            result.message = (
                f"Cannot execute trajectory while in ERROR state: {self._error_reason}"
            )
            goal_handle.abort()
            return result

        if AUTHORIZED_SOURCE.get(self._state) != source:
            result = ExecuteTrajectory.Result()
            result.success = False
            result.message = (
                f"Source '{source}' not authorised in state {self._state.name}"
            )
            goal_handle.abort()
            return result

        waypoints = [list(pt.positions) for pt in goal_handle.request.trajectory.points]
        try:
            self._arm.move_angular_trajectory(waypoints, blocking=False)
            feedback = ExecuteTrajectory.Feedback()
            deadline = time.monotonic() + KinovaArm.ACTION_TIMEOUT_DURATION
            while not self._arm.ready():
                if self._state == ArmState.ERROR:
                    goal_handle.abort()
                    result = ExecuteTrajectory.Result()
                    result.success = False
                    result.message = self._error_reason
                    return result
                if time.monotonic() >= deadline:
                    self.get_logger().error(
                        "Trajectory action timed out waiting for arm ready signal"
                    )
                    self._error_reason = "Trajectory action timed out"
                    self._transition_to(ArmState.ERROR)
                    goal_handle.abort()
                    result = ExecuteTrajectory.Result()
                    result.success = False
                    result.message = self._error_reason
                    return result
                try:
                    state = self._arm.get_state()
                except Exception as e:
                    self.get_logger().warn(
                        f"get_state() failed in feedback loop ({e!r}) — skipping cycle",
                        throttle_duration_sec=1.0,
                    )
                    time.sleep(FEEDBACK_RATE)
                    continue
                feedback.joint_states.header.stamp = self.get_clock().now().to_msg()
                feedback.joint_states.position = state["position"].tolist()
                feedback.joint_states.velocity = state["velocity"].tolist()
                feedback.joint_states.effort = state["effort"].tolist()
                goal_handle.publish_feedback(feedback)
                time.sleep(FEEDBACK_RATE)
            goal_handle.succeed()
            result = ExecuteTrajectory.Result()
            result.success = True
        except Exception as e:
            self.get_logger().error(f"Trajectory execution failed: {e}")
            self._transition_to(ArmState.ERROR)
            goal_handle.abort()
            result = ExecuteTrajectory.Result()
            result.success = False
            result.message = str(e)

        return result

    # -------------------------------------------------------------------------
    # Timer callbacks
    # -------------------------------------------------------------------------

    def _publish_joint_states(self):
        """Publish current joint states and end-effector force at 100 Hz."""
        stamp = self.get_clock().now().to_msg()

        joint_msg = JointState()
        joint_msg.header.stamp = stamp
        force_msg = Vector3Stamped()
        vel_msg = TwistStamped()
        pos_msg = PoseStamped()
        force_msg.header.stamp = stamp
        vel_msg.header.stamp = stamp
        pos_msg.header.stamp = stamp

        if self._arm:
            try:
                state = self._arm.get_state()
                self._last_feedback_time = time.monotonic()
                self._latest_imu_data = state["imu"]
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
                        f"Arm communication lost — no feedback for {COMMS_TIMEOUT_S}s"
                    )
                    self._error_reason = "Arm communication timeout"
                    self._transition_to(ArmState.ERROR)
                return

            if self._collision_checker is not None and self._state != ArmState.ERROR:
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

            joint_msg.name = [
                f"joint_{i + 1}" for i in range(self._arm.actuator_count)
            ] + ["robotiq_85_left_knuckle_joint"]
            joint_msg.position = state["position"].tolist() + [state["gripper_pos"]]
            joint_msg.velocity = state["velocity"].tolist() + [0.0]
            joint_msg.effort = state["effort"].tolist() + [0.0]

            force = state["ee_force"]
            force_msg.vector.x = force[0]
            force_msg.vector.y = force[1]
            force_msg.vector.z = force[2]

            ee_pos = state["ee_pos"]
            pos_msg.pose.position.x = ee_pos[0]
            pos_msg.pose.position.y = ee_pos[1]
            pos_msg.pose.position.z = ee_pos[2]
            pos_msg.pose.orientation.x = ee_pos[3]
            pos_msg.pose.orientation.y = ee_pos[4]
            pos_msg.pose.orientation.z = ee_pos[5]
            pos_msg.pose.orientation.w = ee_pos[6]

            ee_velocity = state["ee_vel"]
            vel_msg.twist.linear.x = ee_velocity[0]
            vel_msg.twist.linear.y = ee_velocity[1]
            vel_msg.twist.linear.z = ee_velocity[2]
            vel_msg.twist.angular.x = np.deg2rad(ee_velocity[3])
            vel_msg.twist.angular.y = np.deg2rad(ee_velocity[4])
            vel_msg.twist.angular.z = np.deg2rad(ee_velocity[5])

        self._joint_state_pub.publish(joint_msg)
        self._ee_vel_pub.publish(vel_msg)
        self._ee_pos_pub.publish(pos_msg)
        self._ee_force_pub.publish(force_msg)

    def _check_twist_timeout(self):
        """Stop the arm if no twist command has arrived recently while in a twist state.

        Prevents runaway motion when a twist publisher dies mid-stream. Does not
        transition to ERROR — the publisher may recover and send new commands.
        """
        if self._state not in _TWIST_STATES:
            return
        if self._last_twist_time is None:
            return
        if time.monotonic() - self._last_twist_time > TWIST_TIMEOUT_S:
            self.get_logger().warn(
                "Twist command timeout — stopping arm to prevent runaway motion.",
                throttle_duration_sec=1.0,
            )
            if self._arm:
                try:
                    self._arm.stop()
                except Exception as e:
                    self.get_logger().warn(
                        f"stop() failed in twist watchdog ({e!r})",
                        throttle_duration_sec=1.0,
                    )
            self._last_twist_time = None  # reset so we only stop once per stale window

    def _check_hardware_fault(self):
        """Poll the Kortex base for hardware fault state at 1 Hz.

        Transitions to ERROR if the arm enters ARMSTATE_IN_FAULT so that
        software state stays consistent with hardware reality.  Skipped when
        already in ERROR to avoid redundant TCP calls.
        """
        if not self._arm or self._state == ArmState.ERROR:
            return
        try:
            state_name, is_faulted = self._arm.get_fault_state()
            self._kortex_arm_state = state_name
            if is_faulted:
                self.get_logger().error(f"Kortex hardware fault detected: {state_name}")
                self._error_reason = f"Kortex hardware fault: {state_name}"
                self._transition_to(ArmState.ERROR)
        except Exception as e:
            self.get_logger().warn(
                f"get_fault_state() failed ({e!r})",
                throttle_duration_sec=5.0,
            )

    def _publish_status(self):
        """Publish the node's diagnostic status at 1 Hz."""
        msg = DiagnosticStatus()
        msg.name = "arm_driver"
        msg.message = self._state.name
        msg.level = (
            DiagnosticStatus.ERROR
            if (self._state == ArmState.ERROR or self._arm is None)
            else DiagnosticStatus.OK
        )
        kv_connected = KeyValue()
        kv_connected.key = "arm_connected"
        kv_connected.value = str(self._arm is not None).lower()
        msg.values.append(kv_connected)
        kv_hw = KeyValue()
        kv_hw.key = "kortex_arm_state"
        kv_hw.value = self._kortex_arm_state
        msg.values.append(kv_hw)
        if self._error_reason:
            kv_err = KeyValue()
            kv_err.key = "error_reason"
            kv_err.value = self._error_reason
            msg.values.append(kv_err)
        self._status_pub.publish(msg)


def main(args=None):
    """Entry point for the arm_driver node."""
    rclpy.init(args=args)
    node = ArmDriverNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
