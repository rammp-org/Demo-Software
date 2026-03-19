import enum
import time

import rclpy
import rclpy.action
import rclpy.node
from arm_interfaces.action import ExecuteTrajectory, ReachPreset
from arm_interfaces.srv import GetSpeedPreset, SetMode, SetSpeedPreset
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from .mock_arm import KinovaArm, SpeedPreset

FEEDBACK_RATE = 0.1  # seconds between feedback publishes during action execution


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
        self._latest_twist: Twist | None = None
        self._latest_twist_time: float = 0.0

        # ReentrantCallbackGroup allows action/service callbacks to run concurrently
        # with the rest of the node (timers, subscribers) on the MultiThreadedExecutor,
        # so that blocking service/action handlers don't starve the joint_states timer.
        self._action_group = ReentrantCallbackGroup()
        self._service_group = ReentrantCallbackGroup()

        self._init_publishers()
        self._init_subscribers()
        self._init_services()
        self._init_actions()
        self._init_timers()
        self._init_arm()

    # -------------------------------------------------------------------------
    # Initialization
    # -------------------------------------------------------------------------

    def _init_publishers(self):
        """Create all ROS publishers."""
        self._joint_state_pub = self.create_publisher(
            JointState, "/arm/joint_states", 10
        )
        self._imu_pub = self.create_publisher(Imu, "/arm/imu", 10)
        self._status_pub = self.create_publisher(DiagnosticStatus, "/arm/status", 10)
        self._ee_force_pub = self.create_publisher(Vector3Stamped, "/arm/ee_force", 10)
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

    def _init_timers(self):
        """Create periodic timers for state publishing and twist streaming."""
        self.create_timer(0.01, self._publish_joint_states)  # 100 Hz
        self.create_timer(1.0, self._publish_status)  # 1 Hz
        self._twist_timer = self.create_timer(0.04, self._stream_twist)  # 25 Hz
        self._twist_timer.cancel()  # only active while in a TWIST state

    def _init_arm(self):
        """Connect to the Kinova arm. Transitions to ERROR on failure."""
        try:
            self._arm = KinovaArm()
            self.get_logger().info("Arm connected successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to arm: {e}")
            self._error_reason = f"Arm connection failed: {e}"
            self._transition_to(ArmState.ERROR)

    # -------------------------------------------------------------------------
    # State machine
    # -------------------------------------------------------------------------

    def _transition_to(self, new_state: ArmState):
        """Transition the node to a new state, logging the change.

        If leaving a TWIST state, clears the cached twist so the streaming timer
        stops sending commands.

        Args:
            new_state: The state to transition to.
        """
        self.get_logger().info(f"State: {self._state.name} -> {new_state.name}")

        if self._arm:
            self._arm.stop()

        old_mode = COMMAND_MODE.get(self._state)
        new_mode = COMMAND_MODE.get(new_state)

        if old_mode != CommandMode.TWIST and new_mode == CommandMode.TWIST:
            self._twist_timer.reset()

        elif new_mode != CommandMode.TWIST:
            self._twist_timer.cancel()
            # Clear the twist cache when leaving a TWIST state so the arm stops if we re-enter a TWIST state without receiving a new command
            self._latest_twist = None
            self._latest_twist_time = 0.0

        self._state = new_state

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
        """Cache the latest Cartesian twist command for the streaming timer.

        The actual ``send_twist`` call happens in :meth:`_stream_twist` at 25 Hz
        so that the arm receives a steady command stream regardless of publisher rate.

        Args:
            msg: Desired end-effector linear and angular velocity.
        """
        self._latest_twist = msg
        self._latest_twist_time = time.monotonic()

    def _handle_joint_position(self, msg: JointState):
        """Command the arm to a target joint position (HIGH_LEVEL).

        Args:
            msg: Desired joint positions in ``msg.position`` (radians).
        """
        self._arm.move_angular(list(msg.position), blocking=False)

    def _handle_cartesian_pose(self, msg: PoseStamped):
        """Command the arm to a target end-effector Cartesian pose (HIGH_LEVEL).

        Args:
            msg: Desired end-effector pose in Cartesian space.
        """
        p = msg.pose.position
        q = msg.pose.orientation
        self._arm.move_cartesian([p.x, p.y, p.z], [q.x, q.y, q.z, q.w], blocking=False)

    def _stream_twist(self):
        """Send the latest cached twist command to the arm at 25 Hz.

        Only active while in a TWIST state (timer is cancelled otherwise). Acts as a
        watchdog: if no fresh twist has been received within 200 ms, sends a
        zero-velocity command and clears the cache so the arm comes to a stop.
        """
        if not self._arm:
            return

        now = time.monotonic()
        if now - self._latest_twist_time > 0.2:
            # Watchdog expired — stop the arm and clear the cache
            self._arm.send_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
            self._latest_twist = None
            return

        if self._latest_twist is None:
            return

        msg = self._latest_twist
        self._arm.send_twist(
            [msg.linear.x, msg.linear.y, msg.linear.z],
            [msg.angular.x, msg.angular.y, msg.angular.z],
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
            # STUB: self._arm.stop()
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

        self._arm.choose_from_speed_presets(preset)
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
            while not self._arm.ready():
                if self._state == ArmState.ERROR:
                    goal_handle.abort()
                    result = ReachPreset.Result()
                    result.success = False
                    result.message = self._error_reason
                    return result
                state = self._arm.get_state()
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
        dispatch = {
            ReachPreset.Goal.PRESET_HOME: self._arm.home,
            ReachPreset.Goal.PRESET_RETRACT: self._arm.retract,
            ReachPreset.Goal.PRESET_ZERO: self._arm.zero,
            ReachPreset.Goal.PRESET_CUP_STABILIZE: self._arm.cup_stabilize,
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
            while not self._arm.ready():
                state = self._arm.get_state()
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
        force_msg.header.stamp = stamp

        if self._arm:
            state = self._arm.get_state()
            joint_msg.position = state["position"].tolist() + [state["gripper_pos"]]
            joint_msg.velocity = state["velocity"].tolist() + [0.0]
            joint_msg.effort = state["effort"].tolist() + [0.0]

            force = self._arm.get_ee_force()
            force_msg.vector.x = force[0]
            force_msg.vector.y = force[1]
            force_msg.vector.z = force[2]

        self._joint_state_pub.publish(joint_msg)
        self._ee_force_pub.publish(force_msg)

    def _publish_status(self):
        """Publish the node's diagnostic status at 1 Hz."""
        msg = DiagnosticStatus()
        msg.name = "arm_driver"
        msg.message = self._state.name
        msg.level = (
            DiagnosticStatus.ERROR
            if self._state == ArmState.ERROR
            else DiagnosticStatus.OK
        )
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
