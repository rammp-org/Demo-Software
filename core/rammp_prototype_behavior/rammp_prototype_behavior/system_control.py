import asyncio
import enum
import threading
from std_msgs.msg import String

import rclpy
import rclpy.action
import rclpy.node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import os
from .action_client.arm_preset_action_client import ArmPreset, ArmPresetActionClient
from .action_client.bring_cup_to_mouth_action_client import BringCupToMouthActionClient
from .action_client.grab_cup_from_table_action_client import (
    GrabCupFromTableActionClient,
)
from .action_client.home_cup_action_client import HomeCupActionClient
from .action_client.pick_up_and_order_action_client import PickUpAndOrderActionClient
from .action_client.pub_cup_back_to_holder_action_client import (
    PutCupBackToHolderActionClient,
)
from .action_client.open_door_action_client import OpenDoorActionClient
from .action_client.chair_curb_traverse_action_client import (
    CurbTraverseDirection,
    ChairCurbTraverseActionClient,
)
from gui_interfaces.srv import UserInputs
from transitions.extensions import HierarchicalMachine as Machine
from .node_name_monitor import NodeNameMonitor
from ament_index_python.packages import get_package_share_directory
from arm_interfaces.srv import SetMode, SetSpeedPreset
from std_srvs.srv import Trigger, SetBool
from std_msgs.msg import Bool
from diagnostic_msgs.msg import DiagnosticStatus


class ArmMode(enum.IntEnum):
    IDLE = SetMode.Request.MODE_IDLE
    OPEN_DOOR = SetMode.Request.MODE_OPEN_DOOR
    ORDER_DRINK = SetMode.Request.MODE_ORDER_DRINK
    DRINKING = SetMode.Request.MODE_DRINKING
    CUP_STABILIZE = SetMode.Request.MODE_CUP_STABILIZE
    MANUAL = SetMode.Request.MODE_MANUAL


class MockTasks(enum.IntEnum):
    OPEN_DOOR = 1
    ORDER_DRINK = 2
    RECEIVE_DRINK = 3
    CUP_STABILIZE = 4
    SIP_DRINK = 5
    PLACE_CUP_BACK = 6
    ARM_MANUAL_CONTROL = 7
    ARM_HOME = 8
    ARM_RETRACT = 9
    BASE_CONTROL = 10
    BASE_CURB_TRAVERSE = 11
    END_TASK = 12  # update this when adding new mock tasks


class MockState:
    def __init__(self, node: rclpy.node.Node, starting_task=MockTasks.OPEN_DOOR):
        self._node = node
        self.is_mock_task_running = False
        self.next_mock_task = starting_task
        self.current_mock_task = None
        self.next_mock_wait_time_counter = 3

    def run_next_mock_task(self):
        if self.is_mock_task_running:
            return
        if self.next_mock_task is MockTasks.END_TASK:
            self._node.get_logger().info("All mock tasks completed.")
            self.next_mock_task = None
            return
        if self.next_mock_task is None:
            return
        if self.next_mock_wait_time_counter > 0:
            self._node.get_logger().info(
                f"Waiting for {self.next_mock_wait_time_counter} seconds before starting next mock task: {self.next_mock_task.name}"
            )
            self.next_mock_wait_time_counter -= 1
            return
        self.current_mock_task = self.next_mock_task
        self.next_mock_task = MockTasks(self.next_mock_task.value + 1)
        self.is_mock_task_running = True

        if self.current_mock_task == MockTasks.OPEN_DOOR:
            self._node.mock_open_door_request()
        elif self.current_mock_task == MockTasks.ORDER_DRINK:
            self._node.mock_order_drink_request()
        elif self.current_mock_task == MockTasks.SIP_DRINK:
            self._node.mock_sip_drink_request()
        elif self.current_mock_task == MockTasks.PLACE_CUP_BACK:
            self._node.mock_place_cup_back_to_holder_request()
        elif self.current_mock_task == MockTasks.RECEIVE_DRINK:
            self._node.mock_receive_drink_request()
        elif self.current_mock_task == MockTasks.CUP_STABILIZE:
            self._node.mock_cup_stabilizer_request()
        elif self.current_mock_task == MockTasks.ARM_MANUAL_CONTROL:
            self._node.mock_arm_manual_control_request()
        elif self.current_mock_task == MockTasks.ARM_RETRACT:
            self._node.mock_arm_retract_request()
        elif self.current_mock_task == MockTasks.ARM_HOME:
            self._node.mock_arm_home_request()
        elif self.current_mock_task == MockTasks.BASE_CONTROL:
            self._node.mock_base_control_request()
        elif self.current_mock_task == MockTasks.BASE_CURB_TRAVERSE:
            self._node.mock_base_curb_navigation_traverse_request()

    def finish_current_mock_task(self):
        if self.is_mock_task_running is False:
            return

        self.is_mock_task_running = False
        self.next_mock_wait_time_counter = (
            3  # reset wait time counter for next mock task
        )
        self._node.get_logger().info(
            f"Finished mock task: {self.current_mock_task.name}"
        )
        self.current_mock_task = None

    def is_mocking_arm_home(self) -> bool:
        return self.current_mock_task == MockTasks.ARM_HOME

    def is_mocking_arm_retract(self) -> bool:
        return self.current_mock_task == MockTasks.ARM_RETRACT


class SystemControl(rclpy.node.Node):
    def __init__(self):
        super().__init__("system_control")
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        self.get_logger().info("System Control Node has been started.")
        share_dir = get_package_share_directory("rammp_prototype_behavior")
        json_path = os.path.join(share_dir, "config/node_name.json")
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()

        self.is_mocking = False
        if self.is_mocking:
            self._mock_state = MockState(self, starting_task=MockTasks.OPEN_DOOR)
        else:
            self._mock_state = MockState(
                self, starting_task=MockTasks.END_TASK
            )  # starting task = END_TASK to disable mock

        self._arm_status = ""
        self._cb_group = ReentrantCallbackGroup()
        self._all_node_ready = False
        self.node_monitor = NodeNameMonitor(self, json_path, self.node_monitor_callback)

        self.current_arm_state = ""
        self.init_state_machine()

        self.init_publisher()
        self.init_subscribers()
        self.init_services()
        self.init_actions_clients()
        self._test_timer = self.create_timer(
            1.0, self.mock_task, callback_group=self._cb_group
        )
        self._test_timer.cancel()  # cancel the timer and reset when system is ready

        self._system_monitor_timer = self.create_timer(
            1.0, self.system_monitor_callback, callback_group=self._cb_group
        )

    def system_monitor_callback(self):
        if self._all_node_ready and self._gui_connected:
            self.ready()

    def mock_task(self):
        self._mock_state.run_next_mock_task()

    def finish_mock_task(self):
        self._mock_state.finish_current_mock_task()

    def _mock_delay(self, seconds, callback):
        """Non-blocking delay: creates a one-shot timer that calls `callback` after `seconds`."""
        if not self.is_mocking:
            return
        timer = None

        def _on_timeout():
            nonlocal timer
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
                timer = None
            callback()

        # comment out to disable mock
        timer = self.create_timer(seconds, _on_timeout, callback_group=self._cb_group)

    ## ---------------------mock testing functions----------------------------------
    def mock_open_door_request(self):
        # for testing open door action, will remove after testing
        self.get_logger().info("Sending mock open door request.")
        self.get_logger().info("send reqArm trigger to enter Arm state first.")
        self.reqArm()  # should enter Arm_retracted state
        self.get_logger().info("then send reqOpenDoor trigger.")
        self.reqOpenDoor()  # should enter Arm_Door_raisingArm state
        self.get_logger().info("reqOpenDoor trigger sent.")

    def mock_order_drink_request(self):
        # for testing order drink action, will remove after testing
        self.get_logger().info("Sending mock order drink request.")
        self.get_logger().info("then send reqOrderDrink trigger.")
        self.reqOrderDrink()  # should enter Arm_OrderDrink_pickCup state
        self.get_logger().info("reqOrderDrink trigger sent.")

    def mock_receive_drink_request(self):
        # for testing receive drink action, will remove after testing
        self.get_logger().info("Sending mock receive drink request.")
        self.detectDrink()  # should enter Arm_OrderDrink_detectingDrink state
        self.get_logger().info("detectDrink trigger sent.")

    def mock_sip_drink_request(self):
        # for testing sip drink action, will remove after testing
        self.get_logger().info("Sending mock sip drink request.")
        self.reqDrink()  # should enter Arm_Drink_sipping state
        self.get_logger().info("reqSipDrink trigger sent.")

    def mock_place_cup_back_to_holder_request(self):
        # for testing place cup back to holder action, will remove after testing
        self.get_logger().info("Sending mock place cup back to holder request.")
        self.placeCupBack()  # should enter Arm_Drink_placingCupBack state
        self.get_logger().info("placeCupBack trigger sent.")

    def mock_cup_stabilizer_request(self):
        # for testing cup stabilizer, will remove after testing
        self.get_logger().info("Sending mock cup stabilizer request.")
        self.reqStabilizeCup()  # should enter Arm_Drink_stabilizingCup state

    def mock_arm_manual_control_request(self):
        # for testing manual control of arm, will remove after testing
        self.get_logger().info("Sending mock arm manual control request.")
        self.reqManualControl()  # should enter Arm_manualControl state

    def mock_arm_retract_request(self):
        # for testing arm retract, will remove after testing
        self.get_logger().info("Sending mock arm retract request.")
        self.set_arm_mode_idle()  # set arm to idle before retracting to avoid potential interference with retracting process
        self.arm_preset_client.set_preset(
            ArmPreset.RETRACT
        )  # should enter Arm_retracted state

    def mock_arm_home_request(self):
        # for testing arm home, will remove after testing
        self.get_logger().info("Sending mock arm home request.")
        self.set_arm_mode_idle()  # set arm to idle before moving to home position to avoid potential interference with homing process
        self.arm_preset_client.set_preset(
            ArmPreset.HOME
        )  # should enter Arm_homed state

    def mock_base_control_request(self):
        # for testing manual control of base, will remove after testing
        self.get_logger().info("Sending mock base manual control request.")
        self.reqChair()
        self.request_manual_seat_control("elevate up")
        self._mock_delay(0.5, lambda: self.request_manual_seat_control("elevate down"))
        self._mock_delay(1.0, lambda: self.base_drive_enable(True))
        self._mock_delay(1.5, lambda: self.base_drive_enable(False))

        self._mock_delay(2.0, lambda: self.base_self_leveling_enable(True))
        self._mock_delay(2.5, lambda: self.base_self_leveling_enable(False))
        self._mock_delay(
            3.0, lambda: self._mock_state.finish_current_mock_task()
        )  # finish mock base control task after testing

    def mock_base_curb_navigation_traverse_request(self):
        # for testing curb traverse action, will remove after testing
        self.get_logger().info("Sending mock curb navigation traverse request.")
        self.reqNav()

    ## -----------------------------end of mock testing functions------------------------

    # -----------------------curb navigation transtion functions-----------------------------
    def on_enter_Nav_descendDetecting(self):
        self.get_logger().info("Detecting curb to prepare for descend.")
        self.enable_curb_detection(
            True
        )  # enable curb detection to detect curb for navigation
        self.get_logger().debug("Mock detecting curb for 5 seconds.")
        self._mock_delay(
            5.0, self.confirm
        )  # should enter Nav_traverse state after detecting curb for testing, will replace with actual logic to determine when curb is detected later

    def on_exit_Nav_descendDetecting(self):
        self.enable_curb_detection(
            False
        )  # ensure curb detection is disabled when exiting detecting curb state

    def on_enter_Nav_ascendDetecting(self):
        self.get_logger().info("Detecting curb to prepare for ascend.")
        self.enable_curb_detection(
            True
        )  # enable curb detection to detect curb for navigation
        self.get_logger().debug("Mock detecting curb for 5 seconds.")
        self._mock_delay(
            5.0, self.confirm
        )  # should enter Nav_ascending state after detecting curb for testing, will replace with actual logic to determine when curb is detected later

    def on_exit_Nav_ascendDetecting(self):
        self.enable_curb_detection(
            False
        )  # ensure curb detection is disabled when exiting detecting curb state

    def on_enter_Nav_ascending(self):
        self.get_logger().info("Starting curb ascend.")
        self.base_self_leveling_enable(False)
        self.request_curb_traverse(CurbTraverseDirection.ASCEND)

    def on_enter_Nav_descending(self):
        self.get_logger().info("Starting curb descend.")
        self.base_self_leveling_enable(False)
        self.request_curb_traverse(CurbTraverseDirection.DESCEND)

    # -----------------------end of curb navigation transtion functions----------------------

    # ---------Arm Pause state transition function calls---------------------------------
    def on_enter_Arm_Paused(self):
        self.get_logger().info("Arm is paused. Waiting for resume command.")
        self.base_drive_enable(False)  # disable drive while arm is paused
        self.enable_cup_stabilizer(
            False
        )  # disable cup stabilizer if it is on when arm is paused to avoid potential interference with pausing arm
        self.set_arm_mode_idle()  # set arm mode to idle when paused to stop any ongoing arm action

    def on_exit_Arm_Paused(self):
        self.base_drive_enable(True)  # re-enable drive when exiting paused state

    # ---------End of Arm Pause state transition function calls--------------------------

    # ---------arm manual control state transition function calls------------------------
    def on_enter_Arm_manual(self):
        self.get_logger().info(
            "Arm is in manual control mode, waiting for manual commands."
        )
        self.base_drive_enable(False)  # disable drive while in manual arm control
        self.set_arm_mode(ArmMode.MANUAL)  # set arm mode to manual for manual control

        # mock 5s manual control, then disable it TODO: remove mock
        self.get_logger().debug("Mock manual control for 5 seconds.")
        self._mock_delay(
            5.0, self.exitManualControl
        )  # should enter Arm_retracted state after manual control for testing, will replace with actual logic to determine when to exit manual control later

    def on_exit_Arm_manual(self):
        self.get_logger().info("Exiting manual control mode, setting arm back to idle.")
        self.base_drive_enable(True)  # re-enable drive when exiting manual arm control
        self.set_arm_mode_idle()  # set arm back to idle after exiting manual control
        self._mock_state.finish_current_mock_task()  # for testing, will remove after testing

    # ---------end of arm manual control state transition function calls-----------------

    # ---------cup stabilizer state transition function calls------------------------
    def on_enter_Arm_cupStabilize_moving(self):
        self.get_logger().info("Moving arm to cup stabilize position.")
        self.base_drive_enable(
            False
        )  # disable drive while moving arm to cup stabilize position
        self.arm_preset_client.set_preset(ArmPreset.CUP_STABILIZE)

    def on_exit_Arm_cupStabilize_moving(self):
        self.base_drive_enable(
            True
        )  # re-enable drive when exiting moving to cup stabilize position state

    def on_enter_Arm_cupStabilize_stabilizing(self):
        self.get_logger().info("Arm is stabilizing cup.")
        self.set_arm_mode(
            ArmMode.CUP_STABILIZE
        )  # set arm mode to cup stabilize when stabilizing cup, will set specific arm preset in the action server later
        self.enable_cup_stabilizer(True)  # enable cup stabilizer to stabilize the cup

        self.get_logger().debug("Mock stabilizing cup for 5 seconds.")
        self._mock_delay(
            5.0, self.reqCupStabilizeOff
        )  # should enter Arm_cupStabilize_homing state after stabilizing cup for testing, will replace with actual logic to determine when to turn off cup stabilizer later

    # def on_exit_Arm_cupStabilize_stabilizing(self):
    #     # ensure cup stabilizer is disabled when exiting stabilizing cup state

    def on_enter_Arm_cupStabilize_homing(self):
        self.get_logger().info("Homing arm after cup stabilization.")
        self.enable_cup_stabilizer(False)
        self.set_arm_mode_idle()  # set arm back to idle when exiting stabilizing cup state

        self.base_drive_enable(
            False
        )  # disable drive while homing arm after cup stabilization
        self.arm_preset_client.set_preset(
            ArmPreset.HOME
        )  # move arm back to home after stabilizing cup

    def on_exit_Arm_cupStabilize_homing(self):
        self.base_drive_enable(
            True
        )  # re-enable drive when exiting homing arm after cup stabilization state

    # ---------end of cup stabilizer state transition function calls-----------------

    # ---------order drink state transition function calls------------------------
    def on_enter_Arm_OrderDrink_pickUpCup(self):
        self.get_logger().info("Picking up cup from table.")
        self.base_drive_enable(False)  # disable drive while picking up cup
        self.set_arm_mode(
            ArmMode.ORDER_DRINK
        )  # set arm mode to order drink when picking up cup, will set specific arm preset in the action server later
        self.pickup_and_order_client.send_goal()

    def on_exit_Arm_OrderDrink_pickUpCup(self):
        self.base_drive_enable(True)  # re-enable drive when exiting pick up cup state

    def on_enter_Arm_OrderDrink_holdingCup(self):
        self.get_logger().info("Holding cup, preparing to release cup.")
        self.set_arm_mode(ArmMode.IDLE)
        # mock wait for 5 seconds to simulate holding cup, will replace with actual logic later
        self.get_logger().debug("Mock holding cup for 5 seconds.")
        self._mock_delay(5.0, self.releaseCupConfirm)  # should enter releasingCup state

    def on_enter_Arm_OrderDrink_releasingCup(self):
        self.get_logger().info("Releasing cup to holder.")
        self.base_drive_enable(False)  # disable drive while releasing cup
        self.close_gripper()  # close gripper to release cup
        self.arm_preset_client.set_preset(ArmPreset.HOME)

    def on_exit_Arm_OrderDrink_releasingCup(self):
        self.base_drive_enable(True)  # re-enable drive when exiting releasing cup state

    def on_enter_Arm_OrderDrink_detectingDrink(self):
        self.get_logger().info("Detecting drink to confirm if the drink is received.")
        self.enable_cup_detection(True)  # enable cup detection
        # mock wait for 5 seconds to simulate drink detection, will replace with actual logic later
        self.get_logger().debug("Mock detecting drink for 5 seconds.")
        self._mock_delay(5.0, self.receiveDrinkConfirm)

    def on_exit_Arm_OrderDrink_detectingDrink(self):
        self.enable_cup_detection(
            False
        )  # ensure cup detection is disabled when exiting detecting drink state

    def on_enter_Arm_OrderDrink_receivingDrink(self):
        self.get_logger().info("Receiving drink.")
        self.base_drive_enable(False)  # disable drive
        self.set_arm_mode(
            ArmMode.ORDER_DRINK
        )  # set arm mode to drinking when receiving drink, will set specific arm preset in the action server later
        self.grab_cup_from_table_client.send_goal()  # reuse grab cup from table action to simulate receiving drink, will replace with actual pickup drink action later

    def on_exit_Arm_OrderDrink_receivingDrink(self):
        self.base_drive_enable(
            True
        )  # re-enable drive when exiting receiving drink state

    def on_enter_Arm_Drink_bringCloser(self):
        self.get_logger().info("Bringing cup closer to prepare for drinking.")
        self.base_drive_enable(False)  # disable drive while bringing cup to mouth
        self.set_arm_mode(
            ArmMode.DRINKING
        )  # set arm mode to drinking when bringing cup to mouth, will set specific arm preset in the action server later
        self.bring_cup_to_mouth_client.send_goal()  # send action goal to bring cup to mouth for drinking

    def on_exit_Arm_Drink_bringCloser(self):
        self.base_drive_enable(
            True
        )  # re-enable drive when exiting bringing cup closer state

    def on_enter_Arm_Drink_sipping(self):
        self.get_logger().info("Sipping drink.")
        # mock wait for 5 seconds to simulate sipping drink, will replace with actual sipping logic later
        self.get_logger().debug("Mock sipping drink for 5 seconds.")
        self._mock_delay(5.0, self.placeCupAway)  # should enter placingCupAway state

    def on_enter_Arm_Drink_placingCupAway(self):
        self.get_logger().info("Placing cup away after drinking.")
        self.base_drive_enable(False)  # disable drive while placing cup away
        self.home_cup_client.send_goal()  # send action goal to move cup back to holder after drinking, will replace with actual place cup logic later

    def on_exit_Arm_Drink_placingCupAway(self):
        self.base_drive_enable(
            True
        )  # re-enable drive when exiting placing cup away state

    def on_enter_Arm_Drink_placingCupBack(self):
        self.get_logger().info("Placing cup back to holder.")
        self.base_drive_enable(False)  # disable drive while placing cup back to holder
        self.set_arm_mode(
            ArmMode.DRINKING
        )  # set arm mode to DRINKING when placing cup back to holder, will set specific arm preset in the action server later
        self.put_cup_back_to_holder_client.send_goal()  # send action goal to place cup back to holder, will replace with actual place cup back logic later

    def on_exit_Arm_Drink_placingCupBack(self):
        self.base_drive_enable(
            True
        )  # re-enable drive when exiting placing cup back to holder state

    def on_enter_Nav_SLOff(self):
        self.get_logger().info("Self-leveling is turned off.")
        self.base_self_leveling_enable(
            False
        )  # disable self-leveling when entering SLOff state

    def on_enter_Nav_SLOn(self):
        self.get_logger().info("Self-leveling is turned on.")
        self.base_self_leveling_enable(
            True
        )  # enable self-leveling when entering SLOn state

    def on_exit_Nav_SLOn(self):
        self.get_logger().info("Exiting self-leveling on state.")
        self.base_self_leveling_enable(
            False
        )  # disable self-leveling when exiting SLOn state

    # --------------------end of order drink state transition function calls --------------------------

    # --------------------door open state transition function calls------------------------
    def on_enter_Arm_Door_raisingArm(self):
        self.get_logger().info("Raising arm to home to open door.")
        self.arm_preset_client.set_preset(ArmPreset.HOME)

    def on_enter_Arm_Door_detecting(self):
        self.get_logger().info("Detecting door button.")
        if not self.enable_door_detection(True):
            self.get_logger().warn("Failed to enable door detection.")
            self.ArmActionFailed()
        # mock wait for 5 seconds to simulate door button detection, will replace with actual detection logic later
        self.get_logger().debug("Mock detecting door button for 5 seconds.")
        self._mock_delay(
            5.0, self.openDoorConfirm
        )  # should enter Arm_Door_opening state

    def on_exit_Arm_Door_detecting(self):
        self.enable_door_detection(
            False
        )  # ensure door detection is disabled when exiting detecting door state

    def on_enter_Arm_Door_opening(self):
        self.get_logger().info("Sending open door action goal.")
        # disable door detection to avoid interference during door opening
        self.base_drive_enable(False)  # disable drive while opening door
        self.set_arm_mode(ArmMode.OPEN_DOOR)
        self.open_door_client.send_goal()

    def on_exit_Arm_Door_opening(self):
        self.base_drive_enable(True)  # re-enable drive when exiting door opening state

    def on_enter_Arm_retracted(self):
        self.get_logger().debug("Arm is retracted, ready for next command.")
        self.set_arm_mode(ArmMode.IDLE)

    def on_enter_Nav_paused(self):
        self.get_logger().debug("Navigation is paused.")
        self.base_drive_enable(False)  # disable base drive to pause navigation

    def on_enter_Arm_retracting(self):
        self.get_logger().info("Retracting arm to prepare for next action.")
        self.arm_preset_client.set_preset(ArmPreset.RETRACT)

    def on_enter_Arm_homing(self):
        self.get_logger().info("Moving arm to home position.")
        self.arm_preset_client.set_preset(ArmPreset.HOME)

    def after_seat_control(self):
        if self._seat_control_request is None:
            self.get_logger().warn("No seat control command to process.")
            return
        self.get_logger().info(
            f"Processing seat control command: {self._seat_control_request}"
        )
        self.request_manual_seat_control(
            self._seat_control_request
        )  # send manual seat control command to base
        self._seat_control_request = None  # reset seat control request after processing

    def on_enter_Arm_canceling(self):
        self.get_logger().info("Preparing to cancel current action.")
        # try to cancel all action, none running action will do nothing, safe to call.
        self.arm_preset_client.cancel()
        self.bring_cup_to_mouth_client.cancel()
        self.grab_cup_from_table_client.cancel()
        self.home_cup_client.cancel()
        self.open_door_client.cancel()
        self.pickup_and_order_client.cancel()
        self.put_cup_back_to_holder_client.cancel()
        self.set_arm_mode(ArmMode.IDLE)

    def on_enter_Nav_canceling(self):
        self.get_logger().info("Navigation canceling requested.")
        self.curb_traverse_client.cancel()
        self.enable_curb_detection(False)

    # --------------------end of Door Open State Transition Functions------------------------

    def init_publisher(self):
        # publishers
        self.base_manual_seat_control_publisher = self.create_publisher(
            String, "/base/manual_seat_control", 10
        )  # message type is placeholder
        self._seat_control_request = None  # to store seat control command for testing, will replace with actual logic to handle different seat control commands later
        self.system_state_publisher = self.create_publisher(
            String, "/system/state", 10
        )  # message type is placeholder
        self.system_state_publisher_timer = self.create_timer(
            0.1, self.publish_system_state, callback_group=self._cb_group
        )

    def publish_system_state(self):
        # Placeholder for publishing system state, will replace with actual system state message later
        msg = String()
        msg.data = self.state
        self.system_state_publisher.publish(msg)

    def init_subscribers(self):
        self.arm_status_subscriber = self.create_subscription(
            DiagnosticStatus,
            "/arm/status",
            self.arm_status_callback,
            10,
            callback_group=self._cb_group,
        )
        self._gui_connected = False
        self.Gui_connection_subscriber = self.create_subscription(
            Bool,
            "/GuiBridge/gui_connection",
            self.Gui_connection_callback,
            10,
            callback_group=self._cb_group,
        )

    def init_services(self):
        self._service_cb_group = ReentrantCallbackGroup()
        self.set_mode_client = self.create_client(
            SetMode, "/arm/set_mode", callback_group=self._service_cb_group
        )
        self.open_gripper_client = self.create_client(
            Trigger, "/arm/open_gripper", callback_group=self._service_cb_group
        )
        self.close_gripper_client = self.create_client(
            Trigger, "/arm/close_gripper", callback_group=self._service_cb_group
        )
        self.set_speed_client = self.create_client(
            SetSpeedPreset,
            "/arm/set_speed_preset",
            callback_group=self._service_cb_group,
        )
        self.door_button_detection_client = self.create_client(
            SetBool, "/arm/door/detection/enable", callback_group=self._service_cb_group
        )
        self.cup_detection_client = self.create_client(
            SetBool,
            "/arm/drink/detection/enable",
            callback_group=self._service_cb_group,
        )
        self.cup_stabilizer_client = self.create_client(
            SetBool,
            "/arm/drink/stabilize/enable",
            callback_group=self._service_cb_group,
        )
        self.curb_detection_client = self.create_client(
            SetBool,
            "/nav/curb/detect",
            callback_group=self._service_cb_group,
        )
        self.base_drive_enable_client = self.create_client(
            SetBool,
            "/base/drive_enable",
            callback_group=self._service_cb_group,
        )
        self.base_self_leveling_client = self.create_client(
            SetBool,
            "/base/self_level_enable",
            callback_group=self._service_cb_group,
        )
        self.create_service(
            UserInputs,
            "/GuiBridge/user_input",
            self._srv_user_inputs_callback,
            callback_group=self._service_cb_group,
        )

    def init_actions_clients(self):
        self.arm_preset_client = ArmPresetActionClient(self)
        self.pickup_and_order_client = PickUpAndOrderActionClient(self)
        self.home_cup_client = HomeCupActionClient(self)
        self.grab_cup_from_table_client = GrabCupFromTableActionClient(self)
        self.put_cup_back_to_holder_client = PutCupBackToHolderActionClient(self)
        self.bring_cup_to_mouth_client = BringCupToMouthActionClient(self)
        self.open_door_client = OpenDoorActionClient(self)
        self.curb_traverse_client = ChairCurbTraverseActionClient(self)

    # ----------Helper functions to call services and actions for state transitions----------------
    def set_arm_mode_idle(self):
        return self.set_arm_mode(ArmMode.IDLE)

    def set_arm_mode(self, mode: ArmMode) -> bool:
        req = SetMode.Request()
        req.mode = mode.value
        future = self.set_mode_client.call_async(req)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait(timeout=5.0)
        self.get_logger().debug(
            "set_arm_mode to " + mode.name + " result: " + str(future.result())
        )
        if not future.done():
            return False
        if future.result() is not None:
            return future.result().success
        else:
            return False

    def open_gripper(self) -> bool:
        future = self.open_gripper_client.call_async(Trigger.Request())
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait(timeout=5.0)
        if not future.done():
            return False
        if future.result() is not None:
            return future.result().success
        else:
            return False

    def enable_door_detection(self, enable: bool) -> bool:
        req = SetBool.Request()
        req.data = enable
        future = self.door_button_detection_client.call_async(req)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait(timeout=5.0)
        if not future.done():
            return False
        if future.result() is not None:
            return future.result().success
        else:
            return False

    def enable_cup_detection(self, enable: bool) -> bool:
        req = SetBool.Request()
        req.data = enable
        future = self.cup_detection_client.call_async(req)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait(timeout=5.0)
        if not future.done():
            return False
        if future.result() is not None:
            return future.result().success
        else:
            return False

    def enable_cup_stabilizer(self, enable: bool) -> bool:
        req = SetBool.Request()
        req.data = enable
        future = self.cup_stabilizer_client.call_async(req)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait(timeout=5.0)
        if not future.done():
            return False
        if future.result() is not None:
            return future.result().success
        else:
            return False

    def close_gripper(self) -> bool:
        future = self.close_gripper_client.call_async(Trigger.Request())
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait(timeout=5.0)
        if not future.done():
            return False
        if future.result() is not None:
            return future.result().success
        else:
            return False

    def enable_curb_detection(self, enable: bool) -> bool:
        req = SetBool.Request()
        req.data = enable
        future = self.curb_detection_client.call_async(req)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait(timeout=5.0)
        if not future.done():
            return False
        if future.result() is not None:
            return future.result().success
        else:
            return False

    def base_drive_enable(self, enable: bool) -> bool:
        req = SetBool.Request()
        req.data = enable
        future = self.base_drive_enable_client.call_async(req)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait(timeout=5.0)
        if not future.done():
            return False
        if future.result() is not None:
            return future.result().success
        else:
            return False

    def base_self_leveling_enable(self, enable: bool) -> bool:
        req = SetBool.Request()
        req.data = enable
        future = self.base_self_leveling_client.call_async(req)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        event.wait(timeout=5.0)
        if not future.done():
            return False
        if future.result() is not None:
            return future.result().success
        else:
            return False

    def request_curb_traverse(self, direction: CurbTraverseDirection):
        self.curb_traverse_client.send_goal(direction)

    def request_manual_seat_control(self, command: str):
        msg = String()
        msg.data = command
        self.base_manual_seat_control_publisher.publish(msg)

    # ----------End of Helper functions to call services and actions for state transitions----------------

    # ----------publisher / service callback functions to process messages and update internal state----------------
    def arm_status_callback(self, msg: DiagnosticStatus):
        # Placeholder for processing arm status messages
        if self._arm_status != msg.message:
            self.get_logger().info(f" arm status: {self._arm_status} --> {msg.message}")
            self._arm_status = msg.message

    def Gui_connection_callback(self, msg: Bool):
        # Placeholder for processing GUI connection status, will replace with actual logic to handle GUI connection status later
        if msg.data:
            if not self._gui_connected:
                self.get_logger().info(
                    "GUI connection state changed: disconnected --> connected"
                )
            self._gui_connected = True
        else:
            if self._gui_connected:
                self.get_logger().warn(
                    "GUI connection state changed: connected --> disconnected"
                )
                self.UIDisconnected()
            self._gui_connected = False

    def _srv_user_inputs_callback(
        self, request: UserInputs.Request, response: UserInputs.Response
    ):
        # Placeholder for processing user inputs from GUI, will replace with actual logic to handle different user inputs later
        self.get_logger().info(f"Received user input: {request.input}")
        match request.input:
            case UserInputs.Request.CHAIR_CONTROL_MAIN:
                self.reqChair()
            case UserInputs.Request.CHAIR_SELFLEVELING_ON:
                self.enableSL()
            case UserInputs.Request.CHAIR_SELFLEVELING_OFF:
                self.disableSL()
            case (
                UserInputs.Request.CHAIR_SEAT_ELEVATE_UP
                | UserInputs.Request.CHAIR_SEAT_ELEVATE_DOWN
                | UserInputs.Request.CHAIR_SEAT_ELEVATE_HOME
                | UserInputs.Request.CHAIR_SEAT_RECLINE_FORWARD
                | UserInputs.Request.CHAIR_SEAT_ELEVATE_RECLINE_BACK
                | UserInputs.Request.CHAIR_SEAT_ELEVATE_RECLINE_HOME
                | UserInputs.Request.CHAIR_SEAT_ELEVATE_LTILT_LEFT
                | UserInputs.Request.CHAIR_SEAT_ELEVATE_LTILT_RIGHT
                | UserInputs.Request.CHAIR_SEAT_ELEVATE_LTILT_HOME
                | UserInputs.Request.CHAIR_SEAT_HOME
            ):
                self._seat_control_request = request.input  # store the seat control command for testing, will replace with actual logic to handle different seat control commands later
                self.seatControl()
            case UserInputs.Request.CHAIR_CURB_NAVIGATION:
                self.reqNav()
            case UserInputs.Request.CHAIR_CURB_ASCEND:
                self.reqAscend()
            case UserInputs.Request.CHAIR_CURB_DESCEND:
                self.reqDescend()
            case UserInputs.Request.CHAIR_CURB_CANCEL:
                self.reqNavCancel()  # should enter Nav_paused state and pause curb traverse for testing, will replace with actual logic to determine when to pause curb traverse later
            case UserInputs.Request.ARM_CONTROL_MAIN:
                self.reqArm()
            case UserInputs.Request.ARM_RETRACT:
                self.retract()
            case UserInputs.Request.ARM_HOME:
                self.reqHome()
            case UserInputs.Request.ARM_MANUAL_ON:
                self.reqManualControl()
            case UserInputs.Request.ARM_MANUAL_OFF:
                self.exitManualControl()
            case UserInputs.Request.ARM_OPEN_DOOR:
                self.reqOpenDoor()
            case UserInputs.Request.ARM_OPEN_DOOR_CONFIRM:
                self.openDoorConfirm()
            case UserInputs.Request.ARM_ORDER_DRINK:
                self.reqOrderDrink()
            case UserInputs.Request.ARM_ORDER_DRINK_RELEASE_CUP:
                self.releaseCupConfirm()
            case UserInputs.Request.ARM_ORDER_DRINK_RECEIVE:
                self.detectDrink()
            case UserInputs.Request.ARM_ORDER_DRINK_RECEIVE_CONFIRM:
                self.receiveDrinkConfirm()
            case UserInputs.Request.ARM_CUP_STABLE_ON:
                self.reqStabilizeCup()
            case UserInputs.Request.ARM_CUP_STABLE_OFF:
                self.reqCupStabilizeOff()
            case UserInputs.Request.ARM_DRINKING_START:
                self.reqDrink()
            case UserInputs.Request.ARM_DRINKING_FINISH:
                self.placeCupAway()
            case UserInputs.Request.ARM_CUP_BACK:
                self.placeCupBack()
            case UserInputs.Request.ARM_CANCEL:
                self.reqArmActionCancel()  # should try to cancel current arm action and enter Arm_paused state for testing, will replace with actual logic to determine when to cancel arm action later
            case UserInputs.Request.RESET:
                self.reset()  # should reset the system and enter init state for testing, will replace with actual logic to determine when to reset system later
            case UserInputs.Request.ESTOP:
                self.eStop()  # should enter error state for testing, will replace with actual logic to determine when to enter error state later
            case UserInputs.Request.CONFIRM:
                self.confirm()  # placeholder for confirm action, will replace with actual logic to handle confirm action later
            case UserInputs.Request.CANCEL:
                self.cancel()  # placeholder for cancel action, will replace with actual logic to handle cancel action later
            case _:
                response.success = False
                response.message = "Unknown command"
                self.get_logger().warn(f"Received unknown command: {request.input}")
                return response
        response.success = True  # always return success.
        response.message = "Command executed"
        return response

    # node name monitor callback
    def node_monitor_callback(self, all_nodes_ready):
        if all_nodes_ready:
            self.get_logger().info("All nodes are ready!")
            self._all_node_ready = True
            self.ready()  # trigger transition to Chair state when all nodes are ready
            # TODO: need check UI connection as well
            # wait 2 senconds to start mock testing here
            # self._test_timer.reset()  # reset and start the timer to run mock tasks

        else:
            self.get_logger().warn("Some nodes are missing!")
            self.eStop()  # trigger transition to error state when nodes are missing
            self._all_node_ready = False

    # ----------End of publisher callback functions to process messages and update internal state----------------

    # ----------state machine conditions----------------
    def is_arm_state_good_for_driving(self):
        # several arm state is good for driving, e.g., retracted, home, homeWithDrink, cub_Stabilizing
        if self.state in [
            "Arm_retracted",
            "Arm_home",
            "Arm_homeWithDrink",
            "Arm_cupStabilize_stabilizing",
        ]:
            self.current_arm_state = self.state
            return True
        return False

    def is_nav_state_good_for_driving(self):
        # Placeholder for actual logic to determine if the navigation state is good for driving
        return True

    def is_arm_at_home(self):
        return self.current_arm_state == "Arm_home"

    def is_arm_stable_cup(self):
        return self.current_arm_state == "Arm_cupStabilize_stabilizing"

    def is_arm_holding_drink(self):
        return self.current_arm_state == "Arm_homeWithDrink"

    def is_arm_retracted(self):
        return self.current_arm_state == "Arm_retracted" or self.current_arm_state == ""

    # ----------End of state machine conditions----------------
    # ----------state machine callbacks----------------
    def on_enter_Chair(self):
        # for testing
        self.get_logger().info("Entering Chair state")
        # TODO: enable chair control

    def on_enter_Error(self):
        self.get_logger().info("Entering Error state")

    def on_enter_Arm(self):
        self.get_logger().info("Entering Arm state")

    # ----------End of state machine callbacks----------------

    # ----------state machine setup----------------
    def init_state_machine(self):
        states = [
            "init",
            "Chair",
            {
                "name": "Arm",
                "initial": "retracted",
                "children": [
                    "homeWithDrink",
                    "home",
                    "homing",
                    "retracting",
                    "retracted",
                    "manual",
                    {
                        "name": "Door",
                        "initial": "raisingArm",
                        "children": ["raisingArm", "detecting", "opening"],
                    },
                    {
                        "name": "OrderDrink",
                        "initial": "pickUpCup",
                        "children": [
                            "pickUpCup",
                            "holdingCup",
                            "releasingCup",
                            "detectingDrink",
                            "receivingDrink",
                        ],
                    },
                    {
                        "name": "Drink",
                        "initial": "bringCloser",
                        "children": [
                            "bringCloser",
                            "sipping",
                            "placingCupAway",
                            "placingCupBack",
                        ],
                    },
                    {
                        "name": "cupStabilize",
                        "initial": "moving",
                        "children": ["moving", "stabilizing", "homing"],
                    },
                    "paused",  # arm error, will pause arm to maintain current state.
                    "canceling",
                ],
            },
            {
                "name": "Nav",
                "initial": "SLOff",
                "children": [
                    "SLOff",
                    "SLOn",
                    "ascendDetecting",
                    "descendDetecting",
                    "ascending",
                    "descending",
                    "canceling",
                    "paused",
                ],
            },
            "Error",  # system error state, will require reset to recover
        ]

        transitions = [
            # arm sub state transitions
            {
                "trigger": "retract",
                "source": ["Arm_paused", "Arm_home", "Arm_manual"],
                "dest": "Arm_retracting",
            },
            {
                "trigger": "retracted",
                "source": "Arm_retracting",
                "dest": "Arm_retracted",
            },
            {
                "trigger": "reqArmActionCancel",
                "source": [
                    "Arm_homing",
                    "Arm_retracting",
                    "Arm_Drink_bringCloser",
                    "Arm_Drink_placingCupAway",
                    "Arm_Drink_placingCupBack",
                    "Arm_OrderDrink_pickUpCup",
                    "Arm_OrderDrink_releasingCup",
                    "Arm_OrderDrink_receivingDrink",
                    "Arm_Door_raisingArm",
                    "Arm_Door_opening",
                    "Arm_cupStabilize_moving",
                    "Arm_cupStabilize_homing",
                ],
                "dest": "Arm_canceling",
            },  # request to cancel current arm action, move to canceling sub state to attempt cancel
            {
                "trigger": "reqArmActionCancel",
                "source": [
                    "Arm_OrderDrink_detectingDrink",
                    "Arm_Door_detecting",
                    "Arm_cupStabilize_stabilizing",
                ],
                "dest": "Arm_paused",
            },  # request to cancel current service, move to paused sub state
            {
                "trigger": "cancel",
                "source": [
                    "Arm_homing",
                    "Arm_retracting",
                    "Arm_Drink_bringCloser",
                    "Arm_Drink_placingCupAway",
                    "Arm_Drink_placingCupBack",
                    "Arm_OrderDrink_pickUpCup",
                    "Arm_OrderDrink_releasingCup",
                    "Arm_OrderDrink_receivingDrink",
                    "Arm_Door_raisingArm",
                    "Arm_Door_opening",
                    "Arm_cupStabilize_moving",
                    "Arm_cupStabilize_homing",
                ],
                "dest": "Arm_canceling",
            },  # request to cancel current arm action, move to canceling sub state to attempt cancel
            {
                "trigger": "cancel",
                "source": [
                    "Arm_OrderDrink_detectingDrink",
                    "Arm_Door_detecting",
                    "Arm_cupStabilize_stabilizing",
                ],
                "dest": "Arm_paused",
            },  # request to cancel current service, move to paused sub state
            {
                "trigger": "reqArmActionCancelSuccess",
                "source": "Arm_canceling",
                "dest": "Arm_paused",
            },  # action canceled successfully, pause arm to maintain current state and allow for retry or other recovery actions
            {
                "trigger": "reqArmActionCancelFailed",
                "source": "Arm_canceling",
                "dest": "Error",
            },  # can not cancel action, treat as error and require reset
            {
                "trigger": "ArmActionFailed",
                "source": "Arm",
                "dest": "Arm_paused",
            },  # action finished with failure, pause arm to maintain current state and allow for retry or other recovery actions
            {
                "trigger": "reqArmActionGoalFailed",
                "source": "Arm",
                "dest": "Arm_paused",
            },  # request action, but fail to start action, pause arm to maintain current state and allow for retry or other recovery actions
            {
                "trigger": "reqHome",
                "source": ["Arm_retracted", "Arm_paused", "Arm_manual"],
                "dest": "Arm_homing",
            },
            {
                "trigger": "homed",
                "source": [
                    "Arm_homing",
                    "Arm_OrderDrink_releasingCup",
                ],
                "dest": "Arm_home",
            },
            # open door
            {
                "trigger": "reqOpenDoor",
                "source": ["Arm_home", "Arm_retracted"],
                "dest": "Arm_Door_raisingArm",
            },
            {
                "trigger": "homed",
                "source": "Arm_Door_raisingArm",
                "dest": "Arm_Door_detecting",
            },
            {
                "trigger": "openDoorConfirm",
                "source": "Arm_Door_detecting",
                "dest": "Arm_Door_opening",
            },
            {
                "trigger": "confirm",
                "source": "Arm_Door_detecting",
                "dest": "Arm_Door_opening",
            },
            {
                "trigger": "doorOpenFinished",
                "source": "Arm_Door_opening",
                "dest": "Arm_retracted",
            },
            # order drink
            {
                "trigger": "reqOrderDrink",
                "source": ["Arm_home", "Arm_retracted"],
                "dest": "Arm_OrderDrink_pickUpCup",
            },
            {
                "trigger": "pickedUpCup",
                "source": "Arm_OrderDrink_pickUpCup",
                "dest": "Arm_OrderDrink_holdingCup",
            },
            {
                "trigger": "releaseCupConfirm",
                "source": "Arm_OrderDrink_holdingCup",
                "dest": "Arm_OrderDrink_releasingCup",
            },
            {
                "trigger": "confirm",
                "source": "Arm_OrderDrink_holdingCup",
                "dest": "Arm_OrderDrink_releasingCup",
            },
            {
                "trigger": "detectDrink",
                "source": "Arm_home",
                "dest": "Arm_OrderDrink_detectingDrink",
            },
            {
                "trigger": "receiveDrinkConfirm",
                "source": "Arm_OrderDrink_detectingDrink",
                "dest": "Arm_OrderDrink_receivingDrink",
            },
            {
                "trigger": "confirm",
                "source": "Arm_OrderDrink_detectingDrink",
                "dest": "Arm_OrderDrink_receivingDrink",
            },
            {
                "trigger": "receivedDrink",
                "source": "Arm_OrderDrink_receivingDrink",
                "dest": "Arm_homeWithDrink",
            },
            # stabilize cup
            {
                "trigger": "reqStabilizeCup",
                "source": "Arm_homeWithDrink",
                "dest": "Arm_cupStabilize_moving",
            },
            {
                "trigger": "cupStable",
                "source": "Arm_cupStabilize_moving",
                "dest": "Arm_cupStabilize_stabilizing",
            },
            {
                "trigger": "reqCupStabilizeOff",
                "source": "Arm_cupStabilize_stabilizing",
                "dest": "Arm_cupStabilize_homing",
            },
            {
                "trigger": "reqHome",
                "source": "Arm_cupStabilize_stabilizing",
                "dest": "Arm_cupStabilize_homing",
            },
            {
                "trigger": "homed",
                "source": "Arm_cupStabilize_homing",
                "dest": "Arm_homeWithDrink",
            },
            # drink
            {
                "trigger": "reqDrink",
                "source": "Arm_homeWithDrink",
                "dest": "Arm_Drink_bringCloser",
            },
            {
                "trigger": "readyForDrink",
                "source": "Arm_Drink_bringCloser",
                "dest": "Arm_Drink_sipping",
            },
            {
                "trigger": "placeCupAway",
                "source": "Arm_Drink_sipping",
                "dest": "Arm_Drink_placingCupAway",
            },
            {
                "trigger": "reqHome",
                "source": "Arm_Drink_sipping",
                "dest": "Arm_Drink_placingCupAway",
            },
            {
                "trigger": "homed",
                "source": "Arm_Drink_placingCupAway",
                "dest": "Arm_homeWithDrink",
            },
            {
                "trigger": "placeCupBack",
                "source": "Arm_homeWithDrink",
                "dest": "Arm_Drink_placingCupBack",
            },
            {
                "trigger": "retract",
                "source": "Arm_homeWithDrink",
                "dest": "Arm_Drink_placingCupBack",
            },
            {
                "trigger": "cupPlacedBack",
                "source": "Arm_Drink_placingCupBack",
                "dest": "Arm_retracted",
            },
            # manual control
            {
                "trigger": "reqManualControl",
                "source": [
                    "Arm_home",
                    "Arm_retracted",
                    "Arm_paused",
                    "Arm_homeWithDrink",
                ],
                "dest": "Arm_manual",
            },
            {
                "trigger": "exitManualControl",
                "source": "Arm_manual",
                "dest": "Arm_paused",
            },
            # global transitions
            {"trigger": "eStop", "source": "*", "dest": "Error"},
            {"trigger": "UIDisconnected", "source": "*", "dest": "Error"},
            {"trigger": "ready", "source": "init", "dest": "Chair"},
            {"trigger": "reset", "source": "Error", "dest": "init"},
            # main state transitions
            {
                "trigger": "reqArm",
                "source": "Chair",
                "dest": "Arm_home",
                "conditions": "is_arm_at_home",
            },
            {
                "trigger": "reqArm",
                "source": "Chair",
                "dest": "Arm_cupStabilize_stabilizing",
                "conditions": "is_arm_stable_cup",
            },
            {
                "trigger": "reqArm",
                "source": "Chair",
                "dest": "Arm_retracted",
                "conditions": "is_arm_retracted",
            },
            {
                "trigger": "reqArm",
                "source": "Chair",
                "dest": "Arm_homeWithDrink",
                "conditions": "is_arm_holding_drink",
            },
            {"trigger": "reqNav", "source": "Chair", "dest": "Nav"},
            {
                "trigger": "reqChair",
                "source": "Arm",
                "dest": "Chair",
                "conditions": "is_arm_state_good_for_driving",
            },
            {
                "trigger": "reqChair",
                "source": ["Nav_SLOff", "Nav_SLOn", "Nav_paused"],
                "dest": "Chair",
            },
            # chair sub state transitions
            {"trigger": "enableSL", "source": "Nav_SLOff", "dest": "Nav_SLOn"},
            {"trigger": "disableSL", "source": "Nav_SLOn", "dest": "Nav_SLOff"},
            {
                "trigger": "seatControl",
                "source": "Chair",
                "dest": "Chair",
                "after": "after_seat_control",
            },  # seat control command, stay in the same state but call after_seat_control function to process the command
            # navigation sub state transitions
            # {
            #     "trigger": "startAscendConfirm",
            #     "source": "Nav_ascendDetecting",
            #     "dest": "Nav_ascending",
            # },
            {
                "trigger": "confirm",
                "source": "Nav_ascendDetecting",
                "dest": "Nav_ascending",
            },
            {
                "trigger": "reqAscend",
                "source": ["Nav_SLOff", "Nav_SLOn"],
                "dest": "Nav_ascendDetecting",
            },
            {
                "trigger": "reqDescend",
                "source": ["Nav_SLOff", "Nav_SLOn"],
                "dest": "Nav_descendDetecting",
            },
            # {
            #     "trigger": "startDescendConfirm",
            #     "source": "Nav_descendDetecting",
            #     "dest": "Nav_descending",
            # },
            {
                "trigger": "confirm",
                "source": "Nav_descendDetecting",
                "dest": "Nav_descending",
            },
            {
                "trigger": "traverseComplete",
                "source": ["Nav_ascending", "Nav_descending"],
                "dest": "Nav_SLOff",
            },
            {
                "trigger": "reqNavCancel",
                "source": [
                    "Nav_ascending",
                    "Nav_descending",
                ],
                "dest": "Nav_canceling",
            },
            {
                "trigger": "Nav_canceled",
                "source": "Nav_canceling",
                "dest": "Nav_paused",
            },
            {
                "trigger": "cancel",
                "source": [
                    "Nav_ascending",
                    "Nav_descending",
                ],
                "dest": "Nav_canceling",
            },
            {
                "trigger": "cancel",
                "source": ["Nav_ascendDetecting", "Nav_descendDetecting"],
                "dest": "Nav_paused",
            },
            {
                "trigger": "reqNavCancel",
                "source": ["Nav_ascendDetecting", "Nav_descendDetecting"],
                "dest": "Nav_paused",
            },
            {"trigger": "reset", "source": "Nav_paused", "dest": "Nav_SLOff"},
        ]

        self.machine = Machine(
            model=self,
            states=states,
            transitions=transitions,
            initial="init",
            ignore_invalid_triggers=True,
            queued=True,  # ensure thread safety for state transitions
        )

    # ----------End of state machine setup----------------


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    node = SystemControl()
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
