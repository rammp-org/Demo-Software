import asyncio
import enum
import threading

import rclpy
import rclpy.action
import rclpy.node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import os
from .actionClient.ArmPresetActionClient import ArmPreset, ArmPresetActionClient
from .actionClient.BringCupToMouthActionClient import BringCupToMouthActionClient
from .actionClient.GrabCupFromTableActionClient import GrabCupFromTableActionClient
from .actionClient.HomeCupActionClient import HomeCupActionClient
from .actionClient.PickUpAndOrderActionClient import PickUpAndOrderActionClient
from .actionClient.PutCupBackToHolderActionClient import PutCupBackToHolderActionClient
from .actionClient.OpenDoorActionClient import OpenDoorActionClient
from transitions.extensions import HierarchicalMachine as Machine
from .node_name_monitor import NodeNameMonitor
from ament_index_python.packages import get_package_share_directory
from arm_interfaces.srv import SetMode, SetSpeedPreset
from std_srvs.srv import Trigger, SetBool
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
    END_TASK = 10  # update this when adding new mock tasks


class MockState:
    def __init__(self, node: rclpy.node.Node):
        self._node = node
        self.is_mock_task_running = False
        self.next_mock_task = MockTasks.OPEN_DOOR
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

    def finish_current_mock_task(self):
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

        self._mock_state = MockState(self)

        self._arm_status = ""
        self._cb_group = ReentrantCallbackGroup()
        self.node_monitor = NodeNameMonitor(self, json_path, self.node_monitor_callback)

        self.init_subscribers()
        self.init_services_clients()
        self.init_actions_clients()
        self.init_state_machine()
        self._test_timer = self.create_timer(
            1.0, self.mock_task, callback_group=self._cb_group
        )
        self._test_timer.cancel()  # cancel the timer and reset when system is ready

    def mock_task(self):
        self._mock_state.run_next_mock_task()

    def finish_mock_task(self):
        self._mock_state.finish_current_mock_task()

    def _mock_delay(self, seconds, callback):
        """Non-blocking delay: creates a one-shot timer that calls `callback` after `seconds`."""
        timer = None

        def _on_timeout():
            nonlocal timer
            if timer is not None:
                timer.cancel()
                self.destroy_timer(timer)
                timer = None
            callback()

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

    ## -----------------------------end of mock testing functions------------------------

    # ---------Arm Pause state transition function calls---------------------------------
    def on_enter_Arm_Paused(self):
        self.get_logger().info("Arm is paused. Waiting for resume command.")
        self.set_arm_mode_idle()  # set arm mode to idle when paused to stop any ongoing arm action

    # ---------End of Arm Pause state transition function calls--------------------------

    # ---------arm manual control state transition function calls------------------------
    def on_enter_Arm_manual(self):
        self.get_logger().info(
            "Arm is in manual control mode, waiting for manual commands."
        )
        self.set_arm_mode(ArmMode.MANUAL)  # set arm mode to manual for manual control

        # mock 5s manual control, then disable it
        self.get_logger().info("Mock manual control for 5 seconds.")
        self._mock_delay(
            5.0, self.exitManualControl
        )  # should enter Arm_retracted state after manual control for testing, will replace with actual logic to determine when to exit manual control later

    def on_exit_Arm_manual(self):
        self.get_logger().info("Exiting manual control mode, setting arm back to idle.")
        self.set_arm_mode_idle()  # set arm back to idle after exiting manual control
        self._mock_state.finish_current_mock_task()  # for testing, will remove after testing

    # ---------end of arm manual control state transition function calls-----------------

    # ---------cup stabilizer state transition function calls------------------------
    def on_enter_Arm_cupStabilize_moving(self):
        self.get_logger().info("Moving arm to cup stabilize position.")
        self.arm_preset_client.set_preset(ArmPreset.CUP_STABILIZE)

    def on_enter_Arm_cupStabilize_stabilizing(self):
        self.get_logger().info("Arm is stabilizing cup.")
        self.set_arm_mode(
            ArmMode.CUP_STABILIZE
        )  # set arm mode to cup stabilize when stabilizing cup, will set specific arm preset in the action server later
        self.enable_cup_stabilizer(True)  # enable cup stabilizer to stabilize the cup

        self.get_logger().info("Mock stabilizing cup for 5 seconds.")
        self._mock_delay(
            5.0, self.reqCupStabilizeOff
        )  # should enter Arm_cupStabilize_homing state after stabilizing cup for testing, will replace with actual logic to determine when to turn off cup stabilizer later

    def on_enter_Arm_cupStabilize_homing(self):
        self.get_logger().info("Homing arm after cup stabilization.")
        self.set_arm_mode_idle()  # set arm to idle before homing to avoid potential interference with homing process
        self.enable_cup_stabilizer(False)  # disable cup stabilizer to allow arm to home
        self.arm_preset_client.set_preset(
            ArmPreset.HOME
        )  # move arm back to home after stabilizing cup

    # ---------end of cup stabilizer state transition function calls-----------------

    # ---------order drink state transition function calls------------------------
    def on_enter_Arm_OrderDrink_pickUpCup(self):
        self.get_logger().info("Picking up cup from table.")
        self.set_arm_mode(
            ArmMode.ORDER_DRINK
        )  # set arm mode to order drink when picking up cup, will set specific arm preset in the action server later
        self.pickup_and_order_client.send_goal()

    def on_enter_Arm_OrderDrink_holdingCup(self):
        self.get_logger().info("Holding cup, preparing to release cup.")
        self.set_arm_mode(ArmMode.IDLE)
        # mock wait for 5 seconds to simulate holding cup, will replace with actual logic later
        self.get_logger().info("Mock holding cup for 5 seconds.")
        self._mock_delay(5.0, self.releaseCupConfirm)  # should enter releasingCup state

    def on_enter_Arm_OrderDrink_releasingCup(self):
        self.get_logger().info("Releasing cup to holder.")
        self.get_logger().info("close gripper first.")
        self.close_gripper()  # close gripper to release cup

        # mock wait for 1 seconds to simulate releasing cup, will replace with actual logic later
        self.get_logger().info("Mock close gripper for 1 seconds.")

        def _after_gripper_delay():
            # move arm to home position after releasing cup to avoid potential collision when bringing cup to mouth later
            self.get_logger().info("Move arm to home position after releasing cup.")
            self.arm_preset_client.set_preset(ArmPreset.HOME)

        self._mock_delay(1.0, _after_gripper_delay)

    def on_enter_Arm_OrderDrink_detectingDrink(self):
        self.get_logger().info("Detecting drink to confirm if the drink is received.")
        self.enable_cup_detection(True)  # enable cup detection
        # mock wait for 5 seconds to simulate drink detection, will replace with actual logic later
        self.get_logger().info("Mock detecting drink for 5 seconds.")

        def _after_drink_detect():
            self.enable_cup_detection(
                False
            )  # disable cup detection after detection process
            self.receiveDrinkConfirm()  # should enter receivingDrink state

        self._mock_delay(5.0, _after_drink_detect)

    def on_enter_Arm_OrderDrink_receivingDrink(self):
        self.get_logger().info("Receiving drink.")
        self.set_arm_mode(
            ArmMode.ORDER_DRINK
        )  # set arm mode to drinking when receiving drink, will set specific arm preset in the action server later
        self.grab_cup_from_table_client.send_goal()  # reuse grab cup from table action to simulate receiving drink, will replace with actual pickup drink action later

    def on_enter_Arm_Drink_bringCloser(self):
        self.get_logger().info("Bringing cup closer to prepare for drinking.")
        self.set_arm_mode(
            ArmMode.DRINKING
        )  # set arm mode to drinking when bringing cup to mouth, will set specific arm preset in the action server later
        self.bring_cup_to_mouth_client.send_goal()  # send action goal to bring cup to mouth for drinking

    def on_enter_Arm_Drink_sipping(self):
        self.get_logger().info("Sipping drink.")
        # mock wait for 5 seconds to simulate sipping drink, will replace with actual sipping logic later
        self.get_logger().info("Mock sipping drink for 5 seconds.")
        self._mock_delay(5.0, self.placeCupAway)  # should enter placingCupAway state

    def on_enter_Arm_Drink_placingCupAway(self):
        self.get_logger().info("Placing cup away after drinking.")
        self.home_cup_client.send_goal()  # send action goal to move cup back to holder after drinking, will replace with actual place cup logic later

    def on_enter_Arm_Drink_placingCupBack(self):
        self.get_logger().info("Placing cup back to holder.")
        self.set_arm_mode(
            ArmMode.DRINKING
        )  # set arm mode to DRINKING when placing cup back to holder, will set specific arm preset in the action server later
        self.put_cup_back_to_holder_client.send_goal()  # send action goal to place cup back to holder, will replace with actual place cup back logic later

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
        self.get_logger().info("Mock detecting door button for 5 seconds.")
        self._mock_delay(
            5.0, self.openDoorConfirm
        )  # should enter Arm_Door_opening state

    def on_enter_Arm_Door_opening(self):
        self.get_logger().info("Sending open door action goal.")
        # disable door detection to avoid interference during door opening
        if not self.enable_door_detection(False):
            self.get_logger().warn("Failed to disable door detection.")
            self.ArmActionFailed()
        self.set_arm_mode(ArmMode.OPEN_DOOR)
        self.open_door_client.send_goal()

    def on_enter_Arm_retracted(self):
        self.get_logger().debug("Arm is retracted, ready for next command.")
        self.set_arm_mode(ArmMode.IDLE)

    # --------------------end of Door Open State Transition Functions------------------------

    def init_subscribers(self):
        self.arm_status_subscriber = self.create_subscription(
            DiagnosticStatus,
            "/arm/status",
            self.arm_status_callback,
            10,
            callback_group=self._cb_group,
        )

    def init_services_clients(self):
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

    def init_actions_clients(self):
        self.arm_preset_client = ArmPresetActionClient(self)
        self.pickup_and_order_client = PickUpAndOrderActionClient(self)
        self.home_cup_client = HomeCupActionClient(self)
        self.grab_cup_from_table_client = GrabCupFromTableActionClient(self)
        self.put_cup_back_to_holder_client = PutCupBackToHolderActionClient(self)
        self.bring_cup_to_mouth_client = BringCupToMouthActionClient(self)
        self.open_door_client = OpenDoorActionClient(self)

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

    # ----------End of Helper functions to call services and actions for state transitions----------------

    # ----------publisher callback functions to process messages and update internal state----------------
    def arm_status_callback(self, msg: DiagnosticStatus):
        # Placeholder for processing arm status messages
        if self._arm_status != msg.message:
            self.get_logger().info(f" arm status: {self._arm_status} --> {msg.message}")
            self._arm_status = msg.message

    # node name monitor callback
    def node_monitor_callback(self, all_nodes_ready):
        if all_nodes_ready:
            self.get_logger().info("All nodes are ready!")
            self.ready()  # trigger transition to Chair state when all nodes are ready
            # TODO: need check UI connection as well
            # wait 2 senconds to start mock testing here
            self._test_timer.reset()  # reset and start the timer to run mock tasks

        else:
            self.get_logger().warn("Some nodes are missing!")
            self.eStop()  # trigger transition to error state when nodes are missing

    # ----------End of publisher callback functions to process messages and update internal state----------------

    # ----------state machine conditions----------------
    def is_arm_state_good_for_driving(self):
        # Placeholder for actual logic to determine if the arm state is good for driving
        return True

    def is_nav_state_good_for_driving(self):
        # Placeholder for actual logic to determine if the navigation state is good for driving
        return True

    def is_arm_at_home(self):
        # Placeholder for actual logic to determine if the arm is at the home position
        return False

    def is_arm_stable_cup(self):
        # Placeholder for actual logic to determine if the arm is stable with a cup
        return False

    def is_arm_holding_drink(self):
        # Placeholder for actual logic to determine if the arm is holding a drink
        return False

    def is_arm_retracted(self):
        # Placeholder for actual logic to determine if the arm is retracted
        return True

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
            {"name": "Chair", "initial": "SLOff", "children": ["SLOn", "SLOff"]},
            {
                "name": "Arm",
                "initial": "retracted",
                "children": [
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
                ],
            },
            {
                "name": "Nav",
                "initial": "detecting",
                "children": ["detecting", "detected", "traverse", "finished", "paused"],
            },
            "Error",  # system error state, will require reset to recover
        ]

        transitions = [
            # arm sub state transitions
            {
                "trigger": "retract",
                "source": [
                    "Arm_Drink_finished",
                    "Arm_paused",
                    "Arm_home",
                ],
                "dest": "Arm_retracting",
            },
            {
                "trigger": "retracted",
                "source": "Arm_retracting",
                "dest": "Arm_retracted",
            },
            {
                "trigger": "reqArmActionCancelSuccess",
                "source": "Arm",
                "dest": "Arm_paused",
            },  # action canceled successfully, pause arm to maintain current state and allow for retry or other recovery actions
            {
                "trigger": "reqArmActionCancelFailed",
                "source": "Arm",
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
                "source": ["Arm_retracted", "Arm_paused"],
                "dest": "Arm_homing",
            },
            {
                "trigger": "reqCupStabilizeOff",
                "source": "Arm_cupStabilize_stabilizing",
                "dest": "Arm_cupStabilize_homing",
            },
            {
                "trigger": "homed",
                "source": [
                    "Arm_homing",
                    "Arm_cupStabilize_homing",
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
                "trigger": "cupReleased",
                "source": "Arm_OrderDrink_releasingCup",
                "dest": "Arm_home",
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
                "trigger": "receivedDrink",
                "source": "Arm_OrderDrink_receivingDrink",
                "dest": "Arm_home",
            },
            # stabilize cup
            {
                "trigger": "reqStabilizeCup",
                "source": "Arm_home",
                "dest": "Arm_cupStabilize_moving",
            },
            {
                "trigger": "cupStable",
                "source": "Arm_cupStabilize_moving",
                "dest": "Arm_cupStabilize_stabilizing",
            },
            # drink
            {
                "trigger": "reqDrink",
                "source": "Arm_home",
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
                "trigger": "homedCup",
                "source": "Arm_Drink_placingCupAway",
                "dest": "Arm_home",
            },
            {
                "trigger": "placeCupBack",
                "source": "Arm_home",
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
                "source": ["Arm_home", "Arm_retracted", "Arm_paused"],
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
                "dest": "Arm_home",
                "conditions": "is_arm_holding_drink",
            },
            {"trigger": "reqtNav", "source": "Chair", "dest": "Nav"},
            {
                "trigger": "reqChair",
                "source": "Arm",
                "dest": "Chair",
                "conditions": "is_arm_state_good_for_driving",
            },
            {
                "trigger": "reqChair",
                "source": "Nav",
                "dest": "Chair",
                "conditions": "is_nav_state_good_for_driving",
            },
            # chair sub state transitions
            {"trigger": "enableSL", "source": "Chair_SLOff", "dest": "Chair_SLOn"},
            {"trigger": "disableSL", "source": "Chair_SLOn", "dest": "Chair_SLOff"},
            {"trigger": "seatControl", "source": "Chair_SLOff", "dest": "Chair_SLOff"},
            # navigation sub state transitions
            {
                "trigger": "curbDetected",
                "source": "Nav_detecting",
                "dest": "Nav_detected",
            },
            {
                "trigger": "startTraverseConfirm",
                "source": "Nav_detected",
                "dest": "Nav_traverse",
            },
            {
                "trigger": "traverseComplete",
                "source": "Nav_traverse",
                "dest": "Nav_finished",
            },
            {"trigger": "reqNavCancel", "source": "Nav_traverse", "dest": "Nav_paused"},
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
