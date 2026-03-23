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


class SystemControl(rclpy.node.Node):
    def __init__(self):
        super().__init__("system_control")
        self.get_logger().info("System Control Node has been started.")
        share_dir = get_package_share_directory("rammp_prototype_behavior")
        json_path = os.path.join(share_dir, "config/node_name.json")
        self.node_monitor = NodeNameMonitor(self, json_path, self.node_monitor_callback)
        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()

        self._arm_status = ""
        self._cb_group = ReentrantCallbackGroup()
        self._last_state = ""

        self.init_subscribers()
        self.init_services_clients()
        self.init_actions_clients()
        self.init_state_machine()
        self._test_timer = self.create_timer(
            1.0, self.log_state, callback_group=self._cb_group
        )  # run simple_test every 10 seconds for testing
        # put test for asyncio actions here for now, will move to separate test file later
        # self.simple_test()

    def log_state(self):
        self.get_logger().info(f"Current state: {self.state}")
        if self.state != self._last_state and self.state == "Chair_SLOff":
            self.get_logger().info("Chair SL has been enabled. mock open door")
            self.mock_open_door_request()
        self._last_state = self.state

    ## mock testing functions
    def mock_open_door_request(self):
        # for testing open door action, will remove after testing
        self.get_logger().info("Sending mock open door request.")
        self.reqArm()  # should enter Arm_retracted state
        self.reqOpenDoor()  # should enter Arm_Door_raisingArm state

    # door open state transition function calls
    def on_enter_Arm_Door_raisingArm(self):
        self.get_logger().info("Raising arm to home to open door.")
        self.arm_preset_client.set_preset(ArmPreset.HOME)
        print("current state after request home:" + self.state)

    def on_enter_Arm_Door_detecting(self):
        self.get_logger().info("Detecting door button.")
        if not self.enable_door_detection(True):
            self.get_logger().warn("Failed to enable door detection.")
            self.ArmActionFailed()
        # mock wait for 5 seconds to simulate door button detection, will replace with actual detection logic later
        self.get_logger().info("Mock detecting door button for 5 seconds.")
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=5))
        self.openDoorConfirm()  # should enter Arm_Door_opening state

    def on_enter_Arm_Door_opening(self):
        self.get_logger().info("Sending open door action goal.")
        # disable door detection to avoid interference during door opening
        if not self.enable_door_detection(False):
            self.get_logger().warn("Failed to disable door detection.")
            self.ArmActionFailed()
        self.set_arm_mode(ArmMode.OPEN_DOOR)
        self.open_door_client.send_goal()

    def on_enter_Arm_retracted(self):
        self.get_logger().info("Arm is retracted, ready for next command.")
        self.set_arm_mode(ArmMode.IDLE)

    # END of Door Open State Transition Functions

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

    def init_actions_clients(self):
        self.arm_preset_client = ArmPresetActionClient(self)
        self.pickup_and_order_client = PickUpAndOrderActionClient(self)
        self.home_cup_client = HomeCupActionClient(self)
        self.grab_cup_from_table_client = GrabCupFromTableActionClient(self)
        self.put_cup_back_to_holder_client = PutCupBackToHolderActionClient(self)
        self.bring_cup_to_mouth_client = BringCupToMouthActionClient(self)
        self.open_door_client = OpenDoorActionClient(self)

    def set_arm_mode(self, mode: ArmMode) -> bool:
        req = SetMode.Request()
        req.mode = mode.value
        future = self.set_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        print("set_arm_mode result: " + str(future.result()))
        if future.result() is not None:
            return future.result().success
        else:
            return False

    def open_gripper(self) -> bool:
        future = self.open_gripper_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().success
        else:
            return False

    def enable_door_detection(self, enable: bool) -> bool:
        req = SetBool.Request()
        req.data = enable
        future = self.door_button_detection_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().success
        else:
            return False

    def close_gripper(self) -> bool:
        future = self.close_gripper_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result().success
        else:
            return False

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
            self.mock_open_door_request()

        else:
            self.get_logger().warn("Some nodes are missing!")
            self.eStop()  # trigger transition to error state when nodes are missing

    # state machine conditions
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

    # state machine callbacks
    def on_enter_Chair(self):
        # for testing
        print("Entering Chair state")
        # TODO: enable chair control

    def on_enter_Error(self):
        print("Entering Error state")

    def on_enter_Arm(self):
        print("Entering Arm state")

    def simple_test(self):
        print("simple test:")
        print("current state:" + self.state)
        print("triggering ready")
        self.ready()
        while self.state != "Chair_SLOff":
            self.get_logger().info("Waiting to enter Chair_SLOff state for testing...")
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1))
        print("current state:" + self.state)
        print("triggering reqArm")
        self.reqArm()
        print("current state:" + self.state)
        print("simple test : request open door")
        self.reqOpenDoor()  # should enter Arm_Door_raisingArm state
        # self.arm_preset_client.set_preset(ArmPreset.HOME)
        print("current state:" + self.state)

    # state machine setup
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
                        "initial": "pickCup",
                        "children": [
                            "pickCup",
                            "releaseCup",
                            "detectingDrink",
                            "pickingUpDrink",
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
                        "children": ["moving", "stable", "homing"],
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
                "dest": "error",
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
                "source": ["Arm_retracted", "Arm_paused", "Arm_cupStabilize_stable"],
                "dest": "Arm_homing",
            },
            {
                "trigger": "reqHome",
                "source": "Arm_cupStabilize_stable",
                "dest": "Arm_cupStabilize_homing",
            },
            {
                "trigger": "homed",
                "source": ["Arm_homing", "Arm_cupStabilize_homing"],
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
                "dest": "Arm_OrderDrink_pickCup",
            },
            {
                "trigger": "releaseCupConfirm",
                "source": "Arm_OrderDrink_pickCup",
                "dest": "Arm_OrderDrink_releaseCup",
            },
            {
                "trigger": "cupReleased",
                "source": "Arm_OrderDrink_releaseCup",
                "dest": "home",
            },
            {
                "trigger": "detectDrink",
                "source": "home",
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
                "dest": "home",
            },
            # stabilize cup
            {
                "trigger": "reqStabilizeCup",
                "source": "Arm_OrderDrink_ordered",
                "dest": "Arm_cupStabilize_moving",
            },
            {
                "trigger": "cupStable",
                "source": "Arm_cupStabilize_moving",
                "dest": "Arm_cupStabilize_stable",
            },
            # drink
            {
                "trigger": "reqDrink",
                "source": "home",
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
                "trigger": "cupPlaced",
                "source": "Arm_Drink_placingCupAway",
                "dest": "home",
            },
            {
                "trigger": "placeCupBack",
                "source": "home",
                "dest": "placingCupBack",
            },
            {
                "trigger": "cupPlacedBack",
                "source": "placingCupBack",
                "dest": "retracted",
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
                "dest": "Arm_cupStabilize_stable",
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
                "dest": "Arm_OrderDrink_ordered",
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
