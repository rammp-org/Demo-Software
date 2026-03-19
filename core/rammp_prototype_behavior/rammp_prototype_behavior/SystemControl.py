import asyncio
import threading

import rclpy
import rclpy.action
import rclpy.node
from .ArmPresetActionClient import ArmPreset, ArmPresetActionClient
from transitions.extensions import HierarchicalMachine as Machine


class SystemControl(rclpy.node.Node):
    def __init__(self):
        super().__init__("system_control")
        self.get_logger().info("System Control Node has been started.")

        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()

        self.init_subscribers()
        self.init_services_clients()
        self.init_actions_clients()
        self.init_state_machine()
        # put test for asyncio actions here for now, will move to separate test file later
        self.simple_test()

    def init_subscribers(self):
        pass

    def init_services_clients(self):
        pass

    def init_actions_clients(self):
        self.arm_preset_client = ArmPresetActionClient(self)

    # state machine conditions
    def is_arm_state_good_for_driving(self):
        # Placeholder for actual logic to determine if the arm state is good for driving
        return True

    def is_nav_state_good_for_driving(self):
        # Placeholder for actual logic to determine if the navigation state is good for driving
        return True

    def is_arm_at_home(self):
        # Placeholder for actual logic to determine if the arm is at the home position
        return True

    def is_arm_stable_cup(self):
        # Placeholder for actual logic to determine if the arm is stable with a cup
        return False

    def is_arm_holding_drink(self):
        # Placeholder for actual logic to determine if the arm is holding a drink
        return False

    def is_arm_retracted(self):
        # Placeholder for actual logic to determine if the arm is retracted
        return False

    # state machine callbacks
    def on_enter_Chair(self):
        # for testing
        print("Entering Chair state")

    def on_enter_Arm(self):
        print("Entering Arm state")

    def simple_test(self):
        print("simple test:")
        print("current state:" + self.state)
        print("triggering ready")
        self.ready()
        print("current state:" + self.state)
        print("triggering reqArm")
        self.reqArm()
        print("current state:" + self.state)
        self.arm_preset_client.set_preset(ArmPreset.HOME)
        print("current state:" + self.state)

        # get current time
        current_time = self.get_clock().now()
        # run function 100 times.
        for i in range(100):
            self.get_node_names_and_namespaces()

        # get now
        new_time = self.get_clock().now()
        # print time difference
        time_diff = new_time - current_time
        print(
            f"Time taken to call get_node_names_and_namespaces 100 times: {time_diff.nanoseconds / 1e6} ms"
        )

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
                        "children": ["raisingArm", "detecting", "opening", "opened"],
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
                        "children": ["moving", "stable"],
                    },
                    "paused",
                ],
            },
            {
                "name": "Nav",
                "initial": "detecting",
                "children": ["detecting", "detected", "traverse", "finished", "paused"],
            },
            "error",
        ]

        transitions = [
            # arm sub state transitions
            {
                "trigger": "retract",
                "source": [
                    "Arm_Door_opened",
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
            {"trigger": "reqArmActionCancel", "source": "Arm", "dest": "Arm_paused"},
            {
                "trigger": "reqHome",
                "source": ["Arm_retracted", "Arm_paused", "Arm_cupStabilize_stable"],
                "dest": "Arm_homing",
            },
            {"trigger": "homed", "source": "Arm_homing", "dest": "Arm_home"},
            # open door
            {
                "trigger": "reqOpenDoor",
                "source": ["Arm_home", "Arm_retracted"],
                "dest": "Arm_Door_raisingArm",
            },
            {
                "trigger": "armRaised",
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
                "dest": "Arm_Door_opened",
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
            {"trigger": "eStop", "source": "*", "dest": "error"},
            {"trigger": "UIDisconnected", "source": "*", "dest": "error"},
            {"trigger": "ready", "source": "init", "dest": "Chair"},
            {"trigger": "reset", "source": "error", "dest": "init"},
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
        )


def main():
    rclpy.init()
    node = SystemControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
