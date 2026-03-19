import rclpy
import rclpy.action
import rclpy.node
from .ArmPresetActionClient import ArmPresetActionClient
from transitions.extensions import HierarchicalMachine as Machine


class SystemControl(rclpy.node.Node):
    def __init__(self):
        super().__init__("system_control")
        self.get_logger().info("System Control Node has been started.")
        self.init_subscribers()
        self.init_services_clients()
        self.init_actions_clients()
        self.init_state_machine()

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

    # state machine setup
    def init_state_machine(self):
        states = [
            "init",
            {"name": "Chair", "initial": "SLOff", "children": ["SLOn", "SLOff"]},
            {
                "name": "Arm",
                "initial": "home",
                "children": [
                    "home",
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
                            "waitingForDrink",
                            "receivingDrink",
                            "ordered",
                        ],
                    },
                    {
                        "name": "Drink",
                        "initial": "bringCloser",
                        "children": [
                            "bringCloser",
                            "drinking",
                            "placingCup",
                            "finished",
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
                "trigger": "placeOrder",
                "source": "Arm_OrderDrink_pickCup",
                "dest": "Arm_OrderDrink_waitingForDrink",
            },
            {
                "trigger": "receiveDrinkConfirm",
                "source": "Arm_OrderDrink_waitingForDrink",
                "dest": "Arm_OrderDrink_receivingDrink",
            },
            {
                "trigger": "orderComplete",
                "source": "Arm_OrderDrink_receivingDrink",
                "dest": "Arm_OrderDrink_ordered",
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
                "source": "Arm_cupStabilize_stable",
                "dest": "Arm_Drink_bringCloser",
            },
            {
                "trigger": "readyForDrink",
                "source": "Arm_Drink_bringCloser",
                "dest": "Arm_Drink_drinking",
            },
            {
                "trigger": "finishedDrink",
                "source": "Arm_Drink_drinking",
                "dest": "Arm_Drink_placingCup",
            },
            {
                "trigger": "cupPlaced",
                "source": "Arm_Drink_placingCup",
                "dest": "Arm_Drink_finished",
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
