# Author: Jimmy Wu, Rajat Kumar Jenamani

# Python 3.10 removed these aliases from collections; kortex_api 2.6.0 still references them
import collections
import collections.abc
import enum

import numpy as np

collections.MutableMapping = collections.abc.MutableMapping
collections.MutableSequence = collections.abc.MutableSequence
collections.MutableSet = collections.abc.MutableSet
collections.Mapping = collections.abc.Mapping
collections.Sequence = collections.abc.Sequence
collections.Callable = collections.abc.Callable


class SpeedPreset(enum.IntEnum):
    DEFAULT = -1  # sentinel for hardware defaults (no soft limits applied)
    LOW = 0
    MEDIUM = 1
    HIGH = 2
    MAX = 3  # sentinel for maximum possible limits (equal to hard limits)


class KinovaArm:
    def __init__(self):
        # Check whether arm is connected

        self.action_count = 0
        self.speed_preset = SpeedPreset.MEDIUM
        self.actuator_count = 7
        self.gripper_position = 0.0  # 1.0 for close, and 0.0 for open

    def set_tool(self, tool):
        print("Does not affect current controller, but setting tool to", tool)

    def disconnect(self):
        print("disconnect")

    def ready(self):
        if self.action_count == 0:
            return True
        else:
            self.action_count -= 1
        return False

    def _execute_reference_action(self, action_name, blocking=True):
        # Retrieve reference action
        self.action_count = 30

    def home(self, blocking=True):
        self._execute_reference_action("Home", blocking=blocking)

    def retract(self, blocking=True):
        self._execute_reference_action("Retract", blocking=blocking)

    def zero(self, blocking=True):
        self._execute_reference_action("Zero", blocking=blocking)

    def cup_stabilize(self, blocking=True):
        self._execute_reference_action("CUP_STABILIZE", blocking=blocking)

    def drink_detection(self, blocking=True):
        self._execute_reference_action("RAMMP_DRINK_DETECTION", blocking=blocking)

    def send_twist(self, linear_xyz, angular_xyz):
        """Send a Cartesian twist velocity command (SINGLE_LEVEL_SERVOING).

        Uses CARTESIAN_REFERENCE_FRAME_MIXED: linear velocity in the base frame,
        angular velocity in the tool frame — standard for joystick teleoperation.

        Args:
            linear_xyz: Linear velocity [vx, vy, vz] in m/s.
            angular_xyz: Angular velocity [wx, wy, wz] in rad/s.
        """
        print("Sending twist command: linear", linear_xyz, "angular", angular_xyz)

    def get_ee_force(self):
        ee_force = np.array([1.0, 2.0, 3.0])
        return ee_force

    def get_state(self):
        q, dq, tau = (
            np.zeros(7),
            np.ones(7),
            np.ones(7) * 2,
        )

        ee_pos = np.ones(7) * 3

        return {
            "position": q,
            "velocity": dq,
            "effort": tau,
            "ee_pos": ee_pos,
            "gripper_pos": self.gripper_position,
        }

    def move_angular_trajectory(self, trajectory_joint_angles, blocking=True):
        assert len(trajectory_joint_angles) > 0, "Invalid trajectory"
        assert (
            len(trajectory_joint_angles[0]) == self.actuator_count
        ), "Invalid number of joint angles"
        print(
            "Moving along angular trajectory with waypoints:", trajectory_joint_angles
        )

    def move_angular(self, joint_angles, blocking=True):
        assert (
            len(joint_angles) == self.actuator_count
        ), "Invalid number of joint angles"
        print("Moving to angular position:", joint_angles)

    def move_cartesian(self, xyz, xyz_quat, blocking=True):
        print("Moving to Cartesian pose: xyz", xyz, "quat", xyz_quat)

    def _gripper_position_command(self, value, blocking=True, timeout=1.0):
        print(f"Setting gripper position to {value} (blocking={blocking})")

    def open_gripper(self, blocking=True):
        self.gripper_position = 0.0
        self._gripper_position_command(0, blocking)

    def close_gripper(self, blocking=True):
        self.gripper_position = 1.0
        self._gripper_position_command(1, blocking)

    def choose_from_speed_presets(self, speed_preset: SpeedPreset):
        if not isinstance(speed_preset, SpeedPreset) or speed_preset in [
            SpeedPreset.DEFAULT,
            SpeedPreset.MAX,
        ]:
            raise ValueError(
                "speed_preset must be SpeedPreset type and not DEFAULT or MAX"
            )
        print(f"Setting speed preset to {speed_preset.name}")
        self.speed_preset = speed_preset

    def get_speed_preset(self):
        return self.speed_preset

    # Rajat ToDo: Check how the following work:
    def pause_action(self):
        print("Pausing current action")

    def resume_action(self):
        print("Resuming current action")

    def stop_action(self):
        print("Stopping current action")

    def stop(self):
        print("Stopping all motion immediately")

    # Not using this as we haven't tested it
    # def apply_emergency_stop(self):
    #     self.base.ApplyEmergencyStop()

    def clear_faults(self):
        print("Clearing faults")


def main():
    arm = KinovaArm()
    try:
        arm.stop()

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
