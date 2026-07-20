#!/usr/bin/env python3
from __future__ import annotations
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import rclpy
from rammp_prototype_interfaces.msg import RAMMPPrototypeState

FC_MOTOR_JS_THRESHOLD = 0.5
AXIS_NEUTRAL_THRESHOLD = 0.15


class ManualControlNode(Node):
    """
    Minimal /joy -> Teensy serial forwarder.

    Forwards gamepad input for joint motors and front casters over serial.
    Drive wheels are not controlled here.
    """

    STATE_IDLE = 1
    STATE_TUNER_MODE = 2

    def __init__(self):
        super().__init__("manual_control_node")
        self.teensy_state: int | None = None
        self.state = self.STATE_IDLE
        self.prev_start_pressed = False
        self._prev_estop_pressed = False
        self._prev_cal_pressed = False
        self.fc_motors_active = False
        self.last_pwm_array = [0, 0, 0, 0, 0, 0]
        self.axes_centered = False

        self.status_pub = self.create_publisher(String, "gamepad_status", 10)
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.state_sub = self.create_subscription(
            RAMMPPrototypeState, "rammp_prototype_state", self.state_callback, 10
        )
        self.serial_commands_pub = self.create_publisher(String, "serial_commands", 10)

    def state_callback(self, msg: RAMMPPrototypeState) -> None:
        self.teensy_state = msg.state

    def _fc_axis_neutral(self, axes_array: list[float]) -> bool:
        axis3 = axes_array[3] if len(axes_array) > 3 else 0.0
        return abs(axis3) <= AXIS_NEUTRAL_THRESHOLD

    def _stop_fc_motors(self) -> None:
        self.serial_commands_pub.publish(String(data="T9:0.00\nT10:0.00\n"))

    def _start_fc_motors(self, direction: int) -> None:
        pwm = 0.2 * direction
        self.serial_commands_pub.publish(
            String(data=f"M9:0\nM10:0\nT9:{pwm:.2f}\nT10:{pwm:.2f}\n")
        )

    def _trigger_estop(self) -> None:
        self.state = self.STATE_IDLE
        self.fc_motors_active = False
        self.last_pwm_array = [0, 0, 0, 0, 0, 0]
        self.serial_commands_pub.publish(
            String(
                data="s:0.0000\nT9:0.00\nT10:0.00\nT1:0\nT2:0\nT3:0\nT4:0\nT5:0\nT6:0\n"
            )
        )

    def joy_callback(self, msg):
        if self.teensy_state is None:
            return
        estop_pressed = len(msg.buttons) > 1 and msg.buttons[1] == 1
        if estop_pressed and not self._prev_estop_pressed:
            self._trigger_estop()
            self._prev_estop_pressed = estop_pressed
            self.prev_start_pressed = len(msg.buttons) > 9 and msg.buttons[9] == 1
            return
        self._prev_estop_pressed = estop_pressed

        start_pressed = msg.buttons[9] == 1

        entered_manual = False
        if start_pressed and not self.prev_start_pressed:
            if self.teensy_state == RAMMPPrototypeState.STATE_ESTOP:
                self.serial_commands_pub.publish(String(data="c\n"))
                self.state = self.STATE_IDLE
                return
            if (
                self.state == self.STATE_IDLE
                and self.teensy_state == RAMMPPrototypeState.STATE_IDLE
            ):
                axes_array = list(msg.axes)
                if not self._fc_axis_neutral(axes_array):
                    self.axes_centered = False
                    self.status_pub.publish(
                        String(
                            data=("Center FC stick (axis 3), then press Start again")
                        )
                    )
                else:
                    self.axes_centered = True
                    self.state = self.STATE_TUNER_MODE
                    self.fc_motors_active = False
                    self.serial_commands_pub.publish(
                        String(data="M1:0\nM2:0\nM3:0\nM4:0\nM5:0\nM6:0\n")
                    )
                    self.serial_commands_pub.publish(
                        String(data="s:0.0000\nT9:0.00\nT10:0.00\n")
                    )
                    entered_manual = True
                    self.get_logger().info("Entered gamepad control mode")
            else:
                self.state = self.STATE_IDLE
                self.axes_centered = False
                self.serial_commands_pub.publish(
                    String(data="T1:0\nT2:0\nT3:0\nT4:0\nT5:0\nT6:0\n")
                )

        self.prev_start_pressed = start_pressed

        if entered_manual:
            return

        if self.state == self.STATE_TUNER_MODE:
            raw_direction = msg.axes[5]

            cal_pressed = msg.buttons[2] == 1
            if cal_pressed and not self._prev_cal_pressed:
                self.serial_commands_pub.publish(String(data="W0:-0.20\n"))
            self._prev_cal_pressed = cal_pressed

            if (abs(raw_direction)) < 0.15:
                direction = 0
            else:
                direction = 1 if raw_direction > 0 else -1
            buttons_array = list(msg.buttons)
            axes_array = list(msg.axes)
            buttons_all_zeros = not any(buttons_array)
            axes_all_zeros = not any(abs(axis) > 0.15 for axis in axes_array)
            if buttons_all_zeros and axes_all_zeros:
                if self.fc_motors_active:
                    self.fc_motors_active = False
                    self._stop_fc_motors()
                return

            if abs(axes_array[3]) > FC_MOTOR_JS_THRESHOLD and not self.fc_motors_active:
                self.fc_motors_active = True
                fc_direction = 1 if axes_array[3] > 0 else -1
                self._start_fc_motors(fc_direction)
            elif abs(axes_array[3]) < FC_MOTOR_JS_THRESHOLD and self.fc_motors_active:
                self.fc_motors_active = False
                self._stop_fc_motors()

            del buttons_array[8 : len(buttons_array)]
            del buttons_array[1:3]
            buttons_array[2], buttons_array[4] = buttons_array[4], buttons_array[2]
            buttons_array[3], buttons_array[5] = buttons_array[5], buttons_array[3]

            pwm_scale = 0.30
            lines = []
            for i in range(len(buttons_array)):
                id = i + 1
                if i == 4 or i == 5:
                    pwm = -1 * (
                        (pwm_scale * direction)
                        if (buttons_array[i] == 1 and direction != 0.0)
                        else 0.0
                    )
                else:
                    pwm = (
                        (pwm_scale * direction)
                        if (buttons_array[i] == 1 and direction != 0.0)
                        else 0.0
                    )

                if pwm != self.last_pwm_array[i]:
                    self.last_pwm_array[i] = pwm
                    lines.append(f"T{id}:{pwm:.2f}\n")
            if len(lines) > 0:
                self.serial_commands_pub.publish(String(data="".join(lines)))


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
