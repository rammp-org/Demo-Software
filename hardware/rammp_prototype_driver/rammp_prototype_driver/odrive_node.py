import json

import rclpy
import serial
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node

from .keyframe import NUM_MOTORS, Keyframe
from .protocol import ProtocolEncoder


def _load_keyframes_from_json(json_path: str) -> list[Keyframe]:
    with open(json_path, "r") as f:
        data = json.load(f)
    keyframes_data = data.get("keyframes", data if isinstance(data, list) else [])
    return [Keyframe.from_dict(d) for d in keyframes_data]


class ODriveNode(Node):
    def __init__(self):
        super().__init__("odrive_node")
        self.keyframes = _load_keyframes_from_json(
            get_package_share_directory("rammp_prototype_driver")
            + "/config/test_odrive.json"
        )
        self.ser = serial.Serial("/dev/ttyACM0", 115200)

    def write_serial_data(self, data):
        if self.ser is None:
            return
        print(data)
        self.ser.write(data)

    def send_sequence(self, keyframes: list[Keyframe], auto_run: bool = True):
        self.write_serial_data(ProtocolEncoder.enter_sequence_mode(True))
        for idx, kf in enumerate(keyframes):
            targets = [t if t is not None else 0.0 for t in kf.targets]
            active = [t is not None for t in kf.targets]
            durations = [
                kf.motor_durations[i]
                if kf.motor_durations[i] is not None
                else kf.duration_ms
                for i in range(NUM_MOTORS)
            ]
            self.write_serial_data(
                ProtocolEncoder.send_keyframe(
                    idx,
                    targets,
                    active,
                    durations,
                    kf.relative,
                    guard_threshold=kf.guard_threshold,
                    guard_condition=kf.guard_condition,
                    odrive_active=kf.odrive_active,
                    odrive_relative=kf.odrive_relative,
                    odrive_targets=kf.odrive_targets,
                )
            )
        if auto_run:
            self.write_serial_data(ProtocolEncoder.seq_auto_run(True))
        self.write_serial_data(ProtocolEncoder.seq_step_forward())

    def send_serial_heartbeat(self):
        if not self.estop:
            self.write_serial_data("c\n")
        else:
            self.write_serial_data("z\n")

    def send_odrive_sequence(self):
        command = input("Waiting to send odrive sequence...")
        if command == "s":
            self.send_sequence(self.keyframes, auto_run=True)
            print("Sequence sent")


def main(args=None):
    rclpy.init(args=args)
    node = ODriveNode()
    node.send_odrive_sequence()
    rclpy.spin(node)
