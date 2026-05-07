import json
import threading

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
        self.serial_timer = self.create_timer(0.02, self.read_serial_data)
        self.heartbeat_timer = self.create_timer(0.5, self.send_serial_heartbeat)
        self._stdin_thread = threading.Thread(target=self._stdin_loop, daemon=True)
        self._stdin_thread.start()

    def read_serial_data(self):
        if self.ser is None:
            return
        if self.ser.in_waiting > 0:
            line = self.ser.readline()
            self.get_logger().info(line.decode("utf-8", errors="replace").strip())

    def write_serial_data(self, data):
        if self.ser is None:
            return
        self.ser.write(data.encode("utf-8"))

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
        self.write_serial_data("c\n")

    def _stdin_loop(self):
        while rclpy.ok():
            try:
                command = input("Type 's' to send odrive sequence: ").strip()
            except EOFError:
                return
            except KeyboardInterrupt:
                return

            if command == "s":
                self.send_sequence(self.keyframes, auto_run=True)
                self.get_logger().info("Sequence sent")

    def send_odrive_sequence(self):
        # Deprecated: use the stdin thread to trigger sending without blocking spin()
        self.get_logger().warn(
            "send_odrive_sequence() is deprecated; use stdin prompt."
        )


def main(args=None):
    rclpy.init(args=args)
    node = ODriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
