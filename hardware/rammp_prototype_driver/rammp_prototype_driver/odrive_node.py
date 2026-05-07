import json
import rclpy
import serial
from serial.tools import list_ports
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

        # Serial settings as ROS params for quick iteration on Jetson.
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 460800)
        port = str(self.get_parameter("port").value)
        baud = int(self.get_parameter("baud").value)

        ports = [p.device for p in list_ports.comports()]
        self.get_logger().info(f"Detected serial ports: {ports}")

        self.keyframes = _load_keyframes_from_json(
            get_package_share_directory("rammp_prototype_driver")
            + "/config/test_odrive.json"
        )
        # NOTE: Base.ino uses Serial.begin(460800) for the Jetson link.
        # Use a short timeout so reads/writes can't hang callbacks.
        self.ser = None
        try:
            self.ser = serial.Serial(
                port, baud, timeout=0.01, write_timeout=0.2, exclusive=True
            )
            self.get_logger().info(f"Opened serial: port={port} baud={baud}")
        except TypeError:
            # 'exclusive' isn't supported on all pyserial/platform combos.
            self.ser = serial.Serial(port, baud, timeout=0.01, write_timeout=0.2)
            self.get_logger().info(
                f"Opened serial (no exclusive): port={port} baud={baud}"
            )
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial {port} @ {baud}: {e}")
            self.get_logger().error(f"Available ports: {ports}")
            self.ser = None

        self.serial_timer = self.create_timer(0.02, self.read_serial_data)
        self.heartbeat_timer = self.create_timer(0.5, self.send_serial_heartbeat)
        # self._stdin_thread = threading.Thread(target=self._stdin_loop, daemon=True)
        # self._stdin_thread.start()

    def read_serial_data(self):
        if self.ser is None:
            return
        try:
            waiting = self.ser.in_waiting
            if waiting <= 0:
                return
            data = self.ser.read(waiting)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial read failed: {e}")
            return

        self.get_logger().info(f"RX {len(data)}B: {repr(data)}")
        text = data.decode("utf-8", errors="replace")
        if text.strip():
            self.get_logger().info(f"RX txt: {text.rstrip()}")

    def write_serial_data(self, data):
        if self.ser is None:
            return

        if isinstance(data, str):
            data = data.encode("utf-8")

        try:
            self.ser.write(data)
        except serial.SerialTimeoutException as e:
            self.get_logger().error(f"Serial write timeout: {e}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write failed: {e}")

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
        if self.ser is None:
            return
        self.write_serial_data("c\n")

    # def _stdin_loop(self):
    #     while rclpy.ok():
    #         try:
    #             command = input("Type 's' to send odrive sequence: ").strip()
    #         except EOFError:
    #             return
    #         except KeyboardInterrupt:
    #             return

    #         if command == "s":
    #             self.send_sequence(self.keyframes, auto_run=True)
    #             self.get_logger().info("Sequence sent")

    # def send_odrive_sequence(self):
    #     # Deprecated: use the stdin thread to trigger sending without blocking spin()
    #     self.get_logger().warn(
    #         "send_odrive_sequence() is deprecated; use stdin prompt."
    #     )


def main(args=None):
    rclpy.init(args=args)
    node = ODriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
