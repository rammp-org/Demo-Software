import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String

from keyboard_driver.key_map import KEY_TO_ACTION


class KeyboardNode(Node):
    def __init__(self):
        super().__init__("keyboard_node")

        # If set, open this device path directly (e.g. /dev/input/event5).
        self.declare_parameter("device_path", "")
        self.device_path = (
            self.get_parameter("device_path").get_parameter_value().string_value
        )

        # Optional name-substring fallback used only if capability-based
        # selection can't single out a device. Leave empty for pure auto-select.
        self.declare_parameter("device_name", "")
        self.device_name = (
            self.get_parameter("device_name").get_parameter_value().string_value
        )

        # Grab the device for exclusive access so keystrokes don't leak to other apps.
        self.declare_parameter("grab_device", False)
        self.grab_device = (
            self.get_parameter("grab_device").get_parameter_value().bool_value
        )

        self._cb_group = ReentrantCallbackGroup()

        # Phase 1 plumbing output: publish the action label per keypress.
        self.event_pub = self.create_publisher(String, "/keyboard/event", 10)

        # evdev's read_loop() blocks, so run it in a daemon thread alongside the
        # rclpy executor. The thread also handles (re)opening the device.
        self._device = None
        self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self._read_thread.start()

        self.get_logger().info("keyboard_node initialized...")

    def _open_device(self):
        from evdev import InputDevice, ecodes, list_devices

        try:
            if self.device_path:
                return self._finalize_device(InputDevice(self.device_path))

            # Auto-select by capability: prefer the node that actually advertises
            # our target keys. Composite keypads expose several event nodes, and
            # the one whose *name* contains "keyboard" is often NOT the one that
            # delivers the keypresses (it's a sibling HID collection).
            target_codes = {
                ecodes.ecodes[name] for name in KEY_TO_ACTION if name in ecodes.ecodes
            }
            best = None  # (rank_tuple, dev)
            available = []
            for path in list_devices():
                try:
                    dev = InputDevice(path)
                except OSError:
                    continue
                keys = set(dev.capabilities().get(ecodes.EV_KEY, []))
                has_all = target_codes.issubset(keys)
                has_any = bool(target_codes & keys)
                name_match = bool(
                    self.device_name and self.device_name.lower() in dev.name.lower()
                )
                available.append(
                    f"{path} -> {dev.name} "
                    f"[has_all={has_all}, has_any={has_any}, name_match={name_match}]"
                )
                rank = (has_all, has_any, name_match)
                if any(rank) and (best is None or rank > best[0]):
                    best = (rank, dev)
            self.get_logger().info(
                "Candidate input devices:\n  " + "\n  ".join(available)
            )
            if best is None:
                self.get_logger().error(
                    "No input device advertises the target keys "
                    f"({sorted(KEY_TO_ACTION)}) or matches "
                    f"device_name='{self.device_name}'."
                )
                return None
            return self._finalize_device(best[1])
        except (OSError, PermissionError) as e:
            self.get_logger().error(
                f"Failed to open input device: {e}. Is the user in the 'input' group?"
            )
            return None

    def _finalize_device(self, dev):
        if self.grab_device:
            dev.grab()
        self.get_logger().info(f"Using keyboard device: {dev.path} ({dev.name})")
        return dev

    def _read_loop(self):
        from evdev import ecodes

        backoff = 1.0
        while rclpy.ok():
            if self._device is None:
                self._device = self._open_device()
                if self._device is None:
                    time.sleep(backoff)
                continue
            try:
                for event in self._device.read_loop():
                    # Only act on key-down (value 1); ignore autorepeat (2) and key-up (0).
                    if event.type != ecodes.EV_KEY or event.value != 1:
                        continue
                    key_name = ecodes.KEY[event.code]
                    if isinstance(key_name, (list, tuple)):
                        key_name = next(
                            (k for k in key_name if k in KEY_TO_ACTION), key_name[0]
                        )
                    action = KEY_TO_ACTION.get(key_name)
                    if action is None:
                        self.get_logger().debug(f"Ignoring key {key_name}")
                        continue
                    self._handle_action(key_name, action)
            except OSError as e:
                self.get_logger().warn(
                    f"Keyboard device read error ({e}); will attempt to reconnect."
                )
                self._device = None

    def _handle_action(self, key_name, action):
        self.get_logger().info(f"Key {key_name} -> action '{action}'")
        msg = String()
        msg.data = action
        self.event_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardNode()
    executor = rclpy.executors.MultiThreadedExecutor()
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
