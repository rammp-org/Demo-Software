from __future__ import annotations

from typing import Optional

from PyQt6.QtCore import QObject, QTimer, pyqtSignal

try:
    import roslibpy
except ImportError:
    roslibpy = None

JS_FRONT = 0
JS_FRONT_LEFT = 1
JS_FRONT_RIGHT = 2
JS_LEFT = 3
JS_RIGHT = 4
JS_BACK_LEFT = 5
JS_BACK_RIGHT = 6
JS_BACK = 7
JS_ORIGIN = 8

INPUT_REMOTE = 1

JOYSTICK_TOPIC = "luci/remote_joystick"
JOYSTICK_MSG_TYPE = "luci_messages/msg/LuciJoystick"
SET_AUTO_SERVICE = "/luci/set_auto_remote_input"
REMOVE_AUTO_SERVICE = "/luci/remove_auto_remote_input"


def _compute_zone(fb: int, lr: int) -> int:
    if fb == 0 and lr == 0:
        return JS_ORIGIN
    if fb > 0 and lr == 0:
        return JS_FRONT
    if fb < 0 and lr == 0:
        return JS_BACK
    if fb == 0 and lr > 0:
        return JS_RIGHT
    if fb == 0 and lr < 0:
        return JS_LEFT
    if fb > 0 and lr > 0:
        return JS_FRONT_RIGHT
    if fb > 0 and lr < 0:
        return JS_FRONT_LEFT
    if fb < 0 and lr > 0:
        return JS_BACK_RIGHT
    return JS_BACK_LEFT


class LuciClient(QObject):
    connected_changed = pyqtSignal(bool)
    error_occurred = pyqtSignal(str)
    _ready_from_thread = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._ros: Optional[roslibpy.Ros] = None
        self._topic: Optional[roslibpy.Topic] = None
        self._connected = False
        self._auto_input_enabled = False

        self._heartbeat_timer = QTimer(self)
        self._heartbeat_timer.setInterval(5)
        self._heartbeat_timer.timeout.connect(self._send_heartbeat)
        self._ready_from_thread.connect(self._finish_connect)
        self._fb = 0
        self._lr = 0

    @property
    def is_connected(self) -> bool:
        return self._connected

    @property
    def available(self) -> bool:
        return roslibpy is not None

    def connect(self, host: str, port: int = 9090):
        if not self.available:
            self.error_occurred.emit(
                "roslibpy not installed — run: pip install roslibpy"
            )
            return
        if self._connected:
            return

        try:
            self._ros = roslibpy.Ros(host=host, port=port)
            self._ros.on_ready(self._on_connected)
            self._ros.run()
        except Exception as e:
            self.error_occurred.emit(f"Connection failed: {e}")

    def _on_connected(self):
        self._topic = roslibpy.Topic(self._ros, JOYSTICK_TOPIC, JOYSTICK_MSG_TYPE)
        self._topic.advertise()
        self._connected = True
        self._ready_from_thread.emit()

    def _finish_connect(self):
        self.connected_changed.emit(True)
        self._enable_auto_input()
        self._heartbeat_timer.start()

    def _enable_auto_input(self):
        if not self._ros or not self._ros.is_connected:
            return
        svc = roslibpy.Service(self._ros, SET_AUTO_SERVICE, "std_srvs/srv/Empty")
        svc.call(
            roslibpy.ServiceRequest(),
            callback=self._on_auto_enabled,
            errback=self._on_service_error,
        )

    def _on_auto_enabled(self, _result):
        self._auto_input_enabled = True

    def _disable_auto_input(self):
        if not self._ros or not self._ros.is_connected:
            return
        svc = roslibpy.Service(self._ros, REMOVE_AUTO_SERVICE, "std_srvs/srv/Empty")
        svc.call(
            roslibpy.ServiceRequest(), callback=lambda _: None, errback=lambda e: None
        )
        self._auto_input_enabled = False

    def _on_service_error(self, error):
        self.error_occurred.emit(f"LUCI service error: {error}")

    def set_drive(self, forward_back: int, left_right: int):
        self._fb = max(-100, min(100, forward_back))
        self._lr = max(-100, min(100, left_right))

    def stop(self):
        self._fb = 0
        self._lr = 0
        self._send_joystick(0, 0)

    def _send_heartbeat(self):
        self._send_joystick(self._fb, self._lr)

    def _send_joystick(self, fb: int, lr: int):
        if not self._topic or not self._connected:
            return
        try:
            self._topic.publish(
                roslibpy.Message(
                    {
                        "forward_back": fb,
                        "left_right": lr,
                        "joystick_zone": _compute_zone(fb, lr),
                        "input_source": INPUT_REMOTE,
                    }
                )
            )
        except Exception:
            pass

    def disconnect(self):
        self._heartbeat_timer.stop()
        self.stop()
        if self._auto_input_enabled:
            self._disable_auto_input()
        if self._topic:
            try:
                self._topic.unadvertise()
            except Exception:
                pass
            self._topic = None
        if self._ros:
            ros = self._ros
            self._ros = None
            try:
                ros.close()
            except Exception:
                pass
            try:
                ros.terminate()
            except Exception:
                pass
        self._connected = False
        self._auto_input_enabled = False
        self.connected_changed.emit(False)
