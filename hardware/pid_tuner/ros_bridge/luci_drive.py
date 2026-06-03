"""
Shared LUCI drive-wheel command arbitration for PID tuner.

All features (Drive Wheels d-pad, /joy manual control, sequence carriage_return)
use one LuciClient connection. Call compute_drive_command() on telemetry, then
LuciClient.request_drive().
"""

from __future__ import annotations

import threading
from dataclasses import dataclass

STATE_AUTO_CURB_CLIMBING = 6
DRIVE_WHEEL_JS_THRESHOLD = 0.25


@dataclass
class ManualDriveInput:
    """Latest /joy stick request (written from manual_control_node thread)."""

    stick_active: bool = False
    forward_back: int = 0
    left_right: int = 0
    _lock: threading.Lock = None  # type: ignore[assignment]

    def __post_init__(self):
        if self._lock is None:
            object.__setattr__(self, "_lock", threading.Lock())

    def set_sticks(self, active: bool, forward_back: int, left_right: int) -> None:
        with self._lock:
            self.stick_active = active
            self.forward_back = int(forward_back)
            self.left_right = int(left_right)

    def clear_sticks(self) -> None:
        self.set_sticks(False, 0, 0)

    def snapshot(self) -> tuple[bool, int, int]:
        with self._lock:
            return self.stick_active, self.forward_back, self.left_right


def compute_drive_command(
    data_store,
    manual: ManualDriveInput,
    *,
    manual_override: bool,
    override_fb: int,
    override_lr: int,
) -> tuple[int, int]:
    """
    Priority: d-pad hold > sequence carriage_return > gamepad sticks > idle.
    """
    if manual_override:
        return int(override_fb), int(override_lr)

    if data_store is None:
        return 0, 0

    if data_store.current_state == STATE_AUTO_CURB_CLIMBING:
        cr = int(data_store.carriage_return_direction)
        if cr != 0:
            return cr, -2
        return 0, 0

    if hasattr(manual, "snapshot"):
        stick_active, stick_fb, stick_lr = manual.snapshot()
    else:
        stick_active = manual.stick_active
        stick_fb = manual.forward_back
        stick_lr = manual.left_right

    if stick_active:
        return stick_fb, stick_lr

    fb_pwm = float(data_store.drive_fb_pwm)
    lr_pwm = float(data_store.drive_lr_pwm)
    if abs(fb_pwm) > 0.001 or abs(lr_pwm) > 0.001:
        fb = max(-100, min(100, int(fb_pwm * 100.0)))
        lr = max(-100, min(100, int(lr_pwm * 100.0)))
        return fb, lr

    return 0, 0
