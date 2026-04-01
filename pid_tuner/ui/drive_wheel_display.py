from PyQt6.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QPushButton,
    QGridLayout,
)
from PyQt6.QtCore import Qt, QTimer, QRectF
from PyQt6.QtGui import QPainter, QColor, QPen, QFont

from ..data.data_store import DataStore
from ..serial_driver.serial_handler import SerialHandler
from ..ros_bridge.luci_client import LuciClient
from .theme import THEME
from .scaling import scaled, SIZES


class ArcTachometer(QWidget):
    def __init__(self, title: str, parent=None):
        super().__init__(parent)
        self.title = title
        self.velocity = 0.0
        self.max_velocity = 2000.0
        self.setMinimumSize(scaled(80), scaled(60))
        self.setMaximumHeight(scaled(80))

    def set_velocity(self, velocity: float):
        if self.velocity != velocity:
            self.velocity = velocity
            self.update()

    def paintEvent(self, a0):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        width = self.width()
        height = self.height()

        arc_width = min(width - scaled(10), (height - scaled(20)) * 2)
        arc_height = arc_width

        x = (width - arc_width) / 2
        y = scaled(5)

        rect = QRectF(x, y, arc_width, arc_height)

        bg_pen = QPen(
            QColor(THEME.surface1),
            scaled(6),
            Qt.PenStyle.SolidLine,
            Qt.PenCapStyle.RoundCap,
        )
        painter.setPen(bg_pen)
        painter.drawArc(rect, 180 * 16, -180 * 16)

        clamped_vel = max(-self.max_velocity, min(self.max_velocity, self.velocity))
        ratio = clamped_vel / self.max_velocity

        if abs(ratio) > 0.01:
            if ratio > 0:
                fill_color = QColor(THEME.green)
                start_angle = 90 * 16
                sweep_angle = -90 * ratio * 16
            else:
                fill_color = QColor(THEME.red)
                start_angle = 90 * 16
                sweep_angle = 90 * abs(ratio) * 16

            fill_pen = QPen(
                fill_color, scaled(6), Qt.PenStyle.SolidLine, Qt.PenCapStyle.RoundCap
            )
            painter.setPen(fill_pen)
            painter.drawArc(rect, int(start_angle), int(sweep_angle))

        painter.setPen(QColor(THEME.text))

        val_font = QFont()
        val_font.setPixelSize(scaled(12))
        val_font.setBold(True)
        painter.setFont(val_font)

        val_text = f"{int(self.velocity)}"
        val_rect = painter.fontMetrics().boundingRect(val_text)
        painter.drawText(
            int(width / 2 - val_rect.width() / 2),
            int(y + arc_height / 2 - scaled(5)),
            val_text,
        )

        title_font = QFont()
        title_font.setPixelSize(scaled(10))
        painter.setFont(title_font)
        painter.setPen(QColor(THEME.subtext1))

        title_rect = painter.fontMetrics().boundingRect(self.title)
        painter.drawText(
            int(width / 2 - title_rect.width() / 2),
            int(y + arc_height / 2 + scaled(12)),
            self.title,
        )


DRIVE_SPEED = 50

_DPAD_BTN = f"""
    QPushButton {{
        background-color: {THEME.surface1};
        color: {THEME.text};
        border: 1px solid {THEME.surface2};
        border-radius: 4px;
        font-size: {SIZES["font_normal"]}pt;
        font-weight: bold;
        min-width: {scaled(36)}px;
        min-height: {scaled(28)}px;
    }}
    QPushButton:hover {{ background-color: {THEME.surface2}; }}
    QPushButton:pressed {{ background-color: {THEME.blue}; color: {THEME.crust}; }}
    QPushButton:disabled {{ color: {THEME.overlay0}; }}
"""


class DriveWheelDisplay(QWidget):
    def __init__(
        self, data_store: DataStore, serial_handler: SerialHandler, parent=None
    ):
        super().__init__(parent)
        self.data_store = data_store
        self._luci = LuciClient(self)
        self._luci.connected_changed.connect(self._on_luci_connection_changed)
        self._luci.error_occurred.connect(self._on_luci_error)
        self._manual_override = False

        self._init_ui()

        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self._update_display)
        self._update_timer.start(100)

    def _init_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(scaled(4))

        arc_row = QHBoxLayout()
        arc_row.setSpacing(scaled(10))
        self.left_arc = ArcTachometer("L")
        self.right_arc = ArcTachometer("R")
        arc_row.addWidget(self.left_arc)
        arc_row.addWidget(self.right_arc)
        root.addLayout(arc_row)

        dpad = QGridLayout()
        dpad.setSpacing(scaled(2))

        self._btn_fwd = QPushButton("▲")
        self._btn_fwd.setStyleSheet(_DPAD_BTN)
        self._btn_fwd.pressed.connect(lambda: self._dpad_press(DRIVE_SPEED, 0))
        self._btn_fwd.released.connect(self._dpad_release)
        self._btn_fwd.setEnabled(False)
        dpad.addWidget(self._btn_fwd, 0, 1)

        self._btn_left = QPushButton("◀")
        self._btn_left.setStyleSheet(_DPAD_BTN)
        self._btn_left.pressed.connect(lambda: self._dpad_press(0, -DRIVE_SPEED))
        self._btn_left.released.connect(self._dpad_release)
        self._btn_left.setEnabled(False)
        dpad.addWidget(self._btn_left, 1, 0)

        self._btn_stop = QPushButton("■")
        self._btn_stop.setStyleSheet(_DPAD_BTN)
        self._btn_stop.clicked.connect(self._dpad_release)
        self._btn_stop.setEnabled(False)
        dpad.addWidget(self._btn_stop, 1, 1)

        self._btn_right = QPushButton("▶")
        self._btn_right.setStyleSheet(_DPAD_BTN)
        self._btn_right.pressed.connect(lambda: self._dpad_press(0, DRIVE_SPEED))
        self._btn_right.released.connect(self._dpad_release)
        self._btn_right.setEnabled(False)
        dpad.addWidget(self._btn_right, 1, 2)

        self._btn_bwd = QPushButton("▼")
        self._btn_bwd.setStyleSheet(_DPAD_BTN)
        self._btn_bwd.pressed.connect(lambda: self._dpad_press(-DRIVE_SPEED, 0))
        self._btn_bwd.released.connect(self._dpad_release)
        self._btn_bwd.setEnabled(False)
        dpad.addWidget(self._btn_bwd, 2, 1)

        root.addLayout(dpad)

    def _on_luci_connection_changed(self, connected: bool):
        for btn in (
            self._btn_fwd,
            self._btn_bwd,
            self._btn_left,
            self._btn_right,
            self._btn_stop,
        ):
            btn.setEnabled(connected)

    def _on_luci_error(self, _msg: str):
        return

    def _dpad_press(self, fb: int, lr: int):
        self._manual_override = True
        self._luci.set_drive(fb, lr)

    def _dpad_release(self):
        self._manual_override = False
        self._luci.stop()

    def _update_display(self):
        self.left_arc.set_velocity(self.data_store.raw_ml_enc_vel)
        self.right_arc.set_velocity(self.data_store.raw_mr_enc_vel)

        if self._luci.is_connected and not self._manual_override:
            fb_pwm = self.data_store.drive_fb_pwm
            lr_pwm = self.data_store.drive_lr_pwm

            if abs(fb_pwm) > 0.001 or abs(lr_pwm) > 0.001:
                fb = int(fb_pwm * 100.0)
                lr = int(lr_pwm * 100.0)
                fb = max(-100, min(100, fb))
                lr = max(-100, min(100, lr))
                self._luci.set_drive(fb, lr)
            else:
                self._luci.set_drive(0, 0)

    def shutdown(self):
        if self._luci.is_connected:
            self._luci.disconnect()
