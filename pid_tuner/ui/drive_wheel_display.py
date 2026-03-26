import math
from PyQt6.QtWidgets import QWidget, QHBoxLayout, QVBoxLayout, QLabel
from PyQt6.QtCore import Qt, QTimer, QRectF
from PyQt6.QtGui import QPainter, QColor, QPen, QFont, QPainterPath

from ..data.data_store import DataStore
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

    def paintEvent(self, event):
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


class DriveWheelDisplay(QWidget):
    def __init__(self, data_store: DataStore, parent=None):
        super().__init__(parent)
        self.data_store = data_store

        self.init_ui()

        self.update_timer = QTimer(self)
        self.update_timer.timeout.connect(self.update_display)
        self.update_timer.start(100)

    def init_ui(self):
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(scaled(10))

        self.left_arc = ArcTachometer("L")
        self.right_arc = ArcTachometer("R")

        layout.addWidget(self.left_arc)
        layout.addWidget(self.right_arc)

    def update_display(self):
        self.left_arc.set_velocity(self.data_store.ml_drive_vel)
        self.right_arc.set_velocity(self.data_store.mr_drive_vel)
