"""
Encoder overview widget showing all 6 encoder positions as horizontal bars.
"""

from PyQt6.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QLabel,
    QFrame,
    QSizePolicy,
)
from PyQt6.QtCore import pyqtSlot, pyqtSignal, Qt, QTimer
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QFont

from .theme import THEME, JOINT_COLORS
from .scaling import SIZES, scaled, scaled_font_size
from ..data.data_store import DataStore
from ..data.joint_config import JOINTS


class EncoderBar(QWidget):
    """A single horizontal bar representing an encoder position."""

    clicked = pyqtSignal(int)  # Emits joint_id when clicked

    # Default range for the bar (can be adjusted)
    DEFAULT_MIN = -50.0
    DEFAULT_MAX = 50.0

    def __init__(self, joint_id: int, name: str, color: str, parent=None):
        super().__init__(parent)
        self._joint_id = joint_id
        self._name = name
        self._color = QColor(color)
        self._value = 0.0
        self._min_val = self.DEFAULT_MIN
        self._max_val = self.DEFAULT_MAX
        self._selected = False

        # Use scaled heights
        self.setMinimumHeight(SIZES["encoder_bar_height"])
        self.setMaximumHeight(SIZES["encoder_bar_max_height"])
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setCursor(Qt.CursorShape.PointingHandCursor)

    def set_value(self, value: float):
        """Set the current encoder value."""
        self._value = value
        self.update()

    def set_range(self, min_val: float, max_val: float):
        """Set the display range."""
        self._min_val = min_val
        self._max_val = max_val
        self.update()

    def set_selected(self, selected: bool):
        """Set whether this bar is selected."""
        self._selected = selected
        self.update()

    def mousePressEvent(self, event):
        """Handle mouse click to select this joint."""
        self.clicked.emit(self._joint_id)

    def paintEvent(self, event):
        """Paint the encoder bar."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        width = self.width()
        height = self.height()
        margin = scaled(2)
        bar_height = height - 2 * margin
        label_width = scaled(40)
        value_width = scaled(55)
        bar_left = label_width + scaled(4)
        bar_width = width - bar_left - value_width - scaled(8)

        # Draw selection highlight
        if self._selected:
            painter.setPen(QPen(QColor(THEME.blue), 2))
            painter.setBrush(Qt.BrushStyle.NoBrush)
            painter.drawRoundedRect(0, 0, width - 1, height - 1, scaled(4), scaled(4))

        # Draw joint label
        painter.setPen(QColor(THEME.subtext1))
        font = QFont()
        font.setPointSize(SIZES["font_small"])
        font.setBold(True)
        painter.setFont(font)
        painter.drawText(
            margin,
            margin,
            label_width,
            bar_height,
            Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignLeft,
            self._name,
        )

        # Draw bar background
        bar_rect_x = bar_left
        bar_rect_y = margin + scaled(2)
        bar_rect_h = bar_height - scaled(4)

        painter.setPen(QPen(QColor(THEME.surface1), 1))
        painter.setBrush(QBrush(QColor(THEME.surface0)))
        painter.drawRoundedRect(
            bar_rect_x, bar_rect_y, bar_width, bar_rect_h, scaled(3), scaled(3)
        )

        # Calculate fill position
        # The bar shows position relative to range, with 0 in the middle
        range_span = self._max_val - self._min_val
        if range_span > 0:
            # Normalize value to 0-1 range
            normalized = (self._value - self._min_val) / range_span
            normalized = max(0.0, min(1.0, normalized))

            # Calculate fill width and position
            center_x = bar_rect_x + bar_width / 2
            fill_width = abs(normalized - 0.5) * bar_width

            if self._value >= 0:
                fill_x = center_x
            else:
                fill_x = center_x - fill_width

            # Draw filled portion
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QBrush(self._color))
            painter.drawRoundedRect(
                int(fill_x),
                bar_rect_y + 1,
                int(fill_width),
                bar_rect_h - 2,
                scaled(2),
                scaled(2),
            )

            # Draw center line
            painter.setPen(QPen(QColor(THEME.overlay0), 1))
            painter.drawLine(
                int(center_x), bar_rect_y, int(center_x), bar_rect_y + bar_rect_h
            )

        # Draw value text
        painter.setPen(QColor(THEME.text))
        font.setBold(False)
        painter.setFont(font)
        value_text = f"{self._value:+.1f}"
        painter.drawText(
            width - value_width - margin,
            margin,
            value_width,
            bar_height,
            Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignRight,
            value_text,
        )

        painter.end()


class EncoderOverview(QWidget):
    """
    Widget showing all 6 encoder positions as horizontal progress bars.

    Features:
    - Shows all encoders at a glance
    - Each bar is color-coded by joint
    - Clicking a bar selects that joint
    - Updates from DataStore
    """

    joint_selected = pyqtSignal(int)  # Emits joint_id when a bar is clicked

    UPDATE_INTERVAL_MS = 100  # 10 Hz update rate

    def __init__(self, data_store: DataStore, parent=None):
        super().__init__(parent)
        self._data_store = data_store
        self._bars = []
        self._selected_joint = 1

        self._setup_ui()
        self._setup_timer()

    def _setup_ui(self):
        """Set up the widget layout."""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(
            SIZES["margin_medium"],
            SIZES["margin_small"],
            SIZES["margin_medium"],
            SIZES["margin_small"],
        )
        layout.setSpacing(SIZES["spacing_medium"])

        # Title label
        title = QLabel("Encoders:")
        title.setStyleSheet(
            f"color: {THEME.subtext1}; font-weight: bold; font-size: {SIZES['font_small']}pt;"
        )
        title.setFixedWidth(scaled(60))
        layout.addWidget(title)

        # Create bars for each joint
        for i, joint in enumerate(JOINTS):
            color = JOINT_COLORS[i] if i < len(JOINT_COLORS) else THEME.text

            bar = EncoderBar(
                joint_id=joint.id,
                name=joint.short_name,
                color=color,
            )
            bar.clicked.connect(self._on_bar_clicked)
            self._bars.append(bar)
            layout.addWidget(bar)

        # Set first bar as selected
        if self._bars:
            self._bars[0].set_selected(True)

    def _setup_timer(self):
        """Set up the update timer."""
        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self._update_values)
        self._update_timer.start(self.UPDATE_INTERVAL_MS)

    def _update_values(self):
        """Update all bar values from the data store."""
        for i, bar in enumerate(self._bars):
            joint_id = i + 1
            joint_data = self._data_store.get_joint(joint_id)
            if joint_data:
                bar.set_value(joint_data.current_position)

    def _on_bar_clicked(self, joint_id: int):
        """Handle bar click to select joint."""
        self.set_selected_joint(joint_id)
        self.joint_selected.emit(joint_id)

    @pyqtSlot(int)
    def set_selected_joint(self, joint_id: int):
        """Set the selected joint (1-indexed)."""
        self._selected_joint = joint_id

        # Update bar selection states
        for i, bar in enumerate(self._bars):
            bar.set_selected(i + 1 == joint_id)

    def set_range(self, min_val: float, max_val: float):
        """Set the display range for all bars."""
        for bar in self._bars:
            bar.set_range(min_val, max_val)
