"""
Encoder overview widget showing all 6 encoder positions as horizontal bars.
"""

from PyQt6.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QLabel,
    QSizePolicy,
    QPushButton,
)
from PyQt6.QtCore import pyqtSlot, pyqtSignal, Qt, QTimer
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QFont

from .theme import THEME, JOINT_COLORS
from .scaling import SIZES, scaled
from ..data.data_store import DataStore
from ..data.joint_config import JOINTS
from ..serial_driver.serial_handler import SerialHandler

# Mode colors for indicator dots
MODE_OPEN_LOOP = 0
MODE_VELOCITY = 1
MODE_POSITION = 2

MODE_COLORS = {
    MODE_OPEN_LOOP: THEME.red,
    MODE_VELOCITY: THEME.yellow,
    MODE_POSITION: THEME.blue,
}


class EncoderBar(QWidget):
    """A single horizontal bar representing an encoder position."""

    clicked = pyqtSignal(int)  # Emits joint_id when clicked
    jog_requested = pyqtSignal(int, float)

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

        # Limit switch indicators (only for carriage joints 5 and 6)
        self._fwd_limit = False
        self._bwd_limit = False
        self._show_limits = joint_id in [5, 6]

        # Mode indicator
        self._mode = MODE_OPEN_LOOP

        # Use scaled heights
        self.setMinimumHeight(SIZES["encoder_bar_height"])
        self.setMaximumHeight(SIZES["encoder_bar_max_height"])
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self.setCursor(Qt.CursorShape.PointingHandCursor)

        self._left_jog_btn = QPushButton("◀", self)
        self._right_jog_btn = QPushButton("▶", self)
        self._setup_jog_buttons()

    def _setup_jog_buttons(self):
        button_style = f"""
            QPushButton {{
                background-color: {THEME.surface1};
                color: {THEME.text};
                border: 1px solid {THEME.surface2};
                border-radius: {scaled(2)}px;
                font-size: {SIZES["font_small"]}pt;
                padding: 0px;
            }}
            QPushButton:hover {{
                background-color: {THEME.surface2};
            }}
            QPushButton:pressed {{
                background-color: {THEME.blue};
                color: {THEME.crust};
            }}
        """
        btn_w = scaled(18)
        btn_h = scaled(16)

        for btn in (self._left_jog_btn, self._right_jog_btn):
            btn.setFixedSize(btn_w, btn_h)
            btn.setStyleSheet(button_style)
            btn.setCursor(Qt.CursorShape.PointingHandCursor)

        self._left_jog_btn.clicked.connect(lambda: self._emit_jog_request(-1.0))
        self._right_jog_btn.clicked.connect(lambda: self._emit_jog_request(1.0))

    def _emit_jog_request(self, direction: float):
        range_span = self._max_val - self._min_val
        step_mag = abs(0.05 * range_span) if range_span != 0 else 50.0
        self.jog_requested.emit(self._joint_id, direction * step_mag)

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

    def set_limits(self, fwd: bool, bwd: bool):
        """Set limit switch states for this bar."""
        self._fwd_limit = fwd
        self._bwd_limit = bwd
        self.update()

    def set_mode(self, mode: int):
        """Set the control mode for this joint."""
        self._mode = mode
        self.update()

    def mousePressEvent(self, a0):
        """Handle mouse click to select this joint."""
        self.clicked.emit(self._joint_id)

    def paintEvent(self, a0):
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
        bar_right = bar_left + max(0, bar_width)

        y = (height - self._left_jog_btn.height()) // 2
        self._left_jog_btn.move(bar_left + scaled(2), y)
        self._right_jog_btn.move(
            bar_right - self._right_jog_btn.width() - scaled(2),
            y,
        )

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
            # Draw danger zones (10% from each limit)
            danger_width = int(bar_width * 0.1)
            danger_color = QColor(THEME.red)
            danger_color.setAlpha(40)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QBrush(danger_color))

            # Left danger zone (near min limit)
            painter.drawRect(bar_rect_x, bar_rect_y, danger_width, bar_rect_h)

            # Right danger zone (near max limit)
            painter.drawRect(
                bar_rect_x + bar_width - danger_width,
                bar_rect_y,
                danger_width,
                bar_rect_h,
            )

            # Draw limit markers (vertical lines at edges)
            painter.setPen(QPen(QColor(THEME.red), 2))
            painter.drawLine(
                bar_rect_x, bar_rect_y, bar_rect_x, bar_rect_y + bar_rect_h
            )
            painter.drawLine(
                bar_rect_x + bar_width,
                bar_rect_y,
                bar_rect_x + bar_width,
                bar_rect_y + bar_rect_h,
            )

            # Normalize value to 0-1 range
            normalized = (self._value - self._min_val) / range_span
            normalized = max(0.0, min(1.0, normalized))

            # Determine fill color based on proximity to limits
            dist_to_min = abs(self._value - self._min_val) / range_span
            dist_to_max = abs(self._value - self._max_val) / range_span

            if dist_to_min < 0.1 or dist_to_max < 0.1:
                # In danger zone - red fill
                fill_color = QColor(THEME.red)
            elif dist_to_min < 0.2 or dist_to_max < 0.2:
                # Approaching limits - yellow fill
                fill_color = QColor(THEME.yellow)
            else:
                # Normal - joint color
                fill_color = self._color

            # Calculate fill width and position relative to range midpoint
            mid_val = (self._min_val + self._max_val) / 2.0
            norm_mid = (
                mid_val - self._min_val
            ) / range_span  # midpoint normalized to [0, 1]
            center_x = bar_rect_x + norm_mid * bar_width

            # Fill from midpoint toward the current normalized position
            fill_width = abs(normalized - norm_mid) * bar_width

            if self._value >= mid_val:
                fill_x = center_x
            else:
                fill_x = center_x - fill_width

            # Draw filled portion with dynamic color
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QBrush(fill_color))
            painter.drawRoundedRect(
                int(fill_x),
                bar_rect_y + 1,
                int(fill_width),
                bar_rect_h - 2,
                scaled(2),
                scaled(2),
            )

            # Draw center line at range midpoint
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

        # Draw limit switch indicators if applicable (for carriage bars)
        if self._show_limits:
            indicator_size = scaled(8)
            y_center = height // 2

            # Backward limit indicator (left side)
            bwd_color = QColor(THEME.red) if self._bwd_limit else QColor(THEME.green)
            painter.setPen(Qt.PenStyle.NoPen)
            painter.setBrush(QBrush(bwd_color))
            painter.drawEllipse(
                bar_left + scaled(2),
                y_center - indicator_size // 2,
                indicator_size,
                indicator_size,
            )

            # Forward limit indicator (right side)
            fwd_color = QColor(THEME.red) if self._fwd_limit else QColor(THEME.green)
            painter.setBrush(QBrush(fwd_color))
            painter.drawEllipse(
                bar_left + bar_width - indicator_size - scaled(2),
                y_center - indicator_size // 2,
                indicator_size,
                indicator_size,
            )

        # Draw mode indicator dot (top-right corner, above bar)
        mode_indicator_size = scaled(10)
        mode_color = QColor(MODE_COLORS.get(self._mode, THEME.overlay0))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(mode_color))
        painter.drawEllipse(
            bar_left + bar_width - mode_indicator_size - scaled(2),
            margin,
            mode_indicator_size,
            mode_indicator_size,
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

    def __init__(
        self, data_store: DataStore, serial_handler: SerialHandler, parent=None
    ):
        super().__init__(parent)
        self._data_store = data_store
        self._serial_handler = serial_handler
        self._bars = []
        self._selected_joint = 1

        self._setup_ui()
        self._setup_timer()

        # Connect to limit switch updates
        self._data_store.limits_updated.connect(self._update_limits)
        self._data_store.config_updated.connect(self._on_config_updated)

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
            bar.jog_requested.connect(self._on_jog_requested)
            self._bars.append(bar)
            layout.addWidget(bar)

        # Set first bar as selected
        if self._bars:
            self._bars[0].set_selected(True)

        # Bootstrap bar ranges from any configs already loaded in DataStore
        for joint in JOINTS:
            self._on_config_updated(joint.id)

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

    def _on_jog_requested(self, joint_id: int, step_size: float):
        self._serial_handler.set_mode(joint_id, MODE_POSITION)

        joint_data = self._data_store.get_joint(joint_id)
        if joint_data is None:
            return

        new_target = joint_data.current_position + step_size
        self._serial_handler.set_target(joint_id, new_target)
        self._data_store.set_target(joint_id, new_target)

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

    def _update_limits(self):
        """Update limit switch indicators on carriage bars."""
        limits = self._data_store.limit_switches
        if len(limits) >= 4:
            # ML carriage (joint 5, index 4): limits[0]=fwd, limits[1]=bwd
            if len(self._bars) > 4:
                self._bars[4].set_limits(limits[0], limits[1])
            # MR carriage (joint 6, index 5): limits[2]=fwd, limits[3]=bwd
            if len(self._bars) > 5:
                self._bars[5].set_limits(limits[2], limits[3])

    def _on_config_updated(self, joint_id: int):
        """Update bar range when config is loaded from Teensy."""
        config = self._data_store.get_config(joint_id)
        if config is not None:
            min_limit = config.pos_limit_min
            max_limit = config.pos_limit_max
            if min_limit != max_limit:
                self._bars[joint_id - 1].set_range(min_limit, max_limit)

    def set_mode_for_joint(self, joint_id: int, mode: int):
        """Set the mode indicator for a specific joint."""
        if 1 <= joint_id <= len(self._bars):
            self._bars[joint_id - 1].set_mode(mode)

    def set_mode_for_all(self, mode: int):
        """Set the mode indicator for all joints."""
        for bar in self._bars:
            bar.set_mode(mode)
