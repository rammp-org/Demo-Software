"""
State indicator widget showing the current system state.
"""

from PyQt6.QtWidgets import QWidget, QHBoxLayout, QLabel, QFrame
from PyQt6.QtCore import pyqtSlot, QTimer, Qt
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush

from .theme import THEME, STATE_COLORS, STATE_NAMES


class StateIndicatorDot(QWidget):
    """A colored circle indicator for the system state."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._color = QColor(THEME.overlay0)
        self._blink_state = True
        self._blinking = False

        self.setFixedSize(20, 20)

        # Blink timer for ESTOP state
        self._blink_timer = QTimer(self)
        self._blink_timer.timeout.connect(self._on_blink)

    def set_color(self, color: str, blink: bool = False):
        """Set the indicator color."""
        self._color = QColor(color)
        self._blinking = blink

        if blink:
            self._blink_timer.start(500)  # Blink every 500ms
        else:
            self._blink_timer.stop()
            self._blink_state = True

        self.update()

    def _on_blink(self):
        """Toggle blink state."""
        self._blink_state = not self._blink_state
        self.update()

    def paintEvent(self, event):
        """Paint the colored circle."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        # Calculate circle dimensions
        size = min(self.width(), self.height()) - 4
        x = (self.width() - size) // 2
        y = (self.height() - size) // 2

        # Draw outer ring
        pen = QPen(QColor(THEME.surface2))
        pen.setWidth(2)
        painter.setPen(pen)

        # Fill color (with blink support)
        if self._blink_state:
            painter.setBrush(QBrush(self._color))
        else:
            painter.setBrush(QBrush(QColor(THEME.surface0)))

        painter.drawEllipse(x, y, size, size)

        painter.end()


class StateIndicator(QWidget):
    """
    Widget showing the current system state with a colored indicator.

    States:
        0: INIT - Blue
        1: IDLE - Green
        2: TUNER_MODE - Yellow
        3: ESTOP - Red (blinking)
        4: SELF_LEVELING - Teal
        5: CONFIGURATION - Mauve
        6: AUTO_CURB_CLIMBING - Peach
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self._current_state = 0

        self._setup_ui()

    def _setup_ui(self):
        """Set up the widget layout."""
        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(8)

        # State label
        self._state_label = QLabel("State:")
        self._state_label.setStyleSheet(f"color: {THEME.subtext1}; font-weight: bold;")
        layout.addWidget(self._state_label)

        # State name
        self._state_name = QLabel("INIT")
        self._state_name.setStyleSheet(f"color: {THEME.text}; font-weight: bold;")
        self._state_name.setMinimumWidth(100)
        layout.addWidget(self._state_name)

        # Indicator dot
        self._indicator = StateIndicatorDot()
        layout.addWidget(self._indicator)

        # Set initial state
        self.set_state(0)

    @pyqtSlot(int)
    def set_state(self, state: int):
        """
        Set the current system state.

        Args:
            state: State number (0-6)
        """
        self._current_state = state

        # Get state info
        state_name = STATE_NAMES.get(state, f"UNKNOWN({state})")
        state_color = STATE_COLORS.get(state, THEME.overlay0)

        # Update display
        self._state_name.setText(state_name)
        self._state_name.setStyleSheet(f"color: {state_color}; font-weight: bold;")

        # ESTOP blinks
        blink = state == 3
        self._indicator.set_color(state_color, blink=blink)

    @property
    def current_state(self) -> int:
        """Get the current state."""
        return self._current_state
