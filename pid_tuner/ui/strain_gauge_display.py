"""
Strain gauge display widget showing 4 strain gauge readings as horizontal bars.
"""

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtCore import pyqtSlot, Qt
from PyQt6.QtGui import QPainter, QColor, QPen, QBrush, QFont

from .theme import THEME
from .scaling import SIZES, scaled
from ..data.data_store import DataStore


class StrainGaugeBar(QWidget):
    """A single horizontal bar representing a strain gauge reading."""

    def __init__(self, name: str, max_scale: float = 4095.0, parent=None):
        super().__init__(parent)
        self._name = name
        self._max_scale = max_scale
        self._value = 0.0

        self.setMinimumHeight(SIZES["encoder_bar_height"])
        self.setMaximumHeight(SIZES["encoder_bar_max_height"])

    def set_value(self, value: float):
        """Set the current strain gauge value."""
        self._value = value
        self.update()

    def set_max_scale(self, max_scale: float):
        """Set the maximum scale for the bar."""
        self._max_scale = max_scale
        self.update()

    def _get_bar_color(self, ratio: float) -> QColor:
        """Get color based on value/max_scale ratio (green to orange gradient)."""
        green = QColor("#a6e3a1")
        orange = QColor("#fab387")

        r = int(green.red() + (orange.red() - green.red()) * ratio)
        g = int(green.green() + (orange.green() - green.green()) * ratio)
        b = int(green.blue() + (orange.blue() - green.blue()) * ratio)

        return QColor(r, g, b)

    def paintEvent(self, event):
        """Paint the strain gauge bar."""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        width = self.width()
        height = self.height()
        margin = scaled(2)
        bar_height = height - 2 * margin
        label_width = scaled(30)
        bar_left = label_width + scaled(4)
        bar_width = width - bar_left - scaled(8)

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

        bar_rect_x = bar_left
        bar_rect_y = margin + scaled(2)
        bar_rect_h = bar_height - scaled(4)

        painter.setPen(QPen(QColor(THEME.surface1), 1))
        painter.setBrush(QBrush(QColor(THEME.surface0)))
        painter.drawRoundedRect(
            bar_rect_x, bar_rect_y, bar_width, bar_rect_h, scaled(3), scaled(3)
        )

        if self._max_scale > 0:
            ratio = max(0.0, min(1.0, self._value / self._max_scale))
            fill_width = int(ratio * bar_width)

            if fill_width > 0:
                fill_color = self._get_bar_color(ratio)
                painter.setPen(Qt.PenStyle.NoPen)
                painter.setBrush(QBrush(fill_color))
                painter.drawRoundedRect(
                    bar_rect_x + 1,
                    bar_rect_y + 1,
                    fill_width - 2,
                    bar_rect_h - 2,
                    scaled(2),
                    scaled(2),
                )

        painter.setPen(QColor(THEME.text))
        font.setBold(False)
        painter.setFont(font)
        value_text = f"{self._value:.1f}"

        text_width = bar_width
        painter.drawText(
            bar_rect_x,
            bar_rect_y,
            text_width,
            bar_rect_h,
            Qt.AlignmentFlag.AlignVCenter | Qt.AlignmentFlag.AlignHCenter,
            value_text,
        )

        painter.end()


class StrainGaugeDisplay(QWidget):
    """
    Widget displaying 4 strain gauge readings as horizontal bars.

    Features:
    - RC, FC, ML, MR strain gauge displays
    - Color gradient from green to orange based on value/max_scale
    - Updates on data store signals
    """

    def __init__(self, data_store: DataStore, max_scale: float = 4095.0, parent=None):
        super().__init__(parent)
        self._data_store = data_store
        self._max_scale = max_scale
        self._bars = {}

        self._setup_ui()

        self._data_store.strain_gauge_updated.connect(self._update_display)

    def _setup_ui(self):
        """Set up the widget layout."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(
            SIZES["margin_medium"],
            SIZES["margin_small"],
            SIZES["margin_medium"],
            SIZES["margin_small"],
        )
        layout.setSpacing(SIZES["spacing_small"])

        title = QLabel("Strain Gauges:")
        title.setStyleSheet(
            f"color: {THEME.subtext1}; font-weight: bold; font-size: {SIZES['font_small']}pt;"
        )
        layout.addWidget(title)

        gauges = [
            ("RC", "sg_rc"),
            ("FC", "sg_fc"),
            ("ML", "sg_ml"),
            ("MR", "sg_mr"),
        ]

        for name, key in gauges:
            bar = StrainGaugeBar(name, self._max_scale)
            self._bars[key] = bar
            layout.addWidget(bar)

    @pyqtSlot()
    def _update_display(self):
        """Update display with latest strain gauge data."""
        self._bars["sg_rc"].set_value(self._data_store.sg_rc_value)
        self._bars["sg_fc"].set_value(self._data_store.sg_fc_value)
        self._bars["sg_ml"].set_value(self._data_store.sg_ml_value)
        self._bars["sg_mr"].set_value(self._data_store.sg_mr_value)

    def set_max_scale(self, max_scale: float):
        """Set the maximum scale for all bars."""
        self._max_scale = max_scale
        for bar in self._bars.values():
            bar.set_max_scale(max_scale)
