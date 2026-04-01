from typing import Any

import pyqtgraph as pg
from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QComboBox,
    QLabel,
)
from PyQt6.QtCore import QTimer

from ..data.data_store import DataStore
from ..data.joint_config import get_joint_names
from .theme import THEME, JOINT_COLORS
from .scaling import SIZES


class SequencePlotter(QWidget):
    TIME_WINDOWS = [5, 10, 20, 30, 60]
    DEFAULT_TIME_WINDOW = 10

    UPDATE_INTERVAL_MS = 50

    def __init__(self, data_store: DataStore, parent=None):
        super().__init__(parent)
        self._data_store = data_store
        self._time_window = self.DEFAULT_TIME_WINDOW
        self._paused = False

        self._setup_ui()
        self._setup_plots()
        self._setup_timer()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
            0,
        )
        layout.setSpacing(SIZES["spacing_small"])

        control_layout = QHBoxLayout()
        control_layout.setSpacing(SIZES["spacing_small"])

        control_layout.addWidget(QLabel("Time:"))
        self._time_window_combo = QComboBox()
        for window in self.TIME_WINDOWS:
            self._time_window_combo.addItem(f"{window}s")
        self._time_window_combo.setCurrentText(f"{self.DEFAULT_TIME_WINDOW}s")
        self._time_window_combo.currentTextChanged.connect(self._on_time_window_changed)
        control_layout.addWidget(self._time_window_combo)

        control_layout.addStretch()

        self._pause_btn = QPushButton("Pause")
        self._pause_btn.setCheckable(True)
        self._pause_btn.toggled.connect(self._on_pause_toggled)
        control_layout.addWidget(self._pause_btn)

        self._clear_btn = QPushButton("Clear")
        self._clear_btn.clicked.connect(self._on_clear_clicked)
        control_layout.addWidget(self._clear_btn)

        layout.addLayout(control_layout)

        self._graphics_layout: Any = pg.GraphicsLayoutWidget()
        self._graphics_layout.setBackground(THEME.mantle)
        layout.addWidget(self._graphics_layout)

    def _setup_plots(self):
        font_size = SIZES["font_small"]
        label_style = {"color": THEME.text, "font-size": f"{font_size}pt"}

        self._plots = []
        self._position_curves = []
        self._target_curves = []

        joint_names = get_joint_names()[:6]

        for i in range(6):
            plot = self._graphics_layout.addPlot(row=i, col=0)

            plot.setLabel("left", joint_names[i], **label_style)

            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.getAxis("left").setPen(pg.mkPen(color=THEME.overlay0))
            plot.getAxis("left").setTextPen(pg.mkPen(color=THEME.text))
            plot.getAxis("bottom").setPen(pg.mkPen(color=THEME.overlay0))
            plot.getAxis("bottom").setTextPen(pg.mkPen(color=THEME.text))

            pos_curve = plot.plot(
                pen=pg.mkPen(color=JOINT_COLORS[i], width=2), name="Pos"
            )
            target_curve = plot.plot(
                pen=pg.mkPen(
                    color=THEME.red, width=2, style=pg.QtCore.Qt.PenStyle.DashLine
                ),
                name="Target",
            )

            self._plots.append(plot)
            self._position_curves.append(pos_curve)
            self._target_curves.append(target_curve)

            if i > 0:
                plot.setXLink(self._plots[0])

            if i < 5:
                plot.hideAxis("bottom")
            else:
                plot.setLabel("bottom", "Time", units="s", **label_style)

    def _setup_timer(self):
        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self._update_plot)
        self._update_timer.start(self.UPDATE_INTERVAL_MS)

    def _update_plot(self):
        if self._paused:
            return

        latest_time = 0

        for i in range(6):
            joint_data = self._data_store.get_joint(i + 1)
            if joint_data:
                timestamps, _, _ = joint_data.get_plot_data()
                if len(timestamps) > 0 and timestamps[-1] > latest_time:
                    latest_time = timestamps[-1]

        if latest_time == 0:
            return

        min_time = latest_time - self._time_window

        for i in range(6):
            joint_data = self._data_store.get_joint(i + 1)
            if not joint_data:
                continue

            timestamps, positions, targets = joint_data.get_plot_data()

            if len(timestamps) == 0:
                self._position_curves[i].setData([], [])
                self._target_curves[i].setData([], [])
                continue

            mask = timestamps >= min_time
            window_times = timestamps[mask]

            self._position_curves[i].setData(window_times, positions[mask])
            self._target_curves[i].setData(window_times, targets[mask])

        self._plots[5].setXRange(min_time, latest_time, padding=0.02)

    def _on_time_window_changed(self, text: str):
        try:
            self._time_window = int(text.replace("s", ""))
        except ValueError:
            self._time_window = self.DEFAULT_TIME_WINDOW

    def _on_pause_toggled(self, checked: bool):
        self._paused = checked
        self._pause_btn.setText("Resume" if checked else "Pause")

    def _on_clear_clicked(self):
        for i in range(6):
            self._data_store.clear_joint(i + 1)
            self._position_curves[i].setData([], [])
            self._target_curves[i].setData([], [])
