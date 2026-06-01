from typing import Any, Dict, Tuple
from ..serial_driver.keyframe import NUM_MOTORS

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
        self._joint_limits: Dict[int, Tuple[float, float]] = {}
        self._target_times: Dict[int, list] = {i: [] for i in range(1, 9)}
        self._target_values: Dict[int, list] = {i: [] for i in range(1, 9)}
        self._last_target: Dict[int, float] = {}

        self._setup_ui()
        self._setup_plots()
        self._setup_timer()

        self._data_store.config_updated.connect(self._on_config_updated)

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
        self._position_curves = {}
        self._target_curves = {}

        self._plot_groups = [
            ("RC", [(1, "RC")]),
            ("FC", [(2, "FC")]),
            ("Legs", [(3, "ML"), (4, "MR")]),
            ("Carriages", [(5, "ML_Car"), (6, "MR_Car")]),
            ("Drive Error", [(7, "Drive_FB"), (8, "Drive_LR")]),
        ]

        for i, (group_name, joints) in enumerate(self._plot_groups):
            plot = self._graphics_layout.addPlot(row=i, col=0)

            plot.setLabel("left", group_name, **label_style)

            plot.showGrid(x=True, y=True, alpha=0.3)
            plot.getAxis("left").setPen(pg.mkPen(color=THEME.overlay0))
            plot.getAxis("left").setTextPen(pg.mkPen(color=THEME.text))
            plot.getAxis("bottom").setPen(pg.mkPen(color=THEME.overlay0))
            plot.getAxis("bottom").setTextPen(pg.mkPen(color=THEME.text))

            if len(joints) > 1:
                legend = plot.addLegend(offset=(10, 10))
                legend.setLabelTextColor(THEME.text)

            for joint_id, joint_name in joints:
                color = JOINT_COLORS[joint_id - 1]
                pos_curve = plot.plot(
                    pen=pg.mkPen(color=color, width=2), name=f"{joint_name} Pos"
                )
                target_curve = plot.plot(
                    pen=pg.mkPen(
                        color=color, width=1, style=pg.QtCore.Qt.PenStyle.DotLine
                    ),
                    name=f"{joint_name} Target",
                )

                self._position_curves[joint_id] = pos_curve
                self._target_curves[joint_id] = target_curve

            plot.enableAutoRange("y", True)
            self._plots.append(plot)

            if i > 0:
                plot.setXLink(self._plots[0])

            if i < 4:
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

        for i in range(min(NUM_MOTORS, 8)):
            joint_data = self._data_store.get_joint(i + 1)
            if joint_data:
                timestamps, _, _ = joint_data.get_plot_data()
                if len(timestamps) > 0 and timestamps[-1] > latest_time:
                    latest_time = timestamps[-1]

        if latest_time == 0:
            return

        min_time = latest_time - self._time_window

        for i in range(min(NUM_MOTORS, 8)):
            joint_id = i + 1
            joint_data = self._data_store.get_joint(joint_id)
            if not joint_data:
                continue

            timestamps, positions, targets = joint_data.get_plot_data()

            if len(timestamps) == 0:
                self._position_curves[joint_id].setData([], [])
                self._target_curves[joint_id].setData([], [])
                continue

            mask = timestamps >= min_time
            window_times = timestamps[mask]

            if len(window_times) == 0:
                self._position_curves[joint_id].setData([], [])
                self._target_curves[joint_id].setData([], [])
                continue

            if joint_id in (7, 8):
                error = targets[mask] - positions[mask]
                self._position_curves[joint_id].setData(window_times, error)
                self._target_curves[joint_id].setData([], [])
            else:
                self._position_curves[joint_id].setData(window_times, positions[mask])

                seq_targets = self._data_store.get_seq_targets()
                if joint_id in seq_targets:
                    target_val = seq_targets[joint_id]
                    prev = self._last_target.get(joint_id)
                    if prev is None or prev != target_val:
                        if prev is not None:
                            self._target_times[joint_id].append(latest_time)
                            self._target_values[joint_id].append(prev)
                        self._target_times[joint_id].append(latest_time)
                        self._target_values[joint_id].append(target_val)
                        self._last_target[joint_id] = target_val

                t_hist = self._target_times[joint_id]
                v_hist = self._target_values[joint_id]
                while t_hist and t_hist[0] < min_time:
                    t_hist.pop(0)
                    v_hist.pop(0)

                if t_hist:
                    plot_t = list(t_hist) + [latest_time]
                    plot_v = list(v_hist) + [v_hist[-1]]
                    self._target_curves[joint_id].setData(plot_t, plot_v)
                else:
                    self._target_curves[joint_id].setData([], [])

        self._plots[4].setXRange(min_time, latest_time, padding=0.02)

    def _on_config_updated(self, joint_id: int):
        config = self._data_store.get_config(joint_id)
        if (
            config
            and hasattr(config, "pos_limit_min")
            and hasattr(config, "pos_limit_max")
        ):
            self._joint_limits[joint_id] = (config.pos_limit_min, config.pos_limit_max)
            self._update_y_ranges()

    def _update_y_ranges(self):
        for i, (group_name, joints) in enumerate(self._plot_groups):
            if any(jid in (7, 8) for jid, _ in joints):
                continue

            min_limit = float("inf")
            max_limit = float("-inf")
            has_limits = False

            for joint_id, _ in joints:
                if joint_id in self._joint_limits:
                    j_min, j_max = self._joint_limits[joint_id]
                    min_limit = min(min_limit, j_min)
                    max_limit = max(max_limit, j_max)
                    has_limits = True

            if has_limits and min_limit < max_limit:
                self._plots[i].enableAutoRange("y", False)
                padding = (max_limit - min_limit) * 0.05
                self._plots[i].setYRange(
                    min_limit - padding, max_limit + padding, padding=0
                )

    def _on_time_window_changed(self, text: str):
        try:
            self._time_window = int(text.replace("s", ""))
        except ValueError:
            self._time_window = self.DEFAULT_TIME_WINDOW

    def _on_pause_toggled(self, checked: bool):
        self._paused = checked
        self._pause_btn.setText("Resume" if checked else "Pause")

    def _on_clear_clicked(self):
        for i in range(min(NUM_MOTORS, 8)):
            joint_id = i + 1
            self._data_store.clear_joint(joint_id)
            self._position_curves[joint_id].setData([], [])
            self._target_curves[joint_id].setData([], [])
            self._target_times[joint_id].clear()
            self._target_values[joint_id].clear()
        self._last_target.clear()
