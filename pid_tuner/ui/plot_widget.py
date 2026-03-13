"""
Real-time plotting widget using pyqtgraph.
"""

import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QPushButton,
    QComboBox,
    QLabel,
    QCheckBox,
)
from PyQt6.QtCore import QTimer

from ..data.data_store import DataStore


class PlotWidget(QWidget):
    """
    Real-time plot widget for displaying encoder data.

    Features:
    - Three stacked plots: Position, Velocity, PWM
    - Dual traces on position: actual (blue) and target (red dashed)
    - Rolling time window (configurable)
    - Auto-scaling with manual override
    - Pause/Resume functionality
    - Toggle visibility for each plot
    """

    # Time window options in seconds
    TIME_WINDOWS = [5, 10, 20, 30, 60]
    DEFAULT_TIME_WINDOW = 10  # seconds

    # Plot update rate
    UPDATE_INTERVAL_MS = 50  # 20 Hz update rate for smooth animation

    def __init__(self, data_store: DataStore, parent=None):
        super().__init__(parent)
        self._data_store = data_store
        self._time_window = self.DEFAULT_TIME_WINDOW
        self._paused = False

        # Visibility flags for each plot
        self._show_position = True
        self._show_velocity = True
        self._show_pwm = True

        self._setup_ui()
        self._setup_plots()
        self._setup_timer()

        # Connect to data store updates
        self._data_store.data_updated.connect(self._on_data_updated)

    def _setup_ui(self):
        """Set up the widget layout."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Control bar
        control_layout = QHBoxLayout()

        # Time window selector
        control_layout.addWidget(QLabel("Time Window:"))
        self._time_window_combo = QComboBox()
        for window in self.TIME_WINDOWS:
            self._time_window_combo.addItem(f"{window}s")
        self._time_window_combo.setCurrentText(f"{self.DEFAULT_TIME_WINDOW}s")
        self._time_window_combo.currentTextChanged.connect(self._on_time_window_changed)
        control_layout.addWidget(self._time_window_combo)

        # Plot visibility toggles
        control_layout.addWidget(QLabel("  Show:"))

        self._pos_checkbox = QCheckBox("Position")
        self._pos_checkbox.setChecked(True)
        self._pos_checkbox.toggled.connect(self._on_pos_toggled)
        control_layout.addWidget(self._pos_checkbox)

        self._vel_checkbox = QCheckBox("Velocity")
        self._vel_checkbox.setChecked(True)
        self._vel_checkbox.toggled.connect(self._on_vel_toggled)
        control_layout.addWidget(self._vel_checkbox)

        self._pwm_checkbox = QCheckBox("PWM")
        self._pwm_checkbox.setChecked(True)
        self._pwm_checkbox.toggled.connect(self._on_pwm_toggled)
        control_layout.addWidget(self._pwm_checkbox)

        control_layout.addStretch()

        # Simulate button - allows plotting targets without serial connection
        self._simulate_btn = QPushButton("Simulate")
        self._simulate_btn.setCheckable(True)
        self._simulate_btn.setToolTip(
            "Enable simulation mode to preview target signals without Teensy connected"
        )
        self._simulate_btn.toggled.connect(self._on_simulate_toggled)
        self._simulate_btn.setStyleSheet(
            "QPushButton:checked { background-color: #4CAF50; color: white; }"
        )
        control_layout.addWidget(self._simulate_btn)

        # Pause/Resume button
        self._pause_btn = QPushButton("Pause")
        self._pause_btn.setCheckable(True)
        self._pause_btn.toggled.connect(self._on_pause_toggled)
        control_layout.addWidget(self._pause_btn)

        # Clear button
        self._clear_btn = QPushButton("Clear")
        self._clear_btn.clicked.connect(self._on_clear_clicked)
        control_layout.addWidget(self._clear_btn)

        # Auto-scale button
        self._autoscale_btn = QPushButton("Auto Scale")
        self._autoscale_btn.clicked.connect(self._on_autoscale_clicked)
        control_layout.addWidget(self._autoscale_btn)

        layout.addLayout(control_layout)

        # Graphics layout for stacked plots
        self._graphics_layout = pg.GraphicsLayoutWidget()
        self._graphics_layout.setBackground("w")
        layout.addWidget(self._graphics_layout)

    def _setup_plots(self):
        """Set up the three stacked pyqtgraph plots."""
        # Position plot (top)
        self._pos_plot = self._graphics_layout.addPlot(row=0, col=0)
        self._pos_plot.setLabel("left", "Position", units="cm")
        self._pos_plot.setTitle("Position vs Target")
        self._pos_plot.showGrid(x=True, y=True, alpha=0.3)
        self._pos_plot.addLegend(offset=(10, 10))

        # Position curves
        self._position_curve = self._pos_plot.plot(
            pen=pg.mkPen(color="b", width=2), name="Position"
        )
        self._target_curve = self._pos_plot.plot(
            pen=pg.mkPen(color="r", width=2, style=pg.QtCore.Qt.PenStyle.DashLine),
            name="Target",
        )

        # Velocity plot (middle)
        self._vel_plot = self._graphics_layout.addPlot(row=1, col=0)
        self._vel_plot.setLabel("left", "Velocity", units="cm/s")
        self._vel_plot.setTitle("Velocity")
        self._vel_plot.showGrid(x=True, y=True, alpha=0.3)

        self._velocity_curve = self._vel_plot.plot(
            pen=pg.mkPen(color=(0, 150, 0), width=2), name="Velocity"
        )

        # PWM plot (bottom)
        self._pwm_plot = self._graphics_layout.addPlot(row=2, col=0)
        self._pwm_plot.setLabel("left", "PWM")
        self._pwm_plot.setLabel("bottom", "Time", units="s")
        self._pwm_plot.setTitle("PWM Output")
        self._pwm_plot.showGrid(x=True, y=True, alpha=0.3)

        self._pwm_curve = self._pwm_plot.plot(
            pen=pg.mkPen(color=(150, 0, 150), width=2), name="PWM"
        )

        # Link X axes so they scroll together
        self._vel_plot.setXLink(self._pos_plot)
        self._pwm_plot.setXLink(self._pos_plot)

        # Hide X labels for top two plots (shared axis)
        self._pos_plot.hideAxis("bottom")
        self._vel_plot.hideAxis("bottom")

    def _setup_timer(self):
        """Set up the update timer."""
        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self._update_plot)
        self._update_timer.start(self.UPDATE_INTERVAL_MS)

    def _on_data_updated(self, joint_id: int):
        """Handle data update signal (not used directly, timer handles updates)."""
        pass

    def _update_plot(self):
        """Update all plots with current data."""
        if self._paused:
            return

        joint_data = self._data_store.get_selected_joint_data()
        timestamps, positions, targets = joint_data.get_plot_data()

        if len(timestamps) == 0:
            return

        # Apply time window - show only last N seconds
        current_time = timestamps[-1] if len(timestamps) > 0 else 0
        min_time = current_time - self._time_window

        # Find data within window
        mask = timestamps >= min_time
        window_times = timestamps[mask]

        # Update position plot
        if self._show_position:
            window_positions = positions[mask]
            window_targets = targets[mask]
            self._position_curve.setData(window_times, window_positions)
            self._target_curve.setData(window_times, window_targets)

        # Update velocity plot
        if self._show_velocity:
            vel_timestamps, velocities = joint_data.get_velocity_data()
            if len(vel_timestamps) > 0:
                vel_mask = vel_timestamps >= min_time
                self._velocity_curve.setData(
                    vel_timestamps[vel_mask], velocities[vel_mask]
                )

        # Update PWM plot
        if self._show_pwm:
            pwm_timestamps, pwms = joint_data.get_pwm_data()
            if len(pwm_timestamps) > 0:
                pwm_mask = pwm_timestamps >= min_time
                self._pwm_curve.setData(pwm_timestamps[pwm_mask], pwms[pwm_mask])

        # Update x-axis range to show rolling window (only on PWM plot, others are linked)
        self._pwm_plot.setXRange(min_time, current_time, padding=0.02)

    def _on_time_window_changed(self, text: str):
        """Handle time window selection change."""
        try:
            self._time_window = int(text.replace("s", ""))
        except ValueError:
            self._time_window = self.DEFAULT_TIME_WINDOW

    def _on_pos_toggled(self, checked: bool):
        """Handle position plot visibility toggle."""
        self._show_position = checked
        self._pos_plot.setVisible(checked)
        self._update_plot_layout()

    def _on_vel_toggled(self, checked: bool):
        """Handle velocity plot visibility toggle."""
        self._show_velocity = checked
        self._vel_plot.setVisible(checked)
        self._update_plot_layout()

    def _on_pwm_toggled(self, checked: bool):
        """Handle PWM plot visibility toggle."""
        self._show_pwm = checked
        self._pwm_plot.setVisible(checked)
        self._update_plot_layout()

    def _update_plot_layout(self):
        """Update plot layout when visibility changes."""
        # Show bottom axis on the lowest visible plot
        self._pos_plot.hideAxis("bottom")
        self._vel_plot.hideAxis("bottom")
        self._pwm_plot.hideAxis("bottom")

        if self._show_pwm:
            self._pwm_plot.showAxis("bottom")
        elif self._show_velocity:
            self._vel_plot.showAxis("bottom")
        elif self._show_position:
            self._pos_plot.showAxis("bottom")

    def _on_pause_toggled(self, checked: bool):
        """Handle pause button toggle."""
        self._paused = checked
        self._pause_btn.setText("Resume" if checked else "Pause")

    def _on_clear_clicked(self):
        """Handle clear button click."""
        self._data_store.clear_joint(self._data_store.selected_joint)
        self._position_curve.setData([], [])
        self._target_curve.setData([], [])
        self._velocity_curve.setData([], [])
        self._pwm_curve.setData([], [])

    def _on_autoscale_clicked(self):
        """Handle auto-scale button click."""
        self._pos_plot.enableAutoRange()
        self._vel_plot.enableAutoRange()
        self._pwm_plot.enableAutoRange()

    def _on_simulate_toggled(self, checked: bool):
        """Handle simulate button toggle."""
        self._data_store.simulation_mode = checked
        if checked:
            # Clear data when starting simulation for clean preview
            self._data_store.clear_joint(self._data_store.selected_joint)

    def set_paused(self, paused: bool):
        """Set the paused state."""
        self._paused = paused
        self._pause_btn.setChecked(paused)

    def set_simulation_mode(self, enabled: bool):
        """Set the simulation mode state (called externally)."""
        self._simulate_btn.setChecked(enabled)
