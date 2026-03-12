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
)
from PyQt6.QtCore import QTimer

from ..data.data_store import DataStore


class PlotWidget(QWidget):
    """
    Real-time plot widget for displaying encoder position vs target.

    Features:
    - Dual traces: actual position (blue) and target (red dashed)
    - Rolling time window (configurable)
    - Auto-scaling with manual override
    - Pause/Resume functionality
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

        self._setup_ui()
        self._setup_plot()
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

        # Plot widget
        self._plot_widget = pg.PlotWidget()
        layout.addWidget(self._plot_widget)

    def _setup_plot(self):
        """Set up the pyqtgraph plot."""
        self._plot_widget.setBackground("w")
        self._plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self._plot_widget.setLabel("left", "Position", units="cm")
        self._plot_widget.setLabel("bottom", "Time", units="s")
        self._plot_widget.setTitle("Encoder Position vs Target")

        # Enable mouse interaction
        self._plot_widget.setMouseEnabled(x=True, y=True)

        # Create plot items
        # Actual position - solid blue line
        self._position_curve = self._plot_widget.plot(
            pen=pg.mkPen(color="b", width=2), name="Position"
        )

        # Target position - dashed red line
        self._target_curve = self._plot_widget.plot(
            pen=pg.mkPen(color="r", width=2, style=pg.QtCore.Qt.PenStyle.DashLine),
            name="Target",
        )

        # Add legend
        self._plot_widget.addLegend()

    def _setup_timer(self):
        """Set up the update timer."""
        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self._update_plot)
        self._update_timer.start(self.UPDATE_INTERVAL_MS)

    def _on_data_updated(self, joint_id: int):
        """Handle data update signal (not used directly, timer handles updates)."""
        pass

    def _update_plot(self):
        """Update the plot with current data."""
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
        window_positions = positions[mask]
        window_targets = targets[mask]

        # Update curves
        self._position_curve.setData(window_times, window_positions)
        self._target_curve.setData(window_times, window_targets)

        # Update x-axis range to show rolling window
        self._plot_widget.setXRange(min_time, current_time, padding=0.02)

    def _on_time_window_changed(self, text: str):
        """Handle time window selection change."""
        try:
            self._time_window = int(text.replace("s", ""))
        except ValueError:
            self._time_window = self.DEFAULT_TIME_WINDOW

    def _on_pause_toggled(self, checked: bool):
        """Handle pause button toggle."""
        self._paused = checked
        self._pause_btn.setText("Resume" if checked else "Pause")

    def _on_clear_clicked(self):
        """Handle clear button click."""
        self._data_store.clear_joint(self._data_store.selected_joint)
        self._position_curve.setData([], [])
        self._target_curve.setData([], [])

    def _on_autoscale_clicked(self):
        """Handle auto-scale button click."""
        self._plot_widget.enableAutoRange()

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
