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
from .theme import get_plot_colors, THEME
from .scaling import SIZES


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
        self._show_imu = False  # IMU plot hidden by default

        self._setup_ui()
        self._setup_plots()
        self._setup_timer()

        # Connect to data store updates
        self._data_store.data_updated.connect(self._on_data_updated)

    def _setup_ui(self):
        """Set up the widget layout."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
            0,
        )
        layout.setSpacing(SIZES["spacing_small"])

        # Control bar
        control_layout = QHBoxLayout()
        control_layout.setSpacing(SIZES["spacing_small"])

        # Time window selector
        control_layout.addWidget(QLabel("Time:"))
        self._time_window_combo = QComboBox()
        for window in self.TIME_WINDOWS:
            self._time_window_combo.addItem(f"{window}s")
        self._time_window_combo.setCurrentText(f"{self.DEFAULT_TIME_WINDOW}s")
        self._time_window_combo.currentTextChanged.connect(self._on_time_window_changed)
        control_layout.addWidget(self._time_window_combo)

        # Plot visibility toggles
        control_layout.addWidget(QLabel("Show:"))

        self._pos_checkbox = QCheckBox("Pos")
        self._pos_checkbox.setChecked(True)
        self._pos_checkbox.toggled.connect(self._on_pos_toggled)
        control_layout.addWidget(self._pos_checkbox)

        self._vel_checkbox = QCheckBox("Vel")
        self._vel_checkbox.setChecked(True)
        self._vel_checkbox.toggled.connect(self._on_vel_toggled)
        control_layout.addWidget(self._vel_checkbox)

        self._pwm_checkbox = QCheckBox("PWM")
        self._pwm_checkbox.setChecked(True)
        self._pwm_checkbox.toggled.connect(self._on_pwm_toggled)
        control_layout.addWidget(self._pwm_checkbox)

        self._imu_checkbox = QCheckBox("IMU")
        self._imu_checkbox.setChecked(False)
        self._imu_checkbox.toggled.connect(self._on_imu_toggled)
        control_layout.addWidget(self._imu_checkbox)

        control_layout.addStretch()

        # Export CSV button
        self._export_btn = QPushButton("Export CSV")
        self._export_btn.setToolTip("Export visible graph data to CSV file")
        self._export_btn.setStyleSheet(
            f"background-color: {THEME.green}; color: {THEME.crust};"
        )
        self._export_btn.clicked.connect(self._on_export_csv)
        control_layout.addWidget(self._export_btn)

        # Simulate button - allows plotting targets without serial connection
        self._simulate_btn = QPushButton("Sim")
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
        self._autoscale_btn = QPushButton("Auto")
        self._autoscale_btn.setToolTip("Auto Scale")
        self._autoscale_btn.clicked.connect(self._on_autoscale_clicked)
        control_layout.addWidget(self._autoscale_btn)

        layout.addLayout(control_layout)

        # Graphics layout for stacked plots
        self._graphics_layout = pg.GraphicsLayoutWidget()
        colors = get_plot_colors()
        self._graphics_layout.setBackground(colors["background"])
        layout.addWidget(self._graphics_layout)

    def _setup_plots(self):
        """Set up the three stacked pyqtgraph plots."""
        colors = get_plot_colors()

        # Common plot styling with scaled fonts
        font_size = SIZES["font_normal"]
        title_size = SIZES["font_medium"]
        label_style = {"color": THEME.text, "font-size": f"{font_size}pt"}
        title_style = {"color": THEME.text, "size": f"{title_size}pt"}

        # Position plot (top)
        self._pos_plot = self._graphics_layout.addPlot(row=0, col=0)
        self._pos_plot.setLabel("left", "Position", units="cm", **label_style)
        self._pos_plot.setTitle("Position", **title_style)
        self._pos_plot.showGrid(x=True, y=True, alpha=0.3)
        self._pos_plot.getAxis("left").setPen(pg.mkPen(color=THEME.overlay0))
        self._pos_plot.getAxis("left").setTextPen(pg.mkPen(color=THEME.text))
        self._pos_plot.getAxis("bottom").setPen(pg.mkPen(color=THEME.overlay0))
        self._pos_plot.addLegend(
            offset=(10, 10), brush=THEME.surface0, pen=THEME.surface1
        )

        # Position curves
        self._position_curve = self._pos_plot.plot(
            pen=pg.mkPen(color=colors["position"], width=2), name="Pos"
        )
        self._target_curve = self._pos_plot.plot(
            pen=pg.mkPen(
                color=colors["target"], width=2, style=pg.QtCore.Qt.PenStyle.DashLine
            ),
            name="Target",
        )
        self._linked_position_curve = self._pos_plot.plot(
            pen=pg.mkPen(color=THEME.yellow, width=2), name="Linked Pos"
        )
        self._linked_target_curve = self._pos_plot.plot(
            pen=pg.mkPen(
                color=THEME.peach, width=2, style=pg.QtCore.Qt.PenStyle.DashLine
            ),
            name="Linked Target",
        )

        # Velocity plot (middle)
        self._vel_plot = self._graphics_layout.addPlot(row=1, col=0)
        self._vel_plot.setLabel("left", "Velocity", units="cm/s", **label_style)
        self._vel_plot.setTitle("Velocity", **title_style)
        self._vel_plot.showGrid(x=True, y=True, alpha=0.3)
        self._vel_plot.getAxis("left").setPen(pg.mkPen(color=THEME.overlay0))
        self._vel_plot.getAxis("left").setTextPen(pg.mkPen(color=THEME.text))
        self._vel_plot.getAxis("bottom").setPen(pg.mkPen(color=THEME.overlay0))
        self._vel_plot.addLegend(
            offset=(10, 10), brush=THEME.surface0, pen=THEME.surface1
        )

        self._velocity_curve = self._vel_plot.plot(
            pen=pg.mkPen(color=colors["velocity"], width=2), name="Vel"
        )
        self._vel_target_curve = self._vel_plot.plot(
            pen=pg.mkPen(
                color=colors["target"], width=2, style=pg.QtCore.Qt.PenStyle.DashLine
            ),
            name="Target",
        )
        self._linked_velocity_curve = self._vel_plot.plot(
            pen=pg.mkPen(color=THEME.yellow, width=2), name="Linked Vel"
        )
        self._linked_vel_target_curve = self._vel_plot.plot(
            pen=pg.mkPen(
                color=THEME.peach, width=2, style=pg.QtCore.Qt.PenStyle.DashLine
            ),
            name="Linked Target",
        )

        # PWM plot (bottom)
        self._pwm_plot = self._graphics_layout.addPlot(row=2, col=0)
        self._pwm_plot.setLabel("left", "PWM", **label_style)
        self._pwm_plot.setLabel("bottom", "Time", units="s", **label_style)
        self._pwm_plot.setTitle("PWM Output", **title_style)
        self._pwm_plot.showGrid(x=True, y=True, alpha=0.3)
        self._pwm_plot.getAxis("left").setPen(pg.mkPen(color=THEME.overlay0))
        self._pwm_plot.getAxis("left").setTextPen(pg.mkPen(color=THEME.text))
        self._pwm_plot.getAxis("bottom").setPen(pg.mkPen(color=THEME.overlay0))
        self._pwm_plot.getAxis("bottom").setTextPen(pg.mkPen(color=THEME.text))
        self._pwm_plot.addLegend(
            offset=(10, 10), brush=THEME.surface0, pen=THEME.surface1
        )

        self._pwm_curve = self._pwm_plot.plot(
            pen=pg.mkPen(color=colors["pwm"], width=2), name="PWM"
        )
        self._pwm_target_curve = self._pwm_plot.plot(
            pen=pg.mkPen(
                color=colors["target"], width=2, style=pg.QtCore.Qt.PenStyle.DashLine
            ),
            name="Target",
        )
        self._linked_pwm_curve = self._pwm_plot.plot(
            pen=pg.mkPen(color=THEME.yellow, width=2), name="Linked PWM"
        )
        self._linked_pwm_target_curve = self._pwm_plot.plot(
            pen=pg.mkPen(
                color=THEME.peach, width=2, style=pg.QtCore.Qt.PenStyle.DashLine
            ),
            name="Linked Target",
        )

        # IMU plot (bottom, hidden by default)
        self._imu_plot = self._graphics_layout.addPlot(row=3, col=0)
        self._imu_plot.setLabel("left", "Angle", units="deg", **label_style)
        self._imu_plot.setLabel("bottom", "Time", units="s", **label_style)
        self._imu_plot.setTitle("IMU Orientation", **title_style)
        self._imu_plot.showGrid(x=True, y=True, alpha=0.3)
        self._imu_plot.getAxis("left").setPen(pg.mkPen(color=THEME.overlay0))
        self._imu_plot.getAxis("left").setTextPen(pg.mkPen(color=THEME.text))
        self._imu_plot.getAxis("bottom").setPen(pg.mkPen(color=THEME.overlay0))
        self._imu_plot.getAxis("bottom").setTextPen(pg.mkPen(color=THEME.text))
        self._imu_plot.addLegend(
            offset=(10, 10), brush=THEME.surface0, pen=THEME.surface1
        )

        # IMU curves (pitch=red, roll=green, yaw=blue)
        self._pitch_curve = self._imu_plot.plot(
            pen=pg.mkPen(color=THEME.red, width=2), name="Pitch"
        )
        self._pitch_target_curve = self._imu_plot.plot(
            pen=pg.mkPen(
                color=THEME.red, width=2, style=pg.QtCore.Qt.PenStyle.DashLine
            ),
            name="T.Pitch",
        )
        self._roll_curve = self._imu_plot.plot(
            pen=pg.mkPen(color=THEME.green, width=2), name="Roll"
        )
        self._roll_target_curve = self._imu_plot.plot(
            pen=pg.mkPen(
                color=THEME.green, width=2, style=pg.QtCore.Qt.PenStyle.DashLine
            ),
            name="T.Roll",
        )
        self._yaw_curve = self._imu_plot.plot(
            pen=pg.mkPen(color=THEME.blue, width=2), name="Yaw"
        )

        # Link X axes so they scroll together
        self._vel_plot.setXLink(self._pos_plot)
        self._pwm_plot.setXLink(self._pos_plot)
        self._imu_plot.setXLink(self._pos_plot)

        # Hide X labels for top plots (shared axis)
        self._pos_plot.hideAxis("bottom")
        self._vel_plot.hideAxis("bottom")

        # Hide IMU plot by default
        self._imu_plot.setVisible(False)

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

        control_mode = self._data_store.control_mode
        linked_joint_id = self._data_store.linked_joint
        has_linked = False

        linked_data = None
        linked_window_times = np.array([])
        linked_pos = np.array([])
        linked_targets = np.array([])
        linked_mask = np.array([], dtype=bool)

        if linked_joint_id != 0:
            linked_data = self._data_store.get_joint(linked_joint_id)
            if linked_data is not None:
                linked_times, l_pos, l_targets = linked_data.get_plot_data()
                if len(linked_times) > 0:
                    has_linked = True
                    linked_mask = linked_times >= min_time
                    linked_window_times = linked_times[linked_mask]
                    linked_pos = l_pos
                    linked_targets = l_targets

        # Clear target curves so they don't stick around in wrong plots
        self._target_curve.setData([], [])
        self._vel_target_curve.setData([], [])
        self._pwm_target_curve.setData([], [])
        self._linked_target_curve.setData([], [])
        self._linked_vel_target_curve.setData([], [])
        self._linked_pwm_target_curve.setData([], [])
        self._pitch_target_curve.setData([], [])
        self._roll_target_curve.setData([], [])

        # Update position plot
        if self._show_position:
            window_positions = positions[mask]
            self._position_curve.setData(window_times, window_positions)
            if control_mode == 2:  # POSITION
                self._target_curve.setData(window_times, targets[mask])

            if has_linked:
                self._linked_position_curve.setData(
                    linked_window_times, linked_pos[linked_mask]
                )
                if control_mode == 2:
                    self._linked_target_curve.setData(
                        linked_window_times, linked_targets[linked_mask]
                    )
            else:
                self._linked_position_curve.setData([], [])

        # Update velocity plot
        if self._show_velocity:
            vel_timestamps, velocities = joint_data.get_velocity_data()
            if len(vel_timestamps) > 0:
                vel_mask = vel_timestamps >= min_time
                self._velocity_curve.setData(
                    vel_timestamps[vel_mask], velocities[vel_mask]
                )
                if control_mode == 1:  # VELOCITY
                    # Targets array has the target velocity if in velocity mode
                    self._vel_target_curve.setData(window_times, targets[mask])

            if has_linked and linked_data is not None:
                l_vel_times, l_vels = linked_data.get_velocity_data()
                if len(l_vel_times) > 0:
                    l_v_mask = l_vel_times >= min_time
                    self._linked_velocity_curve.setData(
                        l_vel_times[l_v_mask], l_vels[l_v_mask]
                    )
                    if control_mode == 1:
                        self._linked_vel_target_curve.setData(
                            linked_window_times, linked_targets[linked_mask]
                        )
            else:
                self._linked_velocity_curve.setData([], [])

        # Update PWM plot
        if self._show_pwm:
            pwm_timestamps, pwms = joint_data.get_pwm_data()
            if len(pwm_timestamps) > 0:
                pwm_mask = pwm_timestamps >= min_time
                self._pwm_curve.setData(pwm_timestamps[pwm_mask], pwms[pwm_mask])
                if control_mode == 0:  # OPEN LOOP (PWM)
                    self._pwm_target_curve.setData(window_times, targets[mask])

            if has_linked and linked_data is not None:
                l_pwm_times, l_pwms = linked_data.get_pwm_data()
                if len(l_pwm_times) > 0:
                    l_p_mask = l_pwm_times >= min_time
                    self._linked_pwm_curve.setData(
                        l_pwm_times[l_p_mask], l_pwms[l_p_mask]
                    )
                    if control_mode == 0:
                        self._linked_pwm_target_curve.setData(
                            linked_window_times, linked_targets[linked_mask]
                        )
            else:
                self._linked_pwm_curve.setData([], [])

        # Update IMU plot
        if self._show_imu:
            imu_data = self._data_store.imu_data
            imu_timestamps, pitch, roll, yaw = imu_data.get_orientation_data()
            if len(imu_timestamps) > 0:
                imu_mask = imu_timestamps >= min_time
                self._pitch_curve.setData(imu_timestamps[imu_mask], pitch[imu_mask])
                self._roll_curve.setData(imu_timestamps[imu_mask], roll[imu_mask])
                self._yaw_curve.setData(imu_timestamps[imu_mask], yaw[imu_mask])

                # Plot IMU targets if in self leveling mode (state == 4)
                if self._data_store.current_state == 4:
                    target_pitch = np.full(
                        len(imu_timestamps[imu_mask]), self._data_store.imu_target_pitch
                    )
                    target_roll = np.full(
                        len(imu_timestamps[imu_mask]), self._data_store.imu_target_roll
                    )
                    self._pitch_target_curve.setData(
                        imu_timestamps[imu_mask], target_pitch
                    )
                    self._roll_target_curve.setData(
                        imu_timestamps[imu_mask], target_roll
                    )
                else:
                    self._pitch_target_curve.setData([], [])
                    self._roll_target_curve.setData([], [])

        # Update x-axis range to show rolling window (on lowest visible plot)
        if self._show_imu:
            self._imu_plot.setXRange(min_time, current_time, padding=0.02)
        else:
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

    def _on_imu_toggled(self, checked: bool):
        """Handle IMU plot visibility toggle."""
        self._show_imu = checked
        self._imu_plot.setVisible(checked)
        self._update_plot_layout()

    def _update_plot_layout(self):
        """Update plot layout when visibility changes."""
        # Show bottom axis on the lowest visible plot
        self._pos_plot.hideAxis("bottom")
        self._vel_plot.hideAxis("bottom")
        self._pwm_plot.hideAxis("bottom")
        self._imu_plot.hideAxis("bottom")

        if self._show_imu:
            self._imu_plot.showAxis("bottom")
        elif self._show_pwm:
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
        self._pitch_curve.setData([], [])
        self._roll_curve.setData([], [])
        self._yaw_curve.setData([], [])

    def _on_autoscale_clicked(self):
        """Handle auto-scale button click."""
        self._pos_plot.enableAutoRange()
        self._vel_plot.enableAutoRange()
        self._pwm_plot.enableAutoRange()
        self._imu_plot.enableAutoRange()

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

    def _on_export_csv(self):
        """Export visible graph data to CSV file."""
        from PyQt6.QtWidgets import QFileDialog, QMessageBox
        import csv
        from datetime import datetime

        # Get save file path
        default_name = (
            f"pid_tuner_export_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        file_path, _ = QFileDialog.getSaveFileName(
            self, "Export Graph Data", default_name, "CSV Files (*.csv);;All Files (*)"
        )

        if not file_path:
            return

        try:
            # Get data from data store
            joint_data = self._data_store.get_selected_joint_data()
            timestamps, positions, targets = joint_data.get_plot_data()
            vel_timestamps, velocities = joint_data.get_velocity_data()
            pwm_timestamps, pwms = joint_data.get_pwm_data()

            if len(timestamps) == 0:
                QMessageBox.warning(self, "Export Error", "No data to export")
                return

            # Build headers and collect data columns
            headers = ["timestamp"]
            rows_data = {
                i: {"timestamp": timestamps[i]} for i in range(len(timestamps))
            }

            if self._show_position:
                headers.extend(["position", "target"])
                for i in range(len(timestamps)):
                    rows_data[i]["position"] = (
                        positions[i] if i < len(positions) else ""
                    )
                    rows_data[i]["target"] = targets[i] if i < len(targets) else ""

            if self._show_velocity:
                headers.append("velocity")
                for i in range(len(timestamps)):
                    # Find matching velocity timestamp
                    vel_val = ""
                    if i < len(velocities):
                        vel_val = velocities[i]
                    rows_data[i]["velocity"] = vel_val

            if self._show_pwm:
                headers.append("pwm")
                for i in range(len(timestamps)):
                    pwm_val = ""
                    if i < len(pwms):
                        pwm_val = pwms[i]
                    rows_data[i]["pwm"] = pwm_val

            # Include linked joint if present
            linked_id = self._data_store.linked_joint
            if linked_id:
                linked_data = self._data_store.get_joint(linked_id)
                if linked_data:
                    l_timestamps, l_positions, l_targets = linked_data.get_plot_data()
                    l_vel_timestamps, l_velocities = linked_data.get_velocity_data()
                    l_pwm_timestamps, l_pwms = linked_data.get_pwm_data()

                    if self._show_position:
                        headers.extend(["linked_position", "linked_target"])
                        for i in range(len(timestamps)):
                            rows_data[i]["linked_position"] = (
                                l_positions[i] if i < len(l_positions) else ""
                            )
                            rows_data[i]["linked_target"] = (
                                l_targets[i] if i < len(l_targets) else ""
                            )

                    if self._show_velocity:
                        headers.append("linked_velocity")
                        for i in range(len(timestamps)):
                            rows_data[i]["linked_velocity"] = (
                                l_velocities[i] if i < len(l_velocities) else ""
                            )

                    if self._show_pwm:
                        headers.append("linked_pwm")
                        for i in range(len(timestamps)):
                            rows_data[i]["linked_pwm"] = (
                                l_pwms[i] if i < len(l_pwms) else ""
                            )

            # Write CSV
            with open(file_path, "w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=headers)
                writer.writeheader()
                for i in range(len(timestamps)):
                    # Filter row to only include headers we have
                    row = {k: rows_data[i].get(k, "") for k in headers}
                    writer.writerow(row)

            QMessageBox.information(
                self,
                "Export Complete",
                f"Data exported to:\n{file_path}\n\n{len(timestamps)} samples exported.",
            )

        except Exception as e:
            QMessageBox.critical(self, "Export Error", f"Failed to export: {str(e)}")
