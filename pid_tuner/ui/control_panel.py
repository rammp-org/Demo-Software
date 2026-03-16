"""
Control panel for setting targets, step inputs, and sine wave inputs.
"""

from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QGroupBox,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QComboBox,
    QDoubleSpinBox,
)
from PyQt6.QtCore import pyqtSignal, QTimer
from PyQt6.QtGui import QDoubleValidator
import math
import numpy as np

from ..data.data_store import DataStore
from ..serial_driver.serial_handler import SerialHandler
from .theme import THEME


# Control mode constants
MODE_OPEN_LOOP = 0
MODE_VELOCITY = 1
MODE_POSITION = 2

# Unit labels for each mode
MODE_UNITS = {
    MODE_OPEN_LOOP: "PWM",
    MODE_VELOCITY: "units/s",
    MODE_POSITION: "ticks",
}

MODE_NAMES = {
    MODE_OPEN_LOOP: "Open Loop",
    MODE_VELOCITY: "Velocity",
    MODE_POSITION: "Position",
}


class ControlPanel(QWidget):
    """
    Control panel for PID tuning inputs.

    Features:
    - Set absolute target position
    - Step inputs (positive/negative)
    - Sine wave inputs with configurable amplitude, frequency, duration
    - Oscillation analysis metrics
    """

    # Default step sizes
    DEFAULT_STEP_SIZES = [10, 50, 100, 500, 1000]

    def __init__(
        self, data_store: DataStore, serial_handler: SerialHandler, parent=None
    ):
        super().__init__(parent)
        self._data_store = data_store
        self._serial_handler = serial_handler

        # Current control mode (tracked locally)
        self._current_mode = MODE_OPEN_LOOP

        # Sine wave state
        self._sine_active = False
        self._sine_timer = QTimer(self)
        self._sine_timer.timeout.connect(self._update_sine_wave)
        self._sine_start_time = 0
        self._sine_amplitude = 0
        self._sine_frequency = 0
        self._sine_duration = 0
        self._sine_center = 0

        self._setup_ui()

    def _setup_ui(self):
        """Set up the control panel layout."""
        layout = QVBoxLayout(self)

        # Current values display
        layout.addWidget(self._create_status_group())

        # Oscillation analysis
        layout.addWidget(self._create_analysis_group())

        # PID Control
        layout.addWidget(self._create_pid_group())

        # Target control
        layout.addWidget(self._create_target_group())

        # Step inputs
        layout.addWidget(self._create_step_group())

        # Sine wave inputs
        layout.addWidget(self._create_sine_group())

        layout.addStretch()

    def _create_status_group(self) -> QGroupBox:
        """Create the current values display group."""
        group = QGroupBox("Current Values")
        layout = QGridLayout(group)

        # Mode indicator
        layout.addWidget(QLabel("Mode:"), 0, 0)
        self._mode_indicator_label = QLabel("Open Loop")
        self._mode_indicator_label.setStyleSheet(
            "font-weight: bold; font-size: 14px; color: cyan;"
        )
        layout.addWidget(self._mode_indicator_label, 0, 1)
        self._mode_unit_label = QLabel("")  # Placeholder, units shown elsewhere
        layout.addWidget(self._mode_unit_label, 0, 2)

        # Position
        layout.addWidget(QLabel("Position:"), 1, 0)
        self._position_label = QLabel("---")
        self._position_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(self._position_label, 1, 1)
        self._position_unit_label = QLabel("ticks")
        layout.addWidget(self._position_unit_label, 1, 2)

        # Velocity
        layout.addWidget(QLabel("Velocity:"), 2, 0)
        self._velocity_label = QLabel("---")
        self._velocity_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(self._velocity_label, 2, 1)
        layout.addWidget(QLabel("units/s"), 2, 2)

        # PWM
        layout.addWidget(QLabel("PWM:"), 3, 0)
        self._pwm_label = QLabel("---")
        self._pwm_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(self._pwm_label, 3, 1)
        self._pwm_percent_label = QLabel("(0%)")
        layout.addWidget(self._pwm_percent_label, 3, 2)

        # Target (label changes based on mode)
        self._target_row_label = QLabel("Target:")
        layout.addWidget(self._target_row_label, 4, 0)
        self._target_label = QLabel("---")
        self._target_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(self._target_label, 4, 1)
        self._target_unit_label = QLabel("PWM")
        layout.addWidget(self._target_unit_label, 4, 2)

        # Error
        layout.addWidget(QLabel("Error:"), 5, 0)
        self._error_label = QLabel("---")
        self._error_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(self._error_label, 5, 1)
        self._error_unit_label = QLabel("")
        layout.addWidget(self._error_unit_label, 5, 2)

        # Update timer
        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._update_status)
        self._status_timer.start(100)  # Update at 10 Hz

        return group

    def _create_analysis_group(self) -> QGroupBox:
        """Create oscillation analysis metrics group."""
        group = QGroupBox("Performance Analysis")
        layout = QGridLayout(group)

        # RMS Error
        layout.addWidget(QLabel("RMS Error:"), 0, 0)
        self._rms_error_label = QLabel("---")
        self._rms_error_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(self._rms_error_label, 0, 1)

        # Peak-to-Peak oscillation
        layout.addWidget(QLabel("Peak-Peak:"), 0, 2)
        self._peak_peak_label = QLabel("---")
        self._peak_peak_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(self._peak_peak_label, 0, 3)

        # Settling indicator (based on recent error variance)
        layout.addWidget(QLabel("Settled:"), 1, 0)
        self._settled_label = QLabel("---")
        self._settled_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(self._settled_label, 1, 1)

        # Zero crossings (oscillation frequency indicator)
        layout.addWidget(QLabel("Oscillations:"), 1, 2)
        self._oscillation_label = QLabel("---")
        self._oscillation_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(self._oscillation_label, 1, 3)

        # Analysis window info
        self._analysis_window_label = QLabel("Window: 2s")
        self._analysis_window_label.setStyleSheet("color: gray; font-size: 10px;")
        layout.addWidget(self._analysis_window_label, 2, 0, 1, 2)

        # Reset analysis button
        self._reset_analysis_btn = QPushButton("Reset")
        self._reset_analysis_btn.clicked.connect(self._on_reset_analysis)
        layout.addWidget(self._reset_analysis_btn, 2, 2, 1, 2)

        return group

    def _create_pid_group(self) -> QGroupBox:
        group = QGroupBox("Mode & PID Control")
        layout = QVBoxLayout(group)

        # Mode Selection row
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Mode:"))
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(["Open Loop (0)", "Velocity (1)", "Position (2)"])
        self._mode_combo.currentIndexChanged.connect(self._on_mode_combo_changed)
        mode_layout.addWidget(self._mode_combo)

        self._set_mode_btn = QPushButton("Set Mode")
        self._set_mode_btn.clicked.connect(self._on_set_mode)
        mode_layout.addWidget(self._set_mode_btn)

        # Clear PID button (resets integrator windup)
        self._clear_pid_btn = QPushButton("Clear PID")
        self._clear_pid_btn.setToolTip("Reset integrator windup and previous error")
        self._clear_pid_btn.clicked.connect(self._on_clear_pid)
        self._clear_pid_btn.setStyleSheet(
            f"background-color: {THEME.yellow}; color: {THEME.crust};"
        )
        mode_layout.addWidget(self._clear_pid_btn)

        layout.addLayout(mode_layout)

        # Position PID row (P, I, D, FF)
        pos_pid_layout = QHBoxLayout()
        pos_pid_layout.addWidget(QLabel("Pos:"))

        # Use QLineEdit for unlimited input
        self._pos_p = QLineEdit("0")
        self._pos_p.setValidator(QDoubleValidator())
        self._pos_p.setMaximumWidth(70)
        self._pos_i = QLineEdit("0")
        self._pos_i.setValidator(QDoubleValidator())
        self._pos_i.setMaximumWidth(70)
        self._pos_d = QLineEdit("0")
        self._pos_d.setValidator(QDoubleValidator())
        self._pos_d.setMaximumWidth(70)
        self._pos_ff = QLineEdit("0")
        self._pos_ff.setValidator(QDoubleValidator())
        self._pos_ff.setMaximumWidth(70)

        pos_pid_layout.addWidget(QLabel("P:"))
        pos_pid_layout.addWidget(self._pos_p)
        pos_pid_layout.addWidget(QLabel("I:"))
        pos_pid_layout.addWidget(self._pos_i)
        pos_pid_layout.addWidget(QLabel("D:"))
        pos_pid_layout.addWidget(self._pos_d)
        pos_pid_layout.addWidget(QLabel("FF:"))
        pos_pid_layout.addWidget(self._pos_ff)

        self._set_pos_pid_btn = QPushButton("Set")
        self._set_pos_pid_btn.clicked.connect(self._on_set_pos_pid)
        pos_pid_layout.addWidget(self._set_pos_pid_btn)
        layout.addLayout(pos_pid_layout)

        # Velocity PID row (P, I, D, FF)
        vel_pid_layout = QHBoxLayout()
        vel_pid_layout.addWidget(QLabel("Vel:"))

        self._vel_p = QLineEdit("0")
        self._vel_p.setValidator(QDoubleValidator())
        self._vel_p.setMaximumWidth(70)
        self._vel_i = QLineEdit("0")
        self._vel_i.setValidator(QDoubleValidator())
        self._vel_i.setMaximumWidth(70)
        self._vel_d = QLineEdit("0")
        self._vel_d.setValidator(QDoubleValidator())
        self._vel_d.setMaximumWidth(70)
        self._vel_ff = QLineEdit("0")
        self._vel_ff.setValidator(QDoubleValidator())
        self._vel_ff.setMaximumWidth(70)

        vel_pid_layout.addWidget(QLabel("P:"))
        vel_pid_layout.addWidget(self._vel_p)
        vel_pid_layout.addWidget(QLabel("I:"))
        vel_pid_layout.addWidget(self._vel_i)
        vel_pid_layout.addWidget(QLabel("D:"))
        vel_pid_layout.addWidget(self._vel_d)
        vel_pid_layout.addWidget(QLabel("FF:"))
        vel_pid_layout.addWidget(self._vel_ff)

        self._set_vel_pid_btn = QPushButton("Set")
        self._set_vel_pid_btn.clicked.connect(self._on_set_vel_pid)
        vel_pid_layout.addWidget(self._set_vel_pid_btn)
        layout.addLayout(vel_pid_layout)

        return group

    def _create_target_group(self) -> QGroupBox:
        """Create the target control group."""
        group = QGroupBox("Target Control")
        layout = QVBoxLayout(group)

        # Target input row - use QLineEdit for unlimited range
        input_layout = QHBoxLayout()
        input_layout.addWidget(QLabel("Target:"))

        self._target_input = QLineEdit("0")
        self._target_input.setValidator(QDoubleValidator())
        self._target_input.setMaximumWidth(120)
        input_layout.addWidget(self._target_input)

        # Dynamic unit label (changes with mode)
        self._target_input_unit_label = QLabel("PWM")
        input_layout.addWidget(self._target_input_unit_label)

        self._set_target_btn = QPushButton("Set Target")
        self._set_target_btn.clicked.connect(self._on_set_target)
        input_layout.addWidget(self._set_target_btn)

        layout.addLayout(input_layout)

        # Quick target buttons row
        quick_layout = QHBoxLayout()

        # Set to current button
        self._set_current_btn = QPushButton("Use Current")
        self._set_current_btn.clicked.connect(self._on_use_current)
        quick_layout.addWidget(self._set_current_btn)

        # Set Zero button
        self._set_zero_btn = QPushButton("Set Zero")
        self._set_zero_btn.clicked.connect(self._on_set_zero)
        self._set_zero_btn.setStyleSheet(
            f"background-color: {THEME.green}; color: {THEME.crust};"
        )
        quick_layout.addWidget(self._set_zero_btn)

        # Disable Motors button
        self._disable_motors_btn = QPushButton("ESTOP (z)")
        self._disable_motors_btn.clicked.connect(self._on_disable_motors)
        self._disable_motors_btn.setStyleSheet(
            f"background-color: {THEME.red}; color: {THEME.crust};"
        )
        quick_layout.addWidget(self._disable_motors_btn)

        # Clear ESTOP button
        self._clear_estop_btn = QPushButton("Clear ESTOP (c)")
        self._clear_estop_btn.clicked.connect(self._on_clear_estop)
        self._clear_estop_btn.setStyleSheet(
            f"background-color: {THEME.peach}; color: {THEME.crust};"
        )
        quick_layout.addWidget(self._clear_estop_btn)

        layout.addLayout(quick_layout)

        return group

    def _create_step_group(self) -> QGroupBox:
        """Create the step input group."""
        group = QGroupBox("Step Input")
        layout = QVBoxLayout(group)

        # Step size input - use QLineEdit for unlimited range
        step_size_layout = QHBoxLayout()
        step_size_layout.addWidget(QLabel("Step Size:"))

        self._step_size_input = QLineEdit("100")
        self._step_size_input.setValidator(QDoubleValidator())
        self._step_size_input.setMaximumWidth(100)
        step_size_layout.addWidget(self._step_size_input)

        # Dynamic unit label
        self._step_unit_label = QLabel("PWM")
        step_size_layout.addWidget(self._step_unit_label)
        step_size_layout.addStretch()
        layout.addLayout(step_size_layout)

        # Step buttons
        btn_layout = QHBoxLayout()

        self._step_neg_btn = QPushButton("Step -")
        self._step_neg_btn.clicked.connect(self._on_step_negative)
        btn_layout.addWidget(self._step_neg_btn)

        self._step_pos_btn = QPushButton("Step +")
        self._step_pos_btn.clicked.connect(self._on_step_positive)
        btn_layout.addWidget(self._step_pos_btn)

        layout.addLayout(btn_layout)

        # Quick step buttons
        quick_layout = QHBoxLayout()
        quick_layout.addWidget(QLabel("Quick:"))
        for size in self.DEFAULT_STEP_SIZES:
            btn = QPushButton(f"+{size}")
            btn.clicked.connect(lambda checked, s=size: self._on_quick_step(s))
            quick_layout.addWidget(btn)
        layout.addLayout(quick_layout)

        return group

    def _create_sine_group(self) -> QGroupBox:
        """Create the sine wave input group."""
        group = QGroupBox("Sine Wave Input")
        layout = QVBoxLayout(group)

        # Parameters grid - use QLineEdit for unlimited range
        params_layout = QGridLayout()

        # Amplitude
        params_layout.addWidget(QLabel("Amplitude:"), 0, 0)
        self._sine_amplitude_input = QLineEdit("500")
        self._sine_amplitude_input.setValidator(QDoubleValidator())
        self._sine_amplitude_input.setMaximumWidth(100)
        params_layout.addWidget(self._sine_amplitude_input, 0, 1)
        self._sine_amplitude_unit_label = QLabel("PWM")
        params_layout.addWidget(self._sine_amplitude_unit_label, 0, 2)

        # Frequency
        params_layout.addWidget(QLabel("Frequency:"), 1, 0)
        self._sine_frequency_input = QLineEdit("0.5")
        self._sine_frequency_input.setValidator(QDoubleValidator())
        self._sine_frequency_input.setMaximumWidth(100)
        params_layout.addWidget(self._sine_frequency_input, 1, 1)
        params_layout.addWidget(QLabel("Hz"), 1, 2)

        # Duration
        params_layout.addWidget(QLabel("Duration:"), 2, 0)
        self._sine_duration_input = QLineEdit("10")
        self._sine_duration_input.setValidator(QDoubleValidator())
        self._sine_duration_input.setMaximumWidth(100)
        params_layout.addWidget(self._sine_duration_input, 2, 1)
        params_layout.addWidget(QLabel("seconds"), 2, 2)

        layout.addLayout(params_layout)

        # Control buttons
        btn_layout = QHBoxLayout()

        self._start_sine_btn = QPushButton("Start Sine")
        self._start_sine_btn.clicked.connect(self._on_start_sine)
        btn_layout.addWidget(self._start_sine_btn)

        self._stop_sine_btn = QPushButton("Stop Sine")
        self._stop_sine_btn.clicked.connect(self._on_stop_sine)
        self._stop_sine_btn.setEnabled(False)
        btn_layout.addWidget(self._stop_sine_btn)

        layout.addLayout(btn_layout)

        # Status
        self._sine_status_label = QLabel("Sine wave: Inactive")
        layout.addWidget(self._sine_status_label)

        return group

    def _update_status(self):
        """Update the status display with current values."""
        joint_data = self._data_store.get_selected_joint_data()

        position = joint_data.current_position
        velocity = joint_data.current_velocity
        pwm = joint_data.current_pwm
        target = joint_data.current_target

        # Calculate error based on mode
        if self._current_mode == MODE_POSITION:
            error = position - target
        elif self._current_mode == MODE_VELOCITY:
            error = velocity - target
        else:  # Open loop - no meaningful error
            error = 0

        # Update labels
        self._position_label.setText(f"{position:.2f}")
        self._velocity_label.setText(f"{velocity:.2f}")
        self._pwm_label.setText(f"{pwm:.2f}")
        self._pwm_percent_label.setText(f"({abs(pwm) / 32767 * 100:.1f}%)")
        self._target_label.setText(f"{target:.2f}")
        self._error_label.setText(f"{error:.2f}")

        # Color the error label based on magnitude
        if abs(error) < 10:
            self._error_label.setStyleSheet(
                "font-weight: bold; font-size: 14px; color: green;"
            )
        elif abs(error) < 100:
            self._error_label.setStyleSheet(
                "font-weight: bold; font-size: 14px; color: orange;"
            )
        else:
            self._error_label.setStyleSheet(
                "font-weight: bold; font-size: 14px; color: red;"
            )

        # Update oscillation analysis
        self._update_analysis(joint_data)

    def _update_analysis(self, joint_data):
        """Update oscillation analysis metrics."""
        # Get recent data (last 2 seconds at ~10Hz = ~20 samples)
        positions = np.array(joint_data.positions)
        targets = np.array(joint_data.targets)
        timestamps = np.array(joint_data.timestamps)

        if len(positions) < 10:
            return  # Not enough data

        # Use last 2 seconds of data
        if len(timestamps) > 0:
            recent_mask = timestamps >= (timestamps[-1] - 2.0)
            recent_positions = positions[recent_mask]
            recent_targets = targets[recent_mask]
        else:
            return

        if len(recent_positions) < 5:
            return

        # Calculate error
        recent_errors = recent_positions - recent_targets

        # RMS Error
        rms_error = np.sqrt(np.mean(recent_errors**2))
        self._rms_error_label.setText(f"{rms_error:.2f}")

        # Peak-to-Peak
        peak_peak = np.max(recent_errors) - np.min(recent_errors)
        self._peak_peak_label.setText(f"{peak_peak:.2f}")

        # Settled check (variance-based)
        variance = np.var(recent_errors)
        if variance < 100:  # Threshold for "settled"
            self._settled_label.setText("Yes")
            self._settled_label.setStyleSheet("font-weight: bold; color: green;")
        else:
            self._settled_label.setText("No")
            self._settled_label.setStyleSheet("font-weight: bold; color: orange;")

        # Zero crossings (oscillation detection)
        if len(recent_errors) > 2:
            zero_crossings = np.sum(np.diff(np.sign(recent_errors)) != 0)
            # Estimate frequency: crossings / 2 / time_window
            time_window = (
                timestamps[recent_mask][-1] - timestamps[recent_mask][0]
                if len(timestamps[recent_mask]) > 1
                else 1
            )
            osc_freq = zero_crossings / 2 / max(time_window, 0.1)
            self._oscillation_label.setText(f"{zero_crossings} ({osc_freq:.1f}Hz)")
        else:
            self._oscillation_label.setText("---")

    def _on_reset_analysis(self):
        """Reset analysis by clearing joint data."""
        joint_id = self._data_store.selected_joint
        self._data_store.clear_joint(joint_id)
        self._rms_error_label.setText("---")
        self._peak_peak_label.setText("---")
        self._settled_label.setText("---")
        self._oscillation_label.setText("---")

    def _on_mode_combo_changed(self, index: int):
        """Handle mode combo box change - update unit labels."""
        self._current_mode = index
        unit = MODE_UNITS.get(index, "")

        # Update all unit labels based on mode
        self._target_unit_label.setText(unit)
        self._target_input_unit_label.setText(unit)
        self._step_unit_label.setText(unit)
        self._sine_amplitude_unit_label.setText(unit)
        self._mode_indicator_label.setText(MODE_NAMES.get(index, "Unknown"))

        # Update error unit based on mode
        if index == MODE_POSITION:
            self._error_unit_label.setText("ticks")
        elif index == MODE_VELOCITY:
            self._error_unit_label.setText("units/s")
        else:
            self._error_unit_label.setText("")

    def _on_clear_pid(self):
        """Send reset PID command to clear integrator windup."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.reset_pid(joint_id)

    def _get_float_from_lineedit(
        self, line_edit: QLineEdit, default: float = 0.0
    ) -> float:
        """Safely get float value from QLineEdit."""
        try:
            return float(line_edit.text())
        except ValueError:
            return default

    def _on_set_target(self):
        """Handle set target button click."""
        target = self._get_float_from_lineedit(self._target_input)
        joint_id = self._data_store.selected_joint

        # Update data store
        self._data_store.set_target(joint_id, target)

        # Send to Teensy
        self._serial_handler.set_target(joint_id, target)

    def _on_use_current(self):
        """Set target input to current position/velocity based on mode."""
        joint_data = self._data_store.get_selected_joint_data()
        if self._current_mode == MODE_VELOCITY:
            self._target_input.setText(str(joint_data.current_velocity))
        else:
            self._target_input.setText(str(joint_data.current_position))

    def _on_set_zero(self):
        """Set target to zero."""
        joint_id = self._data_store.selected_joint

        # Update data store
        self._data_store.set_target(joint_id, 0)

        # Update input field
        self._target_input.setText("0")

        # Send to Teensy
        self._serial_handler.set_target(joint_id, 0)

    def _on_disable_motors(self):
        """Send disable motors command."""
        self._serial_handler.disable_motors()

    def _on_clear_estop(self):
        """Send clear ESTOP command."""
        self._serial_handler.clear_estop()

    def _on_set_mode(self):
        """Send mode set command."""
        joint_id = self._data_store.selected_joint
        mode = self._mode_combo.currentIndex()
        self._serial_handler.set_mode(joint_id, mode)

    def _on_set_pos_pid(self):
        """Send position PID gains and feed-forward."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.set_pid(
            joint_id, "P", self._get_float_from_lineedit(self._pos_p)
        )
        self._serial_handler.set_pid(
            joint_id, "I", self._get_float_from_lineedit(self._pos_i)
        )
        self._serial_handler.set_pid(
            joint_id, "D", self._get_float_from_lineedit(self._pos_d)
        )
        self._serial_handler.set_feed_forward(
            joint_id, "F", self._get_float_from_lineedit(self._pos_ff)
        )

    def _on_set_vel_pid(self):
        """Send velocity PID gains and feed-forward."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.set_pid(
            joint_id, "p", self._get_float_from_lineedit(self._vel_p)
        )
        self._serial_handler.set_pid(
            joint_id, "i", self._get_float_from_lineedit(self._vel_i)
        )
        self._serial_handler.set_pid(
            joint_id, "d", self._get_float_from_lineedit(self._vel_d)
        )
        self._serial_handler.set_feed_forward(
            joint_id, "f", self._get_float_from_lineedit(self._vel_ff)
        )

    def _on_step_positive(self):
        """Handle positive step button click."""
        step = self._get_float_from_lineedit(self._step_size_input, 100.0)
        self._apply_step(step)

    def _on_step_negative(self):
        """Handle negative step button click."""
        step = -self._get_float_from_lineedit(self._step_size_input, 100.0)
        self._apply_step(step)

    def _on_quick_step(self, step: float):
        """Handle quick step button click."""
        self._apply_step(step)

    def _apply_step(self, step: float):
        """Apply a step to the current target."""
        joint_id = self._data_store.selected_joint
        joint_data = self._data_store.get_selected_joint_data()

        new_target = joint_data.current_target + step

        # Update data store
        self._data_store.set_target(joint_id, new_target)

        # Update input field
        self._target_input.setText(str(new_target))

        # Send to Teensy
        self._serial_handler.set_target(joint_id, new_target)

    def _on_start_sine(self):
        """Start sine wave input."""
        if self._sine_active:
            return

        joint_data = self._data_store.get_selected_joint_data()

        self._sine_amplitude = self._get_float_from_lineedit(
            self._sine_amplitude_input, 500.0
        )
        self._sine_frequency = self._get_float_from_lineedit(
            self._sine_frequency_input, 0.5
        )
        self._sine_duration = self._get_float_from_lineedit(
            self._sine_duration_input, 10.0
        )
        self._sine_center = joint_data.current_target
        self._sine_start_time = 0
        self._sine_active = True

        # Update UI
        self._start_sine_btn.setEnabled(False)
        self._stop_sine_btn.setEnabled(True)
        self._sine_status_label.setText(
            f"Sine wave: Active (A={self._sine_amplitude}, f={self._sine_frequency}Hz)"
        )

        # Start timer - update at ~50 Hz for smooth sine wave
        self._sine_timer.start(20)

    def _on_stop_sine(self):
        """Stop sine wave input."""
        self._sine_active = False
        self._sine_timer.stop()

        # Update UI
        self._start_sine_btn.setEnabled(True)
        self._stop_sine_btn.setEnabled(False)
        self._sine_status_label.setText("Sine wave: Inactive")

        # Set target back to center
        joint_id = self._data_store.selected_joint
        self._data_store.set_target(joint_id, self._sine_center)
        self._serial_handler.set_target(joint_id, self._sine_center)
        self._target_input.setText(str(self._sine_center))

    def _update_sine_wave(self):
        """Update sine wave target position."""
        if not self._sine_active:
            return

        self._sine_start_time += 0.02  # 20ms per update

        # Check if duration exceeded
        if self._sine_start_time >= self._sine_duration:
            self._on_stop_sine()
            return

        # Calculate sine value
        t = self._sine_start_time
        value = self._sine_amplitude * math.sin(2 * math.pi * self._sine_frequency * t)
        target = self._sine_center + value

        # Update
        joint_id = self._data_store.selected_joint
        self._data_store.set_target(joint_id, target)
        self._serial_handler.set_target(joint_id, target)

        # Update time remaining
        remaining = self._sine_duration - self._sine_start_time
        self._sine_status_label.setText(
            f"Sine wave: Active ({remaining:.1f}s remaining)"
        )
