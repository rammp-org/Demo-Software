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
import math

from ..data.data_store import DataStore
from ..serial_driver.serial_handler import SerialHandler
from .theme import THEME


class ControlPanel(QWidget):
    """
    Control panel for PID tuning inputs.

    Features:
    - Set absolute target position
    - Step inputs (positive/negative)
    - Sine wave inputs with configurable amplitude, frequency, duration
    """

    # Default step sizes
    DEFAULT_STEP_SIZES = [0.5, 1.0, 2.0, 5.0, 10.0]

    def __init__(
        self, data_store: DataStore, serial_handler: SerialHandler, parent=None
    ):
        super().__init__(parent)
        self._data_store = data_store
        self._serial_handler = serial_handler

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

        # Position
        layout.addWidget(QLabel("Position:"), 0, 0)
        self._position_label = QLabel("---")
        self._position_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(self._position_label, 0, 1)
        layout.addWidget(QLabel("cm"), 0, 2)

        # Target
        layout.addWidget(QLabel("Target:"), 1, 0)
        self._target_label = QLabel("---")
        self._target_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(self._target_label, 1, 1)
        layout.addWidget(QLabel("cm"), 1, 2)

        # Error
        layout.addWidget(QLabel("Error:"), 2, 0)
        self._error_label = QLabel("---")
        self._error_label.setStyleSheet("font-weight: bold; font-size: 14px;")
        layout.addWidget(self._error_label, 2, 1)
        layout.addWidget(QLabel("cm"), 2, 2)

        # Update timer
        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._update_status)
        self._status_timer.start(100)  # Update at 10 Hz

        return group

    def _create_pid_group(self) -> QGroupBox:
        group = QGroupBox("Mode & PID Control")
        layout = QVBoxLayout(group)

        # Mode Selection
        mode_layout = QHBoxLayout()
        mode_layout.addWidget(QLabel("Mode:"))
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(["Open Loop (0)", "Velocity (1)", "Position (2)"])
        mode_layout.addWidget(self._mode_combo)

        self._set_mode_btn = QPushButton("Set Mode")
        self._set_mode_btn.clicked.connect(self._on_set_mode)
        mode_layout.addWidget(self._set_mode_btn)
        layout.addLayout(mode_layout)

        # Position PID
        pos_pid_layout = QHBoxLayout()
        pos_pid_layout.addWidget(QLabel("Pos PID:"))
        self._pos_p = QDoubleSpinBox()
        self._pos_p.setDecimals(4)
        self._pos_p.setRange(0, 1000)
        self._pos_i = QDoubleSpinBox()
        self._pos_i.setDecimals(4)
        self._pos_i.setRange(0, 1000)
        self._pos_d = QDoubleSpinBox()
        self._pos_d.setDecimals(4)
        self._pos_d.setRange(0, 1000)

        pos_pid_layout.addWidget(QLabel("P:"))
        pos_pid_layout.addWidget(self._pos_p)
        pos_pid_layout.addWidget(QLabel("I:"))
        pos_pid_layout.addWidget(self._pos_i)
        pos_pid_layout.addWidget(QLabel("D:"))
        pos_pid_layout.addWidget(self._pos_d)

        self._set_pos_pid_btn = QPushButton("Set")
        self._set_pos_pid_btn.clicked.connect(self._on_set_pos_pid)
        pos_pid_layout.addWidget(self._set_pos_pid_btn)
        layout.addLayout(pos_pid_layout)

        # Velocity PID
        vel_pid_layout = QHBoxLayout()
        vel_pid_layout.addWidget(QLabel("Vel PID:"))
        self._vel_p = QDoubleSpinBox()
        self._vel_p.setDecimals(4)
        self._vel_p.setRange(0, 1000)
        self._vel_i = QDoubleSpinBox()
        self._vel_i.setDecimals(4)
        self._vel_i.setRange(0, 1000)
        self._vel_d = QDoubleSpinBox()
        self._vel_d.setDecimals(4)
        self._vel_d.setRange(0, 1000)

        vel_pid_layout.addWidget(QLabel("P:"))
        vel_pid_layout.addWidget(self._vel_p)
        vel_pid_layout.addWidget(QLabel("I:"))
        vel_pid_layout.addWidget(self._vel_i)
        vel_pid_layout.addWidget(QLabel("D:"))
        vel_pid_layout.addWidget(self._vel_d)

        self._set_vel_pid_btn = QPushButton("Set")
        self._set_vel_pid_btn.clicked.connect(self._on_set_vel_pid)
        vel_pid_layout.addWidget(self._set_vel_pid_btn)
        layout.addLayout(vel_pid_layout)

        return group

    def _create_target_group(self) -> QGroupBox:
        """Create the target control group."""
        group = QGroupBox("Target Control")
        layout = QVBoxLayout(group)

        # Target input row
        input_layout = QHBoxLayout()
        input_layout.addWidget(QLabel("Target:"))

        self._target_input = QDoubleSpinBox()
        self._target_input.setRange(-1000.0, 1000.0)
        self._target_input.setSingleStep(0.5)
        self._target_input.setDecimals(2)
        self._target_input.setValue(0.0)
        input_layout.addWidget(self._target_input)

        input_layout.addWidget(QLabel("cm"))

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

        # Step size input
        step_size_layout = QHBoxLayout()
        step_size_layout.addWidget(QLabel("Step Size:"))

        self._step_size_input = QDoubleSpinBox()
        self._step_size_input.setRange(0.1, 1000.0)
        self._step_size_input.setSingleStep(0.5)
        self._step_size_input.setDecimals(1)
        self._step_size_input.setValue(1.0)
        step_size_layout.addWidget(self._step_size_input)

        step_size_layout.addWidget(QLabel("cm"))
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

        # Parameters grid
        params_layout = QGridLayout()

        # Amplitude
        params_layout.addWidget(QLabel("Amplitude:"), 0, 0)
        self._sine_amplitude_input = QDoubleSpinBox()
        self._sine_amplitude_input.setRange(0.1, 100.0)
        self._sine_amplitude_input.setSingleStep(0.5)
        self._sine_amplitude_input.setDecimals(1)
        self._sine_amplitude_input.setValue(5.0)
        params_layout.addWidget(self._sine_amplitude_input, 0, 1)
        params_layout.addWidget(QLabel("cm"), 0, 2)

        # Frequency
        params_layout.addWidget(QLabel("Frequency:"), 1, 0)
        self._sine_frequency_input = QDoubleSpinBox()
        self._sine_frequency_input.setRange(0.01, 10.0)
        self._sine_frequency_input.setSingleStep(0.1)
        self._sine_frequency_input.setValue(0.5)
        self._sine_frequency_input.setDecimals(2)
        params_layout.addWidget(self._sine_frequency_input, 1, 1)
        params_layout.addWidget(QLabel("Hz"), 1, 2)

        # Duration
        params_layout.addWidget(QLabel("Duration:"), 2, 0)
        self._sine_duration_input = QDoubleSpinBox()
        self._sine_duration_input.setRange(1.0, 120.0)
        self._sine_duration_input.setSingleStep(1.0)
        self._sine_duration_input.setValue(10.0)
        self._sine_duration_input.setDecimals(1)
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
        target = joint_data.current_target
        error = position - target

        self._position_label.setText(f"{position}")
        self._target_label.setText(f"{target}")
        self._error_label.setText(f"{error}")

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

    def _on_set_target(self):
        """Handle set target button click."""
        target = self._target_input.value()
        joint_id = self._data_store.selected_joint

        # Update data store
        self._data_store.set_target(joint_id, target)

        # Send to Teensy
        self._serial_handler.set_target(joint_id, target)

    def _on_use_current(self):
        """Set target input to current position."""
        joint_data = self._data_store.get_selected_joint_data()
        self._target_input.setValue(joint_data.current_position)

    def _on_set_zero(self):
        """Set target to zero."""
        joint_id = self._data_store.selected_joint

        # Update data store
        self._data_store.set_target(joint_id, 0)

        # Update input field
        self._target_input.setValue(0)

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
        """Send position PID gains."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.set_pid(joint_id, "P", self._pos_p.value())
        self._serial_handler.set_pid(joint_id, "I", self._pos_i.value())
        self._serial_handler.set_pid(joint_id, "D", self._pos_d.value())

    def _on_set_vel_pid(self):
        """Send velocity PID gains."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.set_pid(joint_id, "p", self._vel_p.value())
        self._serial_handler.set_pid(joint_id, "i", self._vel_i.value())
        self._serial_handler.set_pid(joint_id, "d", self._vel_d.value())

    def _on_step_positive(self):
        """Handle positive step button click."""
        step = self._step_size_input.value()
        self._apply_step(step)

    def _on_step_negative(self):
        """Handle negative step button click."""
        step = -self._step_size_input.value()
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
        self._target_input.setValue(new_target)

        # Send to Teensy
        self._serial_handler.set_target(joint_id, new_target)

    def _on_start_sine(self):
        """Start sine wave input."""
        if self._sine_active:
            return

        joint_data = self._data_store.get_selected_joint_data()

        self._sine_amplitude = self._sine_amplitude_input.value()
        self._sine_frequency = self._sine_frequency_input.value()
        self._sine_duration = self._sine_duration_input.value()
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
        self._target_input.setValue(self._sine_center)

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
