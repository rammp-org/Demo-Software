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
    QScrollArea,
    QSizePolicy,
    QFrame,
    QCheckBox,
)
from PyQt6.QtCore import pyqtSignal, QTimer, Qt
from PyQt6.QtGui import QDoubleValidator
import math
import numpy as np

from ..data.data_store import DataStore
from ..serial_driver.serial_handler import SerialHandler
from .theme import THEME
from .scaling import SIZES, scaled
from .imu_display import IMUDisplay


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

        # Timed step state
        self._step_timer = QTimer(self)
        self._step_timer.setSingleShot(True)
        self._step_timer.timeout.connect(self._on_step_complete)

        self._setup_ui()

        # Connect to direction updates
        self._data_store.directions_updated.connect(self._update_direction_indicator)

        # Connect to config updates
        self._data_store.config_updated.connect(self._on_config_updated)

    def _setup_ui(self):
        """Set up the control panel layout with scroll area."""
        # Main layout for this widget
        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.setSpacing(0)

        # Create scroll area
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        scroll_area.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAsNeeded)
        scroll_area.setFrameShape(QFrame.Shape.NoFrame)

        # Container widget for scroll area content
        scroll_content = QWidget()
        layout = QVBoxLayout(scroll_content)
        layout.setContentsMargins(
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
        )
        layout.setSpacing(SIZES["spacing_medium"])

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

        # Self Leveling inputs
        layout.addWidget(self._create_self_leveling_group())

        # IMU Display
        self._imu_display = IMUDisplay(self._data_store)
        layout.addWidget(self._imu_display)

        layout.addStretch()

        # Set scroll content and add to outer layout
        scroll_area.setWidget(scroll_content)
        outer_layout.addWidget(scroll_area)

    def _create_status_group(self) -> QGroupBox:
        """Create the current values display group."""
        group = QGroupBox("Current Values")
        layout = QGridLayout(group)
        layout.setSpacing(SIZES["spacing_small"])

        # Dynamic font size for value labels
        value_font_size = SIZES["font_medium"]
        value_style = f"font-weight: bold; font-size: {value_font_size}pt;"

        # Mode indicator
        layout.addWidget(QLabel("Mode:"), 0, 0)
        self._mode_indicator_label = QLabel("Open Loop")
        self._mode_indicator_label.setStyleSheet(f"{value_style} color: cyan;")
        layout.addWidget(self._mode_indicator_label, 0, 1)
        self._mode_unit_label = QLabel("")  # Placeholder, units shown elsewhere
        layout.addWidget(self._mode_unit_label, 0, 2)

        # Position
        layout.addWidget(QLabel("Position:"), 1, 0)
        self._position_label = QLabel("---")
        self._position_label.setStyleSheet(value_style)
        layout.addWidget(self._position_label, 1, 1)
        self._position_unit_label = QLabel("ticks")
        layout.addWidget(self._position_unit_label, 1, 2)

        # Velocity
        layout.addWidget(QLabel("Velocity:"), 2, 0)
        self._velocity_label = QLabel("---")
        self._velocity_label.setStyleSheet(value_style)
        layout.addWidget(self._velocity_label, 2, 1)
        layout.addWidget(QLabel("units/s"), 2, 2)

        # PWM
        layout.addWidget(QLabel("PWM:"), 3, 0)
        self._pwm_label = QLabel("---")
        self._pwm_label.setStyleSheet(value_style)
        layout.addWidget(self._pwm_label, 3, 1)
        self._pwm_percent_label = QLabel("(0%)")
        layout.addWidget(self._pwm_percent_label, 3, 2)

        # Target (label changes based on mode)
        self._target_row_label = QLabel("Target:")
        layout.addWidget(self._target_row_label, 4, 0)
        self._target_label = QLabel("---")
        self._target_label.setStyleSheet(value_style)
        layout.addWidget(self._target_label, 4, 1)
        self._target_unit_label = QLabel("PWM")
        layout.addWidget(self._target_unit_label, 4, 2)

        # Error
        layout.addWidget(QLabel("Error:"), 5, 0)
        self._error_label = QLabel("---")
        self._error_label.setStyleSheet(value_style)
        layout.addWidget(self._error_label, 5, 1)
        self._error_unit_label = QLabel("")
        layout.addWidget(self._error_unit_label, 5, 2)

        # Store value style for dynamic updates
        self._value_style = value_style

        # Update timer
        self._status_timer = QTimer(self)
        self._status_timer.timeout.connect(self._update_status)
        self._status_timer.start(100)  # Update at 10 Hz

        return group

    def _create_analysis_group(self) -> QGroupBox:
        """Create oscillation analysis metrics group."""
        group = QGroupBox("Performance Analysis")
        layout = QGridLayout(group)
        layout.setSpacing(SIZES["spacing_small"])

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
        small_font = SIZES["font_small"]
        self._analysis_window_label.setStyleSheet(
            f"color: gray; font-size: {small_font}pt;"
        )
        layout.addWidget(self._analysis_window_label, 2, 0, 1, 2)

        # Reset analysis button
        self._reset_analysis_btn = QPushButton("Reset")
        self._reset_analysis_btn.clicked.connect(self._on_reset_analysis)
        layout.addWidget(self._reset_analysis_btn, 2, 2, 1, 2)

        return group

    def _create_pid_group(self) -> QGroupBox:
        group = QGroupBox("Mode & PID Control")
        layout = QVBoxLayout(group)
        layout.setSpacing(SIZES["spacing_medium"])

        # Get scaled input width
        pid_input_width = SIZES["input_min_width"]

        # Mode Selection row
        mode_layout = QHBoxLayout()
        mode_layout.setSpacing(SIZES["spacing_small"])
        mode_layout.addWidget(QLabel("Mode:"))
        self._mode_combo = QComboBox()
        self._mode_combo.addItems(["Open Loop (0)", "Velocity (1)", "Position (2)"])
        self._mode_combo.currentIndexChanged.connect(self._on_mode_combo_changed)
        self._mode_combo.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
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
        pos_pid_layout.setSpacing(SIZES["spacing_small"])
        pos_pid_layout.addWidget(QLabel("Pos:"))

        # Use QLineEdit for unlimited input with flexible sizing
        self._pos_p = QLineEdit("0")
        self._pos_p.setValidator(QDoubleValidator())
        self._pos_p.setMinimumWidth(pid_input_width)
        self._pos_p.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        self._pos_i = QLineEdit("0")
        self._pos_i.setValidator(QDoubleValidator())
        self._pos_i.setMinimumWidth(pid_input_width)
        self._pos_i.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        self._pos_d = QLineEdit("0")
        self._pos_d.setValidator(QDoubleValidator())
        self._pos_d.setMinimumWidth(pid_input_width)
        self._pos_d.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        self._pos_ff = QLineEdit("0")
        self._pos_ff.setValidator(QDoubleValidator())
        self._pos_ff.setMinimumWidth(pid_input_width)
        self._pos_ff.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        self._pos_lpf = QLineEdit("1.0")
        self._pos_lpf.setValidator(QDoubleValidator())
        self._pos_lpf.setMinimumWidth(pid_input_width)
        self._pos_lpf.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )

        pos_pid_layout.addWidget(QLabel("P:"))
        pos_pid_layout.addWidget(self._pos_p)
        pos_pid_layout.addWidget(QLabel("I:"))
        pos_pid_layout.addWidget(self._pos_i)
        pos_pid_layout.addWidget(QLabel("D:"))
        pos_pid_layout.addWidget(self._pos_d)
        pos_pid_layout.addWidget(QLabel("FF:"))
        pos_pid_layout.addWidget(self._pos_ff)
        pos_pid_layout.addWidget(QLabel("LPF α:"))
        pos_pid_layout.addWidget(self._pos_lpf)

        self._set_pos_pid_btn = QPushButton("Set")
        self._set_pos_pid_btn.clicked.connect(self._on_set_pos_pid)
        pos_pid_layout.addWidget(self._set_pos_pid_btn)
        layout.addLayout(pos_pid_layout)

        # Velocity PID row (P, I, D, FF)
        vel_pid_layout = QHBoxLayout()
        vel_pid_layout.setSpacing(SIZES["spacing_small"])
        vel_pid_layout.addWidget(QLabel("Vel:"))

        self._vel_p = QLineEdit("0")
        self._vel_p.setValidator(QDoubleValidator())
        self._vel_p.setMinimumWidth(pid_input_width)
        self._vel_p.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        self._vel_i = QLineEdit("0")
        self._vel_i.setValidator(QDoubleValidator())
        self._vel_i.setMinimumWidth(pid_input_width)
        self._vel_i.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        self._vel_d = QLineEdit("0")
        self._vel_d.setValidator(QDoubleValidator())
        self._vel_d.setMinimumWidth(pid_input_width)
        self._vel_d.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        self._vel_ff = QLineEdit("0")
        self._vel_ff.setValidator(QDoubleValidator())
        self._vel_ff.setMinimumWidth(pid_input_width)
        self._vel_ff.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        self._vel_lpf = QLineEdit("1.0")
        self._vel_lpf.setValidator(QDoubleValidator())
        self._vel_lpf.setMinimumWidth(pid_input_width)
        self._vel_lpf.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )

        vel_pid_layout.addWidget(QLabel("P:"))
        vel_pid_layout.addWidget(self._vel_p)
        vel_pid_layout.addWidget(QLabel("I:"))
        vel_pid_layout.addWidget(self._vel_i)
        vel_pid_layout.addWidget(QLabel("D:"))
        vel_pid_layout.addWidget(self._vel_d)
        vel_pid_layout.addWidget(QLabel("FF:"))
        vel_pid_layout.addWidget(self._vel_ff)
        vel_pid_layout.addWidget(QLabel("LPF α:"))
        vel_pid_layout.addWidget(self._vel_lpf)

        self._set_vel_pid_btn = QPushButton("Set")
        self._set_vel_pid_btn.clicked.connect(self._on_set_vel_pid)
        vel_pid_layout.addWidget(self._set_vel_pid_btn)
        layout.addLayout(vel_pid_layout)

        # Config row (Save/Load EEPROM)
        config_layout = QHBoxLayout()
        config_layout.setSpacing(SIZES["spacing_small"])

        self._load_config_btn = QPushButton("Load from EEPROM")
        self._load_config_btn.setToolTip("Request PID configuration from EEPROM")
        self._load_config_btn.clicked.connect(self._on_load_config)
        config_layout.addWidget(self._load_config_btn)

        self._save_config_btn = QPushButton("Save to EEPROM")
        self._save_config_btn.setToolTip("Save current PID configuration to EEPROM")
        self._save_config_btn.clicked.connect(self._on_save_config)
        self._save_config_btn.setStyleSheet(
            f"background-color: {THEME.blue}; color: {THEME.crust};"
        )
        config_layout.addWidget(self._save_config_btn)

        layout.addLayout(config_layout)

        return group

    def _create_target_group(self) -> QGroupBox:
        """Create the target control group."""
        group = QGroupBox("Target Control")
        layout = QVBoxLayout(group)
        layout.setSpacing(SIZES["spacing_medium"])

        # Target input row - use QLineEdit for unlimited range
        input_layout = QHBoxLayout()
        input_layout.setSpacing(SIZES["spacing_small"])

        target_grid = QGridLayout()
        target_grid.setSpacing(SIZES["spacing_small"])

        # Primary target
        target_grid.addWidget(QLabel("Primary:"), 0, 0)
        self._target_input = QLineEdit("0")
        self._target_input.setValidator(QDoubleValidator())
        self._target_input.setMinimumWidth(SIZES["input_preferred_width"])
        self._target_input.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        target_grid.addWidget(self._target_input, 0, 1)

        # Dynamic unit label (changes with mode)
        self._target_input_unit_label = QLabel("PWM")
        target_grid.addWidget(self._target_input_unit_label, 0, 2)

        # Linked target (Optional)
        self._linked_target_label = QLabel("Linked:")
        target_grid.addWidget(self._linked_target_label, 1, 0)
        self._linked_target_input = QLineEdit("0")
        self._linked_target_input.setValidator(QDoubleValidator())
        self._linked_target_input.setMinimumWidth(SIZES["input_preferred_width"])
        self._linked_target_input.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        target_grid.addWidget(self._linked_target_input, 1, 1)
        self._linked_target_unit_label = QLabel("PWM")
        target_grid.addWidget(self._linked_target_unit_label, 1, 2)

        layout.addLayout(target_grid)

        # Invert linked checkbox and Set button
        action_layout = QHBoxLayout()
        self._invert_linked_cb = QCheckBox("Invert Linked")
        self._invert_linked_cb.setToolTip("Invert the target sent to the linked joint")
        action_layout.addWidget(self._invert_linked_cb)

        self._set_target_btn = QPushButton("Set Target(s)")
        self._set_target_btn.clicked.connect(self._on_set_target)
        action_layout.addWidget(self._set_target_btn)

        layout.addLayout(action_layout)

        # Listen to linked joint changes to show/hide the secondary input
        self._data_store.linked_joint_changed.connect(self._on_linked_joint_changed)
        self._on_linked_joint_changed(self._data_store.linked_joint)

        # Quick target buttons row
        quick_layout = QHBoxLayout()
        quick_layout.setSpacing(SIZES["spacing_small"])

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

        layout.addLayout(quick_layout)

        # Motor config row (Home and Direction)
        motor_config_layout = QHBoxLayout()
        motor_config_layout.setSpacing(SIZES["spacing_small"])

        # Home Position button
        self._home_btn = QPushButton("Home Position")
        self._home_btn.setToolTip("Zero encoder position for selected joint")
        self._home_btn.clicked.connect(self._on_home_position)
        self._home_btn.setStyleSheet(
            f"background-color: {THEME.blue}; color: {THEME.crust};"
        )
        motor_config_layout.addWidget(self._home_btn)

        # Direction toggle with indicator
        motor_config_layout.addWidget(QLabel("Dir:"))
        self._dir_indicator = QLabel("->")
        self._dir_indicator.setStyleSheet(
            f"font-size: {SIZES['font_medium']}pt; font-weight: bold; color: {THEME.green};"
        )
        motor_config_layout.addWidget(self._dir_indicator)

        self._dir_btn = QPushButton("Flip")
        self._dir_btn.setToolTip("Flip motor direction (saved to EEPROM)")
        self._dir_btn.clicked.connect(self._on_toggle_direction)
        motor_config_layout.addWidget(self._dir_btn)

        motor_config_layout.addStretch()

        motor_config_layout.addWidget(QLabel("Input LPF α:"))
        self._input_lpf = QLineEdit("0.5")
        self._input_lpf.setValidator(QDoubleValidator())
        self._input_lpf.setMinimumWidth(SIZES["input_min_width"])
        self._input_lpf.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        motor_config_layout.addWidget(self._input_lpf)

        self._set_input_lpf_btn = QPushButton("Set")
        self._set_input_lpf_btn.clicked.connect(self._on_set_input_lpf)
        motor_config_layout.addWidget(self._set_input_lpf_btn)

        layout.addLayout(motor_config_layout)

        # Encoder Direction row
        enc_config_layout = QHBoxLayout()
        enc_config_layout.setSpacing(SIZES["spacing_small"])

        enc_config_layout.addStretch()  # Push to right to align with motor dir

        # Encoder Direction toggle with indicator
        enc_config_layout.addWidget(QLabel("Enc Dir:"))
        self._enc_dir_indicator = QLabel("->")
        self._enc_dir_indicator.setStyleSheet(
            f"font-size: {SIZES['font_medium']}pt; font-weight: bold; color: {THEME.green};"
        )
        enc_config_layout.addWidget(self._enc_dir_indicator)

        self._enc_dir_btn = QPushButton("Flip Enc")
        self._enc_dir_btn.setToolTip("Flip encoder direction (saved to EEPROM)")
        self._enc_dir_btn.clicked.connect(self._on_toggle_encoder_direction)
        enc_config_layout.addWidget(self._enc_dir_btn)

        layout.addLayout(enc_config_layout)

        # Safety buttons row (separate for emphasis)
        safety_layout = QHBoxLayout()
        safety_layout.setSpacing(SIZES["spacing_small"])

        # Disable Motors button
        self._disable_motors_btn = QPushButton("ESTOP (z)")
        self._disable_motors_btn.clicked.connect(self._on_disable_motors)
        self._disable_motors_btn.setStyleSheet(
            f"background-color: {THEME.red}; color: {THEME.crust};"
        )
        safety_layout.addWidget(self._disable_motors_btn)

        # Clear ESTOP button
        self._clear_estop_btn = QPushButton("Clear ESTOP (c)")
        self._clear_estop_btn.clicked.connect(self._on_clear_estop)
        self._clear_estop_btn.setStyleSheet(
            f"background-color: {THEME.peach}; color: {THEME.crust};"
        )
        safety_layout.addWidget(self._clear_estop_btn)

        layout.addLayout(safety_layout)

        return group

    def _create_step_group(self) -> QGroupBox:
        """Create the step/jog input group with timed step support."""
        group = QGroupBox("Step/Jog Input")
        layout = QVBoxLayout(group)
        layout.setSpacing(SIZES["spacing_medium"])

        # Amplitude and Duration inputs
        params_layout = QGridLayout()
        params_layout.setSpacing(SIZES["spacing_small"])

        params_layout.addWidget(QLabel("Amplitude:"), 0, 0)
        self._step_amplitude = QLineEdit("100")
        self._step_amplitude.setValidator(QDoubleValidator())
        self._step_amplitude.setMinimumWidth(SIZES["input_min_width"])
        self._step_amplitude.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        params_layout.addWidget(self._step_amplitude, 0, 1)
        self._step_unit_label = QLabel("PWM")
        params_layout.addWidget(self._step_unit_label, 0, 2)

        params_layout.addWidget(QLabel("Duration:"), 1, 0)
        self._step_duration = QLineEdit("0.5")
        self._step_duration.setValidator(QDoubleValidator())
        self._step_duration.setMinimumWidth(SIZES["input_min_width"])
        self._step_duration.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        params_layout.addWidget(self._step_duration, 1, 1)
        params_layout.addWidget(QLabel("sec"), 1, 2)

        layout.addLayout(params_layout)

        # Step buttons row
        step_btn_layout = QHBoxLayout()
        step_btn_layout.setSpacing(SIZES["spacing_small"])

        self._step_neg_btn = QPushButton("Step -")
        self._step_neg_btn.clicked.connect(self._on_step_negative)
        step_btn_layout.addWidget(self._step_neg_btn)

        self._step_pos_btn = QPushButton("Step +")
        self._step_pos_btn.clicked.connect(self._on_step_positive)
        step_btn_layout.addWidget(self._step_pos_btn)

        layout.addLayout(step_btn_layout)

        # Quick step buttons (percentages of amplitude)
        quick_layout = QHBoxLayout()
        quick_layout.setSpacing(SIZES["spacing_small"])
        quick_layout.addWidget(QLabel("Quick:"))
        for pct in [-100, -50, 50, 100, 200]:
            btn = QPushButton(f"{pct:+}%")
            btn.clicked.connect(lambda checked, p=pct: self._on_quick_step_percent(p))
            quick_layout.addWidget(btn)
        layout.addLayout(quick_layout)

        return group

    def _create_sine_group(self) -> QGroupBox:
        """Create the sine wave input group."""
        group = QGroupBox("Sine Wave Input")
        layout = QVBoxLayout(group)
        layout.setSpacing(SIZES["spacing_medium"])

        # Parameters grid - use QLineEdit for unlimited range
        params_layout = QGridLayout()
        params_layout.setSpacing(SIZES["spacing_small"])

        # Amplitude
        params_layout.addWidget(QLabel("Amplitude:"), 0, 0)
        self._sine_amplitude_input = QLineEdit("500")
        self._sine_amplitude_input.setValidator(QDoubleValidator())
        self._sine_amplitude_input.setMinimumWidth(SIZES["input_min_width"])
        self._sine_amplitude_input.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        params_layout.addWidget(self._sine_amplitude_input, 0, 1)
        self._sine_amplitude_unit_label = QLabel("PWM")
        params_layout.addWidget(self._sine_amplitude_unit_label, 0, 2)

        # Frequency
        params_layout.addWidget(QLabel("Frequency:"), 1, 0)
        self._sine_frequency_input = QLineEdit("0.5")
        self._sine_frequency_input.setValidator(QDoubleValidator())
        self._sine_frequency_input.setMinimumWidth(SIZES["input_min_width"])
        self._sine_frequency_input.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        params_layout.addWidget(self._sine_frequency_input, 1, 1)
        params_layout.addWidget(QLabel("Hz"), 1, 2)

        # Duration
        params_layout.addWidget(QLabel("Duration:"), 2, 0)
        self._sine_duration_input = QLineEdit("10")
        self._sine_duration_input.setValidator(QDoubleValidator())
        self._sine_duration_input.setMinimumWidth(SIZES["input_min_width"])
        self._sine_duration_input.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        params_layout.addWidget(self._sine_duration_input, 2, 1)
        params_layout.addWidget(QLabel("seconds"), 2, 2)

        layout.addLayout(params_layout)

        # Control buttons
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(SIZES["spacing_small"])

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

    def _create_self_leveling_group(self) -> QGroupBox:
        """Create the self-leveling group."""
        group = QGroupBox("Self Leveling")
        layout = QVBoxLayout(group)
        layout.setSpacing(SIZES["spacing_medium"])

        # Parameter grid
        params_layout = QGridLayout()
        params_layout.setSpacing(SIZES["spacing_small"])

        params_layout.addWidget(QLabel("Target Pitch:"), 0, 0)
        self._target_pitch_input = QLineEdit("0.0")
        self._target_pitch_input.setValidator(QDoubleValidator())
        self._target_pitch_input.setMinimumWidth(SIZES["input_min_width"])
        self._target_pitch_input.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        params_layout.addWidget(self._target_pitch_input, 0, 1)
        params_layout.addWidget(QLabel("deg"), 0, 2)

        params_layout.addWidget(QLabel("Target Roll:"), 1, 0)
        self._target_roll_input = QLineEdit("0.0")
        self._target_roll_input.setValidator(QDoubleValidator())
        self._target_roll_input.setMinimumWidth(SIZES["input_min_width"])
        self._target_roll_input.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        params_layout.addWidget(self._target_roll_input, 1, 1)
        params_layout.addWidget(QLabel("deg"), 1, 2)

        self._use_current_imu_btn = QPushButton("Use Current")
        self._use_current_imu_btn.setToolTip("Set Target to current IMU Pitch & Roll")
        self._use_current_imu_btn.clicked.connect(self._on_use_current_imu)
        params_layout.addWidget(self._use_current_imu_btn, 0, 3, 2, 1)  # span 2 rows

        layout.addLayout(params_layout)

        # Start/Stop Buttons
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(SIZES["spacing_small"])

        self._start_leveling_btn = QPushButton("Start Leveling")
        self._start_leveling_btn.setStyleSheet(
            f"background-color: {THEME.teal}; color: {THEME.crust};"
        )
        self._start_leveling_btn.clicked.connect(self._on_start_leveling)
        btn_layout.addWidget(self._start_leveling_btn)

        self._stop_leveling_btn = QPushButton("Stop Leveling")
        self._stop_leveling_btn.clicked.connect(self._on_stop_leveling)
        btn_layout.addWidget(self._stop_leveling_btn)

        layout.addLayout(btn_layout)

        return group

    def _on_start_leveling(self):
        """Enable self leveling mode."""
        pitch = self._get_float_from_lineedit(self._target_pitch_input, 0.0)
        roll = self._get_float_from_lineedit(self._target_roll_input, 0.0)

        # Save targets to data store so plot can read them
        self._data_store.imu_target_pitch = pitch
        self._data_store.imu_target_roll = roll

        self._serial_handler.set_imu_target(pitch, roll)
        self._serial_handler.set_self_leveling(True)

    def _on_stop_leveling(self):
        """Disable self leveling mode."""
        self._serial_handler.set_self_leveling(False)

    def _on_use_current_imu(self):
        """Set IMU targets to current IMU orientation."""
        pitch = self._data_store.imu_pitch
        roll = self._data_store.imu_roll
        self._target_pitch_input.setText(f"{pitch:.2f}")
        self._target_roll_input.setText(f"{roll:.2f}")

        # If currently leveling, update targets immediately
        if self._data_store.current_state == 4:  # SELF_LEVELING
            self._on_start_leveling()

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
        value_font_size = SIZES["font_medium"]
        if abs(error) < 10:
            self._error_label.setStyleSheet(
                f"font-weight: bold; font-size: {value_font_size}pt; color: green;"
            )
        elif abs(error) < 100:
            self._error_label.setStyleSheet(
                f"font-weight: bold; font-size: {value_font_size}pt; color: orange;"
            )
        else:
            self._error_label.setStyleSheet(
                f"font-weight: bold; font-size: {value_font_size}pt; color: red;"
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
        self._data_store.control_mode = index
        unit = MODE_UNITS.get(index, "")

        # Update all unit labels based on mode
        self._target_unit_label.setText(unit)
        self._target_input_unit_label.setText(unit)
        self._linked_target_unit_label.setText(unit)
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

    def _on_linked_joint_changed(self, linked_joint_id: int):
        """Update UI based on whether a linked joint is active."""
        has_linked = linked_joint_id != 0
        self._linked_target_label.setVisible(has_linked)
        self._linked_target_input.setVisible(has_linked)
        self._linked_target_unit_label.setVisible(has_linked)
        self._invert_linked_cb.setVisible(has_linked)

    def _on_clear_pid(self):
        """Send reset PID command to clear integrator windup."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.reset_pid(joint_id)
        if self._data_store.linked_joint != 0:
            self._serial_handler.reset_pid(self._data_store.linked_joint)

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

        linked_joint_id = self._data_store.linked_joint
        if linked_joint_id != 0:
            # If the user typed something into the linked input, use it.
            # Otherwise use the primary target, potentially inverted.
            linked_text = self._linked_target_input.text().strip()

            # Simple check: if the user actually wrote a different value specifically in the linked box
            # we should respect it. However, keeping them synced when using step inputs is usually desired.
            # Let's read whatever is currently in the linked_target_input field.
            linked_target = self._get_float_from_lineedit(self._linked_target_input)

            self._data_store.set_target(linked_joint_id, linked_target)
            self._serial_handler.set_target(linked_joint_id, linked_target)

    def _on_use_current(self):
        """Set target input to current position/velocity based on mode."""
        joint_data = self._data_store.get_selected_joint_data()
        if self._current_mode == MODE_VELOCITY:
            self._target_input.setText(str(joint_data.current_velocity))
        else:
            self._target_input.setText(str(joint_data.current_position))

        linked_joint_id = self._data_store.linked_joint
        if linked_joint_id != 0:
            linked_data = self._data_store.get_joint(linked_joint_id)
            if linked_data is not None:
                if self._current_mode == MODE_VELOCITY:
                    self._linked_target_input.setText(str(linked_data.current_velocity))
                else:
                    self._linked_target_input.setText(str(linked_data.current_position))

    def _on_set_zero(self):
        """Set target to zero."""
        joint_id = self._data_store.selected_joint

        # Update data store
        self._data_store.set_target(joint_id, 0)
        # Update input field
        self._target_input.setText("0")
        # Send to Teensy
        self._serial_handler.set_target(joint_id, 0)

        linked_joint_id = self._data_store.linked_joint
        if linked_joint_id != 0:
            self._data_store.set_target(linked_joint_id, 0)
            self._linked_target_input.setText("0")
            self._serial_handler.set_target(linked_joint_id, 0)

    def _on_disable_motors(self):
        """Send disable motors command."""
        self._serial_handler.disable_motors()

    def _on_clear_estop(self):
        """Send clear ESTOP command."""
        self._serial_handler.clear_estop()

    def _on_home_position(self):
        """Send home/zero position command."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.home_position(joint_id)
        # Also reset target input to 0
        self._target_input.setText("0")
        self._data_store.set_target(joint_id, 0)

        linked_joint_id = self._data_store.linked_joint
        if linked_joint_id != 0:
            self._serial_handler.home_position(linked_joint_id)
            self._linked_target_input.setText("0")
            self._data_store.set_target(linked_joint_id, 0)

    def _on_toggle_direction(self):
        """Toggle motor direction for selected joint."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.toggle_direction(joint_id)

        linked_joint_id = self._data_store.linked_joint
        if linked_joint_id != 0:
            self._serial_handler.toggle_direction(linked_joint_id)

    def _on_toggle_encoder_direction(self):
        """Toggle encoder direction for selected joint."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.toggle_encoder_direction(joint_id)

        linked_joint_id = self._data_store.linked_joint
        if linked_joint_id != 0:
            self._serial_handler.toggle_encoder_direction(linked_joint_id)

    def _update_direction_indicator(self):
        """Update the direction indicator based on current motor direction."""
        joint_id = self._data_store.selected_joint
        directions = self._data_store.motor_directions
        if joint_id <= len(directions):
            direction = directions[joint_id - 1]
            if direction >= 0:
                self._dir_indicator.setText("->")
                self._dir_indicator.setStyleSheet(
                    f"font-size: {SIZES['font_medium']}pt; font-weight: bold; color: {THEME.green};"
                )
            else:
                self._dir_indicator.setText("<-")
                self._dir_indicator.setStyleSheet(
                    f"font-size: {SIZES['font_medium']}pt; font-weight: bold; color: {THEME.yellow};"
                )

        enc_directions = self._data_store.encoder_directions
        if joint_id <= len(enc_directions):
            enc_direction = enc_directions[joint_id - 1]
            if enc_direction >= 0:
                self._enc_dir_indicator.setText("->")
                self._enc_dir_indicator.setStyleSheet(
                    f"font-size: {SIZES['font_medium']}pt; font-weight: bold; color: {THEME.green};"
                )
            else:
                self._enc_dir_indicator.setText("<-")
                self._enc_dir_indicator.setStyleSheet(
                    f"font-size: {SIZES['font_medium']}pt; font-weight: bold; color: {THEME.yellow};"
                )

    def _on_set_mode(self):
        """Send mode set command."""
        joint_id = self._data_store.selected_joint
        mode = self._mode_combo.currentIndex()
        self._serial_handler.set_mode(joint_id, mode)
        if self._data_store.linked_joint != 0:
            self._serial_handler.set_mode(self._data_store.linked_joint, mode)

    def _on_set_pos_pid(self):
        """Send position PID gains and feed-forward."""
        joint_id = self._data_store.selected_joint
        p = self._get_float_from_lineedit(self._pos_p)
        i = self._get_float_from_lineedit(self._pos_i)
        d = self._get_float_from_lineedit(self._pos_d)
        ff = self._get_float_from_lineedit(self._pos_ff)
        lpf = self._get_float_from_lineedit(self._pos_lpf, 1.0)

        self._serial_handler.set_pid(joint_id, "P", p)
        self._serial_handler.set_pid(joint_id, "I", i)
        self._serial_handler.set_pid(joint_id, "D", d)
        self._serial_handler.set_feed_forward(joint_id, "F", ff)
        self._serial_handler.set_pos_lpf(joint_id, lpf)

        linked_joint_id = self._data_store.linked_joint
        if linked_joint_id != 0:
            self._serial_handler.set_pid(linked_joint_id, "P", p)
            self._serial_handler.set_pid(linked_joint_id, "I", i)
            self._serial_handler.set_pid(linked_joint_id, "D", d)
            self._serial_handler.set_feed_forward(linked_joint_id, "F", ff)
            self._serial_handler.set_pos_lpf(linked_joint_id, lpf)

    def _on_set_vel_pid(self):
        """Send velocity PID gains and feed-forward."""
        joint_id = self._data_store.selected_joint
        p = self._get_float_from_lineedit(self._vel_p)
        i = self._get_float_from_lineedit(self._vel_i)
        d = self._get_float_from_lineedit(self._vel_d)
        ff = self._get_float_from_lineedit(self._vel_ff)
        lpf = self._get_float_from_lineedit(self._vel_lpf, 1.0)

        self._serial_handler.set_pid(joint_id, "p", p)
        self._serial_handler.set_pid(joint_id, "i", i)
        self._serial_handler.set_pid(joint_id, "d", d)
        self._serial_handler.set_feed_forward(joint_id, "f", ff)
        self._serial_handler.set_vel_lpf(joint_id, lpf)

        linked_joint_id = self._data_store.linked_joint
        if linked_joint_id != 0:
            self._serial_handler.set_pid(linked_joint_id, "p", p)
            self._serial_handler.set_pid(linked_joint_id, "i", i)
            self._serial_handler.set_pid(linked_joint_id, "d", d)
            self._serial_handler.set_feed_forward(linked_joint_id, "f", ff)
            self._serial_handler.set_vel_lpf(linked_joint_id, lpf)

    def _on_set_input_lpf(self):
        """Send motor input LPF alpha."""
        joint_id = self._data_store.selected_joint
        lpf = self._get_float_from_lineedit(self._input_lpf, 0.5)

        self._serial_handler.set_input_lpf(joint_id, lpf)

        linked_joint_id = self._data_store.linked_joint
        if linked_joint_id != 0:
            self._serial_handler.set_input_lpf(linked_joint_id, lpf)

    def _on_load_config(self):
        """Request PID configuration from EEPROM."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.get_config(joint_id)

        linked_joint_id = self._data_store.linked_joint
        if linked_joint_id != 0:
            self._serial_handler.get_config(linked_joint_id)

    def _on_save_config(self):
        """Save current PID configuration to EEPROM."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.save_config(joint_id)

        linked_joint_id = self._data_store.linked_joint
        if linked_joint_id != 0:
            self._serial_handler.save_config(linked_joint_id)

    def _on_config_updated(self, joint_id: int):
        """Update PID input fields when configuration is loaded from EEPROM."""
        if joint_id == self._data_store.selected_joint:
            config = self._data_store.get_config(joint_id)
            if config is not None:
                self._pos_p.setText(f"{config.pos_p:g}")
                self._pos_i.setText(f"{config.pos_i:g}")
                self._pos_d.setText(f"{config.pos_d:g}")
                self._pos_ff.setText(f"{config.pos_ff:g}")
                self._pos_lpf.setText(f"{config.pos_lpf_alpha:g}")

                self._vel_p.setText(f"{config.vel_p:g}")
                self._vel_i.setText(f"{config.vel_i:g}")
                self._vel_d.setText(f"{config.vel_d:g}")
                self._vel_ff.setText(f"{config.vel_ff:g}")
                self._vel_lpf.setText(f"{config.vel_lpf_alpha:g}")

                self._input_lpf.setText(f"{config.input_lpf_alpha:g}")

    def _on_step_positive(self):
        """Handle positive step button click - timed step."""
        amplitude = self._get_float_from_lineedit(self._step_amplitude, 100.0)
        self._execute_timed_step(amplitude)

    def _on_step_negative(self):
        """Handle negative step button click - timed step."""
        amplitude = -self._get_float_from_lineedit(self._step_amplitude, 100.0)
        self._execute_timed_step(amplitude)

    def _on_quick_step_percent(self, percent: int):
        """Handle quick step button click (percentage of amplitude)."""
        base_amplitude = self._get_float_from_lineedit(self._step_amplitude, 100.0)
        actual_amplitude = base_amplitude * percent / 100
        self._execute_timed_step(actual_amplitude)

    def _execute_timed_step(self, amplitude: float):
        """Execute a timed step: send target, wait duration, return to zero."""
        duration = self._get_float_from_lineedit(self._step_duration, 0.5)
        joint_id = self._data_store.selected_joint
        linked_joint_id = self._data_store.linked_joint
        invert = self._invert_linked_cb.isChecked()

        # Update primary target input display to match step
        self._target_input.setText(str(amplitude))

        # Send step command primary
        self._serial_handler.set_target(joint_id, amplitude)
        self._data_store.set_target(joint_id, amplitude)

        # Send to linked if active
        if linked_joint_id != 0:
            linked_amp = -amplitude if invert else amplitude
            self._linked_target_input.setText(str(linked_amp))
            self._serial_handler.set_target(linked_joint_id, linked_amp)
            self._data_store.set_target(linked_joint_id, linked_amp)

        # Start timer to return to zero
        self._step_timer.start(int(duration * 1000))

    def _on_step_complete(self):
        """Handle step completion - return to zero."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.set_target(joint_id, 0)
        self._target_input.setText("0")
        self._data_store.set_target(joint_id, 0)

        linked_joint_id = self._data_store.linked_joint
        if linked_joint_id != 0:
            self._serial_handler.set_target(linked_joint_id, 0)
            self._linked_target_input.setText("0")
            self._data_store.set_target(linked_joint_id, 0)

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

        linked_joint_id = self._data_store.linked_joint
        if linked_joint_id != 0:
            linked_data = self._data_store.get_joint(linked_joint_id)
            if linked_data is not None:
                self._sine_linked_center = linked_data.current_target
            else:
                self._sine_linked_center = 0.0

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

        linked_joint_id = self._data_store.linked_joint
        if linked_joint_id != 0:
            self._data_store.set_target(linked_joint_id, self._sine_linked_center)
            self._serial_handler.set_target(linked_joint_id, self._sine_linked_center)
            self._linked_target_input.setText(str(self._sine_linked_center))

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

        linked_joint_id = self._data_store.linked_joint
        if linked_joint_id != 0:
            invert = self._invert_linked_cb.isChecked()
            linked_val = -value if invert else value
            linked_target = self._sine_linked_center + linked_val
            self._data_store.set_target(linked_joint_id, linked_target)
            self._serial_handler.set_target(linked_joint_id, linked_target)

        # Update time remaining
        remaining = self._sine_duration - self._sine_start_time
        self._sine_status_label.setText(
            f"Sine wave: Active ({remaining:.1f}s remaining)"
        )
