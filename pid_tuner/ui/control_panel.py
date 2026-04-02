"""
Control panel for setting targets, step inputs, and sine wave inputs.
"""

from __future__ import annotations

from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QGridLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QSpinBox,
    QScrollArea,
    QSizePolicy,
    QFrame,
    QMenu,
)
from PyQt6.QtCore import pyqtSignal, QTimer, Qt, QSettings
from PyQt6.QtGui import QDoubleValidator, QAction
import json
import math
import os
import numpy as np

from ..data.data_store import DataStore
from ..serial_driver.serial_handler import SerialHandler
from ..serial_driver.protocol import ProtocolEncoder
from .theme import THEME
from .scaling import SIZES, scaled
from .imu_display import IMUDisplay
from .imu_3d_widget import IMU3DWidget
from .collapsible_group import CollapsibleGroupBox
from .strain_gauge_display import StrainGaugeDisplay


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

# Mode colors for visibility enhancement
MODE_COLORS = {
    MODE_OPEN_LOOP: THEME.red,
    MODE_VELOCITY: THEME.yellow,
    MODE_POSITION: THEME.blue,
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

    # Signal emitted when control mode changes
    mode_changed = pyqtSignal(int)

    # Default step sizes
    DEFAULT_STEP_SIZES = [10, 50, 100, 500, 1000]

    def __init__(
        self,
        data_store: DataStore,
        serial_handler: SerialHandler,
        settings: "QSettings | None" = None,
        parent=None,
    ):
        super().__init__(parent)
        self._data_store = data_store
        self._serial_handler = serial_handler
        self._settings = settings

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

        # Connect mode banner to data store — only updates when data_store.control_mode
        # is explicitly set from a confirmed source (not the send path)
        self._data_store.mode_changed.connect(self._on_mode_confirmed)

    def _setup_ui(self):
        """Set up the control panel layout with scroll area."""
        # Main layout for this widget
        outer_layout = QVBoxLayout(self)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        outer_layout.setSpacing(0)

        # Toolbar row at the top (outside scroll area)
        toolbar_layout = QHBoxLayout()
        toolbar_layout.setContentsMargins(
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
            0,
        )
        toolbar_layout.setSpacing(SIZES["spacing_small"])

        # Panels visibility menu button
        self._panels_btn = QPushButton("Panels")
        self._panels_btn.setToolTip("Show/hide control panels")
        self._panels_menu = QMenu(self)
        self._panels_btn.setMenu(self._panels_menu)
        toolbar_layout.addWidget(self._panels_btn)

        toolbar_layout.addStretch()
        outer_layout.addLayout(toolbar_layout)

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

        # Mode banner (prominent mode indicator at top)
        layout.addWidget(self._create_mode_banner())

        # Initialize panel registry (name -> widget mapping)
        self._panels: dict[str, QWidget] = {}

        # Current values display
        self._status_group = self._create_status_group()
        layout.addWidget(self._status_group)
        self._panels["Current Values"] = self._status_group

        # Oscillation analysis (default collapsed)
        self._analysis_group = self._create_analysis_group()
        self._analysis_group.setCollapsed(True)
        layout.addWidget(self._analysis_group)
        self._panels["Performance Analysis"] = self._analysis_group

        # PID Control
        self._pid_group = self._create_pid_group()
        layout.addWidget(self._pid_group)
        self._panels["Mode & PID Control"] = self._pid_group

        # Target control
        self._target_group = self._create_target_group()
        layout.addWidget(self._target_group)
        self._panels["Target Control"] = self._target_group

        # Step inputs
        self._step_group = self._create_step_group()
        layout.addWidget(self._step_group)
        self._panels["Step/Jog Input"] = self._step_group

        # Quick Jog (hold-to-jog PWM buttons)
        self._quick_jog_group = self._create_quick_jog_group()
        layout.addWidget(self._quick_jog_group)
        self._panels["Quick Jog"] = self._quick_jog_group

        # Sine wave inputs (default collapsed)
        self._sine_group = self._create_sine_group()
        self._sine_group.setCollapsed(True)
        layout.addWidget(self._sine_group)
        self._panels["Sine Wave Input"] = self._sine_group

        self._leveling_group = self._create_self_leveling_group()
        layout.addWidget(self._leveling_group)
        self._panels["Self Leveling"] = self._leveling_group

        self._stored_seq_group = self._create_stored_sequences_group()
        layout.addWidget(self._stored_seq_group)
        self._panels["Stored Sequences"] = self._stored_seq_group

        # IMU Display (wrapped in collapsible group)
        self._imu_display = IMUDisplay(self._data_store)
        self._imu_display_group = CollapsibleGroupBox("IMU Display")
        self._imu_display_group.addWidget(self._imu_display)
        layout.addWidget(self._imu_display_group)
        self._panels["IMU Display"] = self._imu_display_group

        # 3D IMU Visualization (wrapped in collapsible group)
        self._imu_3d_widget = IMU3DWidget(self._data_store)
        self._imu_3d_group = CollapsibleGroupBox("3D IMU Visualization")
        self._imu_3d_group.addWidget(self._imu_3d_widget)
        layout.addWidget(self._imu_3d_group)
        self._panels["3D IMU Visualization"] = self._imu_3d_group

        # Strain Gauge Display (wrapped in collapsible group)
        self._strain_gauge_display = StrainGaugeDisplay(self._data_store)
        self._strain_gauge_group = CollapsibleGroupBox("Strain Gauges")
        self._strain_gauge_group.addWidget(self._strain_gauge_display)
        layout.addWidget(self._strain_gauge_group)
        self._panels["Strain Gauges"] = self._strain_gauge_group

        layout.addStretch()

        # Set scroll content and add to outer layout
        scroll_area.setWidget(scroll_content)
        outer_layout.addWidget(scroll_area)

        # Build the panels visibility menu
        self._build_panels_menu()

        # Restore saved panel states (must be after menu is built)
        self._restore_panel_settings()

    def _build_panels_menu(self):
        """Build the panels visibility menu with checkboxes for each panel."""
        self._panels_menu.clear()
        self._panel_actions: dict[str, QAction] = {}

        for panel_name, panel_widget in self._panels.items():
            action = QAction(panel_name, self)
            action.setCheckable(True)
            action.setChecked(panel_widget.isVisible())
            # Use a closure to capture the widget and name references
            action.triggered.connect(
                lambda checked, w=panel_widget, name=panel_name: (
                    self._on_panel_visibility_changed(name, w, checked)
                )
            )
            self._panels_menu.addAction(action)
            self._panel_actions[panel_name] = action

            # Connect collapsed_changed signal for CollapsibleGroupBox panels
            if isinstance(panel_widget, CollapsibleGroupBox):
                panel_widget.collapsed_changed.connect(
                    lambda collapsed, name=panel_name: self._on_panel_collapsed_changed(
                        name, collapsed
                    )
                )

        # Add separator and utility actions
        self._panels_menu.addSeparator()

        # Show All action
        show_all_action = QAction("Show All", self)
        show_all_action.triggered.connect(self._show_all_panels)
        self._panels_menu.addAction(show_all_action)

        # Hide All action
        hide_all_action = QAction("Hide All", self)
        hide_all_action.triggered.connect(self._hide_all_panels)
        self._panels_menu.addAction(hide_all_action)

    def _on_panel_visibility_changed(self, name: str, widget: QWidget, visible: bool):
        """Handle panel visibility change - update widget and save setting."""
        widget.setVisible(visible)
        self._save_panel_settings()

    def _on_panel_collapsed_changed(self, name: str, collapsed: bool):
        """Handle panel collapsed state change - save setting."""
        self._save_panel_settings()

    def _save_panel_settings(self):
        """Save panel visibility and collapsed states to settings."""
        if self._settings is None:
            return

        # Save visibility states
        hidden_panels = [
            name for name, widget in self._panels.items() if not widget.isVisible()
        ]
        self._settings.setValue("hidden_panels", hidden_panels)

        # Save collapsed states
        collapsed_panels = [
            name
            for name, widget in self._panels.items()
            if isinstance(widget, CollapsibleGroupBox) and widget.isCollapsed()
        ]
        self._settings.setValue("collapsed_panels", collapsed_panels)

    def _restore_panel_settings(self):
        """Restore panel visibility and collapsed states from settings."""
        if self._settings is None:
            return

        # Restore visibility states
        hidden_panels = self._settings.value("hidden_panels", [])
        if hidden_panels:
            for panel_name in hidden_panels:
                if panel_name in self._panels:
                    self._panels[panel_name].setVisible(False)
                    if panel_name in self._panel_actions:
                        self._panel_actions[panel_name].setChecked(False)

        # Restore collapsed states
        collapsed_panels = self._settings.value("collapsed_panels", [])
        if collapsed_panels:
            for panel_name in collapsed_panels:
                if panel_name in self._panels:
                    widget = self._panels[panel_name]
                    if isinstance(widget, CollapsibleGroupBox):
                        widget.setCollapsed(True)

        # Also handle panels that should be expanded (were collapsed by default but user expanded)
        # by checking what's NOT in collapsed_panels
        if collapsed_panels is not None:  # Only if we have saved state
            for panel_name, widget in self._panels.items():
                if isinstance(widget, CollapsibleGroupBox):
                    if panel_name not in collapsed_panels:
                        widget.setCollapsed(False)

    def _show_all_panels(self):
        """Show all panels and update menu checkboxes."""
        for panel_name, panel_widget in self._panels.items():
            panel_widget.setVisible(True)
            if panel_name in self._panel_actions:
                self._panel_actions[panel_name].setChecked(True)
        self._save_panel_settings()

    def _hide_all_panels(self):
        """Hide all panels and update menu checkboxes."""
        for panel_name, panel_widget in self._panels.items():
            panel_widget.setVisible(False)
            if panel_name in self._panel_actions:
                self._panel_actions[panel_name].setChecked(False)
        self._save_panel_settings()

    def _create_mode_banner(self) -> QFrame:
        """Create a prominent mode indicator banner."""
        self._mode_banner = QFrame()
        self._mode_banner.setFixedHeight(scaled(40))
        self._mode_banner.setStyleSheet(f"""
            QFrame {{
                background-color: {THEME.overlay0};
                border-radius: {scaled(4)}px;
            }}
        """)

        layout = QHBoxLayout(self._mode_banner)
        layout.setContentsMargins(SIZES["margin_medium"], 0, SIZES["margin_medium"], 0)

        self._mode_banner_label = QLabel("MODE: ---")
        self._mode_banner_label.setStyleSheet(f"""
            font-size: {SIZES["font_large"]}pt;
            font-weight: bold;
            color: {THEME.crust};
        """)
        self._mode_banner_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        layout.addWidget(self._mode_banner_label)

        return self._mode_banner

    def _set_mode_banner_pending(self, mode: int):
        """Show the mode banner in a dimmed 'pending' state after sending a command."""
        mode_name = MODE_NAMES.get(mode, "Unknown")
        self._mode_banner.setStyleSheet(f"""
            QFrame {{
                background-color: {THEME.overlay0};
                border-radius: {scaled(4)}px;
            }}
        """)
        self._mode_banner_label.setText(f"MODE: {mode_name.upper()} (pending...)")

    def _on_mode_confirmed(self, mode: int):
        """Update the mode banner when the data store reports a confirmed mode.

        This fires when telemetry arrives with per-motor mode values (59-field
        packet), so the banner reflects what the robot is actually doing, not
        just what was last sent.
        """
        mode_name = MODE_NAMES.get(mode, "Unknown")
        mode_color = MODE_COLORS.get(mode, THEME.overlay0)
        self._mode_banner.setStyleSheet(f"""
            QFrame {{
                background-color: {mode_color};
                border-radius: {scaled(4)}px;
            }}
        """)
        self._mode_banner_label.setText(f"MODE: {mode_name.upper()}")

    def _create_status_group(self) -> CollapsibleGroupBox:
        """Create the current values display group."""
        group = CollapsibleGroupBox("Current Values")
        content = QWidget()
        layout = QGridLayout(content)
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

        group.addWidget(content)
        return group

    def _create_analysis_group(self) -> CollapsibleGroupBox:
        """Create oscillation analysis metrics group."""
        group = CollapsibleGroupBox("Performance Analysis")
        content = QWidget()
        layout = QGridLayout(content)
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

        group.addWidget(content)
        return group

    def _create_pid_group(self) -> CollapsibleGroupBox:
        group = CollapsibleGroupBox("Mode & PID Control")
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setSpacing(SIZES["spacing_medium"])

        # Get scaled input width
        pid_input_width = SIZES["input_min_width"]

        # Mode buttons row — each button sends the mode immediately when pressed
        mode_layout = QHBoxLayout()
        mode_layout.setSpacing(SIZES["spacing_small"])

        self._open_loop_btn = QPushButton("Open Loop")
        self._open_loop_btn.setToolTip("Set open-loop PWM mode (mode 0)")
        self._open_loop_btn.clicked.connect(lambda: self._on_set_mode(MODE_OPEN_LOOP))
        self._open_loop_btn.setStyleSheet(f"""
            QPushButton {{ background-color: {MODE_COLORS[MODE_OPEN_LOOP]}; color: {THEME.crust}; font-weight: bold; }}
            QPushButton:hover {{ background-color: {THEME.maroon}; }}
            QPushButton:pressed {{ background-color: {THEME.flamingo}; }}
        """)
        mode_layout.addWidget(self._open_loop_btn)

        self._cl_vel_btn = QPushButton("CL Vel")
        self._cl_vel_btn.setToolTip("Set closed-loop velocity mode (mode 1)")
        self._cl_vel_btn.clicked.connect(lambda: self._on_set_mode(MODE_VELOCITY))
        self._cl_vel_btn.setStyleSheet(f"""
            QPushButton {{ background-color: {MODE_COLORS[MODE_VELOCITY]}; color: {THEME.crust}; font-weight: bold; }}
            QPushButton:hover {{ background-color: {THEME.peach}; }}
            QPushButton:pressed {{ background-color: {THEME.rosewater}; }}
        """)
        mode_layout.addWidget(self._cl_vel_btn)

        self._cl_pos_btn = QPushButton("CL Pos")
        self._cl_pos_btn.setToolTip("Set closed-loop position mode (mode 2)")
        self._cl_pos_btn.clicked.connect(lambda: self._on_set_mode(MODE_POSITION))
        self._cl_pos_btn.setStyleSheet(f"""
            QPushButton {{ background-color: {MODE_COLORS[MODE_POSITION]}; color: {THEME.crust}; font-weight: bold; }}
            QPushButton:hover {{ background-color: {THEME.sapphire}; }}
            QPushButton:pressed {{ background-color: {THEME.lavender}; }}
        """)
        mode_layout.addWidget(self._cl_pos_btn)

        # Clear PID button (resets integrator windup)
        self._clear_pid_btn = QPushButton("Clear PID")
        self._clear_pid_btn.setToolTip("Reset integrator windup and previous error")
        self._clear_pid_btn.clicked.connect(self._on_clear_pid)
        self._clear_pid_btn.setStyleSheet(f"""
            QPushButton {{ background-color: {THEME.yellow}; color: {THEME.crust}; }}
            QPushButton:hover {{ background-color: {THEME.peach}; }}
            QPushButton:pressed {{ background-color: {THEME.rosewater}; }}
        """)
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
        self._pos_ramp = QLineEdit("0.0")
        self._pos_ramp.setValidator(QDoubleValidator())
        self._pos_ramp.setMinimumWidth(pid_input_width)
        self._pos_ramp.setSizePolicy(
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
        pos_pid_layout.addWidget(QLabel("Ramp:"))
        pos_pid_layout.addWidget(self._pos_ramp)

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
        self._vel_ramp = QLineEdit("0.0")
        self._vel_ramp.setValidator(QDoubleValidator())
        self._vel_ramp.setMinimumWidth(pid_input_width)
        self._vel_ramp.setSizePolicy(
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
        vel_pid_layout.addWidget(QLabel("Ramp:"))
        vel_pid_layout.addWidget(self._vel_ramp)

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
        self._save_config_btn.setStyleSheet(f"""
            QPushButton {{ background-color: {THEME.blue}; color: {THEME.crust}; }}
            QPushButton:hover {{ background-color: {THEME.sapphire}; }}
            QPushButton:pressed {{ background-color: {THEME.lavender}; }}
        """)
        config_layout.addWidget(self._save_config_btn)

        self._save_pos_btn = QPushButton("Save Robot Position State")
        self._save_pos_btn.setToolTip(
            "Save current encoder positions and config to EEPROM for ALL motors"
        )
        self._save_pos_btn.clicked.connect(self._on_save_robot_state)
        self._save_pos_btn.setStyleSheet(f"""
            QPushButton {{ background-color: {THEME.mauve}; color: {THEME.crust}; }}
            QPushButton:hover {{ background-color: {THEME.pink}; }}
            QPushButton:pressed {{ background-color: {THEME.flamingo}; }}
        """)
        config_layout.addWidget(self._save_pos_btn)

        layout.addLayout(config_layout)

        group.addWidget(content)
        return group

    def _create_target_group(self) -> CollapsibleGroupBox:
        """Create the target control group."""
        group = CollapsibleGroupBox("Target Control")
        content = QWidget()
        layout = QVBoxLayout(content)
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

        layout.addLayout(target_grid)

        action_layout = QHBoxLayout()
        self._set_target_btn = QPushButton("Set Target(s)")
        self._set_target_btn.clicked.connect(self._on_set_target)
        action_layout.addWidget(self._set_target_btn)

        layout.addLayout(action_layout)

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
        self._set_zero_btn.setStyleSheet(f"""
            QPushButton {{ background-color: {THEME.green}; color: {THEME.crust}; }}
            QPushButton:hover {{ background-color: {THEME.teal}; }}
            QPushButton:pressed {{ background-color: {THEME.sky}; }}
        """)
        quick_layout.addWidget(self._set_zero_btn)

        layout.addLayout(quick_layout)

        # Motor config row (Home and Direction)
        motor_config_layout = QHBoxLayout()
        motor_config_layout.setSpacing(SIZES["spacing_small"])

        # Home Position button
        self._home_btn = QPushButton("Home Position")
        self._home_btn.setToolTip("Zero encoder position for selected joint")
        self._home_btn.clicked.connect(self._on_home_position)
        self._home_btn.setStyleSheet(f"""
            QPushButton {{ background-color: {THEME.blue}; color: {THEME.crust}; }}
            QPushButton:hover {{ background-color: {THEME.sapphire}; }}
            QPushButton:pressed {{ background-color: {THEME.lavender}; }}
        """)
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

        # Position Limits row
        limits_layout = QHBoxLayout()
        limits_layout.setSpacing(SIZES["spacing_small"])

        limits_layout.addWidget(QLabel("Min Limit:"))
        self._pos_limit_min = QSpinBox()
        self._pos_limit_min.setRange(-1000000, 1000000)
        self._pos_limit_min.setValue(0)
        self._pos_limit_min.setMinimumWidth(SIZES["input_min_width"])
        limits_layout.addWidget(self._pos_limit_min)

        self._set_min_limit_btn = QPushButton("Set Min")
        self._set_min_limit_btn.clicked.connect(self._on_set_min_limit)
        limits_layout.addWidget(self._set_min_limit_btn)

        limits_layout.addStretch()

        limits_layout.addWidget(QLabel("Max Limit:"))
        self._pos_limit_max = QSpinBox()
        self._pos_limit_max.setRange(-1000000, 1000000)
        self._pos_limit_max.setValue(0)
        self._pos_limit_max.setMinimumWidth(SIZES["input_min_width"])
        limits_layout.addWidget(self._pos_limit_max)

        self._set_max_limit_btn = QPushButton("Set Max")
        self._set_max_limit_btn.clicked.connect(self._on_set_max_limit)
        limits_layout.addWidget(self._set_max_limit_btn)

        layout.addLayout(limits_layout)

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

        # Position Offset row
        offset_layout = QHBoxLayout()
        offset_layout.setSpacing(SIZES["spacing_small"])

        offset_layout.addWidget(QLabel("Set Position As:"))
        self._position_offset_input = QLineEdit("0")
        self._position_offset_input.setValidator(QDoubleValidator())
        self._position_offset_input.setMinimumWidth(SIZES["input_min_width"])
        self._position_offset_input.setToolTip(
            "Set the current physical position to this value.\n"
            "Example: If motor is at +25 ticks and you enter 150,\n"
            "the position will now read 150 ticks.\n"
            "Note: Requires firmware support for 'O' command."
        )
        offset_layout.addWidget(self._position_offset_input)

        self._set_offset_btn = QPushButton("Set Offset")
        self._set_offset_btn.setToolTip("Set current position to the specified value")
        self._set_offset_btn.clicked.connect(self._on_set_position_offset)
        self._set_offset_btn.setStyleSheet(f"""
            QPushButton {{ background-color: {THEME.mauve}; color: {THEME.crust}; }}
            QPushButton:hover {{ background-color: {THEME.pink}; }}
            QPushButton:pressed {{ background-color: {THEME.flamingo}; }}
        """)
        offset_layout.addWidget(self._set_offset_btn)

        offset_layout.addStretch()

        layout.addLayout(offset_layout)

        group.addWidget(content)
        return group

    def _create_step_group(self) -> CollapsibleGroupBox:
        """Create the step/jog input group with timed step support."""
        group = CollapsibleGroupBox("Step/Jog Input")
        content = QWidget()
        layout = QVBoxLayout(content)
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

        group.addWidget(content)
        return group

    def _create_quick_jog_group(self) -> CollapsibleGroupBox:
        """Create quick jog buttons for open-loop PWM control (hold to jog)."""
        group = CollapsibleGroupBox("Quick Jog (Open Loop)")
        content = QWidget()
        layout = QVBoxLayout(content)
        layout.setSpacing(SIZES["spacing_medium"])

        # Info label
        info_label = QLabel("Hold button to jog, release to stop")
        info_label.setStyleSheet(
            f"color: {THEME.subtext0}; font-size: {SIZES['font_small']}pt;"
        )
        layout.addWidget(info_label)

        # Jog buttons row
        btn_layout = QHBoxLayout()
        btn_layout.setSpacing(SIZES["spacing_small"])

        # PWM values as fractions of max PWM (32767)
        jog_values = [(-0.2, "-20%"), (-0.1, "-10%"), (0.1, "+10%"), (0.2, "+20%")]

        for pwm_fraction, label in jog_values:
            btn = QPushButton(label)
            btn.setAutoRepeat(False)  # We handle press/release manually
            # Connect press and release events
            btn.pressed.connect(lambda pf=pwm_fraction: self._on_jog_pressed(pf))
            btn.released.connect(self._on_jog_released)
            btn_layout.addWidget(btn)

        layout.addLayout(btn_layout)

        # Stop button
        self._jog_stop_btn = QPushButton("STOP (PWM=0)")
        self._jog_stop_btn.setStyleSheet(f"""
            QPushButton {{ background-color: {THEME.red}; color: {THEME.crust}; }}
            QPushButton:hover {{ background-color: {THEME.maroon}; }}
            QPushButton:pressed {{ background-color: {THEME.flamingo}; }}
        """)
        self._jog_stop_btn.clicked.connect(self._on_jog_stop)
        layout.addWidget(self._jog_stop_btn)

        group.addWidget(content)
        return group

    def _on_jog_pressed(self, pwm_fraction: float):
        """Handle jog button press - start jogging."""
        joint_id = self._data_store.selected_joint

        # Ensure open-loop mode is active before jogging
        self._serial_handler.set_mode(joint_id, MODE_OPEN_LOOP)

        # Send PWM fraction directly (-1.0 to 1.0); Teensy scales internally
        self._serial_handler.set_target(joint_id, pwm_fraction)
        self._data_store.set_target(joint_id, pwm_fraction)

    def _on_jog_released(self):
        """Handle jog button release - stop jogging."""
        self._on_jog_stop()

    def _on_jog_stop(self):
        """Stop all jogging - set PWM to 0."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.set_target(joint_id, 0)
        self._data_store.set_target(joint_id, 0)

    def _create_sine_group(self) -> CollapsibleGroupBox:
        """Create the sine wave input group."""
        group = CollapsibleGroupBox("Sine Wave Input")
        content = QWidget()
        layout = QVBoxLayout(content)
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

        group.addWidget(content)
        return group

    def _create_self_leveling_group(self) -> CollapsibleGroupBox:
        """Create the self-leveling group."""
        group = CollapsibleGroupBox("Self Leveling")
        content = QWidget()
        layout = QVBoxLayout(content)
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
        self._start_leveling_btn.setStyleSheet(f"""
            QPushButton {{ background-color: {THEME.teal}; color: {THEME.crust}; }}
            QPushButton:hover {{ background-color: {THEME.green}; }}
            QPushButton:pressed {{ background-color: {THEME.sky}; }}
        """)
        self._start_leveling_btn.clicked.connect(self._on_start_leveling)
        btn_layout.addWidget(self._start_leveling_btn)

        self._stop_leveling_btn = QPushButton("Stop Leveling")
        self._stop_leveling_btn.clicked.connect(self._on_stop_leveling)
        btn_layout.addWidget(self._stop_leveling_btn)

        layout.addLayout(btn_layout)

        group.addWidget(content)
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

    NUM_STORED_SLOTS = 6

    def _create_stored_sequences_group(self) -> CollapsibleGroupBox:
        group = CollapsibleGroupBox("Stored Sequences")
        content = QWidget()
        grid = QGridLayout(content)
        grid.setSpacing(SIZES["spacing_small"])
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 0)

        self._stored_seq_paths: list[str | None] = [None] * self.NUM_STORED_SLOTS
        self._stored_seq_choose_btns: list[QPushButton] = []
        self._stored_seq_play_btns: list[QPushButton] = []

        for i in range(self.NUM_STORED_SLOTS):
            choose_btn = QPushButton(f"Slot {i + 1}: Empty")
            choose_btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {THEME.surface0};
                    color: {THEME.subtext0};
                    border: 1px dashed {THEME.surface2};
                    text-align: left;
                    padding: 4px 8px;
                }}
                QPushButton:hover {{ background-color: {THEME.surface1}; }}
                QPushButton:pressed {{ background-color: {THEME.surface2}; }}
            """)
            choose_btn.clicked.connect(lambda _, idx=i: self._on_choose_sequence(idx))
            grid.addWidget(choose_btn, i, 0)
            self._stored_seq_choose_btns.append(choose_btn)

            play_btn = QPushButton("▶")
            play_btn.setEnabled(False)
            play_btn.setFixedWidth(scaled(36))
            play_btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {THEME.green};
                    color: {THEME.crust};
                    font-weight: bold;
                    border-radius: 3px;
                }}
                QPushButton:hover {{ background-color: {THEME.teal}; }}
                QPushButton:pressed {{ background-color: {THEME.sky}; }}
                QPushButton:disabled {{ background-color: {THEME.surface1}; color: {THEME.overlay0}; }}
            """)
            play_btn.clicked.connect(
                lambda _, idx=i: self._on_play_stored_sequence(idx)
            )
            grid.addWidget(play_btn, i, 1)
            self._stored_seq_play_btns.append(play_btn)

        group.addWidget(content)
        return group

    def _on_choose_sequence(self, slot_idx: int):
        from PyQt6.QtWidgets import QFileDialog

        path, _ = QFileDialog.getOpenFileName(
            self,
            f"Choose Sequence for Slot {slot_idx + 1}",
            "",
            "JSON Files (*.json);;All Files (*)",
            options=QFileDialog.Option.DontUseNativeDialog,
        )
        if not path:
            return
        self._stored_seq_paths[slot_idx] = path
        name = os.path.basename(path).replace(".json", "")
        btn = self._stored_seq_choose_btns[slot_idx]
        btn.setText(f"Slot {slot_idx + 1}: {name}")
        btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {THEME.surface0};
                color: {THEME.text};
                border: 1px solid {THEME.surface2};
                text-align: left;
                padding: 4px 8px;
            }}
            QPushButton:hover {{ background-color: {THEME.surface1}; }}
            QPushButton:pressed {{ background-color: {THEME.surface2}; }}
        """)
        self._stored_seq_play_btns[slot_idx].setEnabled(True)

    def _on_play_stored_sequence(self, slot_idx: int):
        from PyQt6.QtWidgets import QMessageBox

        path = self._stored_seq_paths[slot_idx]
        if not path or not os.path.exists(path):
            return

        try:
            with open(path, "r") as f:
                data = json.load(f)
        except (json.JSONDecodeError, IOError):
            QMessageBox.warning(self, "Load Error", f"Failed to read {path}")
            return

        keyframes_data = data if isinstance(data, list) else data.get("keyframes", [])
        if not keyframes_data:
            QMessageBox.warning(self, "Empty Sequence", "No keyframes found in file.")
            return

        from .sequence_editor import Keyframe, NUM_MOTORS

        keyframes = [Keyframe.from_dict(d) for d in keyframes_data]

        self._serial_handler.enter_sequence_mode(True)
        for idx, kf in enumerate(keyframes):
            targets = [t if t is not None else 0.0 for t in kf.targets]
            active = [t is not None for t in kf.targets]
            durations = [
                kf.motor_durations[i]
                if kf.motor_durations[i] is not None
                else kf.duration_ms
                for i in range(NUM_MOTORS)
            ]
            self._serial_handler.send_keyframe(
                idx, targets, active, durations, kf.relative
            )

        self._serial_handler.seq_auto_run(True)

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

    def _on_home_position(self):
        """Send home/zero position command."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.home_position(joint_id)
        # Also reset target input to 0
        self._target_input.setText("0")
        self._data_store.set_target(joint_id, 0)

    def _on_toggle_direction(self):
        """Toggle motor direction for selected joint."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.toggle_direction(joint_id)

    def _on_toggle_encoder_direction(self):
        """Toggle encoder direction for selected joint."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.toggle_encoder_direction(joint_id)

    def _on_set_position_offset(self):
        """Set the current position to a specified value (position offset)."""
        desired_position = self._get_float_from_lineedit(
            self._position_offset_input, 0.0
        )
        joint_id = self._data_store.selected_joint

        # Send offset command to Teensy
        # The Teensy will calculate: offset = desired_position - current_raw_position
        self._serial_handler.set_position_offset(joint_id, desired_position)

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

    def _on_set_mode(self, mode: int):
        """Send mode set command and update local unit labels.

        The mode banner is NOT updated here — it only updates when the robot
        confirms the mode via a CONFIG echo (_on_config_updated).
        """
        joint_id = self._data_store.selected_joint
        self._serial_handler.set_mode(joint_id, mode)

        # Update local tracking for unit labels (send-side bookkeeping only).
        # Do NOT set data_store.control_mode here — that is set exclusively from
        # incoming telemetry so the mode banner only reflects what the robot confirms.
        self._current_mode = mode
        unit = MODE_UNITS.get(mode, "")
        self._target_unit_label.setText(unit)
        self._target_input_unit_label.setText(unit)
        self._step_unit_label.setText(unit)
        self._sine_amplitude_unit_label.setText(unit)
        self._mode_indicator_label.setText(MODE_NAMES.get(mode, "Unknown"))
        if mode == MODE_POSITION:
            self._error_unit_label.setText("ticks")
        elif mode == MODE_VELOCITY:
            self._error_unit_label.setText("units/s")
        else:
            self._error_unit_label.setText("")

        # Mark banner as pending confirmation from robot
        self._set_mode_banner_pending(mode)
        self.mode_changed.emit(mode)

    def _on_set_pos_pid(self):
        """Send position PID gains and feed-forward."""
        joint_id = self._data_store.selected_joint
        p = self._get_float_from_lineedit(self._pos_p)
        i = self._get_float_from_lineedit(self._pos_i)
        d = self._get_float_from_lineedit(self._pos_d)
        ff = self._get_float_from_lineedit(self._pos_ff)
        lpf = self._get_float_from_lineedit(self._pos_lpf, 1.0)
        ramp = self._get_float_from_lineedit(self._pos_ramp, 0.0)

        self._serial_handler.set_pid(joint_id, "P", p)
        self._serial_handler.set_pid(joint_id, "I", i)
        self._serial_handler.set_pid(joint_id, "D", d)
        self._serial_handler.set_feed_forward(joint_id, "F", ff)
        self._serial_handler.set_pos_lpf(joint_id, lpf)
        self._serial_handler.set_pos_ramp_rate(joint_id, ramp)

    def _on_set_vel_pid(self):
        """Send velocity PID gains and feed-forward."""
        joint_id = self._data_store.selected_joint
        p = self._get_float_from_lineedit(self._vel_p)
        i = self._get_float_from_lineedit(self._vel_i)
        d = self._get_float_from_lineedit(self._vel_d)
        ff = self._get_float_from_lineedit(self._vel_ff)
        lpf = self._get_float_from_lineedit(self._vel_lpf, 1.0)
        ramp = self._get_float_from_lineedit(self._vel_ramp, 0.0)

        self._serial_handler.set_pid(joint_id, "p", p)
        self._serial_handler.set_pid(joint_id, "i", i)
        self._serial_handler.set_pid(joint_id, "d", d)
        self._serial_handler.set_feed_forward(joint_id, "f", ff)
        self._serial_handler.set_vel_lpf(joint_id, lpf)
        self._serial_handler.set_vel_ramp_rate(joint_id, ramp)

    def _on_set_input_lpf(self):
        """Send motor input LPF alpha."""
        joint_id = self._data_store.selected_joint
        lpf = self._get_float_from_lineedit(self._input_lpf, 0.5)

        self._serial_handler.set_input_lpf(joint_id, lpf)

    def _on_load_config(self):
        """Request PID configuration from EEPROM."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.get_config(joint_id)

    def _on_save_config(self):
        """Save current PID configuration to EEPROM."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.save_config(joint_id)

    def _on_save_robot_state(self):
        """Save current position and configuration for ALL motors (K0)."""
        self._serial_handler.save_config(0)

    def _on_set_min_limit(self):
        """Set the minimum position limit."""
        joint_id = self._data_store.selected_joint
        limit = self._pos_limit_min.value()
        self._serial_handler.set_pos_limit_min(joint_id, limit)

    def _on_set_max_limit(self):
        """Set the maximum position limit."""
        joint_id = self._data_store.selected_joint
        limit = self._pos_limit_max.value()
        self._serial_handler.set_pos_limit_max(joint_id, limit)

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
                self._pos_ramp.setText(f"{config.pos_max_ramp_rate:g}")

                self._vel_p.setText(f"{config.vel_p:g}")
                self._vel_i.setText(f"{config.vel_i:g}")
                self._vel_d.setText(f"{config.vel_d:g}")
                self._vel_ff.setText(f"{config.vel_ff:g}")
                self._vel_lpf.setText(f"{config.vel_lpf_alpha:g}")
                self._vel_ramp.setText(f"{config.vel_max_ramp_rate:g}")

                self._input_lpf.setText(f"{config.input_lpf_alpha:g}")

                # Update limits
                self._pos_limit_min.setValue(config.pos_limit_min)
                self._pos_limit_max.setValue(config.pos_limit_max)

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

    def _get_step_center(self, joint_data) -> float:
        """Return the current actual value to center a step around, based on mode."""
        if self._current_mode == MODE_POSITION:
            return joint_data.current_position
        elif self._current_mode == MODE_VELOCITY:
            return joint_data.current_velocity
        else:
            return 0.0  # Open loop: steps are absolute PWM offsets from 0

    def _execute_timed_step(self, amplitude: float):
        """Execute a timed step: send center+amplitude, wait duration, return to center."""
        duration = self._get_float_from_lineedit(self._step_duration, 0.5)
        joint_id = self._data_store.selected_joint

        # Capture the current actual position/velocity as the center to step around
        joint_data = self._data_store.get_selected_joint_data()
        self._step_center = self._get_step_center(joint_data)
        target = self._step_center + amplitude

        # Update primary target input display
        self._target_input.setText(f"{target:.4g}")

        # Send step command to primary joint
        self._serial_handler.set_target(joint_id, target)
        self._data_store.set_target(joint_id, target)

        # Start timer to return to center
        self._step_timer.start(int(duration * 1000))

    def _on_step_complete(self):
        """Handle step completion - return to pre-step center."""
        joint_id = self._data_store.selected_joint
        center = getattr(self, "_step_center", 0.0)
        self._serial_handler.set_target(joint_id, center)
        self._target_input.setText(f"{center:.4g}")
        self._data_store.set_target(joint_id, center)

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
