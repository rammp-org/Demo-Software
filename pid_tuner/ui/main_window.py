"""
Main application window for PID Tuner.
"""

from PyQt6.QtWidgets import (
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QSplitter,
    QGroupBox,
    QLabel,
    QComboBox,
    QPushButton,
    QStatusBar,
    QMessageBox,
    QSizePolicy,
    QFrame,
    QTabWidget,
    QCheckBox,
)
from PyQt6.QtCore import Qt, QTimer, QSettings
from PyQt6.QtGui import QShortcut, QKeySequence

from pid_tuner.data.data_store import DataStore
from pid_tuner.data.joint_config import (
    get_joint_names,
    get_joint_id_from_index,
    get_joint_info,
)
from pid_tuner.serial_driver.serial_handler import SerialHandler
from pid_tuner.ui.plot_widget import PlotWidget
from pid_tuner.ui.sequence_plotter import SequencePlotter  # pyright: ignore[reportMissingImports]
from pid_tuner.ui.control_panel import ControlPanel
from pid_tuner.ui.config_viewer import ConfigViewerWidget
from pid_tuner.ui.serial_console import SerialConsole
from pid_tuner.ui.state_indicator import StateIndicator
from pid_tuner.ui.encoder_overview import EncoderOverview
from pid_tuner.ui.drive_wheel_display import DriveWheelDisplay
from pid_tuner.ui.sequence_editor import SequenceEditor
from pid_tuner.ui.theme import get_application_stylesheet, THEME
from pid_tuner.ui.scaling import SIZES, scaled


class MainWindow(QMainWindow):
    """
    Main window for the PID Tuner application.
    """

    DEFAULT_BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
    DEFAULT_BAUD_RATE = 115200

    def __init__(self):
        super().__init__()

        # Initialize settings storage
        self._settings = QSettings("MEBot", "PIDTuner")

        # Create core components
        self._data_store = DataStore()
        self._serial_handler = SerialHandler()
        self._keepalive_timer = QTimer(self)
        self._keepalive_timer.setInterval(400)
        self._keepalive_timer.timeout.connect(self._on_keepalive_tick)

        # Connect serial signals
        self._serial_handler.data_received.connect(self._on_data_received)
        self._serial_handler.config_received.connect(self._data_store.set_config)
        self._serial_handler.raw_lines_received.connect(self._on_raw_lines_received)
        self._serial_handler.connection_changed.connect(self._on_connection_changed)
        self._serial_handler.error_occurred.connect(self._on_error)

        # Forward sequence status from serial to data_store
        self._serial_handler.seq_status_received.connect(
            self._data_store.seq_status_updated
        )

        # Apply Catppuccin theme
        self.setStyleSheet(get_application_stylesheet())

        self._setup_ui()
        self._setup_status_bar()

        # Restore saved settings (or default to maximized)
        self._restore_settings()

        # Refresh ports on startup
        self._refresh_ports()

    def _setup_ui(self):
        """Set up the main window UI."""
        self.setWindowTitle("PID Tuner for MEBot/RAMMP")
        # Use scaled minimum size for better laptop support
        self.setMinimumSize(SIZES["window_min_width"], SIZES["window_min_height"])

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(SIZES["spacing_small"])
        main_layout.setContentsMargins(
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
        )

        # EStop bar - always visible at the very top
        main_layout.addWidget(self._create_estop_bar())

        # Top bar - connection, joint selection, and state indicator
        main_layout.addWidget(self._create_top_bar())

        self._top_splitter = QSplitter(Qt.Orientation.Horizontal)

        overview_widget = QWidget()
        overview_layout = QHBoxLayout(overview_widget)
        overview_layout.setContentsMargins(0, 0, 0, 0)
        overview_layout.setSpacing(SIZES["spacing_medium"])

        self._encoder_overview = EncoderOverview(self._data_store, self._serial_handler)
        self._encoder_overview.joint_selected.connect(self._on_encoder_bar_clicked)
        overview_layout.addWidget(self._encoder_overview, stretch=1)

        self._drive_wheel_display = DriveWheelDisplay(
            self._data_store, self._serial_handler
        )
        overview_layout.addWidget(self._drive_wheel_display)

        self._top_splitter.addWidget(overview_widget)

        self._serial_console = SerialConsole()
        self._serial_console.command_sent.connect(self._serial_handler.send_raw)
        self._top_splitter.addWidget(self._serial_console)

        self._top_splitter.setSizes([100, 100])

        main_layout.addWidget(self._top_splitter)

        main_splitter = QSplitter(Qt.Orientation.Horizontal)
        self._main_splitter = main_splitter

        self._left_tabs = QTabWidget()
        self._left_tabs.setStyleSheet(f"""
            QTabWidget::pane {{
                border: 1px solid {THEME.surface1};
                background-color: {THEME.base};
            }}
            QTabBar::tab {{
                background-color: {THEME.surface0};
                color: {THEME.subtext1};
                padding: 5px 14px;
                border: 1px solid {THEME.surface1};
                border-bottom: none;
                border-top-left-radius: 4px;
                border-top-right-radius: 4px;
            }}
            QTabBar::tab:selected {{
                background-color: {THEME.base};
                color: {THEME.text};
                font-weight: bold;
            }}
            QTabBar::tab:hover:!selected {{
                background-color: {THEME.surface1};
            }}
        """)

        self._plot_widget = PlotWidget(self._data_store)
        self._left_tabs.addTab(self._plot_widget, "Live Plot")

        self._sequence_plotter = SequencePlotter(self._data_store)
        self._left_tabs.addTab(self._sequence_plotter, "Sequence Plotter")

        main_splitter.addWidget(self._left_tabs)

        self._right_tabs = QTabWidget()
        self._right_tabs.setStyleSheet(self._left_tabs.styleSheet())

        self._control_panel = ControlPanel(
            self._data_store, self._serial_handler, self._settings
        )
        self._control_panel.mode_changed.connect(
            self._encoder_overview.set_mode_for_all
        )
        self._right_tabs.addTab(self._control_panel, "Controls")

        self._sequence_editor = SequenceEditor(self._data_store, self._serial_handler)
        self._right_tabs.addTab(self._sequence_editor, "Sequences")

        self._config_viewer = ConfigViewerWidget(self._data_store, self._serial_handler)
        self._right_tabs.addTab(self._config_viewer, "Config")

        self._right_tabs.setMinimumWidth(SIZES["control_panel_min_width"])
        self._right_tabs.setSizePolicy(
            QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding
        )
        main_splitter.addWidget(self._right_tabs)

        # Set initial splitter sizes - give more to plot area
        # These are relative weights, will be adjusted based on actual window size
        main_splitter.setSizes([600, SIZES["control_panel_preferred_width"]])
        main_splitter.setStretchFactor(0, 2)  # Plot area stretches more
        main_splitter.setStretchFactor(1, 1)  # Control panel stretches less

        main_layout.addWidget(main_splitter)

    def _create_estop_bar(self) -> QFrame:
        """Create the always-visible EStop bar at the top of the window."""
        btn_height = scaled(32)

        bar = QFrame()
        bar.setStyleSheet(f"""
            QFrame {{
                background-color: {THEME.surface0};
                border-bottom: 2px solid {THEME.red};
            }}
        """)
        bar.setFixedHeight(btn_height + SIZES["margin_small"] * 2)

        layout = QHBoxLayout(bar)
        layout.setContentsMargins(
            SIZES["margin_medium"],
            SIZES["margin_small"],
            SIZES["margin_medium"],
            SIZES["margin_small"],
        )
        layout.setSpacing(SIZES["spacing_medium"])

        estop_btn = QPushButton("ESTOP  (z)")
        estop_btn.setFixedHeight(btn_height)
        estop_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {THEME.red};
                color: {THEME.crust};
                font-weight: bold;
                font-size: {SIZES["font_large"]}pt;
                border-radius: {scaled(4)}px;
                padding: 0 {SIZES["margin_medium"]}px;
            }}
            QPushButton:hover {{
                background-color: {THEME.maroon};
            }}
            QPushButton:pressed {{
                background-color: {THEME.flamingo};
            }}
        """)
        estop_btn.clicked.connect(self._on_estop)
        layout.addWidget(estop_btn)

        clear_btn = QPushButton("Clear ESTOP  (c)")
        clear_btn.setFixedHeight(btn_height)
        clear_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {THEME.peach};
                color: {THEME.crust};
                font-weight: bold;
                font-size: {SIZES["font_large"]}pt;
                border-radius: {scaled(4)}px;
                padding: 0 {SIZES["margin_medium"]}px;
            }}
            QPushButton:hover {{
                background-color: {THEME.yellow};
            }}
            QPushButton:pressed {{
                background-color: {THEME.rosewater};
            }}
        """)
        clear_btn.clicked.connect(self._on_clear_estop)
        layout.addWidget(clear_btn)

        self._keepalive_checkbox = QCheckBox("Keep Alive")
        self._keepalive_checkbox.setStyleSheet(f"""
            QCheckBox {{
                color: {THEME.text};
                font-weight: bold;
                font-size: {SIZES["font_large"]}pt;
                spacing: {scaled(6)}px;
            }}
            QCheckBox::indicator {{
                width: {scaled(18)}px;
                height: {scaled(18)}px;
                border: 1px solid {THEME.surface2};
                border-radius: {scaled(3)}px;
                background-color: {THEME.base};
            }}
            QCheckBox::indicator:checked {{
                background-color: {THEME.green};
                border-color: {THEME.green};
            }}
        """)
        self._keepalive_checkbox.toggled.connect(self._on_keepalive_toggled)
        layout.addWidget(self._keepalive_checkbox)

        layout.addStretch()

        # Keyboard shortcuts
        self._estop_shortcut = QShortcut(QKeySequence("z"), self)
        self._estop_shortcut.activated.connect(self._on_estop)
        self._clear_estop_shortcut = QShortcut(QKeySequence("c"), self)
        self._clear_estop_shortcut.activated.connect(self._on_clear_estop)

        return bar

    def _on_estop(self):
        """Send emergency stop command."""
        self._serial_handler.disable_motors()

    def _on_clear_estop(self):
        """Send clear ESTOP command."""
        self._serial_handler.clear_estop()

    def _on_keepalive_toggled(self, enabled: bool):
        if enabled:
            self._keepalive_timer.start()
        else:
            self._keepalive_timer.stop()

    def _on_keepalive_tick(self):
        if not self._serial_handler.is_connected:
            return
        if self._data_store.current_state != 4:
            self._serial_handler.clear_estop()

    def _create_top_bar(self) -> QWidget:
        """Create the top control bar."""
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(SIZES["spacing_medium"])

        # Serial connection group
        serial_group = QGroupBox("Serial Connection")
        serial_layout = QHBoxLayout(serial_group)
        serial_layout.setSpacing(SIZES["spacing_small"])

        # Port selection
        serial_layout.addWidget(QLabel("Port:"))
        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(scaled(120))
        self._port_combo.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        serial_layout.addWidget(self._port_combo)

        # Refresh button
        self._refresh_btn = QPushButton("Refresh")
        self._refresh_btn.clicked.connect(self._refresh_ports)
        serial_layout.addWidget(self._refresh_btn)

        # Baud rate
        serial_layout.addWidget(QLabel("Baud:"))
        self._baud_combo = QComboBox()
        for baud in self.DEFAULT_BAUD_RATES:
            self._baud_combo.addItem(str(baud))
        self._baud_combo.setCurrentText(str(self.DEFAULT_BAUD_RATE))
        serial_layout.addWidget(self._baud_combo)

        # Connect/Disconnect buttons
        self._connect_btn = QPushButton("Connect")
        self._connect_btn.clicked.connect(self._on_connect)
        serial_layout.addWidget(self._connect_btn)

        self._disconnect_btn = QPushButton("Disconnect")
        self._disconnect_btn.clicked.connect(self._on_disconnect)
        self._disconnect_btn.setEnabled(False)
        serial_layout.addWidget(self._disconnect_btn)

        layout.addWidget(serial_group)

        # Joint selection group
        joint_group = QGroupBox("Joint Selection")
        joint_layout = QHBoxLayout(joint_group)
        joint_layout.setSpacing(SIZES["spacing_small"])

        joint_layout.addWidget(QLabel("Joint:"))
        self._joint_combo = QComboBox()
        for name in get_joint_names():
            self._joint_combo.addItem(name)
        self._joint_combo.currentIndexChanged.connect(self._on_joint_changed)
        self._joint_combo.setMinimumWidth(scaled(120))
        self._joint_combo.setSizePolicy(
            QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed
        )
        joint_layout.addWidget(self._joint_combo)

        # Joint description
        self._joint_desc_label = QLabel("")
        self._joint_desc_label.setStyleSheet(
            f"color: gray; font-style: italic; font-size: {SIZES['font_small']}pt;"
        )
        joint_layout.addWidget(self._joint_desc_label)
        self._update_joint_description()

        layout.addWidget(joint_group)

        layout.addStretch()

        # State indicator (right side of top bar)
        self._state_indicator = StateIndicator()
        self._data_store.state_changed.connect(self._state_indicator.set_state)
        layout.addWidget(self._state_indicator)

        return widget

    def _setup_status_bar(self):
        """Set up the status bar."""
        self._status_bar = QStatusBar()
        self.setStatusBar(self._status_bar)

        # Connection status indicator
        self._connection_label = QLabel("Disconnected")
        self._connection_label.setStyleSheet("color: red;")
        self._status_bar.addPermanentWidget(self._connection_label)

        # Data rate indicator
        self._data_rate_label = QLabel("0 Hz")
        self._status_bar.addPermanentWidget(self._data_rate_label)

        # Data rate calculation
        self._data_count = 0
        self._rate_timer = QTimer(self)
        self._rate_timer.timeout.connect(self._update_data_rate)
        self._rate_timer.start(1000)  # Update every second

    def _refresh_ports(self):
        """Refresh the list of available serial ports."""
        ports = SerialHandler.list_available_ports()
        self._port_combo.clear()
        self._port_combo.addItems(ports)

        if not ports:
            self._status_bar.showMessage("No serial ports found", 3000)

    def _on_connect(self):
        """Handle connect button click."""
        port = self._port_combo.currentText()
        if not port:
            QMessageBox.warning(self, "Error", "Please select a serial port")
            return

        baud_rate = int(self._baud_combo.currentText())

        if self._serial_handler.connect(port, baud_rate):
            self._status_bar.showMessage(f"Connected to {port}", 3000)
        else:
            QMessageBox.warning(self, "Error", f"Failed to connect to {port}")

    def _on_disconnect(self):
        """Handle disconnect button click."""
        self._serial_handler.disconnect_port()
        self._status_bar.showMessage("Disconnected", 3000)

    def _on_connection_changed(self, connected: bool):
        """Handle connection state change."""
        self._connect_btn.setEnabled(not connected)
        self._disconnect_btn.setEnabled(connected)
        self._port_combo.setEnabled(not connected)
        self._baud_combo.setEnabled(not connected)
        self._refresh_btn.setEnabled(not connected)

        if connected:
            self._connection_label.setText("Connected")
            self._connection_label.setStyleSheet("color: green;")
            # Turn off simulation mode when connected to real device
            self._data_store.simulation_mode = False
            self._plot_widget.set_simulation_mode(False)
            QTimer.singleShot(500, self._autoload_config)
        else:
            self._connection_label.setText("Disconnected")
            self._connection_label.setStyleSheet("color: red;")

    def _autoload_config(self):
        if not self._serial_handler.is_connected:
            return
        self._config_viewer._on_load_all()

    def _on_data_received(self, data):
        """Handle incoming encoder data."""
        self._data_store.process_encoder_data(data)
        self._data_count += 1

    def _on_raw_lines_received(self, lines: list):
        """Handle batched raw serial lines for console display."""
        self._serial_console.append_lines(lines)

    def _on_error(self, error_msg: str):
        """Handle serial error."""
        self._status_bar.showMessage(f"Error: {error_msg}", 5000)

    def _on_joint_changed(self, index: int):
        """Handle joint selection change."""
        joint_id = get_joint_id_from_index(index)
        self._data_store.selected_joint = joint_id
        self._update_joint_description()

        # Sync encoder overview selection
        self._encoder_overview.set_selected_joint(joint_id)

        # Clear plot data for new joint
        self._data_store.clear_joint(joint_id)

    def _on_encoder_bar_clicked(self, joint_id: int):
        """Handle encoder bar click to select joint."""
        # Update joint combo box (0-indexed)
        self._joint_combo.setCurrentIndex(joint_id - 1)

    def _update_joint_description(self):
        """Update the joint description label."""
        joint_id = get_joint_id_from_index(self._joint_combo.currentIndex())
        joint_info = get_joint_info(joint_id)
        self._joint_desc_label.setText(joint_info.description)

    def _update_data_rate(self):
        """Update the data rate display."""
        rate = self._data_count
        self._data_count = 0
        self._data_rate_label.setText(f"{rate} Hz")

    def _restore_settings(self):
        """Restore saved window configuration."""
        # Window geometry
        geometry = self._settings.value("geometry")
        if geometry:
            self.restoreGeometry(geometry)
        else:
            # Default to maximized if no saved geometry
            self.showMaximized()

        # Window state (maximized/minimized)
        state = self._settings.value("windowState")
        if state:
            self.restoreState(state)

        # Splitter sizes
        main_sizes = self._settings.value("main_splitter")
        if main_sizes:
            try:
                self._main_splitter.setSizes([int(s) for s in main_sizes])
            except (TypeError, ValueError):
                pass

        top_sizes = self._settings.value("top_splitter")
        if top_sizes:
            try:
                self._top_splitter.setSizes([int(s) for s in top_sizes])
            except (TypeError, ValueError):
                pass

        # Last serial port
        last_port = self._settings.value("last_port", "")
        if last_port:
            index = self._port_combo.findText(last_port)
            if index >= 0:
                self._port_combo.setCurrentIndex(index)

        # Last baud rate
        last_baud = self._settings.value("last_baud", "115200")
        self._baud_combo.setCurrentText(str(last_baud))

        # Last joint selection
        last_joint = self._settings.value("last_joint", 0, type=int)
        if 0 <= last_joint < self._joint_combo.count():
            self._joint_combo.setCurrentIndex(last_joint)

    def _save_settings(self):
        """Save current window configuration."""
        self._settings.setValue("geometry", self.saveGeometry())
        self._settings.setValue("windowState", self.saveState())
        self._settings.setValue("main_splitter", self._main_splitter.sizes())
        self._settings.setValue("top_splitter", self._top_splitter.sizes())
        self._settings.setValue("last_port", self._port_combo.currentText())
        self._settings.setValue("last_baud", self._baud_combo.currentText())
        self._settings.setValue("last_joint", self._joint_combo.currentIndex())

    def closeEvent(self, a0):
        self._save_settings()
        self._drive_wheel_display.shutdown()
        self._serial_handler.disconnect_port()
        if a0 is not None:
            a0.accept()
