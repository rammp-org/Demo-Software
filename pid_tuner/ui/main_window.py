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
)
from PyQt6.QtCore import Qt, QTimer

from ..data.data_store import DataStore
from ..data.joint_config import get_joint_names, get_joint_id_from_index, get_joint_info
from ..serial.serial_handler import SerialHandler
from .plot_widget import PlotWidget
from .control_panel import ControlPanel
from .serial_console import SerialConsole


class MainWindow(QMainWindow):
    """
    Main window for the PID Tuner application.
    """

    DEFAULT_BAUD_RATES = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
    DEFAULT_BAUD_RATE = 115200

    def __init__(self):
        super().__init__()

        # Create core components
        self._data_store = DataStore()
        self._serial_handler = SerialHandler()

        # Connect serial signals
        self._serial_handler.data_received.connect(self._on_data_received)
        self._serial_handler.raw_line_received.connect(self._on_raw_line_received)
        self._serial_handler.connection_changed.connect(self._on_connection_changed)
        self._serial_handler.error_occurred.connect(self._on_error)

        self._setup_ui()
        self._setup_status_bar()

        # Refresh ports on startup
        self._refresh_ports()

    def _setup_ui(self):
        """Set up the main window UI."""
        self.setWindowTitle("PID Tuner for MEBot/RAMMP")
        self.setMinimumSize(1200, 800)

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        main_layout = QVBoxLayout(central_widget)

        # Top bar - connection and joint selection
        main_layout.addWidget(self._create_top_bar())

        # Main content area with vertical splitter (plot+console on left, controls on right)
        main_splitter = QSplitter(Qt.Orientation.Horizontal)

        # Left side - Plot and Console in vertical splitter
        left_splitter = QSplitter(Qt.Orientation.Vertical)

        # Plot widget
        self._plot_widget = PlotWidget(self._data_store)
        left_splitter.addWidget(self._plot_widget)

        # Serial console
        self._serial_console = SerialConsole()
        left_splitter.addWidget(self._serial_console)

        # Set plot to take 70% of vertical space, console 30%
        left_splitter.setSizes([500, 200])

        main_splitter.addWidget(left_splitter)

        # Right side - Control panel
        self._control_panel = ControlPanel(self._data_store, self._serial_handler)
        self._control_panel.setMaximumWidth(400)
        main_splitter.addWidget(self._control_panel)

        # Set initial splitter sizes (70% left, 30% controls)
        main_splitter.setSizes([700, 300])

        main_layout.addWidget(main_splitter)

    def _create_top_bar(self) -> QWidget:
        """Create the top control bar."""
        widget = QWidget()
        layout = QHBoxLayout(widget)
        layout.setContentsMargins(0, 0, 0, 0)

        # Serial connection group
        serial_group = QGroupBox("Serial Connection")
        serial_layout = QHBoxLayout(serial_group)

        # Port selection
        serial_layout.addWidget(QLabel("Port:"))
        self._port_combo = QComboBox()
        self._port_combo.setMinimumWidth(150)
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

        joint_layout.addWidget(QLabel("Joint:"))
        self._joint_combo = QComboBox()
        for name in get_joint_names():
            self._joint_combo.addItem(name)
        self._joint_combo.currentIndexChanged.connect(self._on_joint_changed)
        self._joint_combo.setMinimumWidth(200)
        joint_layout.addWidget(self._joint_combo)

        # Joint description
        self._joint_desc_label = QLabel("")
        self._joint_desc_label.setStyleSheet("color: gray; font-style: italic;")
        joint_layout.addWidget(self._joint_desc_label)
        self._update_joint_description()

        layout.addWidget(joint_group)

        layout.addStretch()

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
        self._serial_handler.disconnect()
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
        else:
            self._connection_label.setText("Disconnected")
            self._connection_label.setStyleSheet("color: red;")

    def _on_data_received(self, data):
        """Handle incoming encoder data."""
        self._data_store.process_encoder_data(data)
        self._data_count += 1

    def _on_raw_line_received(self, line: str):
        """Handle raw serial line for console display."""
        self._serial_console.append_line(line)

    def _on_error(self, error_msg: str):
        """Handle serial error."""
        self._status_bar.showMessage(f"Error: {error_msg}", 5000)

    def _on_joint_changed(self, index: int):
        """Handle joint selection change."""
        joint_id = get_joint_id_from_index(index)
        self._data_store.selected_joint = joint_id
        self._update_joint_description()

        # Clear plot data for new joint
        self._data_store.clear_joint(joint_id)

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

    def closeEvent(self, event):
        """Handle window close event."""
        self._serial_handler.disconnect()
        event.accept()
