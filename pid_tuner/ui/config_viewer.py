"""
Config Viewer Widget for displaying all motor configurations in a table.

This provides a debug view showing the full MotorConfig struct for all 6 joints,
allowing comparison and verification of configuration values.
"""

from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QTableWidget,
    QTableWidgetItem,
    QPushButton,
    QLabel,
    QHeaderView,
    QGroupBox,
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QColor, QBrush

from ..data.data_store import DataStore
from ..serial_driver.serial_handler import SerialHandler
from ..data.joint_config import JOINTS
from .theme import THEME
from .scaling import SIZES, scaled


# Column definitions for the config table.
# Columns with attr starting with "_dir_" are sourced from DataStore direction lists,
# not from ConfigData, and are handled separately in _on_directions_updated.
CONFIG_COLUMNS = [
    ("Joint", "joint"),
    ("mot_dir", "_dir_motor"),
    ("enc_dir", "_dir_encoder"),
    ("pos_p", "pos_p"),
    ("pos_i", "pos_i"),
    ("pos_d", "pos_d"),
    ("pos_ff", "pos_ff"),
    ("vel_p", "vel_p"),
    ("vel_i", "vel_i"),
    ("vel_d", "vel_d"),
    ("vel_ff", "vel_ff"),
    ("pos_lpf", "pos_lpf_alpha"),
    ("vel_lpf", "vel_lpf_alpha"),
    ("in_lpf", "input_lpf_alpha"),
    ("pos_min", "pos_limit_min"),
    ("pos_max", "pos_limit_max"),
]

# Columns that come from DataStore direction lists rather than ConfigData
_DIRECTION_ATTRS = {"_dir_motor", "_dir_encoder"}

# Colors for direction values
_DIR_POS_COLOR = QColor("#a6e3a1")  # green  — forward (+1)
_DIR_NEG_COLOR = QColor("#f38ba8")  # red    — reversed (-1)


class ConfigViewerWidget(QWidget):
    """
    Tabular debug view showing all motor configurations.
    Displays the full MotorConfig struct for all 6 joints.
    """

    HIGHLIGHT_COLOR = QColor("#f9e2af")  # Catppuccin Yellow for highlighting changes
    NORMAL_BG = QColor(THEME.surface0)
    HEADER_BG = QColor(THEME.surface1)

    def __init__(
        self, data_store: DataStore, serial_handler: SerialHandler, parent=None
    ):
        super().__init__(parent)
        self._data_store = data_store
        self._serial_handler = serial_handler

        # Store previous values to detect changes
        self._previous_values: dict[int, dict[str, float]] = {}

        # Timer for sequential config loading
        self._load_timer = QTimer(self)
        self._load_timer.timeout.connect(self._load_next_config)
        self._load_queue: list[int] = []

        self._setup_ui()

        # Connect to config and direction updates
        self._data_store.config_updated.connect(self._on_config_updated)
        self._data_store.directions_updated.connect(self._on_directions_updated)

    def _setup_ui(self):
        """Set up the widget layout."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(SIZES["spacing_small"])

        # Create group box
        group = QGroupBox("Configuration Viewer")
        group_layout = QVBoxLayout(group)
        group_layout.setSpacing(SIZES["spacing_small"])

        # Header with buttons
        header_layout = QHBoxLayout()
        header_layout.setSpacing(SIZES["spacing_small"])

        # Info label
        self._info_label = QLabel("View all motor configurations")
        self._info_label.setStyleSheet(f"color: {THEME.subtext0};")
        header_layout.addWidget(self._info_label)

        header_layout.addStretch()

        # Load All button
        self._load_all_btn = QPushButton("Load All")
        self._load_all_btn.setToolTip("Request configuration from all 6 motors")
        self._load_all_btn.clicked.connect(self._on_load_all)
        self._load_all_btn.setStyleSheet(
            f"background-color: {THEME.blue}; color: {THEME.crust};"
        )
        header_layout.addWidget(self._load_all_btn)

        # Refresh button (single selected joint)
        self._refresh_btn = QPushButton("Refresh Selected")
        self._refresh_btn.setToolTip("Refresh config for currently selected joint")
        self._refresh_btn.clicked.connect(self._on_refresh_selected)
        header_layout.addWidget(self._refresh_btn)

        # Clear highlights button
        self._clear_btn = QPushButton("Clear Highlights")
        self._clear_btn.setToolTip("Clear change highlighting")
        self._clear_btn.clicked.connect(self._clear_highlights)
        header_layout.addWidget(self._clear_btn)

        group_layout.addLayout(header_layout)

        # Create table
        self._table = QTableWidget()
        self._table.setColumnCount(len(CONFIG_COLUMNS))
        self._table.setRowCount(len(JOINTS))

        # Set headers
        headers = [col[0] for col in CONFIG_COLUMNS]
        self._table.setHorizontalHeaderLabels(headers)

        # Configure table appearance
        self._table.setAlternatingRowColors(True)
        self._table.setStyleSheet(f"""
            QTableWidget {{
                background-color: {THEME.base};
                alternate-background-color: {THEME.surface0};
                gridline-color: {THEME.surface1};
                color: {THEME.text};
            }}
            QTableWidget::item {{
                padding: 4px;
            }}
            QTableWidget::item:selected {{
                background-color: {THEME.surface2};
            }}
            QHeaderView::section {{
                background-color: {THEME.surface1};
                color: {THEME.text};
                padding: 4px;
                border: 1px solid {THEME.surface2};
                font-weight: bold;
            }}
        """)

        # Set column widths
        header = self._table.horizontalHeader()
        header.setSectionResizeMode(0, QHeaderView.ResizeMode.ResizeToContents)
        for i in range(1, len(CONFIG_COLUMNS)):
            header.setSectionResizeMode(i, QHeaderView.ResizeMode.Stretch)

        # Set minimum height to show all 6 rows + header comfortably
        # Row height ~25px, header ~30px, some padding = ~200px minimum
        self._table.setMinimumHeight(scaled(200))

        # Initialize rows with joint names
        for row, joint in enumerate(JOINTS):
            # Joint name column (non-editable)
            joint_item = QTableWidgetItem(f"{joint.id} {joint.short_name}")
            joint_item.setFlags(joint_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
            joint_item.setBackground(QBrush(self.HEADER_BG))
            self._table.setItem(row, 0, joint_item)

            # Initialize data columns with "---"
            for col in range(1, len(CONFIG_COLUMNS)):
                item = QTableWidgetItem("---")
                item.setFlags(item.flags() & ~Qt.ItemFlag.ItemIsEditable)
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self._table.setItem(row, col, item)

        group_layout.addWidget(self._table)

        # Status label
        self._status_label = QLabel("")
        self._status_label.setStyleSheet(
            f"color: {THEME.subtext0}; font-size: {SIZES['font_small']}pt;"
        )
        group_layout.addWidget(self._status_label)

        layout.addWidget(group)

    def _on_load_all(self):
        """Load configuration from all 6 motors sequentially."""
        self._load_queue = list(range(1, 7))  # Joints 1-6
        self._info_label.setText("Loading configurations...")
        self._load_all_btn.setEnabled(False)

        # Start loading with a small delay between requests
        self._load_timer.start(100)  # 100ms between requests

    def _load_next_config(self):
        """Load the next config in the queue."""
        if not self._load_queue:
            self._load_timer.stop()
            self._info_label.setText("All configurations loaded")
            self._load_all_btn.setEnabled(True)
            return

        joint_id = self._load_queue.pop(0)
        self._serial_handler.get_config(joint_id)
        self._status_label.setText(f"Loading joint {joint_id}...")

    def _on_refresh_selected(self):
        """Refresh config for the currently selected joint."""
        joint_id = self._data_store.selected_joint
        self._serial_handler.get_config(joint_id)
        self._status_label.setText(f"Refreshing joint {joint_id}...")

    def _on_config_updated(self, joint_id: int):
        """Handle config update from data store."""
        config = self._data_store.get_config(joint_id)
        if config is None:
            return

        row = joint_id - 1  # Convert to 0-indexed

        # Get previous values for this joint
        prev = self._previous_values.get(joint_id, {})

        # Update each column (skip direction columns — sourced from DataStore, not ConfigData)
        for col, (_, attr) in enumerate(CONFIG_COLUMNS[1:], start=1):
            if attr in _DIRECTION_ATTRS:
                continue
            value = getattr(config, attr, None)
            if value is not None:
                # Format the value appropriately
                if attr in ("pos_limit_min", "pos_limit_max"):
                    text = f"{int(value)}"
                elif attr in ("pos_lpf_alpha", "vel_lpf_alpha", "input_lpf_alpha"):
                    text = f"{value:.3f}"
                else:
                    text = f"{value:.4g}"

                item = self._table.item(row, col)
                if item:
                    item.setText(text)

                    # Highlight if value changed
                    prev_val = prev.get(attr)
                    if prev_val is not None and prev_val != value:
                        item.setBackground(QBrush(self.HIGHLIGHT_COLOR))
                        item.setForeground(QBrush(QColor(THEME.crust)))
                    else:
                        item.setBackground(QBrush(Qt.GlobalColor.transparent))
                        item.setForeground(QBrush(QColor(THEME.text)))

        # Store current values as previous for next comparison
        self._previous_values[joint_id] = {
            attr: getattr(config, attr, None) for _, attr in CONFIG_COLUMNS[1:]
        }

        self._status_label.setText(f"Joint {joint_id} config loaded")

    def _on_directions_updated(self):
        """Refresh motor and encoder direction columns from DataStore."""
        motor_dirs = self._data_store.motor_directions
        encoder_dirs = self._data_store.encoder_directions

        for col, (_, attr) in enumerate(CONFIG_COLUMNS[1:], start=1):
            if attr not in _DIRECTION_ATTRS:
                continue
            for row in range(self._table.rowCount()):
                dirs = motor_dirs if attr == "_dir_motor" else encoder_dirs
                value = dirs[row] if row < len(dirs) else None
                item = self._table.item(row, col)
                if item is None or value is None:
                    continue
                text = f"{value:+d}"
                item.setText(text)
                # Color-code: green for +1 (forward), red for -1 (reversed)
                bg = _DIR_POS_COLOR if value >= 0 else _DIR_NEG_COLOR
                item.setBackground(QBrush(bg))
                item.setForeground(QBrush(QColor(THEME.crust)))

    def _clear_highlights(self):
        """Clear all change highlighting."""
        for row in range(self._table.rowCount()):
            for col in range(1, self._table.columnCount()):
                item = self._table.item(row, col)
                if item:
                    item.setBackground(QBrush(Qt.GlobalColor.transparent))
                    item.setForeground(QBrush(QColor(THEME.text)))

        self._previous_values.clear()
        self._status_label.setText("Highlights cleared")
