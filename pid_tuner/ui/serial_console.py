"""
Serial console widget for displaying raw serial data.

Supports custom text filtering with the following syntax:
    - Empty string: show all messages
    - "text": show lines containing "text" (case-insensitive substring match)
    - "^PREFIX": show lines starting with "PREFIX"
    - "!text": hide lines containing "text"
    - "!^PREFIX": hide lines starting with "PREFIX"
"""

from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QTextEdit,
    QPushButton,
    QCheckBox,
    QLabel,
    QSpinBox,
    QLineEdit,
    QComboBox,
)
from PyQt6.QtCore import pyqtSlot, pyqtSignal
from PyQt6.QtGui import QTextCursor, QFont

from .theme import THEME, get_console_stylesheet


class SerialConsole(QWidget):
    """
    Serial console widget for displaying raw serial communication.

    Features:
    - Display incoming serial data
    - Auto-scroll
    - Pause/Resume
    - Send raw commands
    - Custom text filter with support for:
        - Substring match (default)
        - Prefix match (^PREFIX)
        - Exclusion (!text or !^PREFIX)
    - Clear
    - Line limit to prevent memory issues
    """

    command_sent = pyqtSignal(str)

    DEFAULT_MAX_LINES = 500

    # Quick filter presets for the dropdown
    FILTER_PRESETS = [
        ("All", ""),
        ("Exclude TELEMETRY", "!^TELEMETRY"),
        ("DEBUG only", "^DEBUG"),
        ("TELEMETRY only", "^TELEMETRY"),
    ]

    def __init__(self, parent=None):
        super().__init__(parent)
        self._paused = False
        self._auto_scroll = True
        self._max_lines = self.DEFAULT_MAX_LINES
        self._line_count = 0
        self._filter_text = ""  # Empty = show all

        self._setup_ui()

    def _setup_ui(self):
        """Set up the widget layout."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        # Control bar
        control_layout = QHBoxLayout()

        control_layout.addWidget(QLabel("Serial Console"))
        control_layout.addStretch()

        # Auto-scroll checkbox
        self._autoscroll_cb = QCheckBox("Auto-scroll")
        self._autoscroll_cb.setChecked(True)
        self._autoscroll_cb.toggled.connect(self._on_autoscroll_toggled)
        control_layout.addWidget(self._autoscroll_cb)

        # Filter presets dropdown
        control_layout.addWidget(QLabel("Presets:"))
        self._filter_preset_combo = QComboBox()
        for display_name, _ in self.FILTER_PRESETS:
            self._filter_preset_combo.addItem(display_name)
        self._filter_preset_combo.currentIndexChanged.connect(self._on_preset_changed)
        self._filter_preset_combo.setMaximumWidth(150)
        control_layout.addWidget(self._filter_preset_combo)

        # Custom filter text input
        control_layout.addWidget(QLabel("Filter:"))
        self._filter_input = QLineEdit()
        self._filter_input.setPlaceholderText("e.g. 'mc debug' or '!^TELEMETRY'")
        self._filter_input.setToolTip(
            "Filter syntax:\n"
            "  text - show lines containing 'text'\n"
            "  ^PREFIX - show lines starting with 'PREFIX'\n"
            "  !text - hide lines containing 'text'\n"
            "  !^PREFIX - hide lines starting with 'PREFIX'"
        )
        self._filter_input.textChanged.connect(self._on_filter_text_changed)
        self._filter_input.setMaximumWidth(200)
        control_layout.addWidget(self._filter_input)

        # Clear filter button
        self._clear_filter_btn = QPushButton("X")
        self._clear_filter_btn.setMaximumWidth(25)
        self._clear_filter_btn.setToolTip("Clear filter")
        self._clear_filter_btn.clicked.connect(self._on_clear_filter)
        control_layout.addWidget(self._clear_filter_btn)

        # Max lines
        control_layout.addWidget(QLabel("Max:"))
        self._max_lines_spin = QSpinBox()
        self._max_lines_spin.setRange(100, 10000)
        self._max_lines_spin.setValue(self.DEFAULT_MAX_LINES)
        self._max_lines_spin.valueChanged.connect(self._on_max_lines_changed)
        self._max_lines_spin.setMaximumWidth(70)
        control_layout.addWidget(self._max_lines_spin)

        # Pause button
        self._pause_btn = QPushButton("Pause")
        self._pause_btn.setCheckable(True)
        self._pause_btn.toggled.connect(self._on_pause_toggled)
        control_layout.addWidget(self._pause_btn)

        # Clear button
        self._clear_btn = QPushButton("Clear")
        self._clear_btn.clicked.connect(self._on_clear_clicked)
        control_layout.addWidget(self._clear_btn)

        layout.addLayout(control_layout)

        # Text display
        self._text_edit = QTextEdit()
        self._text_edit.setReadOnly(True)
        self._text_edit.setFont(QFont("Courier New", 9))
        self._text_edit.setStyleSheet(get_console_stylesheet())
        layout.addWidget(self._text_edit)

        # Raw Command Input
        input_layout = QHBoxLayout()
        input_layout.addWidget(QLabel("Send:"))
        self._cmd_input = QLineEdit()
        self._cmd_input.returnPressed.connect(self._on_send_command)
        input_layout.addWidget(self._cmd_input)

        self._send_btn = QPushButton("Send")
        self._send_btn.clicked.connect(self._on_send_command)
        input_layout.addWidget(self._send_btn)

        layout.addLayout(input_layout)

    @pyqtSlot(str)
    def append_line(self, line: str):
        """
        Append a line to the console.

        Args:
            line: Text line to append
        """
        if self._paused:
            return

        # Apply filter
        if not self._should_show_line(line):
            return

        # Remove oldest lines if at limit
        if self._line_count >= self._max_lines:
            self._remove_first_line()
        else:
            self._line_count += 1

        # Append the new line
        self._text_edit.append(line.rstrip())

        # Auto-scroll to bottom
        if self._auto_scroll:
            self._text_edit.moveCursor(QTextCursor.MoveOperation.End)

    def _should_show_line(self, line: str) -> bool:
        """
        Check if line passes the current filter.

        Filter syntax:
            - Empty string: show all
            - "text": show lines containing "text" (case-insensitive)
            - "^PREFIX": show lines starting with "PREFIX" (case-sensitive)
            - "!text": hide lines containing "text"
            - "!^PREFIX": hide lines starting with "PREFIX"
        """
        if not self._filter_text:
            return True

        filter_str = self._filter_text.strip()
        if not filter_str:
            return True

        # Check for exclusion mode
        exclude_mode = filter_str.startswith("!")
        if exclude_mode:
            filter_str = filter_str[1:]

        # Check for prefix mode
        prefix_mode = filter_str.startswith("^")
        if prefix_mode:
            filter_str = filter_str[1:]

        # Apply filter
        if prefix_mode:
            matches = line.startswith(filter_str)
        else:
            # Case-insensitive substring match
            matches = filter_str.lower() in line.lower()

        # Return based on exclusion mode
        return not matches if exclude_mode else matches

    def _remove_first_line(self):
        """Remove the first line from the text edit."""
        cursor = self._text_edit.textCursor()
        cursor.movePosition(QTextCursor.MoveOperation.Start)
        cursor.movePosition(
            QTextCursor.MoveOperation.Down, QTextCursor.MoveMode.KeepAnchor
        )
        cursor.removeSelectedText()
        cursor.deleteChar()  # Remove the newline

    def _on_autoscroll_toggled(self, checked: bool):
        """Handle auto-scroll checkbox toggle."""
        self._auto_scroll = checked

    def _on_preset_changed(self, index: int):
        """Handle filter preset dropdown change."""
        if 0 <= index < len(self.FILTER_PRESETS):
            _, filter_text = self.FILTER_PRESETS[index]
            self._filter_input.setText(filter_text)
            self._filter_text = filter_text

    def _on_filter_text_changed(self, text: str):
        """Handle custom filter text change."""
        self._filter_text = text

    def _on_clear_filter(self):
        """Clear the filter text."""
        self._filter_input.clear()
        self._filter_text = ""
        self._filter_preset_combo.setCurrentIndex(0)  # Reset to "All"

    def _on_max_lines_changed(self, value: int):
        """Handle max lines change."""
        self._max_lines = value

    def _on_pause_toggled(self, checked: bool):
        """Handle pause button toggle."""
        self._paused = checked
        self._pause_btn.setText("Resume" if checked else "Pause")

    def _on_clear_clicked(self):
        """Handle clear button click."""
        self._text_edit.clear()
        self._line_count = 0

    def _on_send_command(self):
        cmd = self._cmd_input.text().strip()
        if cmd:
            self.command_sent.emit(cmd)
            # local echo
            self._text_edit.append(f"> {cmd}")
            self._cmd_input.clear()
