"""
Serial console widget for displaying raw serial data.
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
)
from PyQt6.QtCore import pyqtSlot, pyqtSignal
from PyQt6.QtGui import QTextCursor, QFont


class SerialConsole(QWidget):
    """
    Serial console widget for displaying raw serial communication.

    Features:
    - Display incoming serial data
    - Auto-scroll
    - Pause/Resume
    - Send raw commands
    - Clear
    - Line limit to prevent memory issues
    """

    command_sent = pyqtSignal(str)

    DEFAULT_MAX_LINES = 500

    def __init__(self, parent=None):
        super().__init__(parent)
        self._paused = False
        self._auto_scroll = True
        self._max_lines = self.DEFAULT_MAX_LINES
        self._line_count = 0

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

        # Max lines
        control_layout.addWidget(QLabel("Max lines:"))
        self._max_lines_spin = QSpinBox()
        self._max_lines_spin.setRange(100, 10000)
        self._max_lines_spin.setValue(self.DEFAULT_MAX_LINES)
        self._max_lines_spin.valueChanged.connect(self._on_max_lines_changed)
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
        self._text_edit.setStyleSheet("""
            QTextEdit {
                background-color: #1e1e1e;
                color: #d4d4d4;
                border: 1px solid #3c3c3c;
            }
        """)
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
