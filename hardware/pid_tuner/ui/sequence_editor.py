"""
Sequence / Trajectory Editor for AUTO_CURB_CLIMBING mode.

Motor order: [RC, FC, ML, MR, ML_Carriage, MR_Carriage, Drive_FB, Drive_LR,
              ODrive_R, ODrive_L]  (matches firmware SEQ_NUM_MOTORS)
All sequence slots use position interpolation during AUTO_CURB_CLIMBING.
"""

from __future__ import annotations

import json
import os
from typing import List, Optional

from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QTableWidget,
    QTableWidgetItem,
    QPushButton,
    QLabel,
    QFileDialog,
    QMessageBox,
    QHeaderView,
    QAbstractItemView,
    QFrame,
    QSpinBox,
    QCheckBox,
)
from PyQt6.QtCore import Qt, pyqtSignal, pyqtSlot, QTimer
from PyQt6.QtGui import QColor, QFont, QBrush

from ..data.data_store import DataStore
from ..serial_driver.serial_handler import SerialHandler
from ..serial_driver.keyframe import Keyframe, NUM_MOTORS
from .theme import THEME, JOINT_COLORS
from .scaling import SIZES, scaled


MOTOR_NAMES = [
    "RC",
    "FC",
    "ML",
    "MR",
    "ML_Car",
    "MR_Car",
    "Drive_FB",
    "Drive_LR",
    "OD_R",
    "OD_L",
]

COL_LABEL = 0
COL_RC = 1
COL_FC = 2
COL_ML = 3
COL_MR = 4
COL_ML_CAR = 5
COL_MR_CAR = 6
COL_DRIVE_FB = 7
COL_DRIVE_LR = 8
COL_OD_R = 9
COL_OD_L = 10
NUM_COLS = 11

INACTIVE_TEXT = "--"


class SequenceEditor(QWidget):
    """
    Keyframe sequence editor widget.

    Allows building sequences of motor position keyframes, uploading them to
    the robot (entering AUTO_CURB_CLIMBING mode), and stepping through them.
    """

    # Emitted when sequence mode should be activated on the robot
    sequence_mode_requested = pyqtSignal(bool)

    def __init__(
        self,
        data_store: DataStore,
        serial_handler: SerialHandler,
        parent=None,
    ):
        super().__init__(parent)
        self._data_store = data_store
        self._serial_handler = serial_handler
        self._keyframes: List[Keyframe] = []
        self._current_file: Optional[str] = None

        # Sequence execution state (mirrored from robot)
        self._robot_step: int = -1
        self._prev_robot_step: int = -1
        self._robot_total: int = 0
        self._robot_state: int = 0
        self._uploaded: bool = False

        # Upload state
        self._upload_pending: List[int] = []  # queue of keyframe indices to ACK
        self._upload_timer = QTimer(self)
        self._upload_timer.setSingleShot(True)
        self._upload_timer.timeout.connect(self._on_upload_timeout)

        self._setup_ui()
        self._wire_signals()
        self._update_button_states()

    # ------------------------------------------------------------------ #
    #  UI construction                                                      #
    # ------------------------------------------------------------------ #

    def _setup_ui(self):
        root = QVBoxLayout(self)
        root.setSpacing(SIZES["spacing_small"])
        root.setContentsMargins(
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
        )

        root.addWidget(self._build_toolbar())
        root.addWidget(self._build_table(), stretch=1)
        root.addWidget(self._build_row_buttons())
        root.addWidget(self._build_step_bar())
        root.addWidget(self._build_status_bar())

    def _build_toolbar(self) -> QFrame:
        bar = QFrame()
        bar.setStyleSheet(f"background-color: {THEME.surface0}; border-radius: 4px;")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
        )
        layout.setSpacing(SIZES["spacing_small"])

        btn_style = f"""
            QPushButton {{
                background-color: {THEME.surface1};
                color: {THEME.text};
                border: 1px solid {THEME.surface2};
                border-radius: 4px;
                padding: 4px 10px;
                font-size: {SIZES["font_normal"]}pt;
            }}
            QPushButton:hover {{
                background-color: {THEME.surface2};
            }}
            QPushButton:pressed {{
                background-color: {THEME.surface0};
            }}
            QPushButton:disabled {{
                color: {THEME.overlay0};
            }}
        """

        self._btn_new = QPushButton("New")
        self._btn_new.setToolTip("Create a blank sequence")
        self._btn_new.setStyleSheet(btn_style)
        self._btn_new.clicked.connect(self._on_new)
        layout.addWidget(self._btn_new)

        self._btn_load = QPushButton("Load")
        self._btn_load.setToolTip("Load sequence from JSON file")
        self._btn_load.setStyleSheet(btn_style)
        self._btn_load.clicked.connect(self._on_load)
        layout.addWidget(self._btn_load)

        self._btn_save = QPushButton("Save")
        self._btn_save.setToolTip("Save sequence to JSON file")
        self._btn_save.setStyleSheet(btn_style)
        self._btn_save.clicked.connect(self._on_save)
        layout.addWidget(self._btn_save)

        self._btn_save_as = QPushButton("Save As…")
        self._btn_save_as.setToolTip("Save sequence to a new JSON file")
        self._btn_save_as.setStyleSheet(btn_style)
        self._btn_save_as.clicked.connect(self._on_save_as)
        layout.addWidget(self._btn_save_as)

        layout.addSpacing(SIZES["spacing_medium"])

        self._btn_send = QPushButton("Send to Robot")
        self._btn_send.setToolTip(
            "Enter AUTO_CURB_CLIMBING mode and upload all keyframes to the robot"
        )
        self._btn_send.setStyleSheet(f"""
            QPushButton {{
                background-color: {THEME.blue};
                color: {THEME.crust};
                border-radius: 4px;
                padding: 4px 12px;
                font-weight: bold;
                font-size: {SIZES["font_normal"]}pt;
            }}
            QPushButton:hover {{ background-color: {THEME.sapphire}; }}
            QPushButton:pressed {{ background-color: {THEME.lavender}; }}
            QPushButton:disabled {{ background-color: {THEME.surface1}; color: {THEME.overlay0}; }}
        """)
        self._btn_send.clicked.connect(self._on_send_to_robot)
        layout.addWidget(self._btn_send)

        self._btn_exit_seq = QPushButton("Exit Sequence")
        self._btn_exit_seq.setToolTip("Send B1:0 to exit AUTO_CURB_CLIMBING mode")
        self._btn_exit_seq.setStyleSheet(f"""
            QPushButton {{
                background-color: {THEME.maroon};
                color: {THEME.crust};
                border-radius: 4px;
                padding: 4px 10px;
                font-size: {SIZES["font_normal"]}pt;
            }}
            QPushButton:hover {{ background-color: {THEME.red}; }}
            QPushButton:pressed {{ background-color: {THEME.flamingo}; }}
            QPushButton:disabled {{ background-color: {THEME.surface1}; color: {THEME.overlay0}; }}
        """)
        self._btn_exit_seq.clicked.connect(self._on_exit_sequence)
        layout.addWidget(self._btn_exit_seq)

        layout.addStretch()

        # File name display
        self._file_label = QLabel("No file")
        self._file_label.setStyleSheet(
            f"color: {THEME.subtext0}; font-style: italic; font-size: {SIZES['font_small']}pt;"
        )
        layout.addWidget(self._file_label)

        return bar

    def _build_table(self) -> QTableWidget:
        headers = ["Label"] + MOTOR_NAMES
        self._table = QTableWidget(0, NUM_COLS)
        self._table.setHorizontalHeaderLabels(headers)
        self._table.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self._table.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._table.setAlternatingRowColors(True)
        vertical_header = self._table.verticalHeader()
        horizontal_header = self._table.horizontalHeader()
        if vertical_header is not None:
            vertical_header.setVisible(True)
        if horizontal_header is not None:
            horizontal_header.setSectionResizeMode(
                COL_LABEL, QHeaderView.ResizeMode.Stretch
            )
            for col in range(COL_RC, NUM_COLS):
                horizontal_header.setSectionResizeMode(
                    col, QHeaderView.ResizeMode.ResizeToContents
                )
        self._table.setStyleSheet(f"""
            QTableWidget {{
                background-color: {THEME.base};
                color: {THEME.text};
                gridline-color: {THEME.surface1};
                border: 1px solid {THEME.surface1};
                font-size: {SIZES["font_normal"]}pt;
            }}
            QTableWidget::item {{
                color: {THEME.text};
            }}
            QTableWidget::item:alternate {{
                background-color: {THEME.mantle};
                color: {THEME.text};
            }}
            QTableWidget::item:selected,
            QTableWidget::item:selected:alternate {{
                background-color: {THEME.blue};
                color: {THEME.base};
            }}
            QHeaderView::section {{
                background-color: {THEME.surface0};
                color: {THEME.subtext1};
                border: 1px solid {THEME.surface1};
                padding: 4px;
                font-weight: bold;
            }}
        """)

        # Color motor column headers to match joint accent colors
        for motor_idx, col in enumerate(range(COL_RC, NUM_COLS)):
            item = self._table.horizontalHeaderItem(col)
            if item:
                item.setForeground(QBrush(QColor(JOINT_COLORS[motor_idx])))

        self._table.itemChanged.connect(self._on_table_item_changed)
        self._table.itemSelectionChanged.connect(self._on_table_selection_changed)

        self._table.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self._table.customContextMenuRequested.connect(self._on_table_context_menu)
        return self._table

    def _build_row_buttons(self) -> QWidget:
        w = QWidget()
        layout = QHBoxLayout(w)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(SIZES["spacing_small"])

        small_style = f"""
            QPushButton {{
                background-color: {THEME.surface1};
                color: {THEME.text};
                border: 1px solid {THEME.surface2};
                border-radius: 3px;
                padding: 3px 8px;
                font-size: {SIZES["font_small"]}pt;
            }}
            QPushButton:hover {{ background-color: {THEME.surface2}; }}
            QPushButton:pressed {{ background-color: {THEME.surface0}; }}
            QPushButton:disabled {{ color: {THEME.overlay0}; }}
        """

        self._btn_add = QPushButton("+ Add Keyframe")
        self._btn_add.setStyleSheet(small_style)
        self._btn_add.clicked.connect(self._on_add_keyframe)
        layout.addWidget(self._btn_add)

        self._btn_insert = QPushButton("↥ Insert Above")
        self._btn_insert.setStyleSheet(small_style)
        self._btn_insert.clicked.connect(self._on_insert_keyframe_above)
        layout.addWidget(self._btn_insert)

        self._btn_remove = QPushButton("− Remove Selected")
        self._btn_remove.setStyleSheet(small_style)
        self._btn_remove.clicked.connect(self._on_remove_keyframe)
        layout.addWidget(self._btn_remove)

        self._btn_capture = QPushButton("⊙ Capture Current Positions")
        self._btn_capture.setToolTip(
            "Fill selected row with live motor positions from telemetry"
        )
        self._btn_capture.setStyleSheet(f"""
            QPushButton {{
                background-color: {THEME.teal};
                color: {THEME.crust};
                border-radius: 3px;
                padding: 3px 10px;
                font-size: {SIZES["font_small"]}pt;
            }}
            QPushButton:hover {{ background-color: {THEME.green}; }}
            QPushButton:pressed {{ background-color: {THEME.teal}; }}
            QPushButton:disabled {{ background-color: {THEME.surface1}; color: {THEME.overlay0}; }}
        """)
        self._btn_capture.clicked.connect(self._on_capture_positions)
        layout.addWidget(self._btn_capture)

        layout.addStretch()
        return w

    def _build_step_bar(self) -> QFrame:
        bar = QFrame()
        bar.setStyleSheet(f"background-color: {THEME.surface0}; border-radius: 4px;")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
        )
        layout.setSpacing(SIZES["spacing_medium"])

        layout.addWidget(QLabel("Step:"))

        self._btn_step_bwd = QPushButton("◀  Step Back")
        self._btn_step_bwd.setToolTip("Step backward to the previous keyframe  (<)")
        self._btn_step_bwd.setStyleSheet(f"""
            QPushButton {{
                background-color: {THEME.peach};
                color: {THEME.crust};
                border-radius: 4px;
                padding: 5px 14px;
                font-weight: bold;
                font-size: {SIZES["font_normal"]}pt;
            }}
            QPushButton:hover {{ background-color: {THEME.yellow}; }}
            QPushButton:pressed {{ background-color: {THEME.peach}; }}
            QPushButton:disabled {{ background-color: {THEME.surface1}; color: {THEME.overlay0}; }}
        """)
        self._btn_step_bwd.clicked.connect(self._on_step_backward)
        layout.addWidget(self._btn_step_bwd)

        self._btn_step_fwd = QPushButton("Step Fwd  ▶")
        self._btn_step_fwd.setToolTip("Step forward to the next keyframe  (>)")
        self._btn_step_fwd.setStyleSheet(f"""
            QPushButton {{
                background-color: {THEME.green};
                color: {THEME.crust};
                border-radius: 4px;
                padding: 5px 14px;
                font-weight: bold;
                font-size: {SIZES["font_normal"]}pt;
            }}
            QPushButton:hover {{ background-color: {THEME.teal}; }}
            QPushButton:disabled {{ background-color: {THEME.surface1}; color: {THEME.overlay0}; }}
        """)
        self._btn_step_fwd.clicked.connect(self._on_step_forward)
        layout.addWidget(self._btn_step_fwd)

        self._btn_step_goto = QPushButton("Go To")
        self._btn_step_goto.setToolTip(
            "Jump directly to selected keyframe index  (@idx)"
        )
        self._btn_step_goto.setStyleSheet(f"""
            QPushButton {{
                background-color: {THEME.blue};
                color: {THEME.crust};
                border-radius: 4px;
                padding: 5px 12px;
                font-weight: bold;
                font-size: {SIZES["font_normal"]}pt;
            }}
            QPushButton:hover {{ background-color: {THEME.sapphire}; }}
            QPushButton:pressed {{ background-color: {THEME.lavender}; }}
            QPushButton:disabled {{ background-color: {THEME.surface1}; color: {THEME.overlay0}; }}
        """)
        self._btn_step_goto.clicked.connect(self._on_step_goto)
        layout.addWidget(self._btn_step_goto)

        self._spin_step_goto = QSpinBox()
        self._spin_step_goto.setRange(0, 0)
        self._spin_step_goto.setFixedWidth(scaled(80))
        self._spin_step_goto.setToolTip("0-based keyframe index")
        self._spin_step_goto.valueChanged.connect(
            lambda _v: self._update_button_states()
        )
        layout.addWidget(self._spin_step_goto)

        self._chk_auto_run = QCheckBox("Auto Run")
        self._chk_auto_run.setToolTip("Automatically advance through sequence")
        self._chk_auto_run.toggled.connect(self._serial_handler.seq_auto_run)
        layout.addWidget(self._chk_auto_run)

        layout.addStretch()
        return bar

    def _build_status_bar(self) -> QLabel:
        self._status_label = QLabel("No sequence active")
        self._status_label.setStyleSheet(
            f"color: {THEME.subtext1}; font-size: {SIZES['font_small']}pt;"
            f" padding: 2px 4px; background-color: {THEME.mantle}; border-radius: 3px;"
        )
        self._status_label.setAlignment(
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter
        )
        self._status_label.setFixedHeight(scaled(22))
        return self._status_label

    # ------------------------------------------------------------------ #
    #  Signal wiring                                                        #
    # ------------------------------------------------------------------ #

    def _wire_signals(self):
        self._serial_handler.seq_ack_received.connect(self._on_seq_ack)
        self._serial_handler.seq_status_received.connect(self._on_seq_status)

    # ------------------------------------------------------------------ #
    #  Table population                                                     #
    # ------------------------------------------------------------------ #

    def _populate_table(self):
        self._table.blockSignals(True)
        self._table.clearContents()
        self._table.setRowCount(3 * len(self._keyframes))
        for keyframe_idx, kf in enumerate(self._keyframes):
            self._set_keyframe_rows(keyframe_idx, kf)
        vertical_labels: List[str] = []
        for i in range(len(self._keyframes)):
            vertical_labels.extend([f"Step {i}", "dur", "guard"])
        self._table.setVerticalHeaderLabels(vertical_labels)
        self._table.blockSignals(False)
        self._sync_step_goto_range()
        self._highlight_active_step()

    def _set_keyframe_rows(self, keyframe_index: int, kf: Keyframe):
        position_row = 3 * keyframe_index
        duration_row = position_row + 1
        guard_row = position_row + 2
        self._table.blockSignals(True)

        label_item = QTableWidgetItem(kf.label or f"Step {keyframe_index}")
        self._table.setItem(position_row, COL_LABEL, label_item)

        for motor_idx in range(NUM_MOTORS):
            col = COL_RC + motor_idx
            val = kf.targets[motor_idx]
            if val is None:
                item = QTableWidgetItem(INACTIVE_TEXT)
                item.setForeground(QBrush(QColor(THEME.overlay0)))
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            else:
                text = f"{val:.1f}"
                if kf.relative[motor_idx]:
                    text = f"Δ{text}"
                item = QTableWidgetItem(text)
                item.setForeground(QBrush(QColor(JOINT_COLORS[motor_idx])))
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            if kf.relative[motor_idx]:
                item.setBackground(QBrush(QColor(THEME.surface1)))
            self._table.setItem(position_row, col, item)

        dur_label_item = QTableWidgetItem("dur")
        dur_label_item.setFlags(dur_label_item.flags() & ~Qt.ItemFlag.ItemIsEditable)
        dur_label_item.setForeground(QBrush(QColor(THEME.overlay0)))
        dur_label_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        dur_label_item.setBackground(QBrush(QColor(THEME.crust)))
        self._table.setItem(duration_row, COL_LABEL, dur_label_item)

        for motor_idx in range(NUM_MOTORS):
            col = COL_RC + motor_idx
            override = kf.motor_durations[motor_idx]
            value = override if override is not None else kf.duration_ms
            dur_item = QTableWidgetItem(str(value))
            dur_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            if override is None:
                dur_item.setForeground(QBrush(QColor(THEME.subtext0)))
            dur_item.setBackground(QBrush(QColor(THEME.crust)))
            self._table.setItem(duration_row, col, dur_item)

        # Guard row
        guard_label_item = QTableWidgetItem("guard")
        guard_label_item.setFlags(
            guard_label_item.flags() & ~Qt.ItemFlag.ItemIsEditable
        )
        guard_label_item.setForeground(QBrush(QColor(THEME.overlay0)))
        guard_label_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        guard_label_item.setBackground(QBrush(QColor(THEME.surface0)))
        self._table.setItem(guard_row, COL_LABEL, guard_label_item)

        for motor_idx in range(NUM_MOTORS):
            col = COL_RC + motor_idx
            condition = kf.guard_condition[motor_idx]
            threshold = kf.guard_threshold[motor_idx]
            if condition == 1:  # GUARD_GREATER_THAN
                text = f">{threshold:.0f}"
                color = THEME.green
            elif condition == 2:  # GUARD_LESS_THAN
                text = f"<{threshold:.0f}"
                color = THEME.peach
            else:
                text = INACTIVE_TEXT
                color = THEME.overlay0
            guard_item = QTableWidgetItem(text)
            guard_item.setForeground(QBrush(QColor(color)))
            guard_item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            guard_item.setBackground(QBrush(QColor(THEME.surface0)))
            self._table.setItem(guard_row, col, guard_item)

        self._table.blockSignals(False)

    def _highlight_active_step(self):
        self._table.blockSignals(True)
        normal_font = QFont()
        bold_font = QFont()
        bold_font.setBold(True)

        direction = 0
        if self._prev_robot_step > self._robot_step:
            direction = -1
        elif self._prev_robot_step < self._robot_step:
            direction = 1

        back_tint = QColor(THEME.blue).lighter(180)
        fwd_tint = QColor(THEME.green).lighter(180)

        active_rows = set()
        if 0 <= self._robot_step < len(self._keyframes):
            active_rows = {
                3 * self._robot_step,
                3 * self._robot_step + 1,
                3 * self._robot_step + 2,
            }

        for row in range(self._table.rowCount()):
            font = bold_font if row in active_rows else normal_font
            for col in range(NUM_COLS):
                try:
                    item = self._table.item(row, col)
                except RuntimeError:
                    continue
                if item is None:
                    continue
                try:
                    item.setFont(font)
                    self._apply_base_cell_background(row, col, item)
                    if row in active_rows:
                        if direction < 0:
                            item.setBackground(QBrush(back_tint))
                        elif direction > 0:
                            item.setBackground(QBrush(fwd_tint))
                except RuntimeError:
                    continue
        self._table.blockSignals(False)

    def _apply_base_cell_background(self, row: int, col: int, item: QTableWidgetItem):
        if row % 3 == 1:  # duration row
            item.setBackground(QBrush(QColor(THEME.crust)))
            return
        if row % 3 == 2:  # guard row
            item.setBackground(QBrush(QColor(THEME.surface0)))
            return

        # position row (row % 3 == 0) — check for relative highlight
        if COL_RC <= col <= COL_OD_L:
            kf_idx = row // 3
            motor_idx = col - COL_RC
            if (
                0 <= kf_idx < len(self._keyframes)
                and self._keyframes[kf_idx].relative[motor_idx]
            ):
                item.setBackground(QBrush(QColor(THEME.surface1)))
                return
        item.setBackground(QBrush())

    def _sync_step_goto_range(self):
        max_idx = max(0, len(self._keyframes) - 1)
        self._spin_step_goto.blockSignals(True)
        self._spin_step_goto.setRange(0, max_idx)
        if self._robot_step >= 0 and self._robot_step <= max_idx:
            self._spin_step_goto.setValue(self._robot_step)
        elif self._spin_step_goto.value() > max_idx:
            self._spin_step_goto.setValue(max_idx)
        self._spin_step_goto.blockSignals(False)

    # ------------------------------------------------------------------ #
    #  Table → data model sync                                             #
    # ------------------------------------------------------------------ #

    @pyqtSlot(QTableWidgetItem)
    def _on_table_item_changed(self, item: QTableWidgetItem):
        row = item.row()
        col = item.column()
        kf_idx = row // 3
        if row < 0 or kf_idx < 0 or kf_idx >= len(self._keyframes):
            return
        kf = self._keyframes[kf_idx]
        text = item.text().strip()
        is_position_row = row % 3 == 0

        if is_position_row and col == COL_LABEL:
            kf.label = text

        elif is_position_row and COL_RC <= col <= COL_OD_L:
            motor_idx = col - COL_RC
            parse_text = text[1:] if text.startswith("Δ") else text
            if parse_text == INACTIVE_TEXT or parse_text == "" or parse_text == "-":
                kf.targets[motor_idx] = None
            else:
                try:
                    kf.targets[motor_idx] = float(parse_text)
                except ValueError:
                    pass

        elif row % 3 == 1 and COL_RC <= col <= COL_OD_L:
            motor_idx = col - COL_RC
            if text in {"", "-", INACTIVE_TEXT}:
                kf.motor_durations[motor_idx] = None
            else:
                try:
                    kf.motor_durations[motor_idx] = max(0, int(float(text)))
                except ValueError:
                    pass

        elif row % 3 == 2 and COL_RC <= col <= COL_OD_L:
            # Guard row edit
            motor_idx = col - COL_RC
            if text.startswith(">"):
                try:
                    kf.guard_condition[motor_idx] = 1  # GUARD_GREATER_THAN
                    kf.guard_threshold[motor_idx] = float(text[1:])
                except ValueError:
                    pass
            elif text.startswith("<"):
                try:
                    kf.guard_condition[motor_idx] = 2  # GUARD_LESS_THAN
                    kf.guard_threshold[motor_idx] = float(text[1:])
                except ValueError:
                    pass
            else:
                kf.guard_condition[motor_idx] = 0  # GUARD_NONE
                kf.guard_threshold[motor_idx] = 0.0

        self._set_keyframe_rows(kf_idx, kf)
        self._uploaded = False
        self._update_button_states()

    def _on_table_context_menu(self, pos):
        item = self._table.itemAt(pos)
        if item is None:
            return

        row = item.row()
        col = item.column()
        if row % 3 != 0 or not (COL_RC <= col <= COL_OD_L):
            return

        kf_idx = row // 3
        if kf_idx < 0 or kf_idx >= len(self._keyframes):
            return

        motor_idx = col - COL_RC
        kf = self._keyframes[kf_idx]
        kf.relative[motor_idx] = not kf.relative[motor_idx]
        self._set_keyframe_rows(kf_idx, kf)
        self._uploaded = False
        self._update_button_states()

    # ------------------------------------------------------------------ #
    #  Toolbar actions                                                      #
    # ------------------------------------------------------------------ #

    def _on_new(self):
        if self._keyframes:
            reply = QMessageBox.question(
                self,
                "New Sequence",
                "Discard current sequence and start fresh?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            )
            if reply != QMessageBox.StandardButton.Yes:
                return
        self._keyframes = []
        self._uploaded = False
        self._current_file = None
        self._file_label.setText("New sequence (unsaved)")
        self._populate_table()
        self._update_button_states()

    def _on_load(self):
        path, _ = QFileDialog.getOpenFileName(
            self,
            "Load Sequence",
            "",
            "JSON Files (*.json);;All Files (*)",
            options=QFileDialog.Option.DontUseNativeDialog,
        )
        if not path:
            return
        try:
            with open(path, "r") as f:
                data = json.load(f)
            keyframes_data = data.get(
                "keyframes", data if isinstance(data, list) else []
            )
            self._keyframes = [Keyframe.from_dict(d) for d in keyframes_data]
            self._uploaded = False
            self._current_file = path
            self._file_label.setText(os.path.basename(path))
            self._populate_table()
            self._update_button_states()
        except Exception as e:
            QMessageBox.critical(self, "Load Error", f"Failed to load sequence:\n{e}")

    def _on_save(self):
        if self._current_file:
            self._save_to_file(self._current_file)
        else:
            self._on_save_as()

    def _on_save_as(self):
        path, _ = QFileDialog.getSaveFileName(
            self,
            "Save Sequence",
            "",
            "JSON Files (*.json);;All Files (*)",
            options=QFileDialog.Option.DontUseNativeDialog,
        )
        if not path:
            return
        if not path.endswith(".json"):
            path += ".json"
        self._save_to_file(path)

    def _save_to_file(self, path: str):
        try:
            data = {
                "name": os.path.splitext(os.path.basename(path))[0],
                "keyframes": [kf.to_dict() for kf in self._keyframes],
            }
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            self._current_file = path
            self._file_label.setText(os.path.basename(path))
            self._set_status(f"Saved to {os.path.basename(path)}")
        except Exception as e:
            QMessageBox.critical(self, "Save Error", f"Failed to save sequence:\n{e}")

    def _on_send_to_robot(self):
        if not self._keyframes:
            QMessageBox.warning(
                self, "No Sequence", "Add at least one keyframe before sending."
            )
            return

        # Enter sequence mode on the robot
        self._serial_handler.enter_sequence_mode(True)
        self._uploaded = False

        # Build the upload queue and send all keyframes
        self._upload_pending = list(range(len(self._keyframes)))
        self._set_status(f"Uploading {len(self._keyframes)} keyframe(s)…")

        for idx, kf in enumerate(self._keyframes):
            targets = [t if t is not None else 0.0 for t in kf.targets]
            active = [t is not None for t in kf.targets]
            durations = [
                kf.motor_durations[i]
                if kf.motor_durations[i] is not None
                else kf.duration_ms
                for i in range(NUM_MOTORS)
            ]
            self._serial_handler.send_keyframe(
                idx,
                targets,
                active,
                durations,
                kf.relative,
                carriage_return=kf.carriage_return,
                guard_threshold=kf.guard_threshold,
                guard_condition=kf.guard_condition,
            )

        # Start a timeout — if not all ACKs arrive within 3s, warn the user
        self._upload_timer.start(3000)
        self._update_button_states()

    def _on_exit_sequence(self):
        self._serial_handler.enter_sequence_mode(False)
        self._uploaded = False
        self._robot_step = -1
        self._prev_robot_step = -1
        self._robot_total = 0
        self._robot_state = 0
        self._highlight_active_step()
        self._set_status("Exited sequence mode")
        self._update_button_states()

    # ------------------------------------------------------------------ #
    #  Row management                                                       #
    # ------------------------------------------------------------------ #

    def _on_add_keyframe(self):
        kf = Keyframe()
        kf.label = f"Step {len(self._keyframes)}"
        # Pre-fill with current live positions if available
        self._apply_live_positions(kf)
        self._keyframes.append(kf)
        keyframe_idx = len(self._keyframes) - 1
        self._populate_table()
        self._table.selectRow(3 * keyframe_idx)
        self._uploaded = False
        self._sync_step_goto_range()
        self._update_button_states()

    def _on_insert_keyframe_above(self):
        row = self._table.currentRow()
        if row < 0:
            QMessageBox.information(self, "Insert Keyframe", "Select a row first.")
            return
        kf_idx = row // 3
        if kf_idx >= len(self._keyframes):
            QMessageBox.information(self, "Insert Keyframe", "Select a row first.")
            return
        kf = Keyframe()
        kf.label = "New Step"
        self._apply_live_positions(kf)
        self._keyframes.insert(kf_idx, kf)
        self._populate_table()
        self._table.selectRow(3 * kf_idx)
        self._uploaded = False
        self._update_button_states()

    def _on_remove_keyframe(self):
        rows = self._table.selectedItems()
        if not rows:
            return
        row = self._table.currentRow()
        kf_idx = row // 3
        if 0 <= kf_idx < len(self._keyframes):
            self._keyframes.pop(kf_idx)
            self._populate_table()
            self._uploaded = False
            self._sync_step_goto_range()
            self._update_button_states()

    def _on_capture_positions(self):
        """Fill selected row with live motor positions from telemetry."""
        row = self._table.currentRow()
        if row < 0:
            QMessageBox.information(self, "Capture Positions", "Select a row first.")
            return
        if row % 3 != 0:
            QMessageBox.information(
                self,
                "Capture Positions",
                "Select a position row (Step N), not the dur/guard row.",
            )
            return
        kf_idx = row // 3
        if kf_idx >= len(self._keyframes):
            QMessageBox.information(self, "Capture Positions", "Select a row first.")
            return
        kf = self._keyframes[kf_idx]
        self._apply_live_positions(kf)
        self._set_keyframe_rows(kf_idx, kf)
        self._set_status(f"Captured live positions into Step {kf_idx}")

    def _apply_live_positions(self, kf: Keyframe):
        for i in range(NUM_MOTORS):
            if i < 8:
                joint_data = self._data_store.get_joint(i + 1)
                if joint_data is None:
                    continue
                if kf.relative[i]:
                    kf.targets[i] = 0.0
                else:
                    kf.targets[i] = round(joint_data.current_position, 1)
            elif i == 8:
                if kf.relative[i]:
                    kf.targets[i] = 0.0
                else:
                    kf.targets[i] = round(self._data_store.odrive_r_pos, 2)
            elif i == 9:
                if kf.relative[i]:
                    kf.targets[i] = 0.0
                else:
                    kf.targets[i] = round(self._data_store.odrive_l_pos, 2)

    # ------------------------------------------------------------------ #
    #  Step commands                                                        #
    # ------------------------------------------------------------------ #

    def _on_step_forward(self):
        self._serial_handler.seq_step_forward()
        self._set_status("Stepping forward…")

    def _on_step_backward(self):
        self._serial_handler.seq_step_backward()
        self._set_status("Stepping backward…")

    def _on_step_goto(self):
        step_idx = self._spin_step_goto.value()
        if 0 <= step_idx < len(self._keyframes):
            self._serial_handler.seq_goto(step_idx)
            self._set_status(f"Jumping to step {step_idx + 1}…")

    def _on_table_selection_changed(self):
        row = self._table.currentRow()
        keyframe_idx = row // 3
        if keyframe_idx >= 0 and keyframe_idx <= self._spin_step_goto.maximum():
            self._spin_step_goto.blockSignals(True)
            self._spin_step_goto.setValue(keyframe_idx)
            self._spin_step_goto.blockSignals(False)
        self._update_button_states()

    # ------------------------------------------------------------------ #
    #  Robot response handlers                                             #
    # ------------------------------------------------------------------ #

    @pyqtSlot(int)
    def _on_seq_ack(self, step_idx: int):
        """Called when the robot ACKs a keyframe upload."""
        if step_idx in self._upload_pending:
            self._upload_pending.remove(step_idx)
        if not self._upload_pending:
            self._upload_timer.stop()
            # Mirror the known sequence state locally so step buttons enable
            self._robot_total = len(self._keyframes)
            self._robot_step = -1
            self._prev_robot_step = -1
            self._robot_state = 0
            self._uploaded = True
            self._set_status(
                f"Upload complete — {len(self._keyframes)} keyframe(s) ready.  "
                "Press Step Fwd to start."
            )
            self._update_button_states()

    @pyqtSlot(int, int, int)
    def _on_seq_status(self, current_step: int, total_steps: int, state: int):
        self._prev_robot_step = self._robot_step
        self._robot_step = current_step
        self._robot_total = total_steps
        self._robot_state = state
        self._uploaded = total_steps > 0
        self._push_seq_targets()
        self._sync_step_goto_range()
        self._highlight_active_step()

        status_prefix = ""
        if self._prev_robot_step > self._robot_step:
            status_prefix = "↑ "
        elif self._prev_robot_step < self._robot_step:
            status_prefix = "↓ "

        if state == 1:
            self._set_status(
                f"{status_prefix}Step {current_step + 1}/{total_steps} — Interpolating…",
                color=THEME.yellow,
            )
        elif state == 2:
            self._set_status(
                f"{status_prefix}Step {current_step + 1}/{total_steps} — Settling (waiting for motors to reach targets)…",
                color=THEME.peach,
            )
        else:
            self._set_status(
                f"{status_prefix}Step {current_step + 1}/{total_steps} — Ready",
                color=THEME.green,
            )
        self._update_button_states()

    def _push_seq_targets(self):
        targets = {}
        step = self._robot_step
        if 0 <= step < len(self._keyframes):
            kf = self._keyframes[step]
            for i in range(NUM_MOTORS):
                if kf.targets[i] is not None:
                    # RoboClaw / drive slots 0–7 → joint ids 1–8; ODrive slots → 9–10
                    targets[i + 1] = kf.targets[i]
        self._data_store.set_seq_targets(targets)

    def _on_upload_timeout(self):
        if self._upload_pending:
            self._set_status(
                f"Warning: no ACK for keyframe(s) {self._upload_pending}",
                color=THEME.red,
            )
            self._upload_pending = []
            self._update_button_states()

    # ------------------------------------------------------------------ #
    #  Button state management                                             #
    # ------------------------------------------------------------------ #

    def _update_button_states(self):
        has_kf = bool(self._keyframes)
        uploading = bool(self._upload_pending)
        seq_active = self._robot_total > 0  # robot has a loaded sequence

        self._btn_save.setEnabled(has_kf)
        self._btn_save_as.setEnabled(has_kf)
        self._btn_send.setEnabled(has_kf and not uploading)
        self._btn_exit_seq.setEnabled(seq_active)
        self._btn_remove.setEnabled(has_kf)
        self._btn_insert.setEnabled(has_kf and self._table.currentRow() >= 0)
        self._btn_capture.setEnabled(has_kf)

        can_step = seq_active and self._robot_state == 0
        self._btn_step_fwd.setEnabled(
            can_step and (self._robot_step < self._robot_total - 1)
        )
        self._btn_step_bwd.setEnabled(can_step and (self._robot_step > 0))

        if self._uploaded:
            self._table.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        else:
            self._table.setEditTriggers(
                QAbstractItemView.EditTrigger.DoubleClicked
                | QAbstractItemView.EditTrigger.EditKeyPressed
            )

        goto_idx = self._spin_step_goto.value()
        can_goto = (
            self._uploaded
            and self._robot_state == 0
            and 0 <= goto_idx < len(self._keyframes)
        )
        self._btn_step_goto.setEnabled(can_goto)

    # ------------------------------------------------------------------ #
    #  Status helpers                                                       #
    # ------------------------------------------------------------------ #

    def _set_status(self, text: str, color: str = ""):
        self._status_label.setText(text)
        c = color or THEME.subtext1
        self._status_label.setStyleSheet(
            f"color: {c}; font-size: {SIZES['font_small']}pt;"
            f" padding: 2px 4px; background-color: {THEME.mantle}; border-radius: 3px;"
        )
