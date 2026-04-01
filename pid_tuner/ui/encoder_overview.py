"""
Encoder overview widget showing all 6 encoder positions in a compact layout,
with combined motion controls and saved position buttons.
"""

from PyQt6.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QLabel,
    QSizePolicy,
    QPushButton,
    QFrame,
    QDoubleSpinBox,
)
from PyQt6.QtCore import pyqtSlot, pyqtSignal, Qt, QTimer
from PyQt6.QtGui import QColor, QFont

from .theme import THEME, JOINT_COLORS
from .scaling import SIZES, scaled
from ..data.data_store import DataStore
from ..data.joint_config import JOINTS
from ..serial_driver.serial_handler import SerialHandler

MODE_OPEN_LOOP = 0
MODE_VELOCITY = 1
MODE_POSITION = 2


class JointBox(QFrame):
    clicked = pyqtSignal(int)

    def __init__(self, joint_id: int, name: str, color: str, parent=None):
        super().__init__(parent)
        self._joint_id = joint_id
        self._name = name
        self._color = color
        self._value = 0.0
        self._selected = False

        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFrameShape(QFrame.Shape.StyledPanel)

        layout = QHBoxLayout(self)
        layout.setContentsMargins(
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
            SIZES["margin_small"],
        )
        layout.setSpacing(SIZES["spacing_small"])

        self._name_label = QLabel(f"{name}:")
        self._name_label.setStyleSheet(f"color: {color}; font-weight: bold;")

        self._value_label = QLabel("0.0")
        self._value_label.setMinimumWidth(scaled(45))
        self._value_label.setAlignment(
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter
        )

        layout.addWidget(self._name_label)
        layout.addWidget(self._value_label)

        self._update_style()

    def set_value(self, value: float):
        self._value = value
        self._value_label.setText(f"{value:+.1f}")

    def set_selected(self, selected: bool):
        self._selected = selected
        self._update_style()

    def _update_style(self):
        if self._selected:
            self.setStyleSheet(f"""
                JointBox {{
                    background-color: {THEME.surface1};
                    border: 1px solid {THEME.blue};
                    border-radius: {scaled(4)}px;
                }}
            """)
        else:
            self.setStyleSheet(f"""
                JointBox {{
                    background-color: {THEME.surface0};
                    border: 1px solid {THEME.surface1};
                    border-radius: {scaled(4)}px;
                }}
                JointBox:hover {{
                    background-color: {THEME.surface1};
                    border: 1px solid {THEME.surface2};
                }}
            """)

    def mousePressEvent(self, a0):
        if a0 is not None and a0.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit(self._joint_id)


class SavedPositionButton(QPushButton):
    def __init__(
        self,
        index: int,
        data_store: DataStore,
        serial_handler: SerialHandler,
        parent=None,
    ):
        super().__init__(f"Pos {index}", parent)
        self._index = index
        self._data_store = data_store
        self._serial_handler = serial_handler
        self._saved_positions = None

        self.setToolTip("Not Set — right-click to save current position")
        self._update_style()

    def _update_style(self):
        if self._saved_positions is None:
            self.setStyleSheet(f"""
                QPushButton {{
                    background-color: {THEME.surface0};
                    color: {THEME.overlay0};
                    border: 1px dashed {THEME.surface2};
                }}
                QPushButton:hover {{
                    background-color: {THEME.surface1};
                }}
            """)
        else:
            self.setStyleSheet("")

    def mousePressEvent(self, e):
        if e is not None and e.button() == Qt.MouseButton.RightButton:
            self._saved_positions = {}
            for i in range(1, 7):
                joint_data = self._data_store.get_joint(i)
                if joint_data:
                    self._saved_positions[i] = joint_data.current_position
            self.setToolTip(f"Saved Pos {self._index}\\nLeft-click to go")
            self._update_style()
        elif e is not None and e.button() == Qt.MouseButton.LeftButton:
            if self._saved_positions is not None:
                for joint_id, pos in self._saved_positions.items():
                    self._serial_handler.set_mode(joint_id, MODE_POSITION)
                    self._serial_handler.set_target(joint_id, pos)
                    self._data_store.set_target(joint_id, pos)
            else:
                super().mousePressEvent(e)
        else:
            super().mousePressEvent(e)


class EncoderOverview(QWidget):
    """
    Widget showing all 6 encoder positions in a compact layout,
    with combined motion controls and saved position buttons.
    """

    joint_selected = pyqtSignal(int)

    UPDATE_INTERVAL_MS = 100

    def __init__(
        self, data_store: DataStore, serial_handler: SerialHandler, parent=None
    ):
        super().__init__(parent)
        self._data_store = data_store
        self._serial_handler = serial_handler
        self._boxes = []
        self._selected_joint = 1

        self._setup_ui()
        self._setup_timer()

        self._data_store.limits_updated.connect(self._update_limits)
        self._data_store.config_updated.connect(self._on_config_updated)

    def _setup_ui(self):
        root = QVBoxLayout(self)
        root.setContentsMargins(
            SIZES["margin_medium"],
            SIZES["margin_small"],
            SIZES["margin_medium"],
            SIZES["margin_small"],
        )
        root.setSpacing(SIZES["spacing_medium"])

        boxes_layout = QHBoxLayout()
        boxes_layout.setSpacing(SIZES["spacing_small"])

        for i, joint in enumerate(JOINTS):
            color = JOINT_COLORS[i] if i < len(JOINT_COLORS) else THEME.text
            box = JointBox(
                joint_id=joint.id,
                name=joint.short_name,
                color=color,
            )
            box.clicked.connect(self._on_box_clicked)
            self._boxes.append(box)
            boxes_layout.addWidget(box)

        boxes_layout.addStretch()
        root.addLayout(boxes_layout)

        controls_layout = QHBoxLayout()
        controls_layout.setSpacing(SIZES["spacing_medium"])

        controls_layout.addWidget(QLabel("Carriage (5,6):"))
        self._carriage_spin = QDoubleSpinBox()
        self._carriage_spin.setRange(-10000, 10000)
        self._carriage_spin.setDecimals(1)
        self._carriage_spin.setSingleStep(10.0)
        controls_layout.addWidget(self._carriage_spin)

        btn_carriage = QPushButton("Go")
        btn_carriage.clicked.connect(self._on_carriage_go)
        controls_layout.addWidget(btn_carriage)

        controls_layout.addSpacing(SIZES["spacing_large"])

        controls_layout.addWidget(QLabel("Legs (3,4):"))
        self._legs_spin = QDoubleSpinBox()
        self._legs_spin.setRange(-10000, 10000)
        self._legs_spin.setDecimals(1)
        self._legs_spin.setSingleStep(10.0)
        controls_layout.addWidget(self._legs_spin)

        btn_legs = QPushButton("Go")
        btn_legs.clicked.connect(self._on_legs_go)
        controls_layout.addWidget(btn_legs)

        controls_layout.addStretch()

        for i in range(1, 7):
            btn = SavedPositionButton(i, self._data_store, self._serial_handler)
            controls_layout.addWidget(btn)

        root.addLayout(controls_layout)

        if self._boxes:
            self._boxes[0].set_selected(True)

    def _setup_timer(self):
        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self._update_values)
        self._update_timer.start(self.UPDATE_INTERVAL_MS)

    def _update_values(self):
        for i, box in enumerate(self._boxes):
            joint_data = self._data_store.get_joint(JOINTS[i].id)
            if joint_data:
                box.set_value(joint_data.current_position)

    def _on_box_clicked(self, joint_id: int):
        self.set_selected_joint(joint_id)
        self.joint_selected.emit(joint_id)

    def _on_carriage_go(self):
        val = self._carriage_spin.value()
        for j in (5, 6):
            self._serial_handler.set_mode(j, MODE_POSITION)
            self._serial_handler.set_target(j, val)
            self._data_store.set_target(j, val)

    def _on_legs_go(self):
        val = self._legs_spin.value()
        for j in (3, 4):
            self._serial_handler.set_mode(j, MODE_POSITION)
            self._serial_handler.set_target(j, val)
            self._data_store.set_target(j, val)

    @pyqtSlot(int)
    def set_selected_joint(self, joint_id: int):
        self._selected_joint = joint_id

        for i, box in enumerate(self._boxes):
            box.set_selected(JOINTS[i].id == joint_id)

    def set_range(self, min_val: float, max_val: float):
        pass

    def _update_limits(self):
        pass

    def _on_config_updated(self, joint_id: int):
        pass

    def set_mode_for_joint(self, joint_id: int, mode: int):
        pass

    def set_mode_for_all(self, mode: int):
        pass
