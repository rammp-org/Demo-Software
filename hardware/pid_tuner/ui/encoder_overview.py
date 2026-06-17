"""
Encoder overview widget showing encoder positions in a compact layout,
with combined motion controls and saved position buttons.

Includes RoboClaw joints 1–8 (JOINTS) and hub motor actuators 9–10.
"""

from PyQt6.QtWidgets import (
    QWidget,
    QHBoxLayout,
    QVBoxLayout,
    QGridLayout,
    QLabel,
    QSizePolicy,
    QPushButton,
    QFrame,
    QDoubleSpinBox,
)
from PyQt6.QtCore import pyqtSlot, pyqtSignal, Qt, QTimer
from PyQt6.QtGui import QColor, QPainter, QPen, QBrush

from .theme import THEME, JOINT_COLORS
from .scaling import SIZES, scaled
from ..data.data_store import DataStore
from ..data.joint_config import JOINTS, is_hub_motor_actuator
from ..serial_driver.serial_handler import SerialHandler

MODE_OPEN_LOOP = 0
MODE_VELOCITY = 1
MODE_POSITION = 2

LIMIT_SWITCH_NAMES = ("ML Fwd", "ML Bwd", "MR Fwd", "MR Bwd")


class JointBox(QFrame):
    clicked = pyqtSignal(int)

    def __init__(self, joint_id: int, name: str, color: str, parent=None):
        super().__init__(parent)
        self._joint_id = joint_id
        self._name = name
        self._color = color
        self._value = 0.0
        self._selected = False
        self._min_val = -10000.0
        self._max_val = 10000.0

        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setFrameShape(QFrame.Shape.StyledPanel)

        self.setFixedHeight(scaled(22))
        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    def set_value(self, value: float):
        self._value = value
        self.update()

    def set_limits(self, min_val: float, max_val: float):
        self._min_val = min_val
        self._max_val = max_val
        self.update()

    def set_selected(self, selected: bool):
        self._selected = selected
        self.update()

    def paintEvent(self, a0):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        rect = self.rect()

        bg_color = QColor(THEME.surface1 if self._selected else THEME.surface0)
        if not self._selected and self.underMouse():
            bg_color = QColor(THEME.surface1)
        painter.fillRect(rect, bg_color)

        range_val = self._max_val - self._min_val
        if range_val > 0:
            pct = (self._value - self._min_val) / range_val
            pct = max(0.0, min(1.0, pct))
        else:
            pct = 0.0

        if pct > 0:
            fill_rect = rect.adjusted(1, 1, -1, -1)
            fill_rect.setWidth(int(fill_rect.width() * pct))
            fill_color = QColor(self._color)
            painter.setBrush(QBrush(fill_color))
            painter.setPen(Qt.PenStyle.NoPen)
            painter.drawRoundedRect(fill_rect, scaled(3), scaled(3))

        border_color = QColor(THEME.blue if self._selected else THEME.surface1)
        if not self._selected and self.underMouse():
            border_color = QColor(THEME.surface2)

        pen = QPen(border_color)
        pen.setWidth(1)
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawRoundedRect(rect.adjusted(0, 0, -1, -1), scaled(4), scaled(4))

        marker_color = QColor(THEME.red)
        painter.setPen(QPen(marker_color, 2))
        painter.drawLine(1, 1, 1, rect.height() - 2)
        painter.drawLine(rect.width() - 2, 1, rect.width() - 2, rect.height() - 2)

        font = self.font()
        font.setBold(True)
        painter.setFont(font)

        name_text = f"{self._name}:"
        value_text = f"{self._value:+.1f}"
        text_rect_left = rect.adjusted(SIZES["margin_small"], 0, 0, 0)
        text_rect_right = rect.adjusted(0, 0, -SIZES["margin_small"], 0)

        painter.setPen(QColor(THEME.crust))
        painter.drawText(
            text_rect_left.adjusted(1, 1, 1, 1),
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            name_text,
        )
        painter.setPen(QColor(THEME.text))
        painter.drawText(
            text_rect_left,
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            name_text,
        )

        font.setBold(False)
        painter.setFont(font)
        painter.setPen(QColor(THEME.crust))
        painter.drawText(
            text_rect_right.adjusted(1, 1, 1, 1),
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            value_text,
        )
        painter.setPen(QColor(THEME.text))
        painter.drawText(
            text_rect_right,
            Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
            value_text,
        )

    def mousePressEvent(self, a0):
        if a0 is not None and a0.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit(self._joint_id)

    def enterEvent(self, event):
        self.update()
        super().enterEvent(event)

    def leaveEvent(self, a0):
        self.update()
        super().leaveEvent(a0)


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
    Widget showing joint and hub motor positions, combined motion controls,
    and saved position buttons.
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

        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)

        self._setup_ui()
        self._setup_timer()

        self._data_store.limits_updated.connect(self._update_limit_switches)
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

        boxes_layout = QVBoxLayout()
        boxes_layout.setSpacing(2)

        for i, joint in enumerate(JOINTS):
            color = JOINT_COLORS[i] if i < len(JOINT_COLORS) else THEME.text
            box = JointBox(
                joint_id=joint.id,
                name=joint.short_name,
                color=color,
            )
            if is_hub_motor_actuator(joint.id):
                box.set_limits(-50.0, 50.0)
            box.clicked.connect(self._on_box_clicked)
            self._boxes.append(box)
            boxes_layout.addWidget(box)

        root.addLayout(boxes_layout, stretch=1)

        controls_layout = QHBoxLayout()
        controls_layout.setSpacing(SIZES["spacing_medium"])

        controls_layout.addWidget(QLabel("Carriage (5,6):"))
        self._carriage_spin = QDoubleSpinBox()
        self._carriage_spin.setRange(-100000, 100000)
        self._carriage_spin.setDecimals(1)
        self._carriage_spin.setSingleStep(500.0)
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

        root.addLayout(controls_layout)

        limit_row = QHBoxLayout()
        limit_row.setSpacing(SIZES["spacing_small"])
        limit_title = QLabel("Limit switches:")
        limit_title.setStyleSheet(
            f"color: {THEME.subtext0}; font-size: {SIZES['font_small']}pt;"
        )
        limit_row.addWidget(limit_title)
        self._limit_switch_labels: list[QLabel] = []
        for name in LIMIT_SWITCH_NAMES:
            name_lbl = QLabel(f"{name}:")
            name_lbl.setStyleSheet(
                f"color: {THEME.subtext1}; font-size: {SIZES['font_small']}pt;"
            )
            limit_row.addWidget(name_lbl)
            val_lbl = QLabel("0")
            val_lbl.setFixedWidth(scaled(16))
            val_lbl.setAlignment(Qt.AlignmentFlag.AlignCenter)
            val_lbl.setStyleSheet(
                f"color: {THEME.text}; font-size: {SIZES['font_small']}pt; font-weight: bold;"
            )
            self._limit_switch_labels.append(val_lbl)
            limit_row.addWidget(val_lbl)
        limit_row.addStretch()
        root.addLayout(limit_row)

        hub_controls = QHBoxLayout()
        hub_controls.setSpacing(SIZES["spacing_medium"])

        hub_controls.addWidget(QLabel("Hub motors (9,10) Δ turns:"))
        self._hub_motor_spin = QDoubleSpinBox()
        self._hub_motor_spin.setRange(-100.0, 100.0)
        self._hub_motor_spin.setDecimals(2)
        self._hub_motor_spin.setSingleStep(0.1)
        self._hub_motor_spin.setValue(0.1)
        self._hub_motor_spin.setToolTip(
            "Relative delta applied to both hub motor R and L (robot-frame turns)"
        )
        hub_controls.addWidget(self._hub_motor_spin)

        btn_hub = QPushButton("Go")
        btn_hub.setToolTip(
            "Position mode: target = current position + delta for HM_R and HM_L"
        )
        btn_hub.clicked.connect(self._on_hub_motor_go)
        hub_controls.addWidget(btn_hub)

        root.addLayout(hub_controls)

        pos_grid = QGridLayout()
        pos_grid.setSpacing(SIZES["spacing_small"])
        for i in range(6):
            btn = SavedPositionButton(i + 1, self._data_store, self._serial_handler)
            pos_grid.addWidget(btn, i // 3, i % 3)

        root.addLayout(pos_grid)

        if self._boxes:
            self._boxes[0].set_selected(True)

    def _setup_timer(self):
        self._update_timer = QTimer(self)
        self._update_timer.timeout.connect(self._update_values)
        self._update_timer.start(self.UPDATE_INTERVAL_MS)

    def _update_values(self):
        for i, joint in enumerate(JOINTS):
            if i >= len(self._boxes):
                break
            joint_data = self._data_store.get_joint(joint.id)
            if joint_data:
                self._boxes[i].set_value(joint_data.current_position)
        self._update_limit_switches()

    def _update_limit_switches(self):
        states = self._data_store.limit_switches
        for i, lbl in enumerate(self._limit_switch_labels):
            active = bool(states[i]) if i < len(states) else False
            lbl.setText("1" if active else "0")
            color = THEME.red if active else THEME.text
            lbl.setStyleSheet(
                f"color: {color}; font-size: {SIZES['font_small']}pt; font-weight: bold;"
            )

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

    def _on_hub_motor_go(self):
        delta = self._hub_motor_spin.value()
        self._serial_handler.clear_estop()
        hub_current = {
            9: self._data_store.hub_motor_l_pos,
            10: self._data_store.hub_motor_r_pos,
        }
        for joint_id in (9, 10):
            target = hub_current[joint_id] + delta
            self._serial_handler.set_mode(joint_id, MODE_POSITION)
            self._serial_handler.set_target(joint_id, target)
            self._data_store.set_target(joint_id, target)

    @pyqtSlot(int)
    def set_selected_joint(self, joint_id: int):
        self._selected_joint = joint_id

        for box in self._boxes:
            box.set_selected(box._joint_id == joint_id)

    def set_range(self, min_val: float, max_val: float):
        pass

    def _update_limits(self):
        """Reserved for joint position limit config updates."""
        pass

    def _on_config_updated(self, joint_id: int):
        config = self._data_store.get_config(joint_id)
        if not config:
            return
        for box in self._boxes:
            if box._joint_id == joint_id:
                box.set_limits(config.pos_limit_min, config.pos_limit_max)
                break

    def set_mode_for_joint(self, joint_id: int, mode: int):
        pass

    def set_mode_for_all(self, mode: int):
        pass
