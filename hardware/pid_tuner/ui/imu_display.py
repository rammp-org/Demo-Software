"""
IMU display widget showing orientation and accelerometer data.
"""

from PyQt6.QtWidgets import (
    QWidget,
    QVBoxLayout,
    QGridLayout,
    QGroupBox,
    QLabel,
)
from PyQt6.QtCore import pyqtSlot

from ..data.data_store import DataStore
from .theme import THEME
from .scaling import SIZES


class IMUDisplay(QWidget):
    """
    Widget displaying IMU orientation and accelerometer data.

    Features:
    - Pitch, Roll, Yaw display with color coding
    - Accelerometer X, Y, Z display
    - Updates on data store signals
    """

    # Thresholds for level detection (degrees)
    LEVEL_THRESHOLD = 2.0
    WARNING_THRESHOLD = 5.0

    def __init__(self, data_store: DataStore, parent=None):
        super().__init__(parent)
        self._data_store = data_store
        self._setup_ui()

        # Connect to data store updates
        self._data_store.imu_updated.connect(self._update_display)

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        group = QGroupBox("IMU Status")
        group_layout = QGridLayout(group)
        group_layout.setSpacing(SIZES["spacing_small"])

        # Orientation section
        group_layout.addWidget(QLabel("Orientation"), 0, 0, 1, 2)

        group_layout.addWidget(QLabel("Pitch:"), 1, 0)
        self._pitch_label = QLabel("---")
        self._pitch_label.setStyleSheet("font-weight: bold;")
        group_layout.addWidget(self._pitch_label, 1, 1)

        group_layout.addWidget(QLabel("Roll:"), 2, 0)
        self._roll_label = QLabel("---")
        self._roll_label.setStyleSheet("font-weight: bold;")
        group_layout.addWidget(self._roll_label, 2, 1)

        group_layout.addWidget(QLabel("Yaw:"), 3, 0)
        self._yaw_label = QLabel("---")
        self._yaw_label.setStyleSheet("font-weight: bold;")
        group_layout.addWidget(self._yaw_label, 3, 1)

        # Accelerometer section
        group_layout.addWidget(QLabel("Accel (m/s^2)"), 4, 0, 1, 2)

        group_layout.addWidget(QLabel("X:"), 5, 0)
        self._ax_label = QLabel("---")
        group_layout.addWidget(self._ax_label, 5, 1)

        group_layout.addWidget(QLabel("Y:"), 6, 0)
        self._ay_label = QLabel("---")
        group_layout.addWidget(self._ay_label, 6, 1)

        group_layout.addWidget(QLabel("Z:"), 7, 0)
        self._az_label = QLabel("---")
        group_layout.addWidget(self._az_label, 7, 1)

        layout.addWidget(group)

    def _get_angle_color(self, angle: float) -> str:
        """Get color for angle based on magnitude."""
        abs_angle = abs(angle)
        if abs_angle < self.LEVEL_THRESHOLD:
            return THEME.green
        elif abs_angle < self.WARNING_THRESHOLD:
            return THEME.yellow
        else:
            return THEME.red

    @pyqtSlot()
    def _update_display(self):
        """Update display with latest IMU data."""
        pitch = self._data_store.imu_pitch
        roll = self._data_store.imu_roll
        yaw = self._data_store.imu_yaw
        ax = self._data_store.imu_ax
        ay = self._data_store.imu_ay
        az = self._data_store.imu_az

        # Update orientation with color coding
        self._pitch_label.setText(f"{pitch:+.2f} deg")
        self._pitch_label.setStyleSheet(
            f"font-weight: bold; color: {self._get_angle_color(pitch)};"
        )

        self._roll_label.setText(f"{roll:+.2f} deg")
        self._roll_label.setStyleSheet(
            f"font-weight: bold; color: {self._get_angle_color(roll)};"
        )

        self._yaw_label.setText(f"{yaw:.1f} deg")

        # Update accelerometer
        self._ax_label.setText(f"{ax:+.2f}")
        self._ay_label.setText(f"{ay:+.2f}")
        self._az_label.setText(f"{az:+.2f}")
