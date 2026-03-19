"""
3D Visualization Widget for IMU Orientation.
Renders the real-time physical orientation and the target orientation.
"""

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QLabel
from PyQt6.QtCore import pyqtSlot
from PyQt6.QtGui import QMatrix4x4, QQuaternion

import pyqtgraph.opengl as gl

from ..data.data_store import DataStore
from .theme import THEME
from .scaling import SIZES


class IMU3DWidget(QWidget):
    """
    Renders 3D coordinate axes showing the actual and target orientation
    of the robot chassis using raw quaternion data to avoid gimbal lock.
    """

    def __init__(self, data_store: DataStore, parent=None):
        super().__init__(parent)
        self._data_store = data_store

        self._setup_ui()

        # Connect to IMU updates
        self._data_store.imu_updated.connect(self._on_imu_updated)

    def _setup_ui(self):
        """Set up the 3D viewer."""
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(SIZES["spacing_small"])

        # Title
        title = QLabel("3D IMU Visualization")
        title.setStyleSheet(f"font-weight: bold; color: {THEME.text};")
        layout.addWidget(title)

        # Create OpenGL widget
        self._view = gl.GLViewWidget()
        self._view.setMinimumHeight(250)
        self._view.opts["distance"] = 3.0  # Camera distance
        self._view.setBackgroundColor(THEME.base)
        layout.addWidget(self._view)

        # Add floor grid
        self._grid = gl.GLGridItem()
        self._grid.setSize(x=5, y=5, z=5)
        self._grid.setSpacing(x=1, y=1, z=1)
        self._view.addItem(self._grid)

        # Actual Orientation Axes (Thick, solid)
        self._actual_axis = gl.GLAxisItem()
        self._actual_axis.setSize(x=1.5, y=1.5, z=1.5)
        self._view.addItem(self._actual_axis)

        # Target Orientation Axes (Thinner, translucent)
        # By default GLAxisItem uses standard RGB for XYZ. We'll use a second axis item.
        self._target_axis = gl.GLAxisItem()
        self._target_axis.setSize(x=1.2, y=1.2, z=1.2)
        self._view.addItem(self._target_axis)

    @pyqtSlot()
    def _on_imu_updated(self):
        """Update the 3D transforms based on new IMU data."""
        # 1. Update Actual Orientation
        qw = self._data_store.imu_qw
        qx = self._data_store.imu_qx
        qy = self._data_store.imu_qy
        qz = self._data_store.imu_qz

        # Create QQuaternion. Note: PyQt uses QQuaternion(w, x, y, z) or QQuaternion(scalar, xpos, ypos, zpos)
        actual_quat = QQuaternion(qw, qx, qy, qz)

        # Apply to actual axis
        actual_transform = QMatrix4x4()
        actual_transform.rotate(actual_quat)
        self._actual_axis.setTransform(actual_transform)

        # 2. Update Target Orientation
        # Replicate the firmware logic to construct the target quaternion
        target_pitch = self._data_store.imu_target_pitch
        target_roll = self._data_store.imu_target_roll

        # In firmware:
        # Pitch = rotation around X.
        # Roll = rotation around Y (offset by 180).
        q_target_pitch = QQuaternion.fromAxisAndAngle(1.0, 0.0, 0.0, target_pitch)
        q_target_roll = QQuaternion.fromAxisAndAngle(0.0, 1.0, 0.0, target_roll - 180.0)

        # Target orientation = q_target_pitch * q_target_roll
        target_quat = q_target_pitch * q_target_roll

        # Apply to target axis
        target_transform = QMatrix4x4()
        target_transform.rotate(target_quat)
        self._target_axis.setTransform(target_transform)
