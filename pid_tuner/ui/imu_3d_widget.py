"""
3D Visualization Widget for IMU Orientation.
Renders the real-time physical orientation and the target orientation.
"""

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLabel
from PyQt6.QtCore import pyqtSlot
from PyQt6.QtGui import QMatrix4x4, QQuaternion

import numpy as np
import pyqtgraph.opengl as gl

from ..data.data_store import DataStore
from .theme import THEME
from .scaling import SIZES


class ThickAxisItem(gl.GLLinePlotItem):
    """
    Custom axis item with configurable line width.
    Creates X (red), Y (green), Z (blue) axes with thick lines.
    """

    def __init__(self, size=1.0, width=3.0, alpha=1.0, **kwargs):
        # Create axis vertices
        # Each axis: origin to endpoint
        verts = np.array(
            [
                # X axis
                [0, 0, 0],
                [size, 0, 0],
                # Y axis
                [0, 0, 0],
                [0, size, 0],
                # Z axis
                [0, 0, 0],
                [0, 0, size],
            ],
            dtype=np.float32,
        )

        # Colors for each axis (with alpha)
        colors = np.array(
            [
                # X axis - red
                [1, 0, 0, alpha],
                [1, 0, 0, alpha],
                # Y axis - green
                [0, 1, 0, alpha],
                [0, 1, 0, alpha],
                # Z axis - blue
                [0, 0, 1, alpha],
                [0, 0, 1, alpha],
            ],
            dtype=np.float32,
        )

        super().__init__(
            pos=verts, color=colors, width=width, mode="lines", antialias=True, **kwargs
        )


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

        # Title row with legend
        title_layout = QHBoxLayout()
        title = QLabel("3D IMU Visualization")
        title.setStyleSheet(f"font-weight: bold; color: {THEME.text};")
        title_layout.addWidget(title)

        title_layout.addStretch()

        # Legend
        legend = QLabel("Thick: Actual | Thin: Target")
        legend.setStyleSheet(
            f"color: {THEME.subtext0}; font-size: {SIZES['font_small']}pt;"
        )
        title_layout.addWidget(legend)

        layout.addLayout(title_layout)

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

        # Actual Orientation Axes (THICK - very visible)
        self._actual_axis = ThickAxisItem(size=1.5, width=5.0, alpha=1.0)
        self._view.addItem(self._actual_axis)

        # Target Orientation Axes (thin, semi-transparent)
        self._target_axis = ThickAxisItem(size=1.2, width=1.5, alpha=0.5)
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
