"""
3D Visualization Widget for IMU Orientation.
Renders the real-time physical orientation and the target orientation.
Also visualizes self-leveling debug data: Z targets for each actuator and attitude errors.
"""

from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel
from PyQt6.QtCore import pyqtSlot
from PyQt6.QtGui import QMatrix4x4, QQuaternion

import numpy as np
import pyqtgraph.opengl as gl

from ..data.data_store import DataStore
from .theme import THEME
from .scaling import SIZES, scaled


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
    Also visualizes self-leveling debug data: Z targets for each actuator.
    """

    # Actuator positions relative to chassis center (cm, scaled down for visualization)
    # Physical: ML(-34, -31), MR(-34, 31), RC(34, 0)
    # Scale factor to fit in 3D view (physical cm -> view units)
    ACTUATOR_SCALE = 0.03  # 34cm -> ~1 unit

    # Actuator XY positions (scaled)
    ACTUATOR_POS_ML = (-34 * 0.03, -31 * 0.03)  # Mid-Left
    ACTUATOR_POS_MR = (-34 * 0.03, 31 * 0.03)  # Mid-Right
    ACTUATOR_POS_RC = (34 * 0.03, 0)  # Rear-Center

    # Z target scale (encoder ticks -> view units).
    # View is ~5 units wide; typical actuator range is ~200 ticks.
    # 200 * 0.05 = 10.0 view units — spans twice the grid, clearly visible.
    Z_SCALE = 0.05

    def __init__(self, data_store: DataStore, parent=None):
        super().__init__(parent)
        self._data_store = data_store

        self._setup_ui()

        # Connect to IMU updates
        self._data_store.imu_updated.connect(self._on_imu_updated)
        # Connect to leveling updates
        self._data_store.leveling_updated.connect(self._on_leveling_updated)

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

        # Z Target visualization bars (vertical lines at actuator positions)
        self._z_bar_ml = self._create_z_bar(
            self.ACTUATOR_POS_ML, (1.0, 0.5, 0.0, 0.8)
        )  # Orange
        self._z_bar_mr = self._create_z_bar(
            self.ACTUATOR_POS_MR, (0.0, 1.0, 0.5, 0.8)
        )  # Cyan
        self._z_bar_rc = self._create_z_bar(
            self.ACTUATOR_POS_RC, (1.0, 0.0, 1.0, 0.8)
        )  # Magenta
        self._view.addItem(self._z_bar_ml)
        self._view.addItem(self._z_bar_mr)
        self._view.addItem(self._z_bar_rc)

        # Add actuator position markers (small spheres at XY positions)
        self._marker_ml = self._create_actuator_marker(
            self.ACTUATOR_POS_ML, (1.0, 0.5, 0.0, 1.0)
        )
        self._marker_mr = self._create_actuator_marker(
            self.ACTUATOR_POS_MR, (0.0, 1.0, 0.5, 1.0)
        )
        self._marker_rc = self._create_actuator_marker(
            self.ACTUATOR_POS_RC, (1.0, 0.0, 1.0, 1.0)
        )
        self._view.addItem(self._marker_ml)
        self._view.addItem(self._marker_mr)
        self._view.addItem(self._marker_rc)

        # Ground plane — semi-transparent grey quad at z=0
        s = 3.0  # half-extent
        gp_verts = np.array(
            [
                [-s, -s, 0],
                [s, -s, 0],
                [s, s, 0],
                [-s, s, 0],
            ],
            dtype=np.float32,
        )
        gp_faces = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32)
        gp_colors = np.array(
            [
                [0.3, 0.3, 0.35, 0.35],
                [0.3, 0.3, 0.35, 0.35],
            ],
            dtype=np.float32,
        )
        gp_mesh = gl.MeshData(vertexes=gp_verts, faces=gp_faces, faceColors=gp_colors)
        self._ground_plane = gl.GLMeshItem(
            meshdata=gp_mesh,
            smooth=False,
            drawFaces=True,
            drawEdges=False,
            glOptions="translucent",
        )
        self._view.addItem(self._ground_plane)

        # Chassis triangle — connects the tops of the three z-bars.
        # Initialized flat; updated live in _on_leveling_updated.
        tri_verts = np.array(
            [
                [self.ACTUATOR_POS_ML[0], self.ACTUATOR_POS_ML[1], 0.001],
                [self.ACTUATOR_POS_MR[0], self.ACTUATOR_POS_MR[1], 0.001],
                [self.ACTUATOR_POS_RC[0], self.ACTUATOR_POS_RC[1], 0.001],
            ],
            dtype=np.float32,
        )
        tri_faces = np.array([[0, 1, 2]], dtype=np.uint32)
        tri_colors = np.array([[0.6, 0.8, 1.0, 0.45]], dtype=np.float32)
        tri_mesh = gl.MeshData(
            vertexes=tri_verts, faces=tri_faces, faceColors=tri_colors
        )
        self._chassis_triangle = gl.GLMeshItem(
            meshdata=tri_mesh,
            smooth=False,
            drawFaces=True,
            drawEdges=True,
            glOptions="translucent",
        )
        self._view.addItem(self._chassis_triangle)

        # Leveling debug info panel (below 3D view)
        self._leveling_panel = self._create_leveling_panel()
        layout.addWidget(self._leveling_panel)

    def _create_z_bar(self, xy_pos: tuple, color: tuple) -> gl.GLLinePlotItem:
        """Create a vertical bar at the given XY position to show Z target."""
        x, y = xy_pos
        # Use a tiny non-zero initial extent so the GL item is painted on the first
        # frame and the vertex buffer is properly initialized in the OpenGL context.
        pts = np.array([[x, y, 0], [x, y, 0.001]], dtype=np.float32)
        return gl.GLLinePlotItem(pos=pts, color=color, width=4.0, antialias=True)

    def _create_actuator_marker(
        self, xy_pos: tuple, color: tuple
    ) -> gl.GLScatterPlotItem:
        """Create a small marker at the actuator XY position."""
        x, y = xy_pos
        pts = np.array([[x, y, 0]], dtype=np.float32)
        colors = np.array([color], dtype=np.float32)
        return gl.GLScatterPlotItem(pos=pts, color=colors, size=8.0, pxMode=True)

    def _create_leveling_panel(self) -> QWidget:
        """Create a panel showing leveling debug values."""
        panel = QWidget()
        layout = QGridLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(SIZES["spacing_small"])

        # Style for labels
        label_style = f"color: {THEME.subtext0}; font-size: {SIZES['font_small']}pt;"
        value_style = f"color: {THEME.text}; font-size: {SIZES['font_small']}pt; font-weight: bold;"

        # Pitch/Roll Error
        layout.addWidget(self._styled_label("Pitch Err:", label_style), 0, 0)
        self._pitch_err_label = QLabel("---")
        self._pitch_err_label.setStyleSheet(value_style)
        layout.addWidget(self._pitch_err_label, 0, 1)

        layout.addWidget(self._styled_label("Roll Err:", label_style), 0, 2)
        self._roll_err_label = QLabel("---")
        self._roll_err_label.setStyleSheet(value_style)
        layout.addWidget(self._roll_err_label, 0, 3)

        # Z Targets
        layout.addWidget(self._styled_label("Z ML:", label_style), 1, 0)
        self._z_ml_label = QLabel("---")
        self._z_ml_label.setStyleSheet(
            f"color: rgb(255, 128, 0); font-size: {SIZES['font_small']}pt; font-weight: bold;"
        )
        layout.addWidget(self._z_ml_label, 1, 1)

        layout.addWidget(self._styled_label("Z RC:", label_style), 1, 2)
        self._z_rc_label = QLabel("---")
        self._z_rc_label.setStyleSheet(
            f"color: rgb(255, 0, 255); font-size: {SIZES['font_small']}pt; font-weight: bold;"
        )
        layout.addWidget(self._z_rc_label, 1, 3)

        layout.addWidget(self._styled_label("Z MR:", label_style), 1, 4)
        self._z_mr_label = QLabel("---")
        self._z_mr_label.setStyleSheet(
            f"color: rgb(0, 255, 128); font-size: {SIZES['font_small']}pt; font-weight: bold;"
        )
        layout.addWidget(self._z_mr_label, 1, 5)

        return panel

    def _styled_label(self, text: str, style: str) -> QLabel:
        """Create a styled QLabel."""
        label = QLabel(text)
        label.setStyleSheet(style)
        return label

    def _update_z_bar(self, bar: gl.GLLinePlotItem, xy_pos: tuple, z_value: float):
        """Update a Z bar's height based on the Z target value."""
        x, y = xy_pos
        # Scale Z value for visualization (ticks -> view units)
        z_scaled = z_value * self.Z_SCALE
        # Ensure the bar has at least a tiny visible extent even at zero
        if abs(z_scaled) < 0.001:
            z_scaled = 0.001
        pts = np.array([[x, y, 0], [x, y, z_scaled]], dtype=np.float32)
        # Pass antialias=True explicitly — setData resets it to False unconditionally
        bar.setData(pos=pts, antialias=True)
        # Force the view to repaint so the new geometry is visible immediately
        if bar.view() is not None:
            bar.view().update()

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

    @pyqtSlot()
    def _on_leveling_updated(self):
        """Update the leveling debug visualization."""
        # Get leveling data from data store
        pitch_err = self._data_store.leveling_pitch_err
        roll_err = self._data_store.leveling_roll_err
        z_ml = self._data_store.z_target_ml
        z_rc = self._data_store.z_target_rc
        z_mr = self._data_store.z_target_mr

        # Update labels
        self._pitch_err_label.setText(f"{pitch_err:.2f}°")
        self._roll_err_label.setText(f"{roll_err:.2f}°")
        self._z_ml_label.setText(f"{z_ml:.0f}")
        self._z_rc_label.setText(f"{z_rc:.0f}")
        self._z_mr_label.setText(f"{z_mr:.0f}")

        # Color the error labels based on magnitude
        pitch_color = self._get_error_color(pitch_err)
        roll_color = self._get_error_color(roll_err)
        self._pitch_err_label.setStyleSheet(
            f"color: {pitch_color}; font-size: {SIZES['font_small']}pt; font-weight: bold;"
        )
        self._roll_err_label.setStyleSheet(
            f"color: {roll_color}; font-size: {SIZES['font_small']}pt; font-weight: bold;"
        )

        # Update 3D Z bars
        self._update_z_bar(self._z_bar_ml, self.ACTUATOR_POS_ML, z_ml)
        self._update_z_bar(self._z_bar_rc, self.ACTUATOR_POS_RC, z_rc)
        self._update_z_bar(self._z_bar_mr, self.ACTUATOR_POS_MR, z_mr)

        # Update chassis triangle — vertices are the tops of the three z-bars
        def _z(v):
            z = v * self.Z_SCALE
            return z if abs(z) >= 0.001 else 0.001

        tri_verts = np.array(
            [
                [self.ACTUATOR_POS_ML[0], self.ACTUATOR_POS_ML[1], _z(z_ml)],
                [self.ACTUATOR_POS_MR[0], self.ACTUATOR_POS_MR[1], _z(z_mr)],
                [self.ACTUATOR_POS_RC[0], self.ACTUATOR_POS_RC[1], _z(z_rc)],
            ],
            dtype=np.float32,
        )
        tri_faces = np.array([[0, 1, 2]], dtype=np.uint32)
        tri_colors = np.array([[0.6, 0.8, 1.0, 0.45]], dtype=np.float32)
        tri_mesh = gl.MeshData(
            vertexes=tri_verts, faces=tri_faces, faceColors=tri_colors
        )
        self._chassis_triangle.setMeshData(meshdata=tri_mesh)
        self._view.update()

    def _get_error_color(self, error: float) -> str:
        """Get color based on error magnitude."""
        abs_err = abs(error)
        if abs_err < 1.0:
            return "rgb(0, 255, 0)"  # Green - good
        elif abs_err < 3.0:
            return "rgb(255, 255, 0)"  # Yellow - warning
        else:
            return "rgb(255, 0, 0)"  # Red - bad
