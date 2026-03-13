"""
Data store for encoder data with rolling buffer.
"""

import time
from collections import deque
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import numpy as np
from PyQt6.QtCore import QObject, pyqtSignal, QTimer

from ..serial_driver.protocol import EncoderData


@dataclass
class JointData:
    """Data for a single joint."""

    joint_id: int
    max_samples: int = 2000  # ~10 seconds at 200Hz

    # Rolling buffers
    timestamps: deque = field(default_factory=lambda: deque(maxlen=2000))
    positions: deque = field(default_factory=lambda: deque(maxlen=2000))
    velocities: deque = field(default_factory=lambda: deque(maxlen=2000))
    pwms: deque = field(default_factory=lambda: deque(maxlen=2000))
    targets: deque = field(default_factory=lambda: deque(maxlen=2000))

    # Current state
    current_position: float = 0.0
    current_velocity: float = 0.0
    current_pwm: float = 0.0
    current_target: float = 0.0

    # Start time for relative timestamps
    _start_time: int = 0
    _start_time_real: float = 0  # For simulation mode

    def __post_init__(self):
        # Reinitialize deques with correct maxlen after dataclass creation
        self.timestamps = deque(maxlen=self.max_samples)
        self.positions = deque(maxlen=self.max_samples)
        self.velocities = deque(maxlen=self.max_samples)
        self.pwms = deque(maxlen=self.max_samples)
        self.targets = deque(maxlen=self.max_samples)
        self._start_time = 0
        self._start_time_real = 0

    def add_sample(
        self,
        timestamp_ms: int,
        position: float,
        velocity: float = 0.0,
        pwm: float = 0.0,
        target: Optional[float] = None,
    ):
        """Add a new sample to the rolling buffer."""
        # Convert timestamp to seconds from start
        if len(self.timestamps) == 0:
            self._start_time = timestamp_ms

        time_s = (timestamp_ms - self._start_time) / 1000.0

        self.timestamps.append(time_s)
        self.positions.append(position)
        self.velocities.append(velocity)
        self.pwms.append(pwm)
        self.current_position = position
        self.current_velocity = velocity
        self.current_pwm = pwm

        # Use provided target or maintain current target
        if target is not None:
            self.current_target = target
        self.targets.append(self.current_target)

    def add_simulated_sample(self, target: float):
        """Add a simulated sample with synthetic timestamp (for preview mode)."""
        # Initialize start time on first sample
        if self._start_time_real == 0:
            self._start_time_real = time.time()

        time_s = time.time() - self._start_time_real

        self.timestamps.append(time_s)
        # In simulation, position follows target (no actual motor)
        self.positions.append(target)
        self.velocities.append(0.0)
        self.pwms.append(0.0)
        self.targets.append(target)
        self.current_position = target
        self.current_velocity = 0.0
        self.current_pwm = 0.0
        self.current_target = target

    def set_target(self, target: float):
        """Set the current target position."""
        self.current_target = target

    def get_plot_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get data arrays for plotting (position data).

        Returns:
            Tuple of (timestamps, positions, targets) as numpy arrays
        """
        return (
            np.array(self.timestamps),
            np.array(self.positions),
            np.array(self.targets),
        )

    def get_velocity_data(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get velocity data for plotting.

        Returns:
            Tuple of (timestamps, velocities) as numpy arrays
        """
        return (np.array(self.timestamps), np.array(self.velocities))

    def get_pwm_data(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get PWM data for plotting.

        Returns:
            Tuple of (timestamps, pwms) as numpy arrays
        """
        return (np.array(self.timestamps), np.array(self.pwms))

    def clear(self):
        """Clear all stored data."""
        self.timestamps.clear()
        self.positions.clear()
        self.velocities.clear()
        self.pwms.clear()
        self.targets.clear()
        self._start_time = 0


class DataStore(QObject):
    """
    Central data store for all joint encoder data.

    Signals:
        data_updated: Emitted when new data is available
        simulation_changed: Emitted when simulation mode changes
        state_changed: Emitted when system state changes
    """

    data_updated = pyqtSignal(int)  # Emits joint_id that was updated
    simulation_changed = pyqtSignal(bool)  # Emitted when simulation mode changes
    state_changed = pyqtSignal(int)  # Emitted when system state changes

    NUM_JOINTS = 6
    DEFAULT_MAX_SAMPLES = 2000  # ~10 seconds at 200Hz
    SIMULATION_UPDATE_MS = 20  # 50 Hz simulation rate

    def __init__(self, parent=None, max_samples: int = DEFAULT_MAX_SAMPLES):
        super().__init__(parent)
        self._joints: List[JointData] = [
            JointData(joint_id=i + 1, max_samples=max_samples)
            for i in range(self.NUM_JOINTS)
        ]
        self._selected_joint: int = 1
        self._simulation_mode: bool = False
        self._current_state: int = 0  # System state from telemetry

        # Simulation timer for generating synthetic data points
        self._simulation_timer = QTimer(self)
        self._simulation_timer.timeout.connect(self._on_simulation_tick)

    @property
    def simulation_mode(self) -> bool:
        """Check if simulation mode is active."""
        return self._simulation_mode

    @simulation_mode.setter
    def simulation_mode(self, enabled: bool):
        """Enable or disable simulation mode."""
        if self._simulation_mode != enabled:
            self._simulation_mode = enabled
            if enabled:
                self._simulation_timer.start(self.SIMULATION_UPDATE_MS)
            else:
                self._simulation_timer.stop()
            self.simulation_changed.emit(enabled)

    def _on_simulation_tick(self):
        """Generate a simulated data point for the selected joint."""
        joint_data = self._joints[self._selected_joint - 1]
        joint_data.add_simulated_sample(joint_data.current_target)
        self.data_updated.emit(self._selected_joint)

    @property
    def selected_joint(self) -> int:
        """Get the currently selected joint (1-indexed)."""
        return self._selected_joint

    @selected_joint.setter
    def selected_joint(self, joint_id: int):
        """Set the currently selected joint (1-indexed)."""
        if 1 <= joint_id <= self.NUM_JOINTS:
            self._selected_joint = joint_id

    @property
    def current_state(self) -> int:
        """Get the current system state."""
        return self._current_state

    def get_joint(self, joint_id: int) -> Optional[JointData]:
        """Get JointData for a specific joint (1-indexed)."""
        if 1 <= joint_id <= self.NUM_JOINTS:
            return self._joints[joint_id - 1]
        return None

    def get_selected_joint_data(self) -> JointData:
        """Get JointData for the currently selected joint."""
        return self._joints[self._selected_joint - 1]

    def process_encoder_data(self, data: EncoderData):
        """
        Process incoming telemetry data and update all joints.

        Args:
            data: Parsed telemetry data from Teensy (positions, velocities, pwms)
        """
        # Update system state
        if data.state != self._current_state:
            self._current_state = data.state
            self.state_changed.emit(data.state)

        # Update joint data
        for i in range(min(len(data.position_values), self.NUM_JOINTS)):
            position = data.position_values[i]
            velocity = data.velocity_values[i] if i < len(data.velocity_values) else 0.0
            pwm = data.pwm_values[i] if i < len(data.pwm_values) else 0.0
            self._joints[i].add_sample(data.timestamp_ms, position, velocity, pwm)

        self.data_updated.emit(self._selected_joint)

    def set_target(self, joint_id: int, target: float):
        """Set target position for a joint."""
        if 1 <= joint_id <= self.NUM_JOINTS:
            self._joints[joint_id - 1].set_target(target)

    def clear_all(self):
        """Clear all stored data."""
        for joint in self._joints:
            joint.clear()

    def clear_joint(self, joint_id: int):
        """Clear data for a specific joint."""
        if 1 <= joint_id <= self.NUM_JOINTS:
            self._joints[joint_id - 1].clear()
