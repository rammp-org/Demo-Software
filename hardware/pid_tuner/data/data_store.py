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
from .joint_config import (
    FC_MOTOR_BACKEND_HUB,
    FC_MOTOR_BACKEND_ODRIVE,
    is_fc_motor_actuator,
)


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

    # Cached numpy arrays (avoid repeated conversions)
    _cache_dirty: bool = True
    _cached_timestamps: np.ndarray = field(default_factory=lambda: np.array([]))
    _cached_positions: np.ndarray = field(default_factory=lambda: np.array([]))
    _cached_velocities: np.ndarray = field(default_factory=lambda: np.array([]))
    _cached_pwms: np.ndarray = field(default_factory=lambda: np.array([]))
    _cached_targets: np.ndarray = field(default_factory=lambda: np.array([]))

    def __post_init__(self):
        # Reinitialize deques with correct maxlen after dataclass creation
        self.timestamps = deque(maxlen=self.max_samples)
        self.positions = deque(maxlen=self.max_samples)
        self.velocities = deque(maxlen=self.max_samples)
        self.pwms = deque(maxlen=self.max_samples)
        self.targets = deque(maxlen=self.max_samples)
        self._start_time = 0
        self._start_time_real = 0
        self._cache_dirty = True
        self._cached_timestamps = np.array([])
        self._cached_positions = np.array([])
        self._cached_velocities = np.array([])
        self._cached_pwms = np.array([])
        self._cached_targets = np.array([])

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

        # Mark cache as dirty
        self._cache_dirty = True

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

        # Mark cache as dirty
        self._cache_dirty = True

    def set_target(self, target: float):
        """Set the current target position."""
        self.current_target = target

    def _rebuild_cache(self):
        """Rebuild the cached numpy arrays from deques."""
        if not self._cache_dirty:
            return

        self._cached_timestamps = np.array(self.timestamps)
        self._cached_positions = np.array(self.positions)
        self._cached_velocities = np.array(self.velocities)
        self._cached_pwms = np.array(self.pwms)
        self._cached_targets = np.array(self.targets)
        self._cache_dirty = False

    def get_plot_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Get data arrays for plotting (position data).

        Uses cached numpy arrays to avoid repeated deque-to-array conversions.

        Returns:
            Tuple of (timestamps, positions, targets) as numpy arrays
        """
        self._rebuild_cache()
        return (
            self._cached_timestamps,
            self._cached_positions,
            self._cached_targets,
        )

    def get_velocity_data(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get velocity data for plotting.

        Uses cached numpy arrays to avoid repeated deque-to-array conversions.

        Returns:
            Tuple of (timestamps, velocities) as numpy arrays
        """
        self._rebuild_cache()
        return (self._cached_timestamps, self._cached_velocities)

    def get_pwm_data(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get PWM data for plotting.

        Uses cached numpy arrays to avoid repeated deque-to-array conversions.

        Returns:
            Tuple of (timestamps, pwms) as numpy arrays
        """
        self._rebuild_cache()
        return (self._cached_timestamps, self._cached_pwms)

    def clear(self):
        """Clear all stored data."""
        self.timestamps.clear()
        self.positions.clear()
        self.velocities.clear()
        self.pwms.clear()
        self.targets.clear()
        self._start_time = 0
        self._start_time_real = 0
        self._cache_dirty = True
        self._cached_timestamps = np.array([])
        self._cached_positions = np.array([])
        self._cached_velocities = np.array([])
        self._cached_pwms = np.array([])
        self._cached_targets = np.array([])


@dataclass
class IMUData:
    """Data for IMU with rolling buffer for graphing."""

    max_samples: int = 2000  # ~10 seconds at 200Hz

    # Rolling buffers
    timestamps: deque = field(default_factory=lambda: deque(maxlen=2000))
    pitch: deque = field(default_factory=lambda: deque(maxlen=2000))
    roll: deque = field(default_factory=lambda: deque(maxlen=2000))
    yaw: deque = field(default_factory=lambda: deque(maxlen=2000))
    ax: deque = field(default_factory=lambda: deque(maxlen=2000))
    ay: deque = field(default_factory=lambda: deque(maxlen=2000))
    az: deque = field(default_factory=lambda: deque(maxlen=2000))

    # Start time for relative timestamps
    _start_time: int = 0

    # Cached numpy arrays
    _cache_dirty: bool = True
    _cached_timestamps: np.ndarray = field(default_factory=lambda: np.array([]))
    _cached_pitch: np.ndarray = field(default_factory=lambda: np.array([]))
    _cached_roll: np.ndarray = field(default_factory=lambda: np.array([]))
    _cached_yaw: np.ndarray = field(default_factory=lambda: np.array([]))
    _cached_ax: np.ndarray = field(default_factory=lambda: np.array([]))
    _cached_ay: np.ndarray = field(default_factory=lambda: np.array([]))
    _cached_az: np.ndarray = field(default_factory=lambda: np.array([]))

    def __post_init__(self):
        # Reinitialize deques with correct maxlen after dataclass creation
        self.timestamps = deque(maxlen=self.max_samples)
        self.pitch = deque(maxlen=self.max_samples)
        self.roll = deque(maxlen=self.max_samples)
        self.yaw = deque(maxlen=self.max_samples)
        self.ax = deque(maxlen=self.max_samples)
        self.ay = deque(maxlen=self.max_samples)
        self.az = deque(maxlen=self.max_samples)
        self._start_time = 0
        self._cache_dirty = True

    def add_sample(
        self,
        timestamp_ms: int,
        pitch: float,
        roll: float,
        yaw: float,
        ax: float,
        ay: float,
        az: float,
    ):
        """Add a new IMU sample to the rolling buffer."""
        if len(self.timestamps) == 0:
            self._start_time = timestamp_ms

        time_s = (timestamp_ms - self._start_time) / 1000.0

        self.timestamps.append(time_s)
        self.pitch.append(pitch)
        self.roll.append(roll)
        self.yaw.append(yaw)
        self.ax.append(ax)
        self.ay.append(ay)
        self.az.append(az)

        self._cache_dirty = True

    def _rebuild_cache(self):
        """Rebuild the cached numpy arrays from deques."""
        if not self._cache_dirty:
            return

        self._cached_timestamps = np.array(self.timestamps)
        self._cached_pitch = np.array(self.pitch)
        self._cached_roll = np.array(self.roll)
        self._cached_yaw = np.array(self.yaw)
        self._cached_ax = np.array(self.ax)
        self._cached_ay = np.array(self.ay)
        self._cached_az = np.array(self.az)
        self._cache_dirty = False

    def get_orientation_data(
        self,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get orientation data for plotting (timestamps, pitch, roll, yaw)."""
        self._rebuild_cache()
        return (
            self._cached_timestamps,
            self._cached_pitch,
            self._cached_roll,
            self._cached_yaw,
        )

    def get_accel_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get acceleration data for plotting (timestamps, ax, ay, az)."""
        self._rebuild_cache()
        return (
            self._cached_timestamps,
            self._cached_ax,
            self._cached_ay,
            self._cached_az,
        )

    def clear(self):
        """Clear all stored data."""
        self.timestamps.clear()
        self.pitch.clear()
        self.roll.clear()
        self.yaw.clear()
        self.ax.clear()
        self.ay.clear()
        self.az.clear()
        self._start_time = 0
        self._cache_dirty = True


class DataStore(QObject):
    """
    Central data store for all joint encoder data.

    Signals:
        data_updated: Emitted when new data is available (throttled to max 20Hz)
        simulation_changed: Emitted when simulation mode changes
        state_changed: Emitted when system state changes
        imu_updated: Emitted when IMU data is updated
        limits_updated: Emitted when limit switch data is updated
        directions_updated: Emitted when motor direction data is updated
    """

    data_updated = pyqtSignal(int)  # Emits joint_id that was updated
    simulation_changed = pyqtSignal(bool)  # Emitted when simulation mode changes
    state_changed = pyqtSignal(int)  # Emitted when system state changes
    imu_updated = pyqtSignal()  # Emitted when IMU data is updated
    limits_updated = pyqtSignal()  # Emitted when limit switch data is updated
    directions_updated = pyqtSignal()  # Emitted when motor direction data is updated
    mode_changed = pyqtSignal(
        int
    )  # Emitted when control mode changes (0: Open, 1: Vel, 2: Pos)

    config_updated = pyqtSignal(int)  # Emits joint_id when config is loaded
    leveling_updated = pyqtSignal()  # Emitted when leveling debug data is updated
    strain_gauge_updated = pyqtSignal()  # Emitted when strain gauge values are updated
    seq_status_updated = pyqtSignal(int, int, int)
    seq_targets_changed = pyqtSignal()
    fc_motor_backend_changed = pyqtSignal(str)  # "hub" or "odrive"

    NUM_JOINTS = 10
    DEFAULT_MAX_SAMPLES = 2000  # ~10 seconds at 200Hz
    SIMULATION_UPDATE_MS = 20  # 50 Hz simulation rate
    DATA_UPDATE_THROTTLE_MS = 50  # Max 20Hz for data_updated signal

    def __init__(self, parent=None, max_samples: int = DEFAULT_MAX_SAMPLES):
        super().__init__(parent)
        self._joints: List[JointData] = [
            JointData(joint_id=i + 1, max_samples=max_samples)
            for i in range(self.NUM_JOINTS)
        ]
        self._selected_joint: int = 1
        self._control_mode: int = 0  # 0: Open, 1: Vel, 2: Pos (for selected joint)
        self._control_modes: List[int] = [0] * 8 + [
            2,
            2,
        ]  # RoboClaw from telemetry; hub motor default position
        self._hub_motor_prev_sample: dict = {}  # joint_id -> (pos, timestamp_ms)
        self._simulation_mode: bool = False
        self._current_state: int = 0
        self._seq_targets: dict = {}

        # IMU data
        self._imu_pitch: float = 0.0
        self._imu_roll: float = 0.0
        self._imu_yaw: float = 0.0
        self._imu_ax: float = 0.0
        self._imu_ay: float = 0.0
        self._imu_az: float = 0.0

        self._imu_qw: float = 1.0
        self._imu_qx: float = 0.0
        self._imu_qy: float = 0.0
        self._imu_qz: float = 0.0

        self._imu_target_pitch: float = 0.0
        self._imu_target_roll: float = 0.0

        # Leveling debug fields
        self._leveling_pitch_err: float = 0.0
        self._leveling_roll_err: float = 0.0
        self._z_target_ml: float = 0.0
        self._z_target_rc: float = 0.0
        self._z_target_mr: float = 0.0

        # Strain gauge (load cell) readings — filtered ADC counts
        self._sg_rc_value: float = 0.0
        self._sg_fc_value: float = 0.0
        self._sg_ml_value: float = 0.0
        self._sg_mr_value: float = 0.0

        # Drive wheel telemetry
        self._drive_fb_pos: float = 0.0
        self._drive_lr_pos: float = 0.0
        self._drive_fb_vel: float = 0.0
        self._drive_lr_vel: float = 0.0
        self._drive_fb_pwm: float = 0.0
        self._drive_lr_pwm: float = 0.0
        self._carriage_return_direction: int = 0
        self._raw_ml_enc_pos: float = 0.0
        self._raw_mr_enc_pos: float = 0.0
        self._raw_ml_enc_vel: float = 0.0
        self._raw_mr_enc_vel: float = 0.0

        # Front caster axes (actuator ids 9=L, 10=R; robot-frame turns)
        self._fc_motor_backend: str = FC_MOTOR_BACKEND_HUB
        self._hub_motor_r_pos: float = 0.0
        self._hub_motor_l_pos: float = 0.0

        # Motor directions (6 motors)
        self._motor_directions: List[int] = [1, 1, 1, 1, 1, 1, 1, 1]
        self._encoder_directions: List[int] = [1, 1, 1, 1, 1, 1, 1, 1]

        # PID configurations (dict of joint_id -> ConfigData)
        self._configs: dict = {}

        # Limit switches (4: ML_fwd, ML_bwd, MR_fwd, MR_bwd)
        self._limit_switches: List[bool] = [False, False, False, False]

        # IMU data history for graphing
        self._imu_data: IMUData = IMUData(max_samples=max_samples)

        # Throttling for data_updated signal
        self._pending_update_joint: Optional[int] = None
        self._update_throttle_timer = QTimer(self)
        self._update_throttle_timer.setSingleShot(True)
        self._update_throttle_timer.timeout.connect(self._emit_pending_update)

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
        self._throttled_data_updated(self._selected_joint)

    def _throttled_data_updated(self, joint_id: int):
        """
        Emit data_updated signal with throttling to prevent UI overload.

        If a signal was recently emitted, this queues the update and waits
        for the throttle timer to expire before emitting again.
        """
        self._pending_update_joint = joint_id

        # If timer is not running, emit immediately and start throttle period
        if not self._update_throttle_timer.isActive():
            self._emit_pending_update()
            self._update_throttle_timer.start(self.DATA_UPDATE_THROTTLE_MS)

    def _emit_pending_update(self):
        """Emit any pending data_updated signal."""
        if self._pending_update_joint is not None:
            self.data_updated.emit(self._pending_update_joint)
            self._pending_update_joint = None

    @property
    def selected_joint(self) -> int:
        """Get the currently selected joint (1-indexed)."""
        return self._selected_joint

    @selected_joint.setter
    def selected_joint(self, joint_id: int):
        """Set the currently selected joint (1-indexed)."""
        if 1 <= joint_id <= self.NUM_JOINTS:
            self._selected_joint = joint_id
            # Sync mode banner to the newly selected joint's last-known mode
            self.control_mode = self._control_modes[joint_id - 1]

    @property
    def control_mode(self) -> int:
        """Get the current control mode."""
        return self._control_mode

    @control_mode.setter
    def control_mode(self, mode: int):
        """Set the current control mode."""
        if self._control_mode != mode:
            self._control_mode = mode
            self.mode_changed.emit(mode)

    @property
    def current_state(self) -> int:
        """Get the current system state."""
        return self._current_state

    @property
    def imu_pitch(self) -> float:
        """Get IMU pitch angle."""
        return self._imu_pitch

    @property
    def imu_roll(self) -> float:
        """Get IMU roll angle."""
        return self._imu_roll

    @property
    def imu_yaw(self) -> float:
        """Get IMU yaw angle."""
        return self._imu_yaw

    @property
    def imu_ax(self) -> float:
        """Get IMU X acceleration."""
        return self._imu_ax

    @property
    def imu_ay(self) -> float:
        """Get IMU Y acceleration."""
        return self._imu_ay

    @property
    def imu_az(self) -> float:
        """Get IMU Z acceleration."""
        return self._imu_az

    @property
    def imu_qw(self) -> float:
        """Get IMU Quaternion W."""
        return self._imu_qw

    @property
    def imu_qx(self) -> float:
        """Get IMU Quaternion X."""
        return self._imu_qx

    @property
    def imu_qy(self) -> float:
        """Get IMU Quaternion Y."""
        return self._imu_qy

    @property
    def imu_qz(self) -> float:
        """Get IMU Quaternion Z."""
        return self._imu_qz

    @property
    def imu_target_pitch(self) -> float:
        """Get IMU target pitch angle."""
        return self._imu_target_pitch

    @imu_target_pitch.setter
    def imu_target_pitch(self, pitch: float):
        self._imu_target_pitch = pitch
        self.imu_updated.emit()

    @property
    def imu_target_roll(self) -> float:
        """Get IMU target roll angle."""
        return self._imu_target_roll

    @imu_target_roll.setter
    def imu_target_roll(self, roll: float):
        self._imu_target_roll = roll
        self.imu_updated.emit()

    @property
    def leveling_pitch_err(self) -> float:
        """Get leveling pitch error (degrees)."""
        return self._leveling_pitch_err

    @property
    def leveling_roll_err(self) -> float:
        """Get leveling roll error (degrees)."""
        return self._leveling_roll_err

    @property
    def z_target_ml(self) -> float:
        """Get Z target for ML (Mid-Left) actuator."""
        return self._z_target_ml

    @property
    def z_target_rc(self) -> float:
        """Get Z target for RC (Rear-Center) actuator."""
        return self._z_target_rc

    @property
    def z_target_mr(self) -> float:
        """Get Z target for MR (Mid-Right) actuator."""
        return self._z_target_mr

    @property
    def sg_rc_value(self) -> float:
        """Get filtered strain gauge reading for RC (Rear Caster)."""
        return self._sg_rc_value

    @property
    def sg_fc_value(self) -> float:
        """Get filtered strain gauge reading for FC (Front Caster)."""
        return self._sg_fc_value

    @property
    def sg_ml_value(self) -> float:
        """Get filtered strain gauge reading for ML (Main Left)."""
        return self._sg_ml_value

    @property
    def sg_mr_value(self) -> float:
        """Get filtered strain gauge reading for MR (Main Right)."""
        return self._sg_mr_value

    @property
    def drive_fb_pos(self) -> float:
        return self._drive_fb_pos

    @property
    def drive_lr_pos(self) -> float:
        return self._drive_lr_pos

    @property
    def drive_fb_vel(self) -> float:
        return self._drive_fb_vel

    @property
    def drive_lr_vel(self) -> float:
        return self._drive_lr_vel

    @property
    def drive_fb_pwm(self) -> float:
        return self._drive_fb_pwm

    @property
    def drive_lr_pwm(self) -> float:
        return self._drive_lr_pwm

    @property
    def carriage_return_direction(self) -> int:
        return self._carriage_return_direction

    @property
    def hub_motor_r_pos(self) -> float:
        return self._hub_motor_r_pos

    @property
    def hub_motor_l_pos(self) -> float:
        return self._hub_motor_l_pos

    @property
    def fc_caster_l_pos(self) -> float:
        """Active front-caster left position (turns), hub or ODrive."""
        return self._hub_motor_l_pos

    @property
    def fc_caster_r_pos(self) -> float:
        """Active front-caster right position (turns), hub or ODrive."""
        return self._hub_motor_r_pos

    @property
    def fc_motor_backend(self) -> str:
        return self._fc_motor_backend

    @fc_motor_backend.setter
    def fc_motor_backend(self, backend: str) -> None:
        if backend not in (FC_MOTOR_BACKEND_HUB, FC_MOTOR_BACKEND_ODRIVE):
            return
        if backend == self._fc_motor_backend:
            return
        self._fc_motor_backend = backend
        self.clear_joint(9)
        self.clear_joint(10)
        self.fc_motor_backend_changed.emit(backend)

    @property
    def uses_hub_motors(self) -> bool:
        return self._fc_motor_backend == FC_MOTOR_BACKEND_HUB

    @property
    def uses_odrive(self) -> bool:
        return self._fc_motor_backend == FC_MOTOR_BACKEND_ODRIVE

    def is_fc_motor_joint(self, joint_id: int) -> bool:
        return is_fc_motor_actuator(joint_id)

    @property
    def odrive_r_pos(self) -> float:
        return self.hub_motor_r_pos

    @property
    def odrive_l_pos(self) -> float:
        return self.hub_motor_l_pos

    @property
    def raw_ml_enc_vel(self) -> float:
        return self._raw_ml_enc_vel

    @property
    def raw_mr_enc_vel(self) -> float:
        return self._raw_mr_enc_vel

    @property
    def motor_directions(self) -> List[int]:
        """Get motor directions (1 or -1 for each of 6 motors)."""
        return self._motor_directions

    @property
    def encoder_directions(self) -> List[int]:
        """Get encoder directions (1 or -1 for each of 6 motors)."""
        return self._encoder_directions

    def get_config(self, joint_id: int):
        """Get configuration data for a joint."""
        return self._configs.get(joint_id)

    def set_config(self, config_data):
        """Store config data and emit signal."""
        self._configs[config_data.joint_id] = config_data
        self.config_updated.emit(config_data.joint_id)

    @property
    def limit_switches(self) -> List[bool]:
        """Get limit switch states [ML_fwd, ML_bwd, MR_fwd, MR_bwd]."""
        return self._limit_switches

    @property
    def imu_data(self) -> IMUData:
        """Get IMU data object for graphing."""
        return self._imu_data

    def get_joint(self, joint_id: int) -> Optional[JointData]:
        """Get JointData for a specific joint (1-indexed)."""
        if 1 <= joint_id <= self.NUM_JOINTS:
            return self._joints[joint_id - 1]
        return None

    def get_selected_joint_data(self) -> JointData:
        """Get JointData for the currently selected joint."""
        return self._joints[self._selected_joint - 1]

    def _feed_hub_motor_joint(
        self,
        joint_id: int,
        timestamp_ms: int,
        position: float,
        pwm: float,
    ):
        """Update JointData for a hub motor axis (joint ids 9–10)."""
        if joint_id < 9 or joint_id > self.NUM_JOINTS:
            return
        prev = self._hub_motor_prev_sample.get(joint_id)
        velocity = 0.0
        if prev is not None:
            prev_pos, prev_ts = prev
            dt_s = (timestamp_ms - prev_ts) / 1000.0
            if dt_s > 0:
                velocity = (position - prev_pos) / dt_s
        self._hub_motor_prev_sample[joint_id] = (position, timestamp_ms)
        self._joints[joint_id - 1].add_sample(timestamp_ms, position, velocity, pwm)
        if joint_id == self._selected_joint:
            self._throttled_data_updated(joint_id)

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

        # Store IMU data
        self._imu_pitch = data.imu_pitch
        self._imu_roll = data.imu_roll
        self._imu_yaw = data.imu_yaw
        self._imu_ax = data.imu_ax
        self._imu_ay = data.imu_ay
        self._imu_az = data.imu_az

        if hasattr(data, "imu_qw"):
            self._imu_qw = data.imu_qw
            self._imu_qx = data.imu_qx
            self._imu_qy = data.imu_qy
            self._imu_qz = data.imu_qz

        # Add IMU sample to history for graphing
        self._imu_data.add_sample(
            data.timestamp_ms,
            data.imu_pitch,
            data.imu_roll,
            data.imu_yaw,
            data.imu_ax,
            data.imu_ay,
            data.imu_az,
        )
        self.imu_updated.emit()

        # Store leveling debug data — only when firmware actually sent the 49-field packet
        if getattr(data, "has_leveling_data", False):
            self._leveling_pitch_err = data.leveling_pitch_err
            self._leveling_roll_err = data.leveling_roll_err
            self._z_target_ml = data.z_target_ml
            self._z_target_rc = data.z_target_rc
            self._z_target_mr = data.z_target_mr
            self.leveling_updated.emit()

        # Store strain gauge readings (present from the 53-field packet onward)
        self._sg_rc_value = data.sg_rc_value
        self._sg_fc_value = data.sg_fc_value
        self._sg_ml_value = data.sg_ml_value
        self._sg_mr_value = data.sg_mr_value
        self.strain_gauge_updated.emit()

        # Store drive wheel telemetry (present from the 63-field packet onward)
        if hasattr(data, "drive_fb_pos"):
            self._drive_fb_pos = data.drive_fb_pos
            self._drive_lr_pos = data.drive_lr_pos
            self._drive_fb_vel = data.drive_fb_vel
            self._drive_lr_vel = data.drive_lr_vel
        if hasattr(data, "drive_fb_pwm"):
            self._drive_fb_pwm = data.drive_fb_pwm
            self._drive_lr_pwm = data.drive_lr_pwm
        if data.carriage_return_direction is not None:
            self._carriage_return_direction = int(data.carriage_return_direction)
        if hasattr(data, "raw_ml_enc_pos"):
            self._raw_ml_enc_pos = data.raw_ml_enc_pos
            self._raw_mr_enc_pos = data.raw_mr_enc_pos
            self._raw_ml_enc_vel = data.raw_ml_enc_vel
            self._raw_mr_enc_vel = data.raw_mr_enc_vel

        if getattr(data, "has_hub_motor_data", False):
            self._hub_motor_r_pos = data.hub_motor_r_pos
            self._hub_motor_l_pos = data.hub_motor_l_pos
            self._feed_hub_motor_joint(9, data.timestamp_ms, self._hub_motor_l_pos, 0.0)
            self._feed_hub_motor_joint(
                10, data.timestamp_ms, self._hub_motor_r_pos, 0.0
            )

        # Feed drive wheel data into JointData for joints 7 (Drive FB) and 8 (Drive LR)
        # so the plotter can display them when selected.
        if hasattr(data, "drive_fb_pos"):
            self._joints[6].add_sample(
                data.timestamp_ms,
                data.drive_fb_pos,
                data.drive_fb_vel,
                data.drive_fb_pwm,
            )
            self._joints[7].add_sample(
                data.timestamp_ms,
                data.drive_lr_pos,
                data.drive_lr_vel,
                data.drive_lr_pwm,
            )

        # Update per-joint control modes from telemetry (59-field packet onward)
        if getattr(data, "control_mode_values", None):
            for i, mode in enumerate(data.control_mode_values[: self.NUM_JOINTS]):
                self._control_modes[i] = mode
        if hasattr(data, "drive_fb_mode"):
            self._control_modes[6] = data.drive_fb_mode
            self._control_modes[7] = data.drive_lr_mode
        if getattr(data, "control_mode_values", None) or hasattr(data, "drive_fb_mode"):
            # Update the selected-joint mode — the property setter emits mode_changed if changed
            self.control_mode = self._control_modes[self._selected_joint - 1]

        # Store motor directions
        if data.direction_values:
            self._motor_directions = data.direction_values
            if hasattr(data, "drive_fb_dir"):
                while len(self._motor_directions) < 8:
                    self._motor_directions.append(1)
                self._motor_directions[6] = data.drive_fb_dir
                self._motor_directions[7] = data.drive_lr_dir

        # Store encoder directions
        if hasattr(data, "encoder_direction_values") and data.encoder_direction_values:
            self._encoder_directions = data.encoder_direction_values
            if hasattr(data, "drive_fb_enc_dir"):
                while len(self._encoder_directions) < 8:
                    self._encoder_directions.append(1)
                self._encoder_directions[6] = data.drive_fb_enc_dir
                self._encoder_directions[7] = data.drive_lr_enc_dir

        if data.direction_values or (
            hasattr(data, "encoder_direction_values") and data.encoder_direction_values
        ):
            self.directions_updated.emit()

        # Store limit switches
        if data.limit_switches:
            self._limit_switches = data.limit_switches
            self.limits_updated.emit()

        # Use throttled emission to prevent UI overload
        self._throttled_data_updated(self._selected_joint)

    def set_target(self, joint_id: int, target: float):
        """Set target position for a joint."""
        if 1 <= joint_id <= self.NUM_JOINTS:
            self._joints[joint_id - 1].set_target(target)

    def clear_all(self):
        """Clear all stored data."""
        for joint in self._joints:
            joint.clear()

    def clear_joint(self, joint_id: int):
        if 1 <= joint_id <= self.NUM_JOINTS:
            self._joints[joint_id - 1].clear()

    def set_seq_targets(self, targets: dict):
        self._seq_targets = targets
        self.seq_targets_changed.emit()

    def get_seq_targets(self) -> dict:
        return self._seq_targets
