"""
Serial communication handler for Teensy communication.

Runs in a separate thread to avoid blocking the UI.
Uses batched emission to prevent GUI overload from fast serial data.
"""

import threading
import time
from typing import Optional, List
from queue import Queue, Empty

import serial
import serial.tools.list_ports
from PyQt6.QtCore import QObject, pyqtSignal, QTimer

from .protocol import (
    ProtocolParser,
    ProtocolEncoder,
    EncoderData,
    ConfigData,
    SeqAckData,
    SeqStatusData,
)


class SerialHandler(QObject):
    """
    Handles serial communication with Teensy in a background thread.

    Signals:
        data_received: Emitted when valid encoder data is received (latest only per batch)
        config_received: Emitted when valid config data is received
        raw_lines_received: Emitted with batched raw lines for console display
        connection_changed: Emitted when connection state changes
        error_occurred: Emitted when an error occurs
    """

    # Qt Signals for thread-safe UI updates
    data_received = pyqtSignal(EncoderData)
    config_received = pyqtSignal(ConfigData)
    raw_lines_received = pyqtSignal(list)  # Batched raw lines for console display
    connection_changed = pyqtSignal(bool)  # True = connected, False = disconnected
    error_occurred = pyqtSignal(str)
    seq_ack_received = pyqtSignal(int)  # step_idx
    seq_status_received = pyqtSignal(int, int, int)

    DEFAULT_BAUD_RATE = 115200
    DEFAULT_TIMEOUT = 0.1  # 100ms read timeout
    BATCH_INTERVAL_MS = (
        50  # 20 Hz batch emission rate (balance responsiveness vs performance)
    )

    def __init__(self, parent=None):
        super().__init__(parent)
        self._serial: Optional[serial.Serial] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._command_queue: Queue = Queue()
        self._lock = threading.Lock()

        # Batching for high-frequency data
        self._line_buffer: List[str] = []
        self._line_buffer_lock = threading.Lock()

        # Timer for batched emission (runs in main/GUI thread)
        self._batch_timer = QTimer(self)
        self._batch_timer.timeout.connect(self._emit_batched_data)
        self._batch_timer.start(self.BATCH_INTERVAL_MS)

    def __del__(self):
        self.disconnect_port()

    @property
    def is_connected(self) -> bool:
        with self._lock:
            return self._serial is not None and self._serial.is_open

    @staticmethod
    def list_available_ports() -> List[str]:
        """List available serial ports."""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]

    def connect(self, port: str, baud_rate: int = DEFAULT_BAUD_RATE) -> bool:
        """
        Connect to the specified serial port.

        Args:
            port: Serial port name (e.g., 'COM3' or '/dev/ttyUSB0')
            baud_rate: Baud rate (default 115200)

        Returns:
            True if connection successful, False otherwise
        """
        if self.is_connected:
            self.disconnect_port()

        try:
            self._serial = serial.Serial(
                port=port,
                baudrate=baud_rate,
                timeout=self.DEFAULT_TIMEOUT,
                write_timeout=1.0,
            )

            # Start reading thread
            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()

            self.connection_changed.emit(True)
            return True

        except serial.SerialException as e:
            self.error_occurred.emit(f"Failed to connect: {e}")
            return False

    def disconnect_port(self):
        """Disconnect from serial port."""
        self._running = False

        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

        with self._lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
                self._serial = None

        self.connection_changed.emit(False)

    def send_command(self, command: bytes):
        """
        Queue a command to send to Teensy.

        Args:
            command: Command bytes to send
        """
        self._command_queue.put(command)

    def _send_command(self, command: bytes):
        self.send_command(command)

    def set_target(self, joint_id: int, target_cm: float):
        """Send target position command."""
        cmd = ProtocolEncoder.set_target(joint_id, target_cm)
        self.send_command(cmd)

    def send_step(self, joint_id: int, step_cm: float):
        """Send step input command."""
        cmd = ProtocolEncoder.step_input(joint_id, step_cm)
        self.send_command(cmd)

    def start_sine(
        self, joint_id: int, amplitude_cm: float, frequency: float, duration: float
    ):
        """Send sine wave start command."""
        cmd = ProtocolEncoder.start_sine_wave(
            joint_id, amplitude_cm, frequency, duration
        )
        self.send_command(cmd)

    def stop_sine(self, joint_id: int):
        """Send sine wave stop command."""
        cmd = ProtocolEncoder.stop_sine_wave(joint_id)
        self.send_command(cmd)

    def set_mode(self, joint_id: int, mode: int):
        """Set control mode (0: Open Loop, 1: Vel, 2: Pos)."""
        cmd = ProtocolEncoder.set_mode(joint_id, mode)
        self.send_command(cmd)

    def set_pid(self, joint_id: int, param: str, value: float):
        """Set PID parameter ('P', 'I', 'D', 'p', 'i', 'd')."""
        cmd = ProtocolEncoder.set_pid(joint_id, param, value)
        self.send_command(cmd)

    def set_pos_lpf(self, joint_id: int, alpha: float):
        """Set position PID output LPF alpha."""
        cmd = ProtocolEncoder.set_pos_lpf(joint_id, alpha)
        self.send_command(cmd)

    def set_vel_lpf(self, joint_id: int, alpha: float):
        """Set velocity PID output LPF alpha."""
        cmd = ProtocolEncoder.set_vel_lpf(joint_id, alpha)
        self.send_command(cmd)

    def set_input_lpf(self, joint_id: int, alpha: float):
        """Set motor input LPF alpha."""
        cmd = ProtocolEncoder.set_input_lpf(joint_id, alpha)
        self.send_command(cmd)

    def disable_motors(self):
        """Send disable motors command (ESTOP)."""
        cmd = ProtocolEncoder.disable_motors()
        self.send_command(cmd)

    def clear_estop(self):
        """Send clear ESTOP command."""
        cmd = ProtocolEncoder.clear_estop()
        self.send_command(cmd)

    def reset_pid(self, joint_id: int):
        """Send reset PID command (clear integrator windup)."""
        cmd = ProtocolEncoder.reset_pid(joint_id)
        self.send_command(cmd)

    def set_feed_forward(self, joint_id: int, param: str, value: float):
        """Set feed-forward gain ('F' for position, 'f' for velocity)."""
        cmd = ProtocolEncoder.set_feed_forward(joint_id, param, value)
        self.send_command(cmd)

    def home_position(self, joint_id: int):
        """Send home/zero position command."""
        cmd = ProtocolEncoder.home_position(joint_id)
        self.send_command(cmd)

    def toggle_direction(self, joint_id: int):
        """Toggle motor direction."""
        cmd = ProtocolEncoder.toggle_direction(joint_id)
        self.send_command(cmd)

    def set_self_leveling(self, enable: bool):
        """Enable or disable self-leveling mode."""
        cmd = ProtocolEncoder.set_self_leveling(enable)
        self.send_command(cmd)

    def set_imu_target(self, pitch: float, roll: float):
        """Set target pitch and roll for self-leveling."""
        cmd = ProtocolEncoder.set_imu_target(pitch, roll)
        self.send_command(cmd)

    def set_position_offset(self, joint_id: int, desired_position: float):
        """Set position offset so current position reads as desired_position."""
        cmd = ProtocolEncoder.set_position_offset(joint_id, desired_position)
        self.send_command(cmd)

    def set_odrive_position(self, axis_id: int, position: float):
        """axis_id 0 = both ODrives; 1 = left; 2 = right (TUNER_MODE on Teensy)."""
        self.send_command(ProtocolEncoder.set_odrive_position(axis_id, position))

    def toggle_encoder_direction(self, joint_id: int):
        """Toggle encoder direction."""
        cmd = ProtocolEncoder.toggle_encoder_direction(joint_id)
        self.send_command(cmd)

    def save_config(self, joint_id: int):
        """Save configuration to EEPROM."""
        cmd = ProtocolEncoder.save_config(joint_id)
        self.send_command(cmd)

    def get_config(self, joint_id: int):
        """Request configuration from EEPROM."""
        cmd = ProtocolEncoder.get_config(joint_id)
        self.send_command(cmd)

    def set_pos_limit_min(self, joint_id: int, limit: int):
        cmd = ProtocolEncoder.set_pos_limit_min(joint_id, limit)
        self.send_command(cmd)

    def set_pos_limit_max(self, joint_id: int, limit: int):
        cmd = ProtocolEncoder.set_pos_limit_max(joint_id, limit)
        self.send_command(cmd)

    def set_pos_ramp_rate(self, joint_id: int, rate: float):
        cmd = ProtocolEncoder.set_pos_ramp_rate(joint_id, rate)
        self.send_command(cmd)

    def set_vel_ramp_rate(self, joint_id: int, rate: float):
        cmd = ProtocolEncoder.set_vel_ramp_rate(joint_id, rate)
        self.send_command(cmd)

    def send_raw(self, cmd_str: str):
        """Send raw serial command."""
        if not cmd_str.endswith("\n"):
            cmd_str += "\n"
        self.send_command(cmd_str.encode("ascii"))

    def _read_loop(self):
        """Background thread loop for reading serial data."""
        buffer = ""

        while self._running:
            # Process any pending commands
            self._process_commands()

            # Read available data
            try:
                with self._lock:
                    if self._serial is None or not self._serial.is_open:
                        break

                    if self._serial.in_waiting > 0:
                        data = self._serial.read(self._serial.in_waiting)
                        buffer += data.decode("ascii", errors="ignore")

                # Process complete lines
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    self._process_line(line)

            except serial.SerialException as e:
                self.error_occurred.emit(f"Serial read error: {e}")
                break
            except Exception as e:
                self.error_occurred.emit(f"Unexpected error: {e}")
                break

            # Small sleep to prevent busy-waiting
            time.sleep(0.001)

        # Cleanup on exit
        self.disconnect_port()

    def _process_commands(self):
        """Send any queued commands."""
        while not self._command_queue.empty():
            try:
                cmd = self._command_queue.get_nowait()
                with self._lock:
                    if self._serial is not None and self._serial.is_open:
                        self._serial.write(cmd)
            except Empty:
                break
            except serial.SerialException as e:
                self.error_occurred.emit(f"Failed to send command: {e}")

    def _process_line(self, line: str):
        """Buffer a complete line of serial data for batched emission."""
        # Buffer the line instead of emitting immediately
        with self._line_buffer_lock:
            self._line_buffer.append(line)

    def _emit_batched_data(self):
        """
        Emit batched data at a controlled rate.

        Called by QTimer in the main/GUI thread to prevent overwhelming the UI.
        Emits all buffered raw lines for console display, but only the latest
        telemetry data for plotting (stale telemetry is irrelevant).
        """
        # Swap out the buffer atomically
        with self._line_buffer_lock:
            lines = self._line_buffer
            self._line_buffer = []

        if not lines:
            return

        # Emit all raw lines as a batch for console display
        self.raw_lines_received.emit(lines)

        # Parse and emit telemetry and config data
        # Only the latest telemetry data is emitted for performance
        # All configs are emitted
        latest_encoder = None
        latest_seq_status = None

        for line in reversed(lines):
            data = ProtocolParser.parse_line(line)
            if data is not None:
                if isinstance(data, EncoderData):
                    if latest_encoder is None:
                        latest_encoder = data
                elif isinstance(data, ConfigData):
                    self.config_received.emit(data)
                elif isinstance(data, SeqAckData):
                    self.seq_ack_received.emit(data.step_idx)
                elif isinstance(data, SeqStatusData):
                    if latest_seq_status is None:
                        latest_seq_status = data

        if latest_encoder is not None:
            self.data_received.emit(latest_encoder)
        if latest_seq_status is not None:
            self.seq_status_received.emit(
                latest_seq_status.current_step,
                latest_seq_status.total_steps,
                latest_seq_status.state,
            )

    def enter_sequence_mode(self, enable: bool):
        """Enter or exit AUTO_CURB_CLIMBING sequence mode."""
        self.send_command(ProtocolEncoder.enter_sequence_mode(enable))

    def send_keyframe(
        self,
        index: int,
        targets: list,
        active: list,
        duration_ms,
        relative=None,
        guard_threshold=None,
        guard_condition=None,
        odrive_active=None,
        odrive_relative=None,
        odrive_targets=None,
    ):
        self.send_command(
            ProtocolEncoder.send_keyframe(
                index,
                targets,
                active,
                duration_ms,
                relative,
                guard_threshold=guard_threshold,
                guard_condition=guard_condition,
                odrive_active=odrive_active,
                odrive_relative=odrive_relative,
                odrive_targets=odrive_targets,
            )
        )

    def seq_auto_run(self, enable: bool):
        self.send_command(ProtocolEncoder.seq_auto_run(enable))

    def seq_step_forward(self):
        """Step forward to the next keyframe."""
        self.send_command(ProtocolEncoder.seq_step_forward())

    def seq_step_backward(self):
        """Step backward to the previous keyframe."""
        self.send_command(ProtocolEncoder.seq_step_backward())

    def seq_goto(self, step_idx: int):
        self._send_command(ProtocolEncoder.seq_goto(step_idx))
