"""
Serial communication handler for Teensy communication.

Runs in a separate thread to avoid blocking the UI.
"""

import threading
import time
from typing import Optional, Callable, List
from queue import Queue, Empty

import serial
import serial.tools.list_ports
from PyQt6.QtCore import QObject, pyqtSignal

from .protocol import ProtocolParser, ProtocolEncoder, EncoderData


class SerialHandler(QObject):
    """
    Handles serial communication with Teensy in a background thread.

    Signals:
        data_received: Emitted when valid encoder data is received
        connection_changed: Emitted when connection state changes
        error_occurred: Emitted when an error occurs
    """

    # Qt Signals for thread-safe UI updates
    data_received = pyqtSignal(EncoderData)
    raw_line_received = pyqtSignal(str)  # Raw serial line for console display
    connection_changed = pyqtSignal(bool)  # True = connected, False = disconnected
    error_occurred = pyqtSignal(str)

    DEFAULT_BAUD_RATE = 115200
    DEFAULT_TIMEOUT = 0.1  # 100ms read timeout

    def __init__(self, parent=None):
        super().__init__(parent)
        self._serial: Optional[serial.Serial] = None
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._command_queue: Queue = Queue()
        self._lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        """Check if serial port is connected and open."""
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
            self.disconnect()

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

    def disconnect(self):
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

    def disable_motors(self):
        """Send disable motors command."""
        cmd = ProtocolEncoder.disable_motors()
        self.send_command(cmd)

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
        self.disconnect()

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
        """Process a complete line of serial data."""
        # Emit raw line for console display
        self.raw_line_received.emit(line)

        # Parse and emit structured data
        data = ProtocolParser.parse_line(line)
        if data is not None:
            self.data_received.emit(data)
