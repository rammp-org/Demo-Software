"""
Serial protocol definitions for Teensy communication.

Teensy -> PC Protocol:
    ENC:<timestamp_ms>,<enc1>,<enc2>,...,<enc12>\n

    Example: ENC:12345,100,-50,320,0,1500,-1200,800,900,15000,-15200,12000,-12500

PC -> Teensy Protocol:
    Set Target: T<joint_id>:<target_ticks>\n
        Example: T7:1500

    Step Input: S<joint_id>:<step_ticks>\n
        Example: S7:100 (step by +100 ticks)

    Sine Wave: W<joint_id>:<amplitude>,<frequency_hz>,<duration_s>\n
        Example: W7:500,0.5,10.0 (500 tick amplitude, 0.5 Hz, 10 seconds)

    Stop Sine: X<joint_id>\n
        Example: X7 (stop sine wave on joint 7)
"""

from dataclasses import dataclass
from typing import Optional, List
import re


@dataclass
class EncoderData:
    """Parsed encoder data from Teensy."""

    timestamp_ms: int
    encoder_values: List[int]  # 12 encoder values (indices 0-11 map to joints 1-12)

    @property
    def num_joints(self) -> int:
        return len(self.encoder_values)

    def get_joint_value(self, joint_id: int) -> Optional[int]:
        """Get encoder value for joint (1-indexed)."""
        if 1 <= joint_id <= len(self.encoder_values):
            return self.encoder_values[joint_id - 1]
        return None


class ProtocolParser:
    """Parse incoming serial data from Teensy."""

    # Pattern for encoder data: ENC:<timestamp>,<enc1>,<enc2>,...,<enc12>
    ENCODER_PATTERN = re.compile(r"^ENC:(\d+),(.+)$")

    @classmethod
    def parse_line(cls, line: str) -> Optional[EncoderData]:
        """
        Parse a line of serial data from Teensy.

        Args:
            line: Raw line from serial (newline stripped)

        Returns:
            EncoderData if valid encoder message, None otherwise
        """
        line = line.strip()

        match = cls.ENCODER_PATTERN.match(line)
        if match:
            try:
                timestamp = int(match.group(1))
                values_str = match.group(2)
                values = [int(v.strip()) for v in values_str.split(",")]

                if len(values) == 12:
                    return EncoderData(timestamp_ms=timestamp, encoder_values=values)
                else:
                    # Handle partial data gracefully
                    return None
            except (ValueError, IndexError):
                return None

        return None


class ProtocolEncoder:
    """Encode commands to send to Teensy."""

    @staticmethod
    def set_target(joint_id: int, target_ticks: int) -> bytes:
        """
        Create command to set target position for a joint.

        Args:
            joint_id: Joint number (1-12)
            target_ticks: Target position in encoder ticks

        Returns:
            Encoded command bytes
        """
        cmd = f"T{joint_id}:{target_ticks}\n"
        return cmd.encode("ascii")

    @staticmethod
    def step_input(joint_id: int, step_ticks: int) -> bytes:
        """
        Create command for step input (relative change).

        Args:
            joint_id: Joint number (1-12)
            step_ticks: Step size in ticks (can be negative)

        Returns:
            Encoded command bytes
        """
        cmd = f"S{joint_id}:{step_ticks}\n"
        return cmd.encode("ascii")

    @staticmethod
    def start_sine_wave(
        joint_id: int, amplitude: int, frequency_hz: float, duration_s: float
    ) -> bytes:
        """
        Create command to start sine wave input.

        Args:
            joint_id: Joint number (1-12)
            amplitude: Amplitude in encoder ticks
            frequency_hz: Frequency in Hz
            duration_s: Duration in seconds

        Returns:
            Encoded command bytes
        """
        cmd = f"W{joint_id}:{amplitude},{frequency_hz:.3f},{duration_s:.1f}\n"
        return cmd.encode("ascii")

    @staticmethod
    def stop_sine_wave(joint_id: int) -> bytes:
        """
        Create command to stop sine wave on a joint.

        Args:
            joint_id: Joint number (1-12)

        Returns:
            Encoded command bytes
        """
        cmd = f"X{joint_id}\n"
        return cmd.encode("ascii")

    @staticmethod
    def disable_motors() -> bytes:
        """
        Create command to disable all motors (emergency stop / safe mode).

        Returns:
            Encoded command bytes
        """
        # Send 'z' command which triggers NO_MOVEMENT in the existing firmware
        cmd = "z\n"
        return cmd.encode("ascii")
