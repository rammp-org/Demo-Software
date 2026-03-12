"""
Serial protocol definitions for Teensy communication.

Teensy -> PC Protocol:
    ENC:<timestamp_ms>,<pos1>,<pos2>,<pos3>,<pos4>,<pos5>,<pos6>\n

    Example: ENC:12345,10.5,-5.2,20.0,15.3,0.5,1.2

PC -> Teensy Protocol:
    Set Target: T<joint_id>:<target_cm>\n
        Example: T1:15.5

    Step Input: S<joint_id>:<step_cm>\n
        Example: S1:1.0 (step by +1.0 cm)

    Sine Wave: W<joint_id>:<amplitude_cm>,<frequency_hz>,<duration_s>\n
        Example: W1:5.0,0.5,10.0 (5.0 cm amplitude, 0.5 Hz, 10 seconds)

    Stop Sine: X<joint_id>\n
        Example: X1 (stop sine wave on joint 1)
"""

from dataclasses import dataclass
from typing import Optional, List
import re


@dataclass
class EncoderData:
    """Parsed position data from Teensy."""

    timestamp_ms: int
    position_values: List[
        float
    ]  # 6 position values in cm (indices 0-5 map to joints 1-6)

    @property
    def num_joints(self) -> int:
        return len(self.position_values)

    def get_joint_value(self, joint_id: int) -> Optional[float]:
        """Get position value for joint (1-indexed)."""
        if 1 <= joint_id <= len(self.position_values):
            return self.position_values[joint_id - 1]
        return None


class ProtocolParser:
    """Parse incoming serial data from Teensy."""

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
                values = [float(v.strip()) for v in values_str.split(",")]

                if len(values) == 6:
                    return EncoderData(timestamp_ms=timestamp, position_values=values)
                else:
                    return None
            except (ValueError, IndexError):
                return None

        return None


class ProtocolEncoder:
    """Encode commands to send to Teensy."""

    @staticmethod
    def set_target(joint_id: int, target_cm: float) -> bytes:
        """
        Create command to set target position for a joint.

        Args:
            joint_id: Joint number (1-6)
            target_cm: Target position in cm

        Returns:
            Encoded command bytes
        """
        cmd = f"T{joint_id}:{target_cm:.2f}\n"
        return cmd.encode("ascii")

    @staticmethod
    def step_input(joint_id: int, step_cm: float) -> bytes:
        """
        Create command for step input (relative change).

        Args:
            joint_id: Joint number (1-6)
            step_cm: Step size in cm (can be negative)

        Returns:
            Encoded command bytes
        """
        cmd = f"S{joint_id}:{step_cm:.2f}\n"
        return cmd.encode("ascii")

    @staticmethod
    def start_sine_wave(
        joint_id: int, amplitude_cm: float, frequency_hz: float, duration_s: float
    ) -> bytes:
        """
        Create command to start sine wave input.

        Args:
            joint_id: Joint number (1-6)
            amplitude_cm: Amplitude in cm
            frequency_hz: Frequency in Hz
            duration_s: Duration in seconds

        Returns:
            Encoded command bytes
        """
        cmd = f"W{joint_id}:{amplitude_cm:.2f},{frequency_hz:.3f},{duration_s:.1f}\n"
        return cmd.encode("ascii")

    @staticmethod
    def stop_sine_wave(joint_id: int) -> bytes:
        """
        Create command to stop sine wave on a joint.

        Args:
            joint_id: Joint number (1-6)

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
        cmd = "z\n"
        return cmd.encode("ascii")
