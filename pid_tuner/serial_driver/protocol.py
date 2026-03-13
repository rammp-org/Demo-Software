"""
Serial protocol definitions for Teensy communication.

Teensy -> PC Protocol:
    TELEMETRY,<timestamp_ms>,<state>,<6 positions>,<6 velocities>,<6 pwms>\n

    Joint order: RC, FC, ML, MR, ML_Carriage, MR_Carriage (1-6)

    Example: TELEMETRY,12345,1,10.5,-5.2,20.0,15.3,0.5,1.2,0.1,0.2,0.3,0.4,0.5,0.6,100,200,300,400,500,600

PC -> Teensy Protocol:
    Set Target: T<joint_id>:<target_cm>\n
        Example: T1:15.5

    Set Mode: M<joint_id>:<mode>\n
        Example: M1:2 (0: OPEN_LOOP, 1: VELOCITY, 2: POSITION)

    Set PID:
        P<joint_id>:<val>, I<joint_id>:<val>, D<joint_id>:<val> (Position PID)
        p<joint_id>:<val>, i<joint_id>:<val>, d<joint_id>:<val> (Velocity PID)
        Example: P1:0.5

    Stop Sine: X<joint_id>\n
        Example: X1 (stop sine wave on joint 1)

    ESTOP: z\n
    Clear ESTOP: c\n
"""

from dataclasses import dataclass, field
from typing import Optional, List
import re


@dataclass
class EncoderData:
    """Parsed position data from Teensy."""

    timestamp_ms: int
    state: int
    position_values: List[float]  # 6 positions
    velocity_values: List[float] = field(default_factory=list)  # 6 velocities
    pwm_values: List[float] = field(default_factory=list)  # 6 pwms

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

    # Matches: TELEMETRY,timestamp,state,<18 float values>
    ENCODER_PATTERN = re.compile(r"^TELEMETRY,(\d+),(\d+),(.+)$")

    NUM_JOINTS = 6

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
                state = int(match.group(2))
                values_str = match.group(3)
                values = [float(v.strip()) for v in values_str.split(",")]

                # Expect 18 values: 6 positions + 6 velocities + 6 pwms
                if len(values) == 18:
                    return EncoderData(
                        timestamp_ms=timestamp,
                        state=state,
                        position_values=values[0:6],
                        velocity_values=values[6:12],
                        pwm_values=values[12:18],
                    )
                # Backwards compatibility: 6 values = positions only
                elif len(values) == 6:
                    return EncoderData(
                        timestamp_ms=timestamp,
                        state=state,
                        position_values=values,
                        velocity_values=[0.0] * cls.NUM_JOINTS,
                        pwm_values=[0.0] * cls.NUM_JOINTS,
                    )
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
        """
        cmd = f"T{joint_id}:{target_cm:.2f}\n"
        return cmd.encode("ascii")

    @staticmethod
    def set_mode(joint_id: int, mode: int) -> bytes:
        """
        Create command to set the control mode of a joint (0: Open Loop, 1: Vel, 2: Pos).
        """
        cmd = f"M{joint_id}:{mode}\n"
        return cmd.encode("ascii")

    @staticmethod
    def set_pid(joint_id: int, param: str, value: float) -> bytes:
        """
        Create command to set a PID parameter.
        param should be 'P', 'I', 'D', 'p', 'i', or 'd'.
        """
        cmd = f"{param}{joint_id}:{value:.4f}\n"
        return cmd.encode("ascii")

    @staticmethod
    def step_input(joint_id: int, step_cm: float) -> bytes:
        """
        Create command for step input (relative change).
        """
        cmd = f"S{joint_id}:{step_cm:.2f}\n"
        return cmd.encode("ascii")

    @staticmethod
    def start_sine_wave(
        joint_id: int, amplitude_cm: float, frequency_hz: float, duration_s: float
    ) -> bytes:
        """
        Create command to start sine wave input.
        """
        cmd = f"W{joint_id}:{amplitude_cm:.2f},{frequency_hz:.3f},{duration_s:.1f}\n"
        return cmd.encode("ascii")

    @staticmethod
    def stop_sine_wave(joint_id: int) -> bytes:
        """
        Create command to stop sine wave on a joint.
        """
        cmd = f"X{joint_id}\n"
        return cmd.encode("ascii")

    @staticmethod
    def disable_motors() -> bytes:
        """
        Create command to disable all motors (emergency stop / safe mode).
        """
        cmd = "z\n"
        return cmd.encode("ascii")

    @staticmethod
    def clear_estop() -> bytes:
        """
        Create command to clear ESTOP state.
        """
        cmd = "c\n"
        return cmd.encode("ascii")
