"""
Joint configuration and naming for the MEBot/RAMMP system.
"""

from dataclasses import dataclass
from typing import Dict, List


@dataclass
class JointInfo:
    """Information about a single joint."""

    id: int
    name: str
    short_name: str
    description: str

    def __str__(self) -> str:
        return f"{self.id}: {self.name}"

    def display_name(self) -> str:
        return f"Joint {self.id}: {self.name}"


# Joint configurations based on EncoderContainer.h mapping
JOINTS: List[JointInfo] = [
    JointInfo(id=1, name="RC", short_name="RC", description="Rear Caster"),
    JointInfo(
        id=2,
        name="FC",
        short_name="FC",
        description="Front Caster",
    ),
    JointInfo(
        id=3,
        name="ML",
        short_name="ML",
        description="Main Left Wheel",
    ),
    JointInfo(id=4, name="MR", short_name="MR", description="Main Right Wheel"),
    JointInfo(
        id=5,
        name="ML Carriage",
        short_name="ML_C",
        description="Main Left Carriage",
    ),
    JointInfo(
        id=6,
        name="MR Carriage",
        short_name="MR_C",
        description="Main Right Carriage",
    ),
    JointInfo(
        id=7,
        name="Drive FB",
        short_name="D_FB",
        description="Forward/Back body-frame velocity controller (output via LUCI)",
    ),
    JointInfo(
        id=8,
        name="Drive LR",
        short_name="D_LR",
        description="Left/Right steering correction controller (output via LUCI)",
    ),
    JointInfo(
        id=9,
        name="ODrive L",
        short_name="OD_L",
        description="Left ODrive axis (actuator 9, robot-frame turns)",
    ),
    JointInfo(
        id=10,
        name="ODrive R",
        short_name="OD_R",
        description="Right ODrive axis (actuator 10, robot-frame turns)",
    ),
]

# Actuator ids 9–10 (motor_map): 9=ODriveL, 10=ODriveR
ODRIVE_ACTUATOR_IDS = (9, 10)

# Create lookup dictionary by ID
JOINT_BY_ID: Dict[int, JointInfo] = {joint.id: joint for joint in JOINTS}


def get_joint_info(joint_id: int) -> JointInfo:
    """Get joint info by ID (1-indexed)."""
    return JOINT_BY_ID.get(joint_id, JOINTS[0])


def get_joint_names() -> List[str]:
    """Get list of joint display names for dropdown."""
    return [joint.display_name() for joint in JOINTS]


def get_joint_id_from_index(index: int) -> int:
    """Convert dropdown index (0-based) to joint ID (1-based)."""
    return index + 1


def is_odrive_actuator(joint_id: int) -> bool:
    """True for ODrive axes (actuator ids 9 and 10)."""
    return joint_id in ODRIVE_ACTUATOR_IDS
