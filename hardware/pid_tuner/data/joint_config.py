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
        description="Left ODrive axis (TUNER_MODE: o1:<pos>)",
    ),
    JointInfo(
        id=10,
        name="ODrive R",
        short_name="OD_R",
        description="Right ODrive axis (TUNER_MODE: o2:<pos>)",
    ),
]

# Teensy CMD_ODRIVE_POS actuator_id (not RoboClaw joint id)
ODRIVE_JOINT_L = 9
ODRIVE_JOINT_R = 10
ODRIVE_AXIS_LEFT = 1
ODRIVE_AXIS_RIGHT = 2


def is_odrive_joint(joint_id: int) -> bool:
    return joint_id in (ODRIVE_JOINT_L, ODRIVE_JOINT_R)


def odrive_axis_id(joint_id: int) -> int:
    if joint_id == ODRIVE_JOINT_L:
        return ODRIVE_AXIS_LEFT
    if joint_id == ODRIVE_JOINT_R:
        return ODRIVE_AXIS_RIGHT
    raise ValueError(f"Not an ODrive joint id: {joint_id}")


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
