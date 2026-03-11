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
    JointInfo(
        id=1, name="RC Top", short_name="RC_T", description="Rear Caster Top Encoder"
    ),
    JointInfo(
        id=2,
        name="FC Bottom",
        short_name="FC_B",
        description="Front Caster Bottom Encoder",
    ),
    JointInfo(
        id=3,
        name="RC Bottom",
        short_name="RC_B",
        description="Rear Caster Bottom Encoder (0-850 range)",
    ),
    JointInfo(
        id=4, name="FC Top", short_name="FC_T", description="Front Caster Top Encoder"
    ),
    JointInfo(
        id=5,
        name="MR Back",
        short_name="MR_B",
        description="Main Right Wheel Back Encoder",
    ),
    JointInfo(
        id=6,
        name="ML Front",
        short_name="ML_F",
        description="Main Left Wheel Front Encoder",
    ),
    JointInfo(
        id=7,
        name="ML Back",
        short_name="ML_B",
        description="Main Left Wheel Back Encoder",
    ),
    JointInfo(
        id=8,
        name="MR Front",
        short_name="MR_F",
        description="Main Right Wheel Front Encoder",
    ),
    JointInfo(
        id=9,
        name="ML Drive Wheel",
        short_name="ML_DW",
        description="Main Left Drive Wheel Encoder",
    ),
    JointInfo(
        id=10,
        name="MR Drive Wheel",
        short_name="MR_DW",
        description="Main Right Drive Wheel Encoder",
    ),
    JointInfo(
        id=11,
        name="ML Carriage",
        short_name="ML_C",
        description="Main Left Carriage Encoder",
    ),
    JointInfo(
        id=12,
        name="MR Carriage",
        short_name="MR_C",
        description="Main Right Carriage Encoder",
    ),
]

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
