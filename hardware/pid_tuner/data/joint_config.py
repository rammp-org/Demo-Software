"""
Joint configuration and naming for the MEBot/RAMMP system.
"""

from dataclasses import dataclass
from typing import Dict, List, Tuple


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


# Front-caster backends share actuator ids 9 (L) and 10 (R) on the wire.
FC_MOTOR_BACKEND_HUB = "hub"
FC_MOTOR_BACKEND_ODRIVE = "odrive"
FC_MOTOR_BACKENDS = (FC_MOTOR_BACKEND_HUB, FC_MOTOR_BACKEND_ODRIVE)

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
        name="Hub Motor L",
        short_name="HM_L",
        description="Left hub motor (actuator 9, robot-frame turns)",
    ),
    JointInfo(
        id=10,
        name="Hub Motor R",
        short_name="HM_R",
        description="Right hub motor (actuator 10, robot-frame turns)",
    ),
]

# Actuator ids 9–10: hub motors or ODrives depending on firmware / GUI selection.
FC_MOTOR_ACTUATOR_IDS = (9, 10)
HUB_MOTOR_ACTUATOR_IDS = FC_MOTOR_ACTUATOR_IDS  # backward compat

FC_MOTOR_JOINT_LABELS: Dict[str, Dict[int, Tuple[str, str, str]]] = {
    FC_MOTOR_BACKEND_HUB: {
        9: (
            "Hub Motor L",
            "HM_L",
            "Left hub motor (actuator 9, robot-frame turns)",
        ),
        10: (
            "Hub Motor R",
            "HM_R",
            "Right hub motor (actuator 10, robot-frame turns)",
        ),
    },
    FC_MOTOR_BACKEND_ODRIVE: {
        9: (
            "ODrive L",
            "ODR_L",
            "Left ODrive front caster (actuator 9, robot-frame turns)",
        ),
        10: (
            "ODrive R",
            "ODR_R",
            "Right ODrive front caster (actuator 10, robot-frame turns)",
        ),
    },
}

FC_MOTOR_SEQUENCE_NAMES: Dict[str, Tuple[str, str]] = {
    FC_MOTOR_BACKEND_HUB: ("HM_R", "HM_L"),
    FC_MOTOR_BACKEND_ODRIVE: ("OD_R", "OD_L"),
}

# Create lookup dictionary by ID
JOINT_BY_ID: Dict[int, JointInfo] = {joint.id: joint for joint in JOINTS}


def _normalize_fc_backend(fc_backend: str) -> str:
    if fc_backend in FC_MOTOR_BACKENDS:
        return fc_backend
    return FC_MOTOR_BACKEND_HUB


def get_joint_info(joint_id: int, fc_backend: str = FC_MOTOR_BACKEND_HUB) -> JointInfo:
    """Get joint info by ID (1-indexed), with FC motor labels for joints 9–10."""
    backend = _normalize_fc_backend(fc_backend)
    if joint_id in FC_MOTOR_JOINT_LABELS.get(backend, {}):
        name, short, desc = FC_MOTOR_JOINT_LABELS[backend][joint_id]
        return JointInfo(id=joint_id, name=name, short_name=short, description=desc)
    return JOINT_BY_ID.get(joint_id, JOINTS[0])


def get_joint_names(fc_backend: str = FC_MOTOR_BACKEND_HUB) -> List[str]:
    """Get list of joint display names for dropdown."""
    backend = _normalize_fc_backend(fc_backend)
    names: List[str] = []
    for joint in JOINTS:
        if joint.id in FC_MOTOR_JOINT_LABELS.get(backend, {}):
            info = get_joint_info(joint.id, backend)
            names.append(info.display_name())
        else:
            names.append(joint.display_name())
    return names


def get_joint_id_from_index(index: int) -> int:
    """Convert dropdown index (0-based) to joint ID (1-based)."""
    return index + 1


def is_fc_motor_actuator(joint_id: int) -> bool:
    """True for front-caster actuators 9 and 10 (hub or ODrive)."""
    return joint_id in FC_MOTOR_ACTUATOR_IDS


def is_hub_motor_actuator(joint_id: int) -> bool:
    """True when joint_id is a front-caster actuator (hub or ODrive)."""
    return is_fc_motor_actuator(joint_id)


def is_odrive_actuator(joint_id: int) -> bool:
    """True when joint_id is a front-caster actuator (hub or ODrive)."""
    return is_fc_motor_actuator(joint_id)


def get_fc_motor_sequence_names(
    fc_backend: str = FC_MOTOR_BACKEND_HUB,
) -> Tuple[str, str]:
    """Sequence table column labels for motors 9 and 10 (R, L order)."""
    backend = _normalize_fc_backend(fc_backend)
    return FC_MOTOR_SEQUENCE_NAMES[backend]
