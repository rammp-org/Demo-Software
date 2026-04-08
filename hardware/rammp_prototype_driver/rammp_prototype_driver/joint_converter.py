"""Per-joint encoder-tick to physical-unit conversion config.

Edit JOINT_CONVERSIONS below to adjust per-joint mapping.
Applied at publish time — raw ticks are preserved internally.
"""

import math
from dataclasses import dataclass


@dataclass
class JointConversion:
    """Linear mapping from encoder ticks to physical units.

    output = output_min + (ticks - tick_min) * scale
    velocity uses the same scale factor.
    """

    tick_min: float
    tick_max: float
    output_min: float  # radians for revolute, meters for prismatic
    output_max: float

    @property
    def scale(self) -> float:
        tick_range = self.tick_max - self.tick_min
        if tick_range == 0.0:
            return 0.0
        return (self.output_max - self.output_min) / tick_range

    def position(self, ticks: float) -> float:
        return self.output_min + (ticks - self.tick_min) * self.scale

    def velocity(self, tick_velocity: float) -> float:
        return tick_velocity * self.scale


# ──────────────────────────────────────────────────────────────────────
#  CONVERSION CONFIGS
#  Outputs radians (revolute) and meters (prismatic) — matches arm
#  driver convention. GuiBridge converts to degrees/cm for Unreal.
# ──────────────────────────────────────────────────────────────────────
#
#  Joint names match URDF joint names.
#
#  Tick ranges and output ranges are PLACEHOLDERS. Ranges
#  are approximate — this is a pure linear scale.
# ──────────────────────────────────────────────────────────────────────

# /base/joint_states topic order — differs from firmware SEAT_DELTAS motor order
BASE_JOINT_ORDER = (
    "front_caster_swing_arm",
    "rear_caster_swing_arm",
    "motor_swing_arm_r",
    "motor_swing_arm_l",
    "dw_main_plate_l",
    "dw_main_plate_r",
    "drive_wheel_l",
    "drive_wheel_r",
)

JOINT_CONVERSIONS: dict[str, JointConversion] = {
    # ── Vertical actuators: ticks → radians ──────────────────────────
    "rear_caster_swing_arm": JointConversion(
        tick_min=0.0,
        tick_max=900.0,
        output_min=math.radians(-5.0),
        output_max=math.radians(75.0),
    ),
    "front_caster_swing_arm": JointConversion(
        tick_min=0.0,
        tick_max=900.0,
        output_min=math.radians(-5.0),
        output_max=math.radians(75.0),
    ),
    "motor_swing_arm_l": JointConversion(
        tick_min=0.0,
        tick_max=700.0,
        output_min=math.radians(0.0),
        output_max=math.radians(65.0),
    ),
    "motor_swing_arm_r": JointConversion(
        tick_min=0.0,
        tick_max=700.0,
        output_min=math.radians(0.0),
        output_max=math.radians(65.0),
    ),
    # ── Carriages: ticks → linear displacement (meters) ──────────────
    #    0-12000ish ticks  →  0-0.30m  (length of carriage travel)
    "dw_main_plate_l": JointConversion(
        tick_min=0.0,
        tick_max=12000.0,
        output_min=0.0,
        output_max=0.30,
    ),
    "dw_main_plate_r": JointConversion(
        tick_min=0.0,
        tick_max=12000.0,
        output_min=0.0,
        output_max=0.30,
    ),
    # ── Drive wheels: ticks → radians (continuous rotation) ──────────
    #    velocity: tick_vel * scale = rad/s
    #    Placeholder: 4096 ticks per revolution (adjust to actual encoder × gear ratio)
    "drive_wheel_l": JointConversion(
        tick_min=0.0,
        tick_max=4096.0,
        output_min=0.0,
        output_max=2.0 * math.pi,
    ),
    "drive_wheel_r": JointConversion(
        tick_min=0.0,
        tick_max=4096.0,
        output_min=0.0,
        output_max=2.0 * math.pi,
    ),
}
