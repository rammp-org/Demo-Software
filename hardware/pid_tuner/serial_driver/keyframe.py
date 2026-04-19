"""
Keyframe data model for the Teensy sequence player.

This module is Qt-free so it can be imported by both the pid_tuner GUI
and the ROS driver without pulling in PyQt6.

Motor order: [RC, FC, ML, MR, ML_Carriage, MR_Carriage, Drive_FB, Drive_LR]
"""

from __future__ import annotations

from typing import List, Optional

# Number of motors controlled by the sequence player (must match firmware SEQ_NUM_MOTORS)
NUM_MOTORS = 8


class Keyframe:
    def __init__(self):
        self.label: str = ""
        self.targets: List[Optional[float]] = [None] * NUM_MOTORS
        self.duration_ms: int = 1000
        self.relative: List[bool] = [False] * NUM_MOTORS
        self.motor_durations: List[Optional[int]] = [None] * NUM_MOTORS
        self.guard_threshold: List[float] = [0.0] * NUM_MOTORS
        self.guard_condition: List[int] = [0] * NUM_MOTORS

    def is_active(self, motor_idx: int) -> bool:
        return self.targets[motor_idx] is not None

    def to_dict(self) -> dict:
        return {
            "label": self.label,
            "targets": [t if t is not None else None for t in self.targets],
            "duration_ms": self.duration_ms,
            "relative": self.relative,
            "motor_durations": [
                d if d is not None else None for d in self.motor_durations
            ],
            "guard_threshold": list(self.guard_threshold),
            "guard_condition": list(self.guard_condition),
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Keyframe":
        kf = cls()
        kf.label = d.get("label", "")
        raw_targets = d.get("targets", [None] * NUM_MOTORS)
        while len(raw_targets) < NUM_MOTORS:
            raw_targets.append(None)
        kf.targets = [
            (float(t) if t is not None else None) for t in raw_targets[:NUM_MOTORS]
        ]
        kf.duration_ms = int(d.get("duration_ms", 1000))
        raw_relative = d.get("relative", [False] * NUM_MOTORS)
        while len(raw_relative) < NUM_MOTORS:
            raw_relative.append(False)
        kf.relative = [bool(r) for r in raw_relative[:NUM_MOTORS]]
        raw_motor_durations = d.get("motor_durations", [None] * NUM_MOTORS)
        while len(raw_motor_durations) < NUM_MOTORS:
            raw_motor_durations.append(None)
        kf.motor_durations = [
            (max(0, int(float(v))) if v is not None else None)
            for v in raw_motor_durations[:NUM_MOTORS]
        ]
        raw_guard_threshold = d.get("guard_threshold", [0.0] * NUM_MOTORS)
        while len(raw_guard_threshold) < NUM_MOTORS:
            raw_guard_threshold.append(0.0)
        kf.guard_threshold = [float(v) for v in raw_guard_threshold[:NUM_MOTORS]]

        raw_guard_condition = d.get("guard_condition", [0] * NUM_MOTORS)
        while len(raw_guard_condition) < NUM_MOTORS:
            raw_guard_condition.append(0)
        kf.guard_condition = [int(v) for v in raw_guard_condition[:NUM_MOTORS]]
        return kf
