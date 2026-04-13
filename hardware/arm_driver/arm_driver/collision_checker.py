"""Collision detection via Pinocchio RNEA torque residuals.

Compares measured joint torques from the arm against torques predicted by the
rigid-body dynamics model.  A large residual indicates an unexpected external
force — i.e., a collision.

Usage::

    checker = CollisionChecker(
        urdf_string=urdf_xml,
        thresholds={"DEFAULT": 100.0, "OPEN_DOOR": 500.0},
    )

    if checker.check(q, dq, tau, state_name="IDLE"):
        # collision detected
"""

import math

import numpy as np
import pinocchio as pin


class CollisionChecker:
    """Detects arm collisions by comparing measured joint torques against a
    Pinocchio RNEA model prediction.

    The residual is the max absolute difference across all joints between the
    measured torque and the model-predicted torque (gravity + Coriolis, zero
    acceleration assumed).  If the residual exceeds the threshold for the
    current arm state, a collision is reported.

    Args:
        urdf_string: URDF XML as a string (e.g. the output of running xacro).
        thresholds: Mapping from ArmState name (e.g. ``"OPEN_DOOR"``) to
            residual threshold in Nm.  Any state not explicitly listed falls
            back to ``thresholds["DEFAULT"]``.
    """

    # Number of configuration/velocity entries belonging to the 7 arm joints.
    # The xacro-generated URDF includes gripper joints as active joints in the
    # Pinocchio model (model.nq > 11, model.nv > 7).  Arm joints always come
    # first in the kinematic chain, so these constants let us slice the correct
    # entries without needing to query joint names at runtime.
    _ARM_NQ = 11  # 4 continuous joints (nq=2 each) + 3 revolute joints (nq=1 each)
    _ARM_NV = 7  # one velocity DOF per joint regardless of joint type

    def __init__(self, urdf_string: str, thresholds: dict[str, float]) -> None:
        if "DEFAULT" not in thresholds:
            raise ValueError("thresholds dict must contain a 'DEFAULT' key")
        self._model = pin.buildModelFromXML(urdf_string)
        self._model_data = self._model.createData()
        self._thresholds = thresholds
        # Full neutral configuration used as the base when building q vectors.
        # Gripper joints stay at their neutral position during arm collision checks.
        self._q_neutral = pin.neutral(self._model)
        # Per-joint residuals from the most recent check() call (7,).
        # Exposed for diagnostics without changing the bool return type.
        self.last_residuals: np.ndarray = np.zeros(self._ARM_NV)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def check(
        self,
        q: np.ndarray,
        dq: np.ndarray,
        tau: np.ndarray,
        state_name: str,
    ) -> bool:
        """Return True if a collision is detected.

        Args:
            q: Joint positions (7,) in radians — arm joints only, no gripper.
            dq: Joint velocities (7,) in rad/s.
            tau: Measured joint torques (7,) in Nm.
            state_name: Current ArmState name, used to look up the threshold.

        Returns:
            True if ``max(|tau_arm - tau_model_arm|) > threshold``.
        """
        q_full = self._build_q_full(q)

        dq_full = np.zeros(self._model.nv)
        dq_full[: self._ARM_NV] = dq

        pin.rnea(
            self._model, self._model_data, q_full, dq_full, np.zeros(self._model.nv)
        )
        tau_model_arm = self._model_data.tau[: self._ARM_NV].copy()

        self.last_residuals = np.abs(tau - tau_model_arm)
        residual = float(np.max(self.last_residuals))
        threshold = self._thresholds.get(state_name, self._thresholds["DEFAULT"])
        return residual > threshold

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _build_q_full(self, q: np.ndarray) -> np.ndarray:
        """Build the full nq configuration vector for RNEA.

        Arm joints occupy the first _ARM_NQ entries; gripper joints are left
        at their neutral positions.
        """
        q_full = self._q_neutral.copy()
        q_full[: self._ARM_NQ] = self._to_q_pin(q)
        return q_full

    @staticmethod
    def _to_q_pin(q: np.ndarray) -> np.ndarray:
        """Convert a 7-element joint angle vector to an 11-element Pinocchio
        configuration for the arm joints.

        The Gen3 URDF has alternating joint types:
        - joints 1, 3, 5, 7 (indices 0, 2, 4, 6) are ``continuous``
          → Pinocchio encodes each as ``[cos(θ), sin(θ)]``
        - joints 2, 4, 6   (indices 1, 3, 5)   are ``revolute``
          → Pinocchio encodes each as ``[θ]``

        The resulting vector has 11 elements.
        """
        return np.array(
            [
                math.cos(q[0]),
                math.sin(q[0]),  # joint_1  (continuous)
                q[1],  # joint_2  (revolute)
                math.cos(q[2]),
                math.sin(q[2]),  # joint_3  (continuous)
                q[3],  # joint_4  (revolute)
                math.cos(q[4]),
                math.sin(q[4]),  # joint_5  (continuous)
                q[5],  # joint_6  (revolute)
                math.cos(q[6]),
                math.sin(q[6]),  # joint_7  (continuous)
            ]
        )
