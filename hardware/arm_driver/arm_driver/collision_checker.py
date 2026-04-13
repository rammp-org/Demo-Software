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

    def __init__(self, urdf_string: str, thresholds: dict[str, float]) -> None:
        if "DEFAULT" not in thresholds:
            raise ValueError("thresholds dict must contain a 'DEFAULT' key")
        self._model = pin.buildModelFromXML(urdf_string)
        self._model_data = self._model.createData()
        self._thresholds = thresholds

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
            True if ``max(|tau - tau_model|) > threshold``.
        """
        q_pin = self._to_q_pin(q)
        pin.rnea(self._model, self._model_data, q_pin, dq, np.zeros(self._model.nv))
        tau_model = self._model_data.tau.copy()

        residual = float(np.max(np.abs(tau - tau_model)))
        threshold = self._thresholds.get(state_name, self._thresholds["DEFAULT"])
        return residual > threshold

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _to_q_pin(q: np.ndarray) -> np.ndarray:
        """Convert a 7-element joint angle vector to a Pinocchio configuration.

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
