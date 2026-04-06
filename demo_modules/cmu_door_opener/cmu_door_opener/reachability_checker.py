"""Lightweight IK-based reachability checker using PyBullet.

Loads a URDF (from string) of the Kinova Gen3 7-DOF arm in a headless
(DIRECT) physics server and uses ``pybullet.calculateInverseKinematics``
to decide whether a target end-effector position is kinematically reachable.

No MoveIt, no ROS dependency, no planning pipeline — just geometry.
"""

import tempfile
import os
import numpy as np
import pybullet as p


class ReachabilityChecker:
    """Check whether the Kinova Gen3 end-effector can reach a given pose.

    Parameters
    ----------
    urdf_string : str
        The full URDF XML as a string (e.g. from ``/robot_description``).
    pos_tol : float
        Position tolerance (metres) for the FK validation step.
    max_iters : int
        Maximum refinement iterations per IK attempt.
    """

    def __init__(
        self,
        urdf_string: str,
        pos_tol: float = 0.01,
        max_iters: int = 50,
    ):
        self.pos_tol = pos_tol
        self.max_iters = max_iters

        # Write URDF to a temp file — PyBullet requires a file path.
        self._urdf_tmp = tempfile.NamedTemporaryFile(
            suffix=".urdf", mode="w", delete=False
        )
        self._urdf_tmp.write(urdf_string)
        self._urdf_tmp.close()

        # Headless physics — no rendering, pure kinematics.
        self._cid = p.connect(p.DIRECT)
        self._robot = p.loadURDF(
            self._urdf_tmp.name, useFixedBase=True, physicsClientId=self._cid
        )

        # Discover joint indices for the 7 arm revolute/continuous joints.
        self._arm_joints: list[int] = []
        self._ee_link: int = -1
        n = p.getNumJoints(self._robot, physicsClientId=self._cid)
        for i in range(n):
            info = p.getJointInfo(self._robot, i, physicsClientId=self._cid)
            joint_name = info[1].decode("utf-8")
            joint_type = info[2]
            if joint_name.startswith("gen3_joint_") and joint_type in (
                p.JOINT_REVOLUTE,
                p.JOINT_PRISMATIC,
            ):
                self._arm_joints.append(i)
            if joint_name == "gen3_end_effector":
                # child link index for this fixed joint
                self._ee_link = i

        # Fallback: search by link name.
        if self._ee_link == -1:
            for i in range(n):
                info = p.getJointInfo(self._robot, i, physicsClientId=self._cid)
                if info[12].decode("utf-8") == "gen3_end_effector_link":
                    self._ee_link = i
                    break

        if not self._arm_joints or self._ee_link == -1:
            raise RuntimeError(
                f"URDF parsing failed: found {len(self._arm_joints)} arm joints, "
                f"ee_link={self._ee_link}"
            )

        # Cache joint limits for randomised seeding.
        self._lower = []
        self._upper = []
        for jid in self._arm_joints:
            info = p.getJointInfo(self._robot, jid, physicsClientId=self._cid)
            lo, hi = info[8], info[9]
            # Continuous joints have lo >= hi in URDF; use full rotation.
            if lo >= hi:
                lo, hi = -np.pi, np.pi
            self._lower.append(lo)
            self._upper.append(hi)
        self._lower = np.array(self._lower)
        self._upper = np.array(self._upper)

    # ------------------------------------------------------------------
    def is_reachable(self, target_pos, target_orn=None, n_seeds: int = 5) -> bool:
        """Return ``True`` if *target_pos* is within the arm's workspace.

        Tries *n_seeds* random initial joint configurations to handle the
        null-space ambiguity of a 7-DOF arm (multiple elbow configs).

        Parameters
        ----------
        target_pos : array-like, shape (3,)
            Target XYZ in the robot base frame (metres).
        target_orn : array-like, shape (4,), optional
            Target orientation as a quaternion [x, y, z, w].  If ``None``,
            only position is checked (orientation left free).
        n_seeds : int
            Number of random restarts.
        """
        target_pos = list(target_pos)

        for _ in range(n_seeds):
            seed = np.random.uniform(self._lower, self._upper)
            for jid, q in zip(self._arm_joints, seed):
                p.resetJointState(self._robot, jid, q, physicsClientId=self._cid)

            if self._ik_converges(target_pos, target_orn):
                return True

        return False

    # ------------------------------------------------------------------
    def _ik_converges(self, target_pos, target_orn) -> bool:
        """Run iterative IK from the current joint state and check FK."""
        kwargs = dict(
            bodyUniqueId=self._robot,
            endEffectorLinkIndex=self._ee_link,
            targetPosition=target_pos,
            physicsClientId=self._cid,
        )
        if target_orn is not None:
            kwargs["targetOrientation"] = list(target_orn)

        for _ in range(self.max_iters):
            joint_angles = p.calculateInverseKinematics(**kwargs)

            for jid, q in zip(self._arm_joints, joint_angles):
                p.resetJointState(self._robot, jid, q, physicsClientId=self._cid)

            ee_state = p.getLinkState(
                self._robot, self._ee_link, physicsClientId=self._cid
            )
            ee_pos = ee_state[4]  # worldLinkFramePosition
            if (
                abs(ee_pos[0] - target_pos[0]) < self.pos_tol
                and abs(ee_pos[1] - target_pos[1]) < self.pos_tol
                and abs(ee_pos[2] - target_pos[2]) < self.pos_tol
            ):
                return True

        return False

    # ------------------------------------------------------------------
    def destroy(self):
        """Disconnect the PyBullet physics server and clean up temp file."""
        if self._cid >= 0:
            p.disconnect(self._cid)
            self._cid = -1
        try:
            os.unlink(self._urdf_tmp.name)
        except OSError:
            pass
