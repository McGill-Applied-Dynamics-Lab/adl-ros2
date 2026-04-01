"""
cartesian_to_joint_trajectory.py
---------------------------------
Converts a Cartesian end-effector trajectory (list of poses + timestamps)
to a joint-space JointTrajectory message using Pinocchio IK.

Designed for the Franka Research 3 with franka_ros2, to be used with
joint_trajectory_controller in position-command mode.

Key design decisions:
  - Uses Pinocchio's damped least-squares IK (avoids singularity blow-up)
  - Seeds each IK solve from the previous solution (keeps joint path smooth)
  - Validates velocity AND acceleration against FR3 hard limits before returning
  - Numerically differentiates joint positions to fill in velocities/accelerations
    so joint_trajectory_controller's spline interpolation has full C2 information

Usage:
    from cartesian_to_joint_trajectory import CartesianToJointTrajectory

    converter = CartesianToJointTrajectory(urdf_path="/path/to/fr3.urdf")
    joint_traj_msg = converter.convert(cartesian_waypoints, timestamps)
    publisher.publish(joint_traj_msg)
"""

from __future__ import annotations

import numpy as np
from dataclasses import dataclass
from typing import List, Optional, Tuple
from pathlib import Path

import pinocchio as pin

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# ---------------------------------------------------------------------------
# FR3 joint limits (from Franka documentation)
# libfranka will fault hard if these are exceeded — we validate against them.
# ---------------------------------------------------------------------------
FR3_JOINT_NAMES = [
    "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4",
    "fr3_joint5", "fr3_joint6", "fr3_joint7",
]

FR3_JOINT_POS_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
FR3_JOINT_POS_MAX = np.array([ 2.8973,  1.7628,  2.8973, -0.0698,  2.8973,  3.7525,  2.8973])

# Conservative velocity limits (95% of Franka's hard limits for safety margin)
FR3_JOINT_VEL_MAX = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]) * 0.95

# Conservative acceleration limits
FR3_JOINT_ACC_MAX = np.array([10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]) * 0.90


@dataclass
class CartesianWaypoint:
    """A single Cartesian waypoint: SE3 pose + timestamp."""
    pose: pin.SE3          # Pinocchio SE3: rotation matrix + translation
    time_from_start: float  # seconds from trajectory start


class CartesianToJointTrajectory:
    """
    Converts Cartesian waypoints to a JointTrajectory ROS2 message
    using Pinocchio IK with warm-starting and limit validation.
    """

    def __init__(
        self,
        urdf_path: str,
        end_effector_frame: str = "fr3_hand_tcp",
        joint_names: Optional[List[str]] = None,
        ik_max_iterations: int = 1000,
        ik_tolerance: float = 1e-6,
        ik_damping: float = 1e-6,
    ):
        """
        Args:
            urdf_path: Absolute path to the FR3 URDF file.
                       Typically found at:
                       $(ros2 pkg prefix franka_description)/share/franka_description/
                       robots/fr3/fr3.urdf
                       Generate from xacro first if needed:
                         xacro fr3.urdf.xacro hand:=true > fr3.urdf
            end_effector_frame: Name of the EE frame in the URDF.
            joint_names: Joint names in order. Defaults to FR3_JOINT_NAMES.
            ik_max_iterations: Max iterations per IK solve.
            ik_tolerance: Convergence tolerance (SE3 error norm).
            ik_damping: Damping factor for damped least-squares IK.
                        Increase (e.g. 1e-4) near singularities.
        """
        self.urdf_path = str(urdf_path)
        self.ee_frame_name = end_effector_frame
        self.joint_names = joint_names or FR3_JOINT_NAMES
        self.ik_max_iter = ik_max_iterations
        self.ik_tol = ik_tolerance
        self.ik_damping = ik_damping

        # Load model
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()

        # Resolve EE frame ID
        if self.ee_frame_name not in self.model.frames:
            # Try to find it
            frame_names = [f.name for f in self.model.frames]
            raise ValueError(
                f"Frame '{self.ee_frame_name}' not found in URDF.\n"
                f"Available frames: {frame_names}"
            )
        self.ee_frame_id = self.model.getFrameId(self.ee_frame_name)

        # Map joint names to Pinocchio joint IDs
        self._joint_ids = []
        for name in self.joint_names:
            if self.model.existJointName(name):
                self._joint_ids.append(self.model.getJointId(name))
            else:
                raise ValueError(f"Joint '{name}' not found in URDF model.")

        # Build index map: joint_name -> index in q vector
        self._q_indices = []
        for jid in self._joint_ids:
            self._q_indices.append(self.model.joints[jid].idx_q)

        print(f"[CartesianToJointTrajectory] Loaded model: {self.model.name}")
        print(f"[CartesianToJointTrajectory] EE frame: {self.ee_frame_name} (id={self.ee_frame_id})")
        print(f"[CartesianToJointTrajectory] Controlling {len(self.joint_names)} joints")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def convert(
        self,
        waypoints: List[CartesianWaypoint],
        q_seed: Optional[np.ndarray] = None,
    ) -> JointTrajectory:
        """
        Convert a list of Cartesian waypoints to a JointTrajectory message.

        Args:
            waypoints: Ordered list of CartesianWaypoint (pose + timestamp).
                       Must have monotonically increasing time_from_start.
            q_seed: Initial joint configuration (7,) for the first IK solve.
                    If None, uses the robot's neutral position.
                    IMPORTANT: pass the robot's current joint state here so
                    the trajectory starts exactly where the robot is.

        Returns:
            JointTrajectory message ready to publish to joint_trajectory_controller.

        Raises:
            ValueError: If IK fails to converge for any waypoint, or if
                        joint limits are violated.
        """
        if len(waypoints) < 2:
            raise ValueError("Need at least 2 waypoints.")

        # Validate timestamps are monotonically increasing
        times = [w.time_from_start for w in waypoints]
        if any(t1 <= t0 for t0, t1 in zip(times[:-1], times[1:])):
            raise ValueError("Waypoint timestamps must be strictly increasing.")

        # Solve IK for all waypoints
        print(f"[CartesianToJointTrajectory] Solving IK for {len(waypoints)} waypoints...")
        q_seed_current = q_seed if q_seed is not None else pin.neutral(self.model)
        joint_positions = []

        for i, wp in enumerate(waypoints):
            q_sol, success, error = self._solve_ik(wp.pose, q_seed_current)
            if not success:
                raise ValueError(
                    f"IK failed to converge at waypoint {i} "
                    f"(t={wp.time_from_start:.3f}s). "
                    f"Final error: {error:.6f}. "
                    f"Consider adjusting the trajectory or IK damping."
                )
            joint_positions.append(q_sol)
            q_seed_current = q_sol  # warm-start next solve from this solution
            if (i + 1) % 10 == 0 or i == len(waypoints) - 1:
                print(f"  IK: {i+1}/{len(waypoints)} waypoints solved")

        joint_positions = np.array(joint_positions)  # (N, 7)
        times_arr = np.array(times)                  # (N,)

        # Numerically differentiate to get velocities and accelerations.
        # joint_trajectory_controller uses these as boundary conditions for
        # its spline interpolation — providing them gives C2 continuity.
        # Without them, the controller assumes zero velocity at every waypoint,
        # which creates unnecessary deceleration/acceleration at each point.
        velocities = self._compute_velocities(joint_positions, times_arr)
        accelerations = self._compute_accelerations(velocities, times_arr)

        # Validate against FR3 limits before touching the robot
        self._validate_limits(joint_positions, velocities, accelerations)

        # Build ROS2 message
        msg = self._build_message(joint_positions, velocities, accelerations, times_arr)
        print(f"[CartesianToJointTrajectory] Trajectory ready: "
              f"{len(waypoints)} points, duration={times[-1]:.2f}s")
        return msg

    def forward_kinematics(self, q: np.ndarray) -> pin.SE3:
        """Compute FK for a given joint configuration. Useful for validation."""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, self.ee_frame_id)
        return self.data.oMf[self.ee_frame_id].copy()

    # ------------------------------------------------------------------
    # IK solver
    # ------------------------------------------------------------------

    def _solve_ik(
        self,
        target_pose: pin.SE3,
        q_init: np.ndarray,
    ) -> Tuple[np.ndarray, bool, float]:
        """
        Solve IK using damped least-squares (Levenberg-Marquardt style).
        Warm-started from q_init.

        Returns:
            (q_solution, converged, final_error_norm)
        """
        q = q_init.copy()
        damping_sq = self.ik_damping ** 2

        for _ in range(self.ik_max_iter):
            # Forward kinematics + frame placement
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacement(self.model, self.data, self.ee_frame_id)

            # SE3 error in local frame
            current_pose = self.data.oMf[self.ee_frame_id]
            error_se3 = current_pose.actInv(target_pose)
            error_vec = pin.log6(error_se3).vector  # 6D twist error

            error_norm = np.linalg.norm(error_vec)
            if error_norm < self.ik_tol:
                return self._extract_joint_positions(q), True, error_norm

            # Jacobian in local frame
            J = pin.computeFrameJacobian(
                self.model, self.data, q, self.ee_frame_id,
                pin.ReferenceFrame.LOCAL
            )

            # Damped least-squares step: dq = J^T (J J^T + λ²I)^{-1} err
            JJT = J @ J.T + damping_sq * np.eye(6)
            dq = J.T @ np.linalg.solve(JJT, error_vec)

            # Integrate on manifold (handles SO3 correctly)
            q = pin.integrate(self.model, q, dq)

            # Clamp to joint position limits (prevents IK from wandering
            # into infeasible configurations during search)
            q_joints = self._extract_joint_positions(q)
            q_joints = np.clip(q_joints, FR3_JOINT_POS_MIN, FR3_JOINT_POS_MAX)
            self._set_joint_positions(q, q_joints)

        # Did not converge
        final_error = np.linalg.norm(pin.log6(
            self.data.oMf[self.ee_frame_id].actInv(target_pose)
        ).vector)
        return self._extract_joint_positions(q), False, final_error

    # ------------------------------------------------------------------
    # Numerical differentiation
    # ------------------------------------------------------------------

    def _compute_velocities(
        self, positions: np.ndarray, times: np.ndarray
    ) -> np.ndarray:
        """
        Compute joint velocities using central differences (interior points)
        and one-sided differences at endpoints.
        Central differences are more accurate for non-uniform time steps.
        """
        N = len(times)
        vels = np.zeros_like(positions)

        for i in range(N):
            if i == 0:
                # Forward difference at start — zero velocity (starts from rest)
                # Change this if you need to start with non-zero velocity
                vels[i] = np.zeros(7)
            elif i == N - 1:
                # Zero velocity at end (stop at goal)
                vels[i] = np.zeros(7)
            else:
                # Central difference: v[i] = (q[i+1] - q[i-1]) / (t[i+1] - t[i-1])
                dt = times[i + 1] - times[i - 1]
                vels[i] = (positions[i + 1] - positions[i - 1]) / dt

        return vels

    def _compute_accelerations(
        self, velocities: np.ndarray, times: np.ndarray
    ) -> np.ndarray:
        """
        Compute joint accelerations using central differences on velocities.
        Zero at endpoints (start and stop smoothly).
        """
        N = len(times)
        accs = np.zeros_like(velocities)

        for i in range(1, N - 1):
            dt = times[i + 1] - times[i - 1]
            accs[i] = (velocities[i + 1] - velocities[i - 1]) / dt

        return accs

    # ------------------------------------------------------------------
    # Limit validation
    # ------------------------------------------------------------------

    def _validate_limits(
        self,
        positions: np.ndarray,
        velocities: np.ndarray,
        accelerations: np.ndarray,
    ) -> None:
        """
        Validate joint positions, velocities, and accelerations against
        FR3 hard limits. Raises ValueError with a clear message if violated.
        """
        errors = []

        # Position limits
        pos_below = positions < FR3_JOINT_POS_MIN
        pos_above = positions > FR3_JOINT_POS_MAX
        if pos_below.any() or pos_above.any():
            for j in range(7):
                if pos_below[:, j].any():
                    worst = positions[:, j].min()
                    errors.append(
                        f"  {FR3_JOINT_NAMES[j]}: position {worst:.4f} rad "
                        f"< min {FR3_JOINT_POS_MIN[j]:.4f} rad"
                    )
                if pos_above[:, j].any():
                    worst = positions[:, j].max()
                    errors.append(
                        f"  {FR3_JOINT_NAMES[j]}: position {worst:.4f} rad "
                        f"> max {FR3_JOINT_POS_MAX[j]:.4f} rad"
                    )

        # Velocity limits
        vel_abs = np.abs(velocities)
        if (vel_abs > FR3_JOINT_VEL_MAX).any():
            for j in range(7):
                worst = vel_abs[:, j].max()
                if worst > FR3_JOINT_VEL_MAX[j]:
                    errors.append(
                        f"  {FR3_JOINT_NAMES[j]}: peak velocity {worst:.4f} rad/s "
                        f"> limit {FR3_JOINT_VEL_MAX[j]:.4f} rad/s. "
                        f"Slow down the trajectory."
                    )

        # Acceleration limits
        acc_abs = np.abs(accelerations)
        if (acc_abs > FR3_JOINT_ACC_MAX).any():
            for j in range(7):
                worst = acc_abs[:, j].max()
                if worst > FR3_JOINT_ACC_MAX[j]:
                    errors.append(
                        f"  {FR3_JOINT_NAMES[j]}: peak acceleration {worst:.4f} rad/s² "
                        f"> limit {FR3_JOINT_ACC_MAX[j]:.4f} rad/s². "
                        f"Add more waypoints or increase segment duration."
                    )

        if errors:
            raise ValueError(
                "Joint limit violations found — trajectory NOT sent to robot:\n"
                + "\n".join(errors)
            )

        print("[CartesianToJointTrajectory] All joint limits OK ✓")

    # ------------------------------------------------------------------
    # Message construction
    # ------------------------------------------------------------------

    def _build_message(
        self,
        positions: np.ndarray,
        velocities: np.ndarray,
        accelerations: np.ndarray,
        times: np.ndarray,
    ) -> JointTrajectory:
        msg = JointTrajectory()
        msg.joint_names = self.joint_names

        for i in range(len(times)):
            point = JointTrajectoryPoint()
            point.positions = positions[i].tolist()
            point.velocities = velocities[i].tolist()
            point.accelerations = accelerations[i].tolist()

            # Convert float seconds to builtin_interfaces/Duration
            secs = int(times[i])
            nsecs = int((times[i] - secs) * 1e9)
            point.time_from_start = Duration(sec=secs, nanosec=nsecs)

            msg.points.append(point)

        return msg

    # ------------------------------------------------------------------
    # Helpers: extract/set joint positions from full Pinocchio q vector
    # ------------------------------------------------------------------

    def _extract_joint_positions(self, q: np.ndarray) -> np.ndarray:
        """Extract the 7 controlled joint positions from Pinocchio's q vector."""
        return np.array([q[idx] for idx in self._q_indices])

    def _set_joint_positions(self, q: np.ndarray, joint_vals: np.ndarray) -> None:
        """Write 7 joint positions back into Pinocchio's q vector in-place."""
        for idx, val in zip(self._q_indices, joint_vals):
            q[idx] = val


# ---------------------------------------------------------------------------
# Convenience helpers for building CartesianWaypoint lists
# ---------------------------------------------------------------------------

def pose_from_matrix(matrix: np.ndarray) -> pin.SE3:
    """Build a Pinocchio SE3 from a 4x4 homogeneous transform matrix."""
    return pin.SE3(matrix[:3, :3], matrix[:3, 3])


def pose_from_rpy_xyz(roll: float, pitch: float, yaw: float,
                      x: float, y: float, z: float) -> pin.SE3:
    """Build a Pinocchio SE3 from RPY angles (radians) and XYZ translation (meters)."""
    R = pin.rpy.rpyToMatrix(roll, pitch, yaw)
    return pin.SE3(R, np.array([x, y, z]))


def pose_from_quaternion_xyz(qx: float, qy: float, qz: float, qw: float,
                              x: float, y: float, z: float) -> pin.SE3:
    """Build a Pinocchio SE3 from a quaternion (x,y,z,w) and XYZ translation."""
    quat = pin.Quaternion(qw, qx, qy, qz)
    quat.normalize()
    return pin.SE3(quat.matrix(), np.array([x, y, z]))
