import numpy as np
try:
    import pinocchio as pin
    _pin_available = True
except ImportError:
    _pin_available = False
from typing import Tuple, Optional
import yaml
from pathlib import Path

from dexrobot_kinematics.utils.types import Position


class HandIKSolver:
    """A Pinocchio-based IK solver for robotic hand that handles joint limits and finger joint synchronization"""

    def __init__(
        self,
        robot,
        handedness: str,
        target_frame: str,
        config_path: Path,
        eps: float = 1e-4,
        max_iter: int = 200,
        damping: float = 0.01,
        dt: float = 1,
    ):
        """
        Args:
            robot: Pinocchio robot wrapper instance
            handedness: Handedness of the robot ("right" or "left")
            target_frame: Target frame name for IK
            config_path: Path to joint limits configuration file
            eps: Convergence threshold
            max_iter: Maximum iterations
            damping: Damping factor for least squares
            dt: Integration time step
        """
        if not _pin_available:
            raise ImportError("pinocchio is required for HandIKSolver but is not installed")
        self.robot = robot
        self.handedness = handedness
        self.target_frame = target_frame
        self.frame_id = robot.model.getFrameId(target_frame)
        
        # Verify frame exists in the model
        if self.frame_id == 0:
            raise ValueError(f"Target frame '{target_frame}' not found in the URDF model")
            
        self.eps = eps
        self.max_iter = max_iter
        self.damping = damping
        self.dt = dt
        self.nq = robot.model.nq
        self.nv = robot.model.nv
        self.joint_lower_limits = robot.model.lowerPositionLimit
        self.joint_upper_limits = robot.model.upperPositionLimit

    def _enforce_joint_limits(self, q: np.ndarray) -> np.ndarray:
        """
        Args:
            q: Joint configuration vector
        Returns:
            Configuration vector with enforced joint limits
        """
        return np.clip(q, self.joint_lower_limits, self.joint_upper_limits)

    def _synchronize_finger_joints(self, q: np.ndarray) -> np.ndarray:
        """
        Args:
            q: Joint configuration vector
        Returns:
            Configuration vector with synchronized joints 3 and 4 for each finger
        """
        q_sync = q.copy()
        prefix = self.handedness[0]  # "r" or "l"
        fingers = [f"{prefix}_f_joint1", f"{prefix}_f_joint2", f"{prefix}_f_joint3", f"{prefix}_f_joint4", f"{prefix}_f_joint5"]

        for finger in fingers:
            j3_joint = f"{finger}_3"
            j4_joint = f"{finger}_4"
            j3_id = self.robot.model.getJointId(j3_joint)
            j4_id = self.robot.model.getJointId(j4_joint)

            # Check if both joints exist
            if j3_id == 0:
                print(f"Warning: Joint '{j3_joint}' not found in the URDF model, cannot synchronize")
                continue
                
            if j4_id == 0:
                print(f"Warning: Joint '{j4_joint}' not found in the URDF model, cannot synchronize")
                continue
                
            # Make sure indices are valid before accessing array
            if j3_id > 0 and j4_id > 0:
                try:
                    avg_angle = (q_sync[j3_id - 1] + q_sync[j4_id - 1]) / 2.0
                    q_sync[j3_id - 1] = q_sync[j4_id - 1] = avg_angle
                except IndexError as e:
                    print(f"Warning: Index error when synchronizing joints {j3_joint} (ID: {j3_id}) and {j4_joint} (ID: {j4_id}): {str(e)}")

        return q_sync

    def solve(
        self,
        target_pos: Position,
        initial_q: Optional[np.ndarray] = None,
        orientation_weight: float = 0.0,
    ) -> Tuple[np.ndarray, bool]:
        """
        Solve inverse kinematics

        Args:
            target_pos: Target end-effector position
            initial_q: Initial joint configuration
            orientation_weight: Weight for orientation constraint
        Returns:
            Tuple of (final joint configuration, success flag)
        """
        try:
            if initial_q is None:
                q = pin.neutral(self.robot.model)
            else:
                q = initial_q.copy()

            target = target_pos.to_array()

            for i in range(self.max_iter):
                pin.forwardKinematics(self.robot.model, self.robot.data, q)
                pin.updateFramePlacements(self.robot.model, self.robot.data)

                current_pos = self.robot.data.oMf[self.frame_id].translation
                err = target - current_pos

                if np.linalg.norm(err) < self.eps:
                    q = self._enforce_joint_limits(q)
                    q = self._synchronize_finger_joints(q)
                    return q, True

                # Get the current orientation of the frame
                R_world_to_local = self.robot.data.oMf[self.frame_id].rotation.T

                # Convert the error to the local frame
                err_local = R_world_to_local @ err

                J = pin.computeFrameJacobian(
                    self.robot.model, self.robot.data, q, self.frame_id
                )[:3, :]

                JJt = J @ J.T
                lambda2 = self.damping**2
                v = J.T @ np.linalg.solve(JJt + lambda2 * np.eye(3), err_local)

                q = pin.integrate(self.robot.model, q, v * self.dt)
                q = self._enforce_joint_limits(q)
                q = self._synchronize_finger_joints(q)

            return q, False

        except IndexError as e:
            print(f"IK solver error: Index out of range for target frame '{self.target_frame}' (ID: {self.frame_id}). This is likely because the frame is missing in the URDF model. Original error: {str(e)}")
            return pin.neutral(self.robot.model), False
        except Exception as e:
            print(f"IK solver error: {type(e).__name__}: {str(e)}")
            return pin.neutral(self.robot.model), False
