import numpy as np
import pytest
from dexrobot_kinematics.system.arm_hand import ArmHandSystem
from dexrobot_kinematics.hand.base import HandKinematicsBase

@pytest.fixture
def arm_hand_system(right_hand):
    """Fixture for arm-hand system"""
    arm_urdf = Path("path/to/arm.urdf")
    return ArmHandSystem(right_hand, arm_urdf, mount_frame="tool0")

def test_system_forward_kinematics(arm_hand_system):
    """Test combined system FK"""
    # Zero configuration
    arm_joints = {name: 0.0 for name in arm_hand_system.arm.model.names[1:]}
    hand_joints = {name: 0.0 for name in arm_hand_system.hand.robot.model.names[1:]}

    poses = arm_hand_system.forward_kinematics(arm_joints, hand_joints)

    assert len(poses) == 5  # All fingertips
    for pose in poses.values():
        assert isinstance(pose, pin.SE3)

def test_grasp_ik_sequential(arm_hand_system):
    """Test sequential grasp IK"""
    # Define grasp targets in world frame
    targets = {
        'thumb': np.array([0.5, -0.02, 0.5]),
        'index': np.array([0.5, 0.02, 0.5])
    }

    arm_joints, hand_joints, success = arm_hand_system.solve_grasp_sequential(
        targets,
        approach_axis=np.array([1, 0, 0])  # Approach from x direction
    )

    assert success

    # Verify result
    poses = arm_hand_system.forward_kinematics(arm_joints, hand_joints)
    for finger, target in targets.items():
        actual_pos = poses[finger].translation
        assert np.allclose(actual_pos, target, atol=0.01)

def test_grasp_ik_combined(arm_hand_system):
    """Test combined grasp IK"""
    targets = {
        'thumb': np.array([0.5, -0.02, 0.5]),
        'index': np.array([0.5, 0.02, 0.5])
    }

    arm_joints, hand_joints, success = arm_hand_system.solve_grasp_combined(
        targets,
        weights={'position': 1.0, 'orientation': 0.1}
    )

    assert success

    # Verify result
    poses = arm_hand_system.forward_kinematics(arm_joints, hand_joints)
    for finger, target in targets.items():
        actual_pos = poses[finger].translation
        assert np.allclose(actual_pos, target, atol=0.01)
