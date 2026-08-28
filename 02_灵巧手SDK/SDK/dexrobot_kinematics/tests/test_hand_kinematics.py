import numpy as np
import pinocchio as pin
import pytest
from pathlib import Path
from dexrobot_kinematics.hand import RightHandKinematics, LeftHandKinematics
from dexrobot_kinematics.utils.types import Position, Pose


@pytest.fixture
def right_hand():
    """Fixture providing initialized right hand kinematics"""
    return RightHandKinematics()

@pytest.fixture
def left_hand():
    """Fixture providing initialized left hand kinematics"""
    return LeftHandKinematics()


def test_forward_kinematics_zero(right_hand):
    """Test FK with zero configuration"""
    # All joints at zero
    joint_angles = {name: 0.0 for name in right_hand.robot.model.names[1:]}

    # Get fingerpad poses
    poses = right_hand.forward_kinematics(joint_angles)

    # Check results
    assert len(poses) == 5  # All fingers
    for pose in poses.values():
        assert isinstance(pose, Pose)
        # Fingertips should be slightly in front of hand base
        assert 0 < pose.position.x < 0.1


def test_forward_kinematics_fingerpad(right_hand):
    """Test the difference between fingerpad and fingertip poses"""
    joint_angles = {name: 0.0 for name in right_hand.robot.model.names[1:]}
    poses_fingerpad = right_hand.forward_kinematics(
        joint_angles, end_effector="fingerpad"
    )
    poses_fingertip = right_hand.forward_kinematics(
        joint_angles, end_effector="fingertip"
    )

    # Check results
    assert len(poses_fingerpad) == 5
    assert len(poses_fingertip) == 5
    for finger in poses_fingerpad.keys():
        pos_fingerpad = poses_fingerpad[finger].position
        pos_fingertip = poses_fingertip[finger].position
        # Fingertip and fingerpad should be close
        assert (
            np.linalg.norm(pos_fingerpad.to_array() - pos_fingertip.to_array()) < 0.05
        )
        # Fingertip should be in front of fingerpad
        if finger == "thumb":
            assert pos_fingertip.y > pos_fingerpad.y
        else:
            assert pos_fingertip.z > pos_fingerpad.z


def test_forward_kinematics_base_pose(right_hand):
    """Test FK with hand base pose"""
    joint_angles = {name: 0.0 for name in right_hand.robot.model.names[1:]}

    # Set base pose (90 deg rotation around y and 1m translation in x)
    R = pin.utils.rpyToMatrix(0, np.pi / 2, 0)
    t = np.array([1.0, 0.0, 0.0])
    base_pose = Pose(position=Position.from_array(t), orientation=R)

    poses = right_hand.forward_kinematics(
        joint_angles, base_pose=base_pose, frame="world"
    )

    # Verify transformation
    for pose in poses.values():
        # Points should be rotated and translated
        assert pose.position.x > 1
        assert -0.1 < pose.position.z < 0


def test_inverse_kinematics_finger(right_hand):
    """Test single finger IK"""
    # Target position for index fingerpad
    joint_angles = {name: 0.0 for name in right_hand.robot.model.names[1:]}
    joint_angles["r_f_joint2_1"] = 0.5
    joint_angles["r_f_joint2_2"] = 0.5
    joint_angles["r_f_joint2_3"] = 0.5
    joint_angles["r_f_joint2_4"] = 0.5
    target = right_hand.forward_kinematics(joint_angles)["index"].position

    joint_angles, success = right_hand.inverse_kinematics_finger(
        finger="index", target_pos=target
    )

    assert success

    # Verify result
    poses = right_hand.forward_kinematics(joint_angles)
    actual_pos = poses["index"].position.to_array()
    assert np.allclose(actual_pos, target.to_array(), atol=0.001)


def test_inverse_kinematics_finger_base_pose(right_hand):
    """Test single finger IK with base pose"""
    joint_angles = {name: 0.0 for name in right_hand.robot.model.names[1:]}
    joint_angles["r_f_joint2_1"] = 0.5
    joint_angles["r_f_joint2_2"] = 0.5
    joint_angles["r_f_joint2_3"] = 0.5
    joint_angles["r_f_joint2_4"] = 0.5
    R = pin.utils.rpyToMatrix(0, np.pi / 2, 0)
    t = np.array([1.0, 0.0, 0.0])
    base_pose = Pose(position=Position.from_array(t), orientation=R)
    target = right_hand.forward_kinematics(joint_angles)["index"].position
    target_world = right_hand.forward_kinematics(joint_angles, base_pose=base_pose, frame="world")["index"].position

    joint_angles, success = right_hand.inverse_kinematics_finger(
        finger="index", target_pos=target
    )
    joint_angles_world, success_world = right_hand.inverse_kinematics_finger(
        finger="index", target_pos=target_world, base_pose=base_pose, frame="world"
    )

    # Result should be invariant to base pose
    assert success
    assert success_world
    assert set(joint_angles.keys()) == set(joint_angles_world.keys())
    for key in joint_angles.keys():
        assert np.isclose(joint_angles[key], joint_angles_world[key], atol=0.001)


def test_inverse_kinematics_finger_unreachable(right_hand):
    """Test IK with unreachable target"""
    # Target too far away
    target = Position.from_array(np.array([0.0, 0.0, 1.0]))

    joint_angles, success = right_hand.inverse_kinematics_finger(
        finger="index", target_pos=target
    )

    assert not success


def test_inverse_kinematics_grasp(right_hand):
    """Test multi-finger grasp IK"""
    # Define pinch grasp targets
    joint_angles = {name: 0.2 for name in right_hand.robot.model.names[1:]}
    joint_angles["r_f_joint3_1"] = 0
    joint_angles["r_f_joint5_1"] = 0.4
    fk_poses = right_hand.forward_kinematics(joint_angles)
    target = {key: value.position for key, value in fk_poses.items()}

    joint_angles, success = right_hand.inverse_kinematics_grasp(target)
    assert success


def test_hand_consistency(right_hand, left_hand):
    """Test consistency between right and left hand kinematics"""
    # Define pinch grasp targets
    joint_angles = {name: 0.2 for name in right_hand.robot.model.names[1:]}
    joint_angles["r_f_joint3_1"] = 0
    joint_angles["r_f_joint5_1"] = 0.4
    fk_poses_right = right_hand.forward_kinematics(joint_angles)
    target_right = {key: value.position for key, value in fk_poses_right.items()}
    joint_angles_left = {f"l_{key[2:]}": value for key, value in joint_angles.items()}
    fk_poses_left = left_hand.forward_kinematics(joint_angles_left)
    target_left = {key: value.position for key, value in fk_poses_left.items()}

    # Verify FK consistency
    for finger in fk_poses_right.keys():
        pos_right = fk_poses_right[finger].position.to_array()
        pos_left = fk_poses_left[finger].position.to_array()
        pos_left[1] = -pos_left[1]
        assert np.allclose(pos_right, pos_left, atol=0.01)

    # Verify IK consistency
    joint_angles, success = right_hand.inverse_kinematics_grasp(target_right)
    assert success
    joint_angles_left, success = left_hand.inverse_kinematics_grasp(target_left)
    assert success
    for key in joint_angles.keys():
        assert np.isclose(joint_angles[key], joint_angles_left[f"l_{key[2:]}"], atol=0.01)
