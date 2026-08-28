"""
Run FK to obtain fingertip and fingerpad positions, from hardware joint angle readings.
"""

import numpy as np
from dexrobot_kinematics.hand import RightHandKinematics
from dexrobot_kinematics.utils.types import Position, Pose
from dexrobot_kinematics.utils.hardware import JointMapping
from pyzlg_dexhand.dexhand_interface import RightDexHand

# Initialize hand
kin = RightHandKinematics()
hand = RightDexHand()
joint_mapping = JointMapping(prefix="r")

# Drive the hand joints to a known position and read feedback
commands = {
    "th_rot": 30,
    "th_mcp": 30,
    "th_dip": 30,
    "ff_spr": 20,
    "ff_mcp": 30,
    "ff_dip": 30,
    "mf_mcp": 30,
    "mf_dip": 30,
    "rf_mcp": 30,
    "rf_dip": 30,
    "lf_mcp": 30,
    "lf_dip": 30,
}
for i in range(10):
    hand.move_joints(**commands)
    hand.clear_errors(clear_all=True)
feedback = hand.get_feedback()
hardware_feedback_angles = {name: joint.angle for (name, joint) in feedback.joints.items()}
joint_angles = joint_mapping.map_feedback(hardware_feedback_angles)

# Define base pose of the hand
R = pin.utils.rpyToMatrix(0, np.pi / 2, 0)
t = np.array([1.0, 0.0, 0.0])
base_pose = Pose(position=Position.from_array(t), orientation=R)

# Run forward kinematics
fingertip_poses_hand = kin.forward_kinematics(joint_angles, frame="hand", end_effector="fingertip")
fingerpad_poses_hand = kin.forward_kinematics(joint_angles, frame="hand", end_effector="fingerpad")
fingertip_poses_world = kin.forward_kinematics(joint_angles, base_pose=base_pose, frame="world", end_effector="fingertip")
fingerpad_poses_world = kin.forward_kinematics(joint_angles, base_pose=base_pose, frame="world", end_effector="fingerpad")

# Print results tables
print("Fingertip poses (hand frame):")
print("Finger | Position (m) | Quaternion")
for finger, pose in fingertip_poses_hand.items():
    print(f"{finger:6} | {pose.position.to_array()} | {pose.orientation.to_quaternion()}")
print("\nFingerpad poses (hand frame):")
print("Finger | Position (m) | Quaternion")
for finger, pose in fingerpad_poses_hand.items():
    print(f"{finger:6} | {pose.position.to_array()} | {pose.orientation.to_quaternion()}")
print("\nFingertip poses (world frame):")
print("Finger | Position (m) | Quaternion")
for finger, pose in fingertip_poses_world.items():
    print(f"{finger:6} | {pose.position.to_array()} | {pose.orientation.to_quaternion()}")
print("\nFingerpad poses (world frame):")
print("Finger | Position (m) | Quaternion")
for finger, pose in fingerpad_poses_world.items():
    print(f"{finger:6} | {pose.position.to_array()} | {pose.orientation.to_quaternion()}")
