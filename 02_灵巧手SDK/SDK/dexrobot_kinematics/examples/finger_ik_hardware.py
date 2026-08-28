"""
Solve IK for a single finger and directly control the hardware.

The code works for the right hand by default, but you can change the hand as needed.
"""

import numpy as np
import time
from dexrobot_kinematics.hand import RightHandKinematics
from dexrobot_kinematics.utils.types import Position
from dexrobot_kinematics.utils.hardware import JointMapping
from pyzlg_dexhand.dexhand_interface import RightDexHand

# Initialize hand
kin = RightHandKinematics()
hand = RightDexHand()
joint_mapping = JointMapping(prefix="r")

# Define a circular trajectory
center = Position(x=0.01, y=0.04, z=0.22)
radius = 0.05
steps = 20

# Simulate motion
for i in range(2 * steps):
    # Calculate position along circle
    if i < steps:
        angle = np.pi * i / steps
    else:
        angle = np.pi * (2 * steps - i) / steps
    target = Position(
        x=center.x + radius * np.sin(angle),
        y=center.y,
        z=center.z + radius * np.cos(angle),
    )

    # Solve IK
    joint_angles, success = kin.inverse_kinematics_finger(
        finger="index", target_pos=target
    )

    print(f"Step {i}: Pos ({target.x:.4f}, {target.y:.4f}, {target.z:.4f}), Angles {list(joint_angles.values())}, Success: {success}")
    # In a real system, you would send these angles to the robot
    commands = joint_mapping.map_command(joint_angles)
    hand.move_joints(**commands)
    hand.clear_errors(clear_all=True)
    time.sleep(0.1)
