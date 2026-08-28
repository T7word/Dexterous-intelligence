[English](README.md) | [中文](README_zh.md)

# DexRobot Kinematics

A Python library for forward and inverse kinematics of robotic hands, particularly designed for DexHand. Provides utilities for both isolated hand operations and integrated arm-hand systems.

## Features

- **Forward Kinematics**: Calculate finger positions and orientations from joint angles
- **Inverse Kinematics**: Solve for joint angles to achieve desired fingertip/fingerpad positions
- **Multi-Finger Grasp Planning**: Define grasp targets for multiple fingers simultaneously
- **Arm-Hand Integration**: Combined kinematics for integrated robotic arm and hand systems
- **Hardware Interface**: Direct integration with DexHand hardware and ROS
- **Support for both left and right hands**: Consistent API across both hand configurations

## Installation

### Prerequisites

- Python 3.8 or newer
- NumPy
- Pinocchio (for robot kinematics)
- PyYAML (for configuration files)

### Basic Installation

```bash
# Clone the repository
git clone https://github.com/dexrobot/dexrobot_kinematics.git
cd dexrobot_kinematics
pip install -e .

# Clone the URDF models repository (required for kinematics)
# In the same parent directory as dexrobot_kinematics
git clone https://github.com/dexrobot/dexrobot_urdf.git
```

## Quick Start

### Initialize a Hand

```python
from dexrobot_kinematics.hand import RightHandKinematics

# Initialize a right hand with default configuration
hand = RightHandKinematics()
```

### Forward Kinematics

Compute fingertip/fingerpad poses from joint angles:

```python
# Define joint angles
joint_angles = {name: 0.0 for name in hand.robot.model.names[1:]}

# Get fingerpad poses in hand frame
poses = hand.forward_kinematics(joint_angles, end_effector="fingerpad")

# Access individual finger poses
thumb_pose = poses["thumb"]
print(f"Thumb position: {thumb_pose.position.x}, {thumb_pose.position.y}, {thumb_pose.position.z}")
```

### Inverse Kinematics for a Single Finger

```python
from dexrobot_kinematics.utils.types import Position

# Define target position for index finger
target_pos = Position(x=0.07, y=0.04, z=0.17)

# Solve IK for index finger
joint_angles, success = hand.inverse_kinematics_finger(
    finger="index",
    target_pos=target_pos
)

if success:
    print("Successfully solved IK!")
    print(f"Joint angles: {joint_angles}")
```

### Multi-Finger Grasping

```python
# Define target positions for thumb and index (for a pinch grasp)
finger_targets = {
    "thumb": Position(x=0.07, y=0.04, z=0.15),
    "index": Position(x=0.07, y=0.04, z=0.17)
}

# Solve IK for grasp
joint_angles, success = hand.inverse_kinematics_grasp(finger_targets)
```

## Key Concepts

### Coordinate Systems

The library uses two primary reference frames:

1. **Hand Frame**: Local coordinate system centered at the hand base
   - X-axis pointing toward palm
   - Y-axis pointing away from thumb (left hand) or toward thumb (right hand)
   - Z-axis pointing toward fingertips

2. **World Frame**: Global coordinate system that can be defined via a base_pose

```python
import numpy as np
import pinocchio as pin
from dexrobot_kinematics.utils.types import Position, Pose

# Define a base pose with rotation and translation
R = pin.utils.rpyToMatrix(0, np.pi/2, 0)  # 90° rotation around Y
t = np.array([1.0, 0.0, 0.0])            # 1m in X direction
base_pose = Pose(position=Position.from_array(t), orientation=R)

# Use in forward kinematics to get poses in world frame
poses = hand.forward_kinematics(joint_angles, base_pose=base_pose, frame="world")
```

### End Effector Types

Two types of end effectors are supported:

- **Fingerpads**: Contact surfaces on each finger, used for grasping
- **Fingertips**: The very ends of each finger

```python
# Get fingerpad poses
fingerpad_poses = hand.forward_kinematics(joint_angles, end_effector="fingerpad")

# Get fingertip poses
fingertip_poses = hand.forward_kinematics(joint_angles, end_effector="fingertip")
```

## Hardware Integration

### Controlling DexHand Hardware

```python
from dexrobot_kinematics.hand import RightHandKinematics
from dexrobot_kinematics.utils.hardware import JointMapping
from pyzlg_dexhand.dexhand_interface import RightDexHand

# Initialize hand objects
kin = RightHandKinematics()
hand = RightDexHand()
joint_mapping = JointMapping(prefix="r")

# Solve IK for a finger position
joint_angles, success = kin.inverse_kinematics_finger(
    finger="index",
    target_pos=Position(x=0.07, y=0.04, z=0.17)
)

# Map URDF joint angles to hardware commands
commands = joint_mapping.map_command(joint_angles)

# Send commands to hardware
hand.move_joints(**commands)
```

### ROS Integration

```python
# See examples/finger_ik_ros.py for a complete ROS node example
```

## Documentation

For full API documentation, refer to the [documentation](https://dexrobot.github.io/dexrobot_kinematics/).

## Examples

The `examples/` directory contains sample scripts demonstrating various use cases:

- `finger_ik_hardware.py`: Controlling hardware using finger IK
- `fk_hardware.py`: Computing FK from hardware joint angles
- `finger_ik_ros.py`: Interfacing with ROS

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.
