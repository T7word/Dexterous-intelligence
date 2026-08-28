"""
Solve IK for a single finger and send the result to ROS/ROS2.

Can be used together with the ROS nodes in dexrobot_mujoco or pyzlg_dexhand to control either the hardware or a simulation.
"""

import numpy as np
import time
from ros_compat import ROSNode
from sensor_msgs.msg import JointState
from dexrobot_kinematics.hand import RightHandKinematics
from dexrobot_kinematics.utils.types import Position
from dexrobot_kinematics.utils.hardware import JointMapping


class DexHandIKNode(ROSNode):
    def __init__(self):
        super().__init__("dexhand_ik_node")
        self.kin = RightHandKinematics()
        self.joint_pub = self.create_publisher(JointState, "joint_commands", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.counter = 0

    def timer_callback(self):
        # Define a circular trajectory
        center = Position(x=0.01, y=0.04, z=0.22)
        radius = 0.05
        steps = 20

        # Calculate position along circle
        i = self.counter
        self.counter += 1
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
        joint_angles, success = self.kin.inverse_kinematics_finger(
            finger="index", target_pos=target
        )

        print(f"Step {i}: Pos ({target.x:.4f}, {target.y:.4f}, {target.z:.4f}), Angles {list(joint_angles.values())}, Success: {success}")

        # Publish joint angles
        msg = JointState()
        msg.header.stamp = self.get_ros_time().to_msg()
        msg.name = list(joint_angles.keys())
        msg.position = list(joint_angles.values())
        self.joint_pub.publish(msg)

if __name__ == "__main__":
    node = DexHandIKNode()
    node.spin()
