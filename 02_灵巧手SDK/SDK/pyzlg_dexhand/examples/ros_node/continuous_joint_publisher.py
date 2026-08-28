#!/usr/bin/env python3
import os
import yaml
import numpy as np
import time
from typing import Dict, List

from ros_compat import ROSNode, ROSTime
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger


class DexHandContinuousPublisher(ROSNode):
    """
    Publishes commands to all DexHand joints continuously at a specified rate.
    Supports various motion patterns and can be configured via command line arguments.
    """

    def __init__(self, config, pattern='sine', amplitude=30.0, period=5.0, offset=0.0):
        """
        Initialize the continuous joint publisher.

        Args:
            config: Configuration dictionary from the YAML file
            pattern: Motion pattern ('sine', 'triangle', 'square', 'hold')
            amplitude: Motion amplitude in degrees
            period: Motion cycle period in seconds
            offset: Phase offset between fingers in radians
        """
        super().__init__("dexhand_continuous_publisher")

        # Store configuration
        self.hands = config.get("hands", ["right"])
        self.rate = config.get("rate", 100.0)
        self.pattern = pattern
        self.amplitude = np.deg2rad(amplitude)  # Convert to radians for ROS
        self.period = period
        self.offset = offset

        # Get joint names for each hand
        self.joint_names = []
        for hand in self.hands:
            prefix = "l" if hand == "left" else "r"
            # All finger joints
            for finger in range(1, 6):  # Fingers 1-5
                for joint in range(1, 5):  # Joints 1-4 per finger
                    joint_name = f"{prefix}_f_joint{finger}_{joint}"
                    self.joint_names.append(joint_name)

        # Calculate update period based on rate
        self.update_period = 1.0 / self.rate

        # Create publisher
        # Create publishers for each hand
        self.publishers = {}
        for hand in self.hands:
            self.publishers[hand] = self.create_publisher(
                JointState,
                f"/{hand}_hand/joint_commands",
                10
            )

        # Initialize timer for continuous publishing
        self.timer = self.create_timer(self.update_period, self.publish_commands)

        # Track time since start
        self.start_time = time.time()

        self.logger.info(
            f"DexHand Continuous Publisher initialized:\n"
            f"  Hands: {', '.join(self.hands)}\n"
            f"  Pattern: {self.pattern}\n"
            f"  Amplitude: {np.rad2deg(self.amplitude)} degrees\n"
            f"  Period: {self.period} seconds\n"
            f"  Update rate: {self.rate} Hz\n"
            f"  Total joints: {len(self.joint_names)}"
        )

    def publish_commands(self):
        """
        Generate and publish commands for all joints based on the selected pattern.
        """
        # Calculate elapsed time
        elapsed = time.time() - self.start_time

        # Create joint state message
        msg = JointState()
        msg.header.stamp = self.get_ros_time().to_msg()

        # Add position for each joint
        for i, joint_name in enumerate(self.joint_names):
            msg.name.append(joint_name)

            # Extract finger and joint indices from name (e.g., "r_f_joint3_2")
            parts = joint_name.split("_")
            if len(parts) >= 4:
                finger_index = int(parts[2][-1]) - 1  # 0-4 for fingers 1-5
                joint_index = int(parts[3]) - 1   # 0-3 for joints 1-4
            else:
                # Default if naming doesn't match pattern
                finger_index = i % 5
                joint_index = i // 5

            # Phase offset based on finger and joint
            phase = self.offset * finger_index

            # Calculate joint position based on pattern
            if self.pattern == 'sine':
                # Sine wave pattern
                value = self.amplitude * np.sin(2 * np.pi * elapsed / self.period + phase)

            elif self.pattern == 'triangle':
                # Triangle wave pattern
                t = (elapsed / self.period + phase / (2 * np.pi)) % 1.0
                if t < 0.5:
                    value = self.amplitude * (4 * t - 1)
                else:
                    value = self.amplitude * (3 - 4 * t)

            elif self.pattern == 'square':
                # Square wave pattern
                t = (elapsed / self.period + phase / (2 * np.pi)) % 1.0
                value = self.amplitude if t < 0.5 else -self.amplitude

            elif self.pattern == 'hold':
                # Hold at maximum position
                value = self.amplitude

            else:
                # Default to sine wave
                value = self.amplitude * np.sin(2 * np.pi * elapsed / self.period + phase)

            # Adjust amplitude based on joint type
            # - Thumb rotation (joint1_1) and finger spread (joint2-5_1) use full amplitude
            # - Metacarpophalangeal joints (joint*_2) use full amplitude
            # - Proximal interphalangeal joints (joint*_3) use 70% amplitude
            # - Distal interphalangeal joints (joint*_4) use 50% amplitude
            scale = 1.0
            if joint_index == 2:  # Joint 3
                scale = 0.7
            elif joint_index == 3:  # Joint 4
                scale = 0.5

            # Add to message
            msg.position.append(value * scale)

        # Get left and right hand joint names
        left_joints = [name for name in msg.name if name.startswith('l_')]
        right_joints = [name for name in msg.name if name.startswith('r_')]
        
        # Publish to each hand's topic with the appropriate joints
        if 'left' in self.hands and left_joints:
            left_msg = JointState()
            left_msg.header = msg.header
            
            # Filter positions for left hand joints
            indices = [i for i, name in enumerate(msg.name) if name.startswith('l_')]
            left_msg.name = [msg.name[i] for i in indices]
            left_msg.position = [msg.position[i] for i in indices]
            
            self.publishers['left'].publish(left_msg)
            
        if 'right' in self.hands and right_joints:
            right_msg = JointState()
            right_msg.header = msg.header
            
            # Filter positions for right hand joints
            indices = [i for i, name in enumerate(msg.name) if name.startswith('r_')]
            right_msg.name = [msg.name[i] for i in indices]
            right_msg.position = [msg.position[i] for i in indices]
            
            self.publishers['right'].publish(right_msg)

    def call_reset_service(self):
        """
        Call the reset_hands service to reset the hand to a known state.
        """
        reset_client = self.create_client(Trigger, "/reset_hands")
        if reset_client.wait_for_service(timeout_sec=1.0):
            request = Trigger.Request()
            result = reset_client.call(request)
            self.logger.info(f"Reset hand result: {result.success}, {result.message}")
            return result.success
        else:
            self.logger.warn("Reset service not available")
            return False


def main():
    import argparse

    parser = argparse.ArgumentParser(description="Continuous joint publisher for DexHand")
    parser.add_argument("--pattern", type=str, default="sine",
                        choices=["sine", "triangle", "square", "hold"],
                        help="Motion pattern")
    parser.add_argument("--amplitude", type=float, default=30.0,
                        help="Motion amplitude in degrees")
    parser.add_argument("--period", type=float, default=5.0,
                        help="Motion cycle period in seconds")
    parser.add_argument("--offset", type=float, default=0.2,
                        help="Phase offset between fingers (radians)")
    parser.add_argument("--config", type=str, default=None,
                        help="Path to config.yaml (default: use package default)")
    args = parser.parse_args()

    # Load configuration
    if args.config:
        config_path = args.config
    else:
        config_path = os.path.join(os.path.dirname(__file__), "../../config", "config.yaml")

    try:
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading config file: {e}")
        return

    # Create and run node
    node = DexHandContinuousPublisher(
        config=config["DexHand"]["ROS_Node"],
        pattern=args.pattern,
        amplitude=args.amplitude,
        period=args.period,
        offset=args.offset
    )

    # Reset hands before starting
    node.call_reset_service()

    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Reset hands on exit
        node.call_reset_service()
        node.shutdown()


if __name__ == "__main__":
    main()
