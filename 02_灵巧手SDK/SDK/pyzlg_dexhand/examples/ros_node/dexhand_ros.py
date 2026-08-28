#!/usr/bin/env python3
from ros_compat import ROSNode
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension
from std_srvs.srv import Trigger
from sensor_msgs.msg import JointState
import argparse
import numpy as np
from typing import Dict, List, Optional
from enum import Enum
import time
import yaml
import os.path

from pyzlg_dexhand.dexhand_interface import (
    DexHandBase,
    LeftDexHand,
    RightDexHand,
    ControlMode,
    ZCANWrapper,
)
from pyzlg_dexhand.dexhand_protocol.constants import MIN_FIRMWARE_VERSION, NUM_MOTORS, NUM_BOARDS

# Import joint names from DexHandBase to maintain compatibility
joint_names = [
    "th_dip",
    "th_mcp",      # Board 0: Thumb
    "th_rot",
    "ff_spr",      # Board 1: Thumb rotation & spread
    "ff_dip",
    "ff_mcp",      # Board 2: First finger
    "mf_dip",
    "mf_mcp",      # Board 3: Middle finger
    "rf_dip",
    "rf_mcp",      # Board 4: Ring finger
    "lf_dip",
    "lf_mcp",      # Board 5: Little finger
]
from pyzlg_dexhand.zcan_wrapper import MockZCANWrapper

from filters import DampedVelocityKalmanFilter


class HardwareMapping(Enum):
    """Mapping between URDF and hardware joints"""

    th_dip = ("th_dip", ["f_joint1_3", "f_joint1_4"])
    th_mcp = ("th_mcp", ["f_joint1_2"])
    th_rot = ("th_rot", ["f_joint1_1"])
    ff_spr = ("ff_spr", ["f_joint2_1", "f_joint4_1", "f_joint5_1"])
    ff_dip = ("ff_dip", ["f_joint2_3", "f_joint2_4"])
    ff_mcp = ("ff_mcp", ["f_joint2_2"])
    mf_dip = ("mf_dip", ["f_joint3_3", "f_joint3_4"])
    mf_mcp = ("mf_mcp", ["f_joint3_2"])
    rf_dip = ("rf_dip", ["f_joint4_3", "f_joint4_4"])
    rc_mcp = ("rf_mcp", ["f_joint4_2"])
    lf_dip = ("lf_dip", ["f_joint5_3", "f_joint5_4"])
    lf_mcp = ("lf_mcp", ["f_joint5_2"])


class JointMapping:
    """Maps between URDF joints and hardware joints"""

    def __init__(self, prefix: str = "l"):
        """Initialize joint mapping"""
        self.prefix = prefix

        # Create mapping from URDF joint names to hardware joints
        self.urdf_to_hw = {}
        for hw_mapping in HardwareMapping:
            dex_joint, urdf_joints = hw_mapping.value
            for urdf_joint in urdf_joints:
                full_name = f"{prefix}_{urdf_joint}"
                self.urdf_to_hw[full_name] = dex_joint

        # Get all possible URDF joint names
        self.joint_names = sorted(list(self.urdf_to_hw.keys()))

    def map_command(self, joint_values: Dict[str, float]) -> Dict[str, float]:
        """Map URDF joint values to hardware commands"""
        # Group joint values by hardware joint
        hw_joint_values = {dex_joint: [] for dex_joint in joint_names}

        for name, value in joint_values.items():
            if name in self.urdf_to_hw:
                dex_joint = self.urdf_to_hw[name]
                hw_joint_values[dex_joint].append(value)

        # Average values for each hardware joint
        command = {}
        for dex_joint in joint_names:
            values = hw_joint_values[dex_joint]
            if values:
                command[dex_joint] = float(np.rad2deg(sum(values) / len(values)))

        return command

    def map_feedback(self, hardware_values: Dict[str, float]) -> Dict[str, float]:
        """Map hardware joint values to URDF joint values"""
        joint_state_dict = {}
        for urdf_joint in self.joint_names:
            dex_joint = self.urdf_to_hw.get(urdf_joint)
            if dex_joint in hardware_values:
                joint_state_dict[urdf_joint] = float(np.deg2rad(hardware_values[dex_joint]))
        return joint_state_dict


class DexHandNode(ROSNode):
    """ROS2 Node for controlling one or both DexHands"""

    def __init__(self, config: dict):
        super().__init__("dexhand")

        # Get configuration values
        hands = config.get("hands", ["right"])
        control_mode = config.get("mode", "impedance_grasp")
        send_rate = config.get("rate", 100.0)
        filter_alpha = config.get("alpha", 0.1)
        self.use_broadcast = config.get("use_broadcast", False)
        
        # Topic configuration
        self.topic_config = {
            "left": {
                "command": config.get("left_hand_command_topic", "/left_hand/joint_commands"),
                "joint_feedback": config.get("left_hand_joint_feedback_topic", "/left_hand/joint_states"),
                "touch_feedback": config.get("left_touch_feedback_topic", "/left_hand/touch_sensors"),
                "motor_feedback": config.get("left_motor_feedback_topic", "/left_hand/motor_feedback")
            },
            "right": {
                "command": config.get("right_hand_command_topic", "/right_hand/joint_commands"),
                "joint_feedback": config.get("right_hand_joint_feedback_topic", "/right_hand/joint_states"),
                "touch_feedback": config.get("right_touch_feedback_topic", "/right_hand/touch_sensors"),
                "motor_feedback": config.get("right_motor_feedback_topic", "/right_hand/motor_feedback")
            }
        }

        self.is_mock = config.get("mock", False)

        # Add feedback configuration
        self.enable_feedback = config.get("enable_feedback", False)
        self.feedback_dt = 1.0 / send_rate  # Time step for Kalman filter

        # Initialize shared ZCAN
        self.zcan = ZCANWrapper() if not self.is_mock else MockZCANWrapper()
        if not self.zcan.open():
            raise RuntimeError("Failed to open ZCAN device")
            
        """
        Tactile Sensor and Motor Feedback Array Layouts
        -----------------------------------------------
        
        Touch Sensor Arrays (/left_touch_sensors and /right_touch_sensors):
          Each array contains 40 values (8 values for each of the 5 fingers)
          Data format per finger (indexed by fingertip_mapping):
            [0] = timestamp (UNIX time in seconds)
            [1] = normal_force (Newtons)
            [2] = normal_force_delta (raw units)
            [3] = tangential_force (Newtons)
            [4] = tangential_force_delta (raw units)
            [5] = direction (0-359 degrees, fingertip is 0, -1 if invalid)
            [6] = proximity (raw units)
            [7] = temperature (Celsius)
          
        Motor Feedback Arrays (/left_motor_feedback and /right_motor_feedback):
          Each array contains 72 values (6 values for each of the 12 motors/joints)
          Motors are organized by joint index as defined in joint_names:
            [0] = th_dip
            [1] = th_mcp 
            [2] = th_rot
            [3] = ff_spr
            [4] = ff_dip
            [5] = ff_mcp
            [6] = mf_dip
            [7] = mf_mcp
            [8] = rf_dip
            [9] = rf_mcp
            [10] = lf_dip
            [11] = lf_mcp
            
          Data format per motor (motor_index * 7):
            [0] = timestamp (UNIX time in seconds)
            [1] = angle (degrees)
            [2] = encoder_position (raw units, 0-4095, angle = encoder/11.38)
            [3] = current (mA)
            [4] = velocity (rpm, 0 if invalid/overflow)
            [5] = error_code (0 if no error)
            [6] = impedance (float, lower values indicate higher resistance to movement)
            
        Finger Mapping (self.fingertip_mapping, for touch sensors only):
          th (thumb): index 0
          ff (index finger): index 1
          mf (middle finger): index 2
          rf (ring finger): index 3
          lf (little finger): index 4
        """

        # Set up control mode
        self.control_mode_map = {
            "zero_torque": ControlMode.ZERO_TORQUE,
            "current": ControlMode.CURRENT,
            "speed": ControlMode.SPEED,
            "hall_position": ControlMode.HALL_POSITION,
            "cascaded_pid": ControlMode.CASCADED_PID,
            "protect_hall_position": ControlMode.PROTECT_HALL_POSITION,
            "mit_torque":ControlMode.MIT_TORQUE,
            "impedance_grasp":ControlMode.IMPEDANCE_GRASP
        }
        self.control_mode = self.control_mode_map.get(
            control_mode, ControlMode.IMPEDANCE_GRASP
        )
        self.filter_alpha = filter_alpha

        # Initialize hands with shared ZCAN
        self.hands = {}
        self.joint_mappings = {}
        self.last_commands = {}

        # Initialize Kalman filters for each joint when feedback is enabled
        self.kalman_filters = {}
        self.last_joint_positions = {}
        self.fingertip_mapping = {
            "th": 0,  # Thumb
            "ff": 1,  # Index
            "mf": 2,  # Middle
            "rf": 3,  # Ring
            "lf": 4,  # Pinky
        }
        
        # Publishers for feedback
        self.joint_state_pubs = {}
        self.touch_sensor_pubs = {}
        self.motor_feedback_pubs = {}

        for hand in hands:
            # Initialize hand with shared ZCAN
            hand_class = LeftDexHand if hand == "left" else RightDexHand
            # Use auto_init=False to explicitly control initialization with proper error handling
            self.hands[hand] = hand_class(self.zcan, auto_init=False)
            if not self.hands[hand].init():
                raise RuntimeError(f"Failed to initialize {hand} hand")
                
            # Check firmware version
            versions = self.hands[hand].get_firmware_versions()
            if versions:
                # Print firmware versions for this hand
                for joint, version in versions.items():
                    if version is not None:
                        self.logger.info(f"{hand} hand joint {joint} firmware version: {version}")
                
                # Get unique versions
                unique_versions = set(v for v in versions.values() if v is not None)
                if len(unique_versions) > 1:
                    self.logger.error(f"{hand} hand has mismatched firmware versions: {unique_versions}")
                    raise RuntimeError(f"{hand} hand has mismatched firmware versions")
                elif len(unique_versions) == 0:
                    self.logger.error(f"Could not read firmware versions for {hand} hand")
                    raise RuntimeError(f"Could not read firmware versions for {hand} hand")
                else:
                    version = list(unique_versions)[0]
                    self.logger.info(f"{hand} hand firmware version: {version}")
                    
                    # Check if firmware version meets minimum requirement
                    if version < MIN_FIRMWARE_VERSION:
                        self.logger.warn(f"{hand} hand firmware version {version} is below minimum recommended version {MIN_FIRMWARE_VERSION}")

            # Initialize joint mapping
            self.joint_mappings[hand] = JointMapping("l" if hand == "left" else "r")

            # Initialize last command
            self.last_commands[hand] = {}

            # Initialize Kalman filters and state tracking for each joint
            if self.enable_feedback:
                self.kalman_filters[hand] = {}
                self.last_joint_positions[hand] = {}

                for joint_name in joint_names:
                    # Parameters for Kalman filter: dt, process_noise_var, measurement_noise_var, damping
                    self.kalman_filters[hand][joint_name] = DampedVelocityKalmanFilter(
                        dt=self.feedback_dt,
                        process_noise_var=100.0,
                        measurement_noise_var=0.1,
                        damping=0.9,
                    )
                    self.last_joint_positions[hand][joint_name] = 0.0
                    
            # Initialize command subscriber with configurable topic
            self.create_subscription(
                JointState, 
                self.topic_config[hand]["command"], 
                lambda msg, h=hand: self.command_callback(msg, h)
            )
            
            # Initialize publishers for feedback
            if self.enable_feedback:
                self.joint_state_pubs[hand] = self.create_publisher(
                    JointState, self.topic_config[hand]["joint_feedback"], 10
                )
                self.touch_sensor_pubs[hand] = self.create_publisher(
                    Float64MultiArray, self.topic_config[hand]["touch_feedback"], 10
                )
                self.motor_feedback_pubs[hand] = self.create_publisher(
                    Float64MultiArray, self.topic_config[hand]["motor_feedback"], 10
                )

        # Initialize reset service
        self.create_service(Trigger, "/reset_hands", self.reset_callback)
        
        reset_client = self.create_client(Trigger, "/reset_hands")
        if reset_client.wait_for_service(timeout_sec=1.0):
            request = Trigger.Request()
            reset_client.call_async(request)
        else:
            self.logger.warn("Reset service not available")

        # Set up command sending timer
        period = 1.0 / send_rate
        self.timer = self.create_timer(period, self.send_commands)

        self.logger.info(
            f"DexHand node initialized:\n"
            f'  Hands: {", ".join(hands)}\n'
            f"  Control mode: {control_mode}\n"
            f"  Send rate: {send_rate} Hz\n"
            f"  Filter alpha: {filter_alpha}\n"
            f"  Feedback enabled: {self.enable_feedback}\n"
            f"  Broadcast mode: {self.use_broadcast}\n"
        )

    def command_callback(self, msg: JointState, hand: str):
        """Handle incoming joint commands for the specified hand"""
        try:
            # Create dictionary of joint values
            joint_values = {}
            for name, pos in zip(msg.name, msg.position):
                if pos != pos:  
                    # NaN detected, setting to 0
                    joint_values[name] = 0.0
                else:
                    joint_values[name] = pos
            
            # Get joint mapping for this hand
            mapping = self.joint_mappings[hand]
            
            # Map to hardware joints
            command = mapping.map_command(joint_values)

            # Apply low-pass filter
            if not self.last_commands[hand]:
                # First command, no filtering
                self.last_commands[hand] = command
            else:
                for joint, value in command.items():
                    if joint not in self.last_commands[hand]:
                        self.last_commands[hand][joint] = value
                    else:
                        self.last_commands[hand][joint] = (
                            1 - self.filter_alpha
                        ) * self.last_commands[hand][
                            joint
                        ] + self.filter_alpha * value
        except Exception as e:
            self.logger.error(f"Error in command callback: {str(e)}")

    def send_commands(self):
        """Send filtered commands to all hands"""
        try:
            # Send commands to each hand
            for hand, hand_interface in self.hands.items():
                command = self.last_commands.get(hand, {})

                # Use move_joints interface with broadcast option if enabled
                # If using broadcast mode, we need to separately clear errors
                # If not using broadcast mode, we can combine with move_joints
                # This avoids the problematic combination of clear_error=True with use_broadcast=True
                hand_interface.move_joints(
                    th_dip=command.get("th_dip"),
                    th_mcp=command.get("th_mcp"),
                    th_rot=command.get("th_rot"),
                    ff_spr=command.get("ff_spr"),
                    ff_dip=command.get("ff_dip"),
                    ff_mcp=command.get("ff_mcp"),
                    mf_dip=command.get("mf_dip"),
                    mf_mcp=command.get("mf_mcp"),
                    rf_dip=command.get("rf_dip"),
                    rf_mcp=command.get("rf_mcp"),
                    lf_dip=command.get("lf_dip"),
                    lf_mcp=command.get("lf_mcp"),
                    control_mode=self.control_mode,
                    use_broadcast=self.use_broadcast,
                    clear_error=not self.use_broadcast,  # Only clear errors with non-broadcast mode
                )
                
                # If using broadcast mode, clear errors separately
                if self.use_broadcast:
                    hand_interface.clear_errors(use_broadcast=False)

                # Note: We've updated the approach to avoid the problematic combination of
                # clear_error=True with use_broadcast=True

                # Get and publish feedback if enabled
                if self.enable_feedback:
                    self.process_and_publish_feedback(hand)

        except Exception as e:
            self.logger.error(f"Error sending commands: {str(e)}")

    def process_and_publish_feedback(self, hand: str):
        """Get feedback from hardware and publish to ROS topics"""
        try:
            # Get feedback from hand
            feedback = self.hands[hand].get_feedback()

            # Process joint positions and estimate velocities
            pos_dict = {}
            vel_dict = {}
            for joint_name, joint_feedback in feedback.joints.items():
                # Get position
                position = joint_feedback.angle
                # Update the Kalman filter with new measurement
                kalman_filter = self.kalman_filters[hand][joint_name]
                if not np.isnan(position):
                    kalman_filter.step(position)
                state = kalman_filter.get_current_state()

                # Extract position (state[0]) and velocity (state[1])
                velocity = state[1][0]  # Extract velocity in deg/s

                # Save position and velocity to dictionaries
                pos_dict[joint_name] = position
                vel_dict[joint_name] = velocity

            # Convert hardware feedback to URDF joint names
            pos_dict_urdf = self.joint_mappings[hand].map_feedback(pos_dict)
            vel_dict_urdf = self.joint_mappings[hand].map_feedback(vel_dict)

            # Construct and publish joint state message
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_ros_time().to_msg()
            joint_state_msg.name = self.joint_mappings[hand].joint_names
            joint_state_msg.position = [pos_dict_urdf.get(joint, 0.0) for joint in joint_state_msg.name]
            joint_state_msg.velocity = [vel_dict_urdf.get(joint, 0.0) for joint in joint_state_msg.name]
            
            self.joint_state_pubs[hand].publish(joint_state_msg)

            # Process touch sensor feedback
            if feedback.touch:
                # Create and document touch sensor message (8 values per finger)
                # Format: [timestamp, normal_force, normal_force_delta, tangential_force, 
                #          tangential_force_delta, direction, proximity, temperature]
                touch_msg = Float64MultiArray()
                touch_msg.layout.dim = [
                    MultiArrayDimension(label="fingers", size=5, stride=8),
                    MultiArrayDimension(label="data", size=8, stride=1)
                ]
                touch_msg.data = [0.0] * 40  # 5 fingers * 8 values per finger
                
                # Create and document motor feedback message (7 values per motor, 12 motors)
                # Format: [timestamp, angle, encoder_value, current, velocity, error_code, impedance]
                motor_msg = Float64MultiArray()
                motor_msg.layout.dim = [
                    MultiArrayDimension(label="motors", size=12, stride=7),
                    MultiArrayDimension(label="data", size=7, stride=1)
                ]
                motor_msg.data = [0.0] * 84  # 12 motors * 7 values per motor

                # Map feedback to separate touch and motor arrays
                for finger_name, touch_data in feedback.touch.items():
                    if finger_name in self.fingertip_mapping:
                        idx = self.fingertip_mapping[finger_name]
                        
                        # Calculate unified timestamp in seconds (UNIX time)
                        timestamp = touch_data.timestamp / 1e9  # Convert nanoseconds to seconds
                        
                        # Set touch sensor data (8 values per finger)
                        touch_msg.data[8*idx] = timestamp
                        touch_msg.data[8*idx+1] = touch_data.normal_force
                        touch_msg.data[8*idx+2] = touch_data.normal_force_delta
                        touch_msg.data[8*idx+3] = touch_data.tangential_force
                        touch_msg.data[8*idx+4] = touch_data.tangential_force_delta
                        
                        # Validate direction value - should be 0-359 degrees
                        direction = touch_data.direction
                        if direction == 65535 or direction > 359:
                            direction = -1  # Use -1 to indicate invalid direction
                        touch_msg.data[8*idx+5] = direction
                        
                        touch_msg.data[8*idx+6] = touch_data.proximity
                        touch_msg.data[8*idx+7] = touch_data.temperature
                        
                # Process all motors using the enhanced JointFeedback data
                current_time = time.time()  # Use current time for consistency
                
                # Populate data for all 12 motors from joint feedback
                for joint_idx, joint_name in enumerate(joint_names):
                    # Get joint feedback data if available
                    if joint_name in feedback.joints:
                        joint_data = feedback.joints[joint_name]
                        motor_msg.data[7*joint_idx] = current_time
                        motor_msg.data[7*joint_idx+1] = joint_data.angle
                        motor_msg.data[7*joint_idx+2] = joint_data.encoder_position or 0
                        motor_msg.data[7*joint_idx+3] = joint_data.current or 0
                        
                        # Handle velocity values (fix negative values from signed interpretation)
                        velocity = joint_data.velocity
                        if velocity is not None and velocity < -10000:  # Likely a misinterpreted large value
                            velocity = 0  # Reset to zero for invalid velocity values
                        motor_msg.data[7*joint_idx+4] = velocity or 0
                        
                        motor_msg.data[7*joint_idx+5] = joint_data.error_code or 0
                        motor_msg.data[7*joint_idx+6] = joint_data.impedance or 0.0

                # Publish both messages
                self.touch_sensor_pubs[hand].publish(touch_msg)
                self.motor_feedback_pubs[hand].publish(motor_msg)

        except Exception as e:
            self.logger.error(f"Error processing feedback: {str(e)}")

    def reset_callback(self, request, response):
        """Reset all hands with bend-straighten sequence"""
        try:
            success = True

            # First bend all joints to 30 degrees
            for hand in self.hands.values():
                hand.move_joints(
                    th_dip=30,
                    th_mcp=30,
                    th_rot=30,
                    ff_spr=30,
                    ff_dip=30,
                    ff_mcp=30,
                    mf_dip=30,
                    mf_mcp=30,
                    rf_dip=30,
                    rf_mcp=30,
                    lf_dip=30,
                    lf_mcp=30,
                    use_broadcast=self.use_broadcast,
                    clear_error=not self.use_broadcast  # Only clear errors with non-broadcast mode
                )
                
                # If using broadcast mode, clear errors separately
                if self.use_broadcast:
                    hand.clear_errors(use_broadcast=False)

            # Wait a moment
            time.sleep(0.5)

            # Then straighten to 0 degrees
            for _ in range(2):
                for hand in self.hands.values():
                    # Reset joints to 0 degrees with clear_error flag
                    hand.reset_joints(use_broadcast=self.use_broadcast)
                    time.sleep(0.2)
                    
                    # Clear errors separately with broadcast=False to ensure it works properly
                    hand.clear_errors(use_broadcast=False)
                        
                    time.sleep(0.2)

            # Clear command history
            for hand in self.hands:
                self.last_commands[hand] = {}

            # Set response
            response.success = success
            response.message = (
                "Hand reset sequence completed successfully"
                if success
                else "Hand reset sequence failed"
            )
            return response

        except Exception as e:
            response.success = False
            response.message = f"Error in reset sequence: {str(e)}"
            return response

    def on_shutdown(self):
        """Clean up on node shutdown"""
        for hand in self.hands.values():
            hand.close()


def main():
    config_path = os.path.join(os.path.dirname(__file__), "../../config", "config.yaml")

    # Load configuration
    try:
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
    except Exception as e:
        print(f"Error loading config file: {e}")
        return

    node = DexHandNode(config=config["DexHand"]["ROS_Node"])
    try:
        node.spin()
    finally:
        node.shutdown()


if __name__ == "__main__":
    main()