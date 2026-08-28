Quickstart Guide
==============

This guide will help you get started with the DexRobot Kinematics library.

Basic Usage
----------

1. **Initialize a Hand**

   You can initialize either a right or left hand:

   .. code-block:: python

      from dexrobot_kinematics.hand import RightHandKinematics, LeftHandKinematics

      # Initialize a right hand with default URDF and configuration
      right_hand = RightHandKinematics()

      # Or initialize a left hand
      left_hand = LeftHandKinematics()

2. **Forward Kinematics**

   Compute poses of all fingerpads from joint angles:

   .. code-block:: python

      # Define joint angles (all zeros for neutral position)
      joint_angles = {name: 0.0 for name in right_hand.robot.model.names[1:]}

      # Get pose of all fingers
      poses = right_hand.forward_kinematics(joint_angles)

      # Access individual finger poses
      thumb_pose = poses["thumb"]
      print(f"Thumb position: {thumb_pose.position.x}, {thumb_pose.position.y}, {thumb_pose.position.z}")

3. **Inverse Kinematics for a Single Finger**

   Calculate joint angles to reach a specific target with one finger:

   .. code-block:: python

      from dexrobot_kinematics.utils.types import Position

      # Define target position for index finger
      target_pos = Position(x=0.07, y=0.04, z=0.17)

      # Solve IK for index finger
      joint_angles, success = right_hand.inverse_kinematics_finger(
          finger="index",
          target_pos=target_pos
      )

      if success:
          print("Successfully solved IK!")
          print(f"Joint angles: {joint_angles}")
      else:
          print("IK solution not found. Target might be unreachable.")

4. **Grasping with Multiple Fingers**

   Solve IK for multiple fingers to create a grasp:

   .. code-block:: python

      # Define target positions for thumb and index (for a pinch grasp)
      finger_targets = {
          "thumb": Position(x=0.07, y=0.04, z=0.15),
          "index": Position(x=0.07, y=0.04, z=0.17)
      }

      # Solve IK for grasp
      joint_angles, success = right_hand.inverse_kinematics_grasp(finger_targets)

      if success:
          print("Successfully solved grasp IK!")

      # Verify the solution with forward kinematics
      poses = right_hand.forward_kinematics(joint_angles)
      for finger, target in finger_targets.items():
          actual = poses[finger].position
          print(f"{finger} - Target: ({target.x}, {target.y}, {target.z}), "
                f"Actual: ({actual.x}, {actual.y}, {actual.z})")

5. **Combined Arm-Hand System**

   To be added in future versions.
