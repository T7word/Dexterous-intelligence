Examples
=======

This section demonstrates how to use the DexRobot Kinematics library with complete example scripts.

.. contents::
   :local:

Hardware control using finger inverse kinematics
---------------

In this example, we solve inverse kinematics for a finger, and send the resulting joint angles to DexHand using the `pyzlg_dexhand` library.

.. literalinclude:: ../../examples/finger_ik_hardware.py
   :language: python
   :linenos:
   :caption: examples/finger_ik_hardware.py

Interfacing inverse kinematics with ROS
---------------

In this example, we solve inverse kinematics for a finger, and publish the resulting joint angles to a ROS topic. The script can be used in combination with ROS nodes in `pyzlg_dexhand` or `dexrobot_mujoco` to control either hardware or simulated hand.

.. literalinclude:: ../../examples/finger_ik_ros.py
   :language: python
   :linenos:
   :caption: examples/finger_ik_ros.py

Forward kinematics based on hardware feedback
---------------

In this example, we read joint angles from DexHand using the `pyzlg_dexhand` library, and compute the forward kinematics to get the fingerpad and fingertip poses.

.. literalinclude:: ../../examples/fk_hardware.py
   :language: python
   :linenos:
   :caption: examples/fk_hardware.py
