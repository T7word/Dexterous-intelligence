Installation
===========

Prerequisites
------------

- Python 3.8 or newer
- NumPy
- Pinocchio (for robot kinematics)
- PyYAML (for configuration files)

Basic Installation
-----------------

You can install DexRobot Kinematics from source:

.. code-block:: bash

   git clone https://github.com/dexrobot/dexrobot_kinematics.git
   cd dexrobot_kinematics
   pip install -e .

The URDF models for DexHand must also be installed. By default, you should clone the `dexrobot_urdf` directory into the same parent directory as `dexrobot_kinematics` (can be overridden in yaml configuration files shown in `dexrobot_kinematics/config`):

.. code-block:: bash

   # In the same parent directory as dexrobot_kinematics
   git clone https://github.com/dexrobot/dexrobot_urdf.git
