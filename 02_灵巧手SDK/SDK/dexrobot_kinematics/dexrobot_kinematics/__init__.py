# dexrobot_kinematics/__init__.py

__version__ = "0.1.0"

# Import utilities (no pin dependency)
from dexrobot_kinematics.utils import Position, Pose

# Try to import hand kinematics (requires pin/pinocchio)
try:
    from dexrobot_kinematics.hand import RightHandKinematics, LeftHandKinematics
except ImportError as e:
    import warnings
    warnings.warn(
        f"Hand kinematics modules could not be imported. "
        f"This is likely because the 'pin' (pinocchio) package is not installed. "
        f"Error: {e}"
    )
    RightHandKinematics = None
    LeftHandKinematics = None

__all__ = [
    '__version__',
    'Position',
    'Pose',
    'RightHandKinematics',
    'LeftHandKinematics',
]
