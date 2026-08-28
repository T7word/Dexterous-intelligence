from typing import Dict, List, Optional
from enum import Enum
import numpy as np
from pyzlg_dexhand.dexhand_interface import DexHandBase


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
        hw_joint_values = {dex_joint: [] for dex_joint in DexHandBase.joint_names}

        for name, value in joint_values.items():
            if name in self.urdf_to_hw:
                dex_joint = self.urdf_to_hw[name]
                hw_joint_values[dex_joint].append(value)

        # Average values for each hardware joint
        command = {}
        for dex_joint in DexHandBase.joint_names:
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
