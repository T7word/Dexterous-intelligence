[English](README.md) | [中文](README_zh.md)

# DexRobot 运动学

一个用于机器人手的正向和逆向运动学的Python库，专为DexHand设计。提供独立手部操作和集成臂手系统的实用工具。

## 功能特点

- **正向运动学**：从关节角度计算手指位置和方向
- **逆向运动学**：求解关节角度以达到所需的指尖/指垫位置
- **多指抓取规划**：同时为多个手指定义抓取目标
- **臂手集成**：集成机器人手臂和手部系统的组合运动学
- **硬件接口**：与DexHand硬件和ROS直接集成
- **支持左手和右手**：两种手部配置下一致的API

## 安装

### 前提条件

- Python 3.8或更新版本
- NumPy
- Pinocchio（用于机器人运动学）
- PyYAML（用于配置文件）

### 基本安装

```bash
# 克隆仓库
git clone https://github.com/dexrobot/dexrobot_kinematics.git
cd dexrobot_kinematics
pip install -e .

# 克隆URDF模型仓库（运动学计算所需）
# 在与dexrobot_kinematics相同的父目录下
git clone https://github.com/dexrobot/dexrobot_urdf.git
```

## 快速开始

### 初始化手部

```python
from dexrobot_kinematics.hand import RightHandKinematics

# 使用默认配置初始化右手
hand = RightHandKinematics()
```

### 正向运动学

从关节角度计算指尖/指垫位姿：

```python
# 定义关节角度
joint_angles = {name: 0.0 for name in hand.robot.model.names[1:]}

# 获取手部参考系中的指垫位姿
poses = hand.forward_kinematics(joint_angles, end_effector="fingerpad")

# 访问单个手指的位姿
thumb_pose = poses["thumb"]
print(f"拇指位置：{thumb_pose.position.x}, {thumb_pose.position.y}, {thumb_pose.position.z}")
```

### 单指逆向运动学

```python
from dexrobot_kinematics.utils.types import Position

# 定义食指的目标位置
target_pos = Position(x=0.07, y=0.04, z=0.17)

# 为食指求解逆运动学
joint_angles, success = hand.inverse_kinematics_finger(
    finger="index",
    target_pos=target_pos
)

if success:
    print("成功求解逆运动学！")
    print(f"关节角度：{joint_angles}")
```

### 多指抓取

```python
# 定义拇指和食指的目标位置（用于精细抓取）
finger_targets = {
    "thumb": Position(x=0.07, y=0.04, z=0.15),
    "index": Position(x=0.07, y=0.04, z=0.17)
}

# 求解抓取的逆运动学
joint_angles, success = hand.inverse_kinematics_grasp(finger_targets)
```

## 关键概念

### 坐标系统

该库使用两种主要参考坐标系：

1. **手部坐标系**：以手部基座为中心的局部坐标系
   - X轴指向手掌
   - Y轴指向拇指远离方向（左手）或拇指方向（右手）
   - Z轴指向指尖

2. **世界坐标系**：可以通过base_pose定义的全局坐标系

```python
import numpy as np
import pinocchio as pin
from dexrobot_kinematics.utils.types import Position, Pose

# 定义带有旋转和平移的基准位姿
R = pin.utils.rpyToMatrix(0, np.pi/2, 0)  # 绕Y轴旋转90°
t = np.array([1.0, 0.0, 0.0])            # X方向1米
base_pose = Pose(position=Position.from_array(t), orientation=R)

# 在正向运动学中使用，获取世界坐标系中的位姿
poses = hand.forward_kinematics(joint_angles, base_pose=base_pose, frame="world")
```

### 末端执行器类型

支持两种类型的末端执行器：

- **指垫**：每个手指的接触表面，用于抓取
- **指尖**：每个手指的最末端

```python
# 获取指垫位姿
fingerpad_poses = hand.forward_kinematics(joint_angles, end_effector="fingerpad")

# 获取指尖位姿
fingertip_poses = hand.forward_kinematics(joint_angles, end_effector="fingertip")
```

## 硬件集成

### 控制DexHand硬件

```python
from dexrobot_kinematics.hand import RightHandKinematics
from dexrobot_kinematics.utils.hardware import JointMapping
from pyzlg_dexhand.dexhand_interface import RightDexHand

# 初始化手部对象
kin = RightHandKinematics()
hand = RightDexHand()
joint_mapping = JointMapping(prefix="r")

# 为手指位置求解逆运动学
joint_angles, success = kin.inverse_kinematics_finger(
    finger="index",
    target_pos=Position(x=0.07, y=0.04, z=0.17)
)

# 将URDF关节角度映射到硬件命令
commands = joint_mapping.map_command(joint_angles)

# 发送命令到硬件
hand.move_joints(**commands)
```

### ROS集成

```python
# 请参见examples/finger_ik_ros.py获取完整的ROS节点示例
```

## 文档

完整的API文档，请参阅[在线文档](https://dexrobot.github.io/dexrobot_kinematics/)。

## 示例

`examples/`目录包含展示各种用例的示例脚本：

- `finger_ik_hardware.py`：使用手指逆运动学控制硬件
- `fk_hardware.py`：从硬件关节角度计算正向运动学
- `finger_ik_ros.py`：与ROS接口对接

## 许可证

本项目采用Apache License 2.0许可 - 详情请参见LICENSE文件。
