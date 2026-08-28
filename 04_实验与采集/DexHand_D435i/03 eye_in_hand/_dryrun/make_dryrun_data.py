"""生成 dry-run 数据:
  - 12 张合成棋盘格图像(不同视角)
  - 12 行 flange 位姿 CSV(单位 m + rad)
模拟眼在手上:相机随末端运动,标定板固定。
"""
from __future__ import annotations

import csv
from pathlib import Path

import numpy as np
import cv2


HERE = Path(__file__).resolve().parent
IMG_DIR = HERE / "imgs"
IMG_DIR.mkdir(parents=True, exist_ok=True)
CSV_PATH = HERE / "flange_pose_samples.csv"


K = np.array(
    [
        [330.89989872, 0.0, 320.08538659],
        [0.0, 327.52309666, 257.4981118],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)

COLS, ROWS, SQUARE = 10, 7, 15.0  # 内角点 + mm
BOARD_W_MM = (COLS + 1) * SQUARE
BOARD_H_MM = (ROWS + 1) * SQUARE

# 标定板世界系下角点(单位 mm)
obj = np.zeros((ROWS * COLS, 3), dtype=np.float32)
flag = 0
for i in range(ROWS):
    for j in range(COLS):
        obj[flag, 0] = (COLS - 1 - j) * SQUARE
        obj[flag, 1] = (ROWS - 1 - i) * SQUARE
        obj[flag, 2] = 0.0
        flag += 1


def rpy_to_R(rx, ry, rz):
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def pose_to_T(x, y, z, rx, ry, rz):
    R = rpy_to_R(rx, ry, rz)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def inv_T(T):
    Rt = T[:3, :3].T
    t = -Rt @ T[:3, 3]
    out = np.eye(4)
    out[:3, :3] = Rt
    out[:3, 3] = t
    return out


rng = np.random.default_rng(42)
poses = []
rows_out = []

for i in range(1, 13):
    # 让 base->flange 在几个不同位姿
    x = rng.uniform(0.4, 0.7)
    y = rng.uniform(-0.2, 0.2)
    z = rng.uniform(0.3, 0.6)
    rx = rng.uniform(-3.1, -2.6)
    ry = rng.uniform(-0.3, 0.3)
    rz = rng.uniform(1.4, 1.7)
    T_base2flange = pose_to_T(x, y, z, rx, ry, rz)
    poses.append(T_base2flange)
    rows_out.append([x, y, z, rx, ry, rz])

    # 假设 cam->gripper 是已知真值(模拟 ground truth)
    # 这里随便给一个固定 cam->gripper 真值用于自检
    T_cam2gripper_gt = pose_to_T(0.02, 0.0, 0.05, 0.0, 0.0, 0.0)
    T_flange2cam = inv_T(T_cam2gripper_gt)
    T_board2base = pose_to_T(0.0, 0.0, 0.0, np.pi, 0.0, 0.0)  # 标定板挂在 base 前方,面朝 base
    T_board2cam = inv_T(T_flange2cam) @ inv_T(T_base2flange) @ T_board2base
    # 简化为只关心 tvec,任意 rvec 也能找到角点
    rvec, _ = cv2.Rodrigues(T_board2cam[:3, :3])
    tvec = T_board2cam[:3, 3].reshape(3, 1) * 1000.0  # mm

    imgpts, _ = cv2.projectPoints(obj, rvec, tvec, K, None)
    img = np.full((480, 848, 3), 60, np.uint8)
    # 直接画圆点代替棋盘格,因为 solvePnP 用像素坐标即可
    for p in imgpts.reshape(-1, 2):
        cv2.circle(img, (int(p[0]), int(p[1])), 3, (0, 255, 0), -1)
    cv2.imwrite(str(IMG_DIR / f"{i}.jpg"), img)

# 写 CSV
with CSV_PATH.open("w", newline="", encoding="utf-8-sig") as f:
    w = csv.writer(f)
    for r in rows_out:
        w.writerow(r)

print("dry-run data ready:")
print(f"  images: {IMG_DIR}")
print(f"  csv:    {CSV_PATH}")