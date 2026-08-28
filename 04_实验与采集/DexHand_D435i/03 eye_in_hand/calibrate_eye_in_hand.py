"""眼在手上 (eye-in-hand) 手眼标定。

输入:
    --images   标定板照片目录(默认 ../01 D435i/img_cam_calibration)
    --pose-csv ROKAE 法兰位姿 CSV(默认 ../02 ROKAE/flange_pose_samples.csv),
                单位 m + rad,无表头,每行 6 列
                flange_x_m, flange_y_m, flange_z_m, flange_rx_rad, flange_ry_rad, flange_rz_rad
    --intrinsics-yaml 相机内参(可选,YAML;不传则使用内置默认值)

标定板类型:
    --board {chessboard, aruco}   (默认 chessboard)
棋盘格附加参数:
    --board-cols          内角点列数(默认 10)
    --board-rows          内角点行数(默认 7)
    --square-size-mm      单元格边长 mm(默认 15.0)
    --board-z-axis {+z,-z}
        标定板世界坐标 Z 轴指向相机一侧(+)还是远离相机一侧(-)。
        这决定了 object_points 的 Z 分量正负,会改变手眼标定结果的旋转方向。
ArUco 附加参数:
    --aruco-dict          ArUco 字典(默认 DICT_6X6_250)
    --marker-length-mm    ArUco 单个标记边长 mm(默认 30.0)
    --marker-spacing-mm   ArUco 板内相邻标记中心距 mm(默认 10.0;即边长的 1/3 左右)
    --board-z-axis        同上

输出:
    cam2gripper (4x4) 齐次变换矩阵,即相机到末端的变换。
    eye-in-hand 时,装在末端上的相机对基座的位姿 = base2gripper @ gripper2camera(eye) = base2gripper @ inv(cam2gripper)。

旋转约定:位姿的 (rx, ry, rz) 按 Rz @ Ry @ Rx(外旋,X-Y-Z 固定轴)解释,与 ROKAE xMate 一致;
求解手眼标定时,需要把轴角 (rvec) 转成旋转矩阵。

注意:
    - 照片文件名必须按采样顺序 1.jpg, 2.jpg, 3.jpg ... 命名,与位姿 CSV 行一一对应。
    - 至少准备 8 组以上有效数据(OpenCV 建议 >= 10 组且姿态变化充分)。
"""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np


HERE = Path(__file__).resolve().parent


# --------------------------------------------------------------------------- #
# 位姿 / 矩阵工具
# --------------------------------------------------------------------------- #


def rpy_to_R(rx: float, ry: float, rz: float) -> np.ndarray:
    """Rz @ Ry @ Rx 固定轴外旋,与 ROKAE xMate SDK 默认约定一致。"""
    cx, sx = np.cos(rx), np.sin(rx)
    cy, sy = np.cos(ry), np.sin(ry)
    cz, sz = np.cos(rz), np.sin(rz)
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
    return Rz @ Ry @ Rx


def pose_to_T(x: float, y: float, z: float, rx: float, ry: float, rz: float) -> np.ndarray:
    R = rpy_to_R(rx, ry, rz)
    t = np.array([x, y, z], dtype=np.float64).reshape(3, 1)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t[:, 0]
    return T


def inv_T(T: np.ndarray) -> np.ndarray:
    Rt = T[:3, :3].T
    t = -Rt @ T[:3, 3]
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = Rt
    out[:3, 3] = t
    return out


# --------------------------------------------------------------------------- #
# 读取位姿
# --------------------------------------------------------------------------- #


def load_flange_poses(csv_path: Path) -> list[np.ndarray]:
    """读取 flange_pose_samples.csv,返回 list of 4x4 T_base2flange(单位 m + rad)。"""
    poses: list[np.ndarray] = []
    with csv_path.open("r", newline="", encoding="utf-8-sig") as f:
        for lineno, raw in enumerate(f, start=1):
            line = raw.strip()
            if not line:
                continue
            parts = [p for p in line.split(",") if p.strip() != ""]
            if len(parts) != 6:
                raise ValueError(
                    f"{csv_path}:{lineno} 期望 6 列,实际 {len(parts)}: {line!r}"
                )
            values = [float(p) for p in parts]
            x, y, z, rx, ry, rz = values
            T = pose_to_T(x, y, z, rx, ry, rz)
            poses.append(T)
    if not poses:
        raise ValueError(f"{csv_path} 中没有位姿数据")
    return poses


# --------------------------------------------------------------------------- #
# 棋盘格检测
# --------------------------------------------------------------------------- #


def build_chessboard_object_points(
    cols: int, rows: int, square_mm: float, z_sign: int
) -> np.ndarray:
    """生成棋盘格角点在标定板世界系下的坐标。

    X 沿列,j=0 在 +X 方向最大;Y 沿行,i=0 在 +Y 方向最大;Z = 0。
    这与旧 camare_calibration_e_t_h.py 的 object_points 保持一致。
    """
    if cols < 2 or rows < 2:
        raise ValueError("cols / rows 必须 >= 2")
    if z_sign not in (+1, -1):
        raise ValueError("z_sign 必须是 +1 或 -1")
    obj = np.zeros((rows * cols, 3), dtype=np.float64)
    flag = 0
    for i in range(rows):
        for j in range(cols):
            obj[flag, 0] = (cols - 1 - j) * square_mm
            obj[flag, 1] = (rows - 1 - i) * square_mm
            obj[flag, 2] = 0.0
            flag += 1
    if z_sign < 0:
        obj[:, 2] = -obj[:, 2]
    return obj


def detect_chessboard(img: np.ndarray, cols: int, rows: int) -> np.ndarray | None:
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    # findChessboardCornersSB 在内角点场景下更稳;若不可用则回退到 findChessboardCorners
    try:
        found, corners = cv2.findChessboardCornersSB(gray, (cols, rows), None)
    except cv2.error:
        found, corners = cv2.findChessboardCorners(gray, (cols, rows), None)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
    corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return corners2.reshape(-1, 2)


def solve_chessboard_pose(
    img: np.ndarray,
    cols: int,
    rows: int,
    square_mm: float,
    z_sign: int,
    K: np.ndarray,
    dist: np.ndarray,
) -> np.ndarray | None:
    corners = detect_chessboard(img, cols, rows)
    if corners is None:
        return None
    obj = build_chessboard_object_points(cols, rows, square_mm, z_sign)
    ok, rvec, tvec = cv2.solvePnP(obj, corners, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    return compose_rt(rvec, tvec)


# --------------------------------------------------------------------------- #
# ArUco 检测
# --------------------------------------------------------------------------- #


ARUCO_DICTS = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


def build_aruco_object_points(
    marker_length_mm: float, spacing_mm: float, z_sign: int
) -> np.ndarray:
    """单标记 ArUco 板的 object points。

    约定:使用 Board.create_single_marker 或 detectMarkers 后,
    每个标记四个角的 world 坐标按 (0,0),(L,0),(L,L),(0,L) 给出(L 为 marker 边长,单位 mm),
    全部 Z=0。spacing 在多标记板里表示相邻标记中心距(留白)。
    这里仅生成单标记版本:4 个角,Z 在 +-z_sign * 0(默认 +z_sign)。
    """
    if marker_length_mm <= 0:
        raise ValueError("marker_length_mm 必须 > 0")
    L = marker_length_mm
    obj = np.array(
        [
            [0.0, 0.0, 0.0],
            [L, 0.0, 0.0],
            [L, L, 0.0],
            [0.0, L, 0.0],
        ],
        dtype=np.float64,
    )
    if z_sign < 0:
        obj[:, 2] = -obj[:, 2]
    return obj


def detect_aruco(
    img: np.ndarray, dict_name: str, K: np.ndarray, dist: np.ndarray, marker_len_mm: float
) -> tuple[np.ndarray, np.ndarray] | tuple[None, None]:
    if not hasattr(cv2.aruco, "ArucoDetector"):
        # 旧版 OpenCV
        aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICTS[dict_name])
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(img, aruco_dict, parameters=params)
    else:
        aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[dict_name])
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        corners, ids, _ = detector.detectMarkers(img)

    if ids is None or len(ids) == 0:
        return None, None

    obj = build_aruco_object_points(marker_len_mm, 0.0, +1)
    # 简单取第一个标记的 4 个角做 solvePnP;多标记板可改为 estimatePoseSingleMarkers
    img_pts = corners[0].reshape(4, 2)
    ok, rvec, tvec = cv2.solvePnP(obj, img_pts, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None, None
    return compose_rt(rvec, tvec), None


def compose_rt(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = tvec[:, 0]
    return T


# --------------------------------------------------------------------------- #
# 内参
# --------------------------------------------------------------------------- #


DEFAULT_K = np.array(
    [
        [330.89989872, 0.0, 320.08538659],
        [0.0, 327.52309666, 257.4981118],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
DEFAULT_DIST = np.zeros((5, 1), dtype=np.float64)


def load_intrinsics(yaml_path: Path | None) -> tuple[np.ndarray, np.ndarray]:
    if yaml_path is None:
        return DEFAULT_K.copy(), DEFAULT_DIST.copy()
    fs = cv2.FileStorage(str(yaml_path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError(f"无法打开内参文件: {yaml_path}")
    K = fs.getNode("camera_matrix").mat()
    D = fs.getNode("distortion_coefficients").mat()
    fs.release()
    if K is None or D is None:
        raise RuntimeError(f"{yaml_path} 中未找到 camera_matrix / distortion_coefficients")
    return K.astype(np.float64), D.astype(np.float64)


# --------------------------------------------------------------------------- #
# 主流程
# --------------------------------------------------------------------------- #


@dataclass
class Args:
    images: Path
    pose_csv: Path
    intrinsics_yaml: Path | None
    board: str
    board_cols: int
    board_rows: int
    square_mm: float
    aruco_dict: str
    marker_length_mm: float
    marker_spacing_mm: float
    z_sign: int


def parse_args() -> Args:
    parser = argparse.ArgumentParser(
        description="眼在手上 (eye-in-hand) 手眼标定",
    )
    parser.add_argument(
        "--images",
        type=Path,
        default=Path(r"C:\Users\sxy18\Desktop\记录留痕\DexHand_D435i\01 D435i\img_cam_calibration"),
        help="标定板照片目录(1.jpg, 2.jpg, ...)",
    )
    parser.add_argument(
        "--pose-csv",
        type=Path,
        default=Path(r"C:\Users\sxy18\Desktop\记录留痕\DexHand_D435i\02 ROKAE\flange_pose_samples.csv"),
        help="ROKAE flange 位姿 CSV (m + rad)",
    )
    parser.add_argument(
        "--intrinsics-yaml",
        type=Path,
        default=None,
        help="可选的相机内参 YAML(包含 camera_matrix 与 distortion_coefficients)",
    )
    parser.add_argument(
        "--board",
        choices=["chessboard", "aruco"],
        default="chessboard",
        help="标定板类型",
    )
    parser.add_argument("--board-cols", type=int, default=10)
    parser.add_argument("--board-rows", type=int, default=7)
    parser.add_argument("--square-size-mm", type=float, default=15.0)
    parser.add_argument(
        "--board-z-axis",
        choices=["+z", "-z"],
        default="+z",
        help="标定板世界系 Z 轴: +z 指向相机一侧(默认);-z 反向",
    )
    parser.add_argument(
        "--aruco-dict",
        default="DICT_6X6_250",
        choices=list(ARUCO_DICTS.keys()),
    )
    parser.add_argument("--marker-length-mm", type=float, default=30.0)
    parser.add_argument("--marker-spacing-mm", type=float, default=10.0)
    a = parser.parse_args()
    z_sign = +1 if a.board_z_axis == "+z" else -1
    return Args(
        images=a.images,
        pose_csv=a.pose_csv,
        intrinsics_yaml=a.intrinsics_yaml,
        board=a.board,
        board_cols=a.board_cols,
        board_rows=a.board_rows,
        square_mm=a.square_size_mm,
        aruco_dict=a.aruco_dict,
        marker_length_mm=a.marker_length_mm,
        marker_spacing_mm=a.marker_spacing_mm,
        z_sign=z_sign,
    )


def list_images(directory: Path) -> list[Path]:
    if not directory.exists():
        raise FileNotFoundError(f"照片目录不存在: {directory}")
    files: list[Path] = []
    for ext in ("jpg", "jpeg", "png", "bmp"):
        files.extend(sorted(directory.glob(f"*.{ext}")))
        files.extend(sorted(directory.glob(f"*.{ext.upper()}")))
    if not files:
        raise FileNotFoundError(f"{directory} 中没有图像文件")
    return files


def main() -> int:
    args = parse_args()
    K, D = load_intrinsics(args.intrinsics_yaml)
    print("Camera matrix K:")
    print(K)
    print("Distortion:")
    print(D.ravel())

    T_base2flange_list = load_flange_poses(args.pose_csv)
    print(f"Loaded {len(T_base2flange_list)} flange poses from {args.pose_csv}")

    image_paths = list_images(args.images)
    print(f"Found {len(image_paths)} images in {args.images}")

    R_gripper2base: list[np.ndarray] = []
    t_gripper2base: list[np.ndarray] = []
    R_target2cam: list[np.ndarray] = []
    t_target2cam: list[np.ndarray] = []

    skipped: list[tuple[int, str]] = []

    n = min(len(image_paths), len(T_base2flange_list))
    for idx in range(n):
        img = cv2.imread(str(image_paths[idx]))
        if img is None:
            skipped.append((idx, "读取失败"))
            continue

        if args.board == "chessboard":
            T_board2cam = solve_chessboard_pose(
                img,
                args.board_cols,
                args.board_rows,
                args.square_mm,
                args.z_sign,
                K,
                D,
            )
            if T_board2cam is None:
                skipped.append((idx, "棋盘格角点未找到"))
                continue
        else:
            T_board2cam, _ = detect_aruco(img, args.aruco_dict, K, D, args.marker_length_mm)
            if T_board2cam is None:
                skipped.append((idx, "ArUco 未检测到"))
                continue

        T_base2flange = T_base2flange_list[idx]
        T_flange2base = inv_T(T_base2flange)

        R_gripper2base.append(T_flange2base[:3, :3])
        t_gripper2base.append(T_flange2base[:3, 3].reshape(3, 1))
        R_target2cam.append(T_board2cam[:3, :3])
        t_target2cam.append(T_board2cam[:3, 3].reshape(3, 1))

    if skipped:
        print("以下样本被跳过(位姿将按通过顺序使用):")
        for i, reason in skipped:
            print(f"  - image[{i}] {image_paths[i].name}: {reason}")

    if len(R_gripper2base) < 3:
        print(f"有效样本数 {len(R_gripper2base)} < 3,无法标定。")
        return 1

    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base,
        t_gripper2base,
        R_target2cam,
        t_target2cam,
    )
    T_cam2gripper = np.eye(4, dtype=np.float64)
    T_cam2gripper[:3, :3] = R_cam2gripper
    T_cam2gripper[:3, 3] = t_cam2gripper[:, 0]

    print("\n=== 眼在手上结果:cam -> gripper ===")
    print(T_cam2gripper)
    print("\n若需要相机在基坐标系下的位姿:base_T_cam = base_T_gripper @ gripper_T_cam")
    print("      其中 gripper_T_cam = inv(cam_T_gripper)")
    print("      base_T_gripper 即某次采样时 flange 在 base 下的位姿(见 CSV)。")
    return 0


if __name__ == "__main__":
    sys.exit(main())