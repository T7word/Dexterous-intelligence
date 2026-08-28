"""实时采集 ArUco 板相对 D435i 相机的位姿。

与 ../02 ROKAE/eye_in_hand_pose_capture.py 同步使用:
    - 这边按 c / 空格键,采集一次当前 ArUco 在相机系下的位姿(同时把彩色帧保存到图片目录)
    - 那边按 c / 空格键,采集一次当前法兰位姿
    两边的 CSV 行号 / 图片编号一一对应。

用法示例:
    python aruco_pose_capture.py ^
        --images ..\01 D435i\img_cam_calibration ^
        --pose-csv aruco_in_camera_pose_samples.csv ^
        --marker-length-mm 30 ^
        --aruco-dict DICT_6X6_250

输出 CSV 与 calibrate_eye_in_hand.py 的 --board aruco 模式完全兼容:
    aruco_x_m, aruco_y_m, aruco_z_m, aruco_rx_rad, aruco_ry_rad, aruco_rz_rad
(平移 m,旋转 rad,由 Rodrigues 反解得到)
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs


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


DEFAULT_K = np.array(
    [
        [330.89989872, 0.0, 320.08538659],
        [0.0, 327.52309666, 257.4981118],
        [0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)
DEFAULT_DIST = np.zeros((5, 1), dtype=np.float64)


def build_aruco_object_points(marker_length_mm: float) -> np.ndarray:
    L = float(marker_length_mm)
    return np.array(
        [
            [0.0, 0.0, 0.0],
            [L, 0.0, 0.0],
            [L, L, 0.0],
            [0.0, L, 0.0],
        ],
        dtype=np.float64,
    )


def rvec_to_euler_xyz(rvec: np.ndarray) -> tuple[float, float, float]:
    """Rodrigues -> (rx, ry, rz),按 Rz @ Ry @ Rx 解释。

    calibrate_eye_in_hand.py 的 pose_to_T 使用相同约定,这里给出"欧拉角"仅方便人眼查看。
    """
    R, _ = cv2.Rodrigues(rvec)
    # R = Rz @ Ry @ Rx
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        rx = math.atan2(R[2, 1], R[2, 2])
        ry = math.atan2(-R[2, 0], sy)
        rz = math.atan2(R[1, 0], R[0, 0])
    else:
        rx = math.atan2(-R[1, 2], R[1, 1])
        ry = math.atan2(-R[2, 0], sy)
        rz = 0.0
    return rx, ry, rz


def detect_and_estimate_pose(
    img: np.ndarray,
    aruco_dict_name: str,
    marker_len_mm: float,
    K: np.ndarray,
    D: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, list[np.ndarray], np.ndarray | None] | None:
    if not hasattr(cv2.aruco, "ArucoDetector"):
        d = cv2.aruco.Dictionary_get(ARUCO_DICTS[aruco_dict_name])
        params = cv2.aruco.DetectorParameters_create()
        corners, ids, _ = cv2.aruco.detectMarkers(img, d, parameters=params)
    else:
        d = cv2.aruco.getPredefinedDictionary(ARUCO_DICTS[aruco_dict_name])
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(d, params)
        corners, ids, _ = detector.detectMarkers(img)
    if ids is None or len(ids) == 0:
        return None

    obj = build_aruco_object_points(marker_len_mm)
    img_pts = corners[0].reshape(4, 2)
    ok, rvec, tvec = cv2.solvePnP(obj, img_pts, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None
    return rvec, tvec, corners, ids


def open_pose_csv(path: Path) -> tuple[object, csv.writer, int]:
    path.parent.mkdir(parents=True, exist_ok=True)
    exists = path.exists() and path.stat().st_size > 0
    f = path.open("a", newline="", encoding="utf-8-sig")
    writer = csv.writer(f)
    if not exists:
        writer.writerow(
            [
                "aruco_x_m",
                "aruco_y_m",
                "aruco_z_m",
                "aruco_rx_rad",
                "aruco_ry_rad",
                "aruco_rz_rad",
            ]
        )
        f.flush()
        return f, writer, 1
    # 估计下一个编号
    with path.open("r", newline="", encoding="utf-8-sig") as old:
        rows = list(csv.reader(old))
    return f, writer, max(1, len(rows))  # header + N 行 => 下一行号 = N+1,但 len(rows) 已含表头


def main() -> int:
    parser = argparse.ArgumentParser(
        description="实时采集 ArUco 板相对 D435i 相机的位姿",
    )
    parser.add_argument(
        "--images",
        type=Path,
        default=Path(r"C:\Users\sxy18\Desktop\记录留痕\DexHand_D435i\01 D435i\img_cam_calibration"),
        help="保存彩色帧的目录(同时被 calibrate_eye_in_hand.py 读取)",
    )
    parser.add_argument(
        "--pose-csv",
        type=Path,
        default=Path("aruco_in_camera_pose_samples.csv"),
        help="ArUco 在相机系下的位姿 CSV",
    )
    parser.add_argument(
        "--intrinsics-yaml",
        type=Path,
        default=None,
        help="可选的相机内参 YAML",
    )
    parser.add_argument("--width", type=int, default=848)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--marker-length-mm", type=float, default=30.0)
    parser.add_argument(
        "--aruco-dict",
        default="DICT_6X6_250",
        choices=list(ARUCO_DICTS.keys()),
    )
    args = parser.parse_args()

    if args.intrinsics_yaml is not None and args.intrinsics_yaml.exists():
        fs = cv2.FileStorage(str(args.intrinsics_yaml), cv2.FILE_STORAGE_READ)
        K = fs.getNode("camera_matrix").mat()
        D = fs.getNode("distortion_coefficients").mat()
        fs.release()
        K = np.asarray(K, dtype=np.float64)
        D = np.asarray(D, dtype=np.float64)
    else:
        K = DEFAULT_K.copy()
        D = DEFAULT_DIST.copy()
    print("K =\n", K)
    print("D =", D.ravel())

    args.images.mkdir(parents=True, exist_ok=True)

    pose_file, writer, sample_id = open_pose_csv(args.pose_csv)
    print(f"pose csv: {args.pose_csv.resolve()}   start at sample_id = {sample_id}")
    print("按 c / 空格:采集一次 ArUco 在相机系下的位姿并保存一帧彩色图")
    print("按 q       :退出")

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)
    pipeline.start(config)

    try:
        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            img = np.asanyarray(color_frame.get_data())

            rvec, tvec, corners, ids = (None, None, None, None)
            result = detect_and_estimate_pose(
                img, args.aruco_dict, args.marker_length_mm, K, D
            )
            if result is not None:
                rvec, tvec, corners, ids = result

            vis = img.copy()
            if corners is not None:
                cv2.aruco.drawDetectedMarkers(vis, corners, ids)
                # 投影坐标轴
                try:
                    cv2.drawFrameAxes(vis, K, D, rvec, tvec, args.marker_length_mm * 1.5)
                except cv2.error:
                    pass

            cv2.putText(
                vis,
                f"sample_id={sample_id}  press c to capture, q to quit",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )
            cv2.imshow("ArUco capture", vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key not in (ord("c"), ord(" ")):
                continue

            if rvec is None:
                print("[warn] 当前帧未检测到 ArUco,跳过。")
                continue

            tvec_m = tvec.reshape(3) / 1000.0  # mm -> m
            rx, ry, rz = rvec_to_euler_xyz(rvec)

            writer.writerow(
                [tvec_m[0], tvec_m[1], tvec_m[2], rx, ry, rz]
            )
            pose_file.flush()

            img_path = args.images / f"{sample_id}.jpg"
            cv2.imwrite(str(img_path), img)

            print(
                f"\n[{sample_id}] captured\n"
                f"  aruco->cam [m]:   x={tvec_m[0]:.4f} y={tvec_m[1]:.4f} z={tvec_m[2]:.4f}\n"
                f"  aruco->cam [deg]: rx={math.degrees(rx):.3f} ry={math.degrees(ry):.3f} rz={math.degrees(rz):.3f}\n"
                f"  saved image: {img_path}"
            )
            sample_id += 1

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C,退出。")
    finally:
        pipeline.stop()
        pose_file.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())