"""按键采集 xMate ER7 Pro 法兰相对基座的当前位姿。

用于眼在手上手眼标定：
  c 或空格键：读取一次 flangeInBase 位姿并追加到 CSV
  q 或 Esc 键：退出并断开机器人

CSV 只保存以下六列，平移单位为 m，旋转单位为 rad：
  flange_x_m, flange_y_m, flange_z_m,
  flange_rx_rad, flange_ry_rad, flange_rz_rad
"""

from __future__ import annotations

import argparse
import csv
import msvcrt
import sys
from pathlib import Path


# 使用工程中随 SDK 提供的 setup_path.py，避免手动复制 .pyd/DLL。
HERE = Path(__file__).resolve().parent
SDK_EXAMPLE = HERE / "ROKAE_xMate_SDK" / "xCoreSDK-Python" / "example"
sys.path.insert(0, str(SDK_EXAMPLE))

import setup_path  # noqa: F401,E402

from Release.windows import xCoreSDK_python  # noqa: E402


def check_ec(ec: dict, operation: str) -> None:
    """检查 xCore SDK 的错误码字典。"""
    code = ec.get("ec", 0)
    if code != 0:
        message = ec.get("message", "<no message>")
        raise RuntimeError(f"{operation} failed: code={code}, message={message}")


def read_robot_sample(robot) -> list[float]:
    """读取一次法兰相对基座的 [x, y, z, rx, ry, rz] 位姿。"""
    ec: dict = {}
    flange_pose = list(
        robot.posture(xCoreSDK_python.CoordinateType.flangeInBase, ec)
    )
    check_ec(ec, "posture(flangeInBase)")
    return flange_pose


def csv_header() -> list[str]:
    return [
        "flange_x_m",
        "flange_y_m",
        "flange_z_m",
        "flange_rx_rad",
        "flange_ry_rad",
        "flange_rz_rad",
    ]


def sample_row(flange_pose: list[float]) -> list[float]:
    if len(flange_pose) != 6:
        raise RuntimeError(f"Unexpected flange pose length: {len(flange_pose)}")
    return flange_pose


def open_csv(path: Path) -> tuple[object, csv.writer, int]:
    """打开只含六列法兰位姿的 CSV，并返回下一个采样编号。"""
    exists = path.exists() and path.stat().st_size > 0
    path.parent.mkdir(parents=True, exist_ok=True)
    sample_count = 0
    if exists:
        with path.open("r", newline="", encoding="utf-8-sig") as old_file:
            reader = csv.reader(old_file)
            header = next(reader, [])
            if header != csv_header():
                raise RuntimeError(
                    f"CSV columns do not match the six-field flange format: {path}. "
                    "Please use a new output filename or retain the existing file as an archive."
                )
            sample_count = sum(1 for _ in reader)

    file = path.open("a", newline="", encoding="utf-8-sig")
    writer = csv.writer(file)
    if not exists:
        writer.writerow(csv_header())
        file.flush()
    return file, writer, sample_count + 1


def connect_robot(robot_ip: str, local_ip: str):
    robot = xCoreSDK_python.xMateErProRobot(robot_ip, local_ip)
    ec: dict = {}
    robot.connectToRobot(ec)
    check_ec(ec, "connectToRobot")
    return robot


def main() -> int:
    parser = argparse.ArgumentParser(
        description="按键采集 xMate ER7 Pro 当前位姿，用于眼在手上手眼标定"
    )
    parser.add_argument("--robot-ip", default="192.168.0.160")
    parser.add_argument(
        "--local-ip",
        default="192.168.0.11",
        help="本机连接机器人控制器的网卡 IP；不确定时可改为空字符串",
    )
    parser.add_argument(
        "--output",
        default=str(HERE / "flange_pose_samples.csv"),
        help="采样 CSV 输出路径",
    )
    args = parser.parse_args()

    robot = None
    csv_file = None
    try:
        print(f"Connecting to robot {args.robot_ip} ...")
        robot = connect_robot(args.robot_ip, args.local_ip)
        print("Connected.")

        output_path = Path(args.output).resolve()
        csv_file, writer, sample_id = open_csv(output_path)
        print(f"Output: {output_path}")
        print("按 c 或空格键采集；按 q 或 Esc 退出。")
        print("请先让机器人完全停止，再按键采集当前位姿。")

        while True:
            key = msvcrt.getwch()

            # Windows 功能键/方向键会返回两个字符，丢弃第二个字符。
            if key in ("\x00", "\xe0"):
                msvcrt.getwch()
                continue

            if key.lower() in ("q", "\x1b"):
                print("退出采集。")
                break

            if key.lower() != "c" and key != " ":
                continue

            try:
                flange_pose = read_robot_sample(robot)
                writer.writerow(sample_row(flange_pose))
                csv_file.flush()

                print(f"\n[{sample_id}] flange pose captured")
                print("  flangeInBase [m,rad]:",
                      [round(v, 9) for v in flange_pose])
                sample_id += 1
            except Exception as exc:
                print(f"\n采集失败：{exc}")

    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，退出采集。")
    except Exception as exc:
        print(f"程序失败：{exc}")
        return 1
    finally:
        if csv_file is not None:
            csv_file.close()
        if robot is not None:
            try:
                robot.disconnectFromRobot({})
            except Exception as exc:
                print(f"断开机器人时出现异常：{exc}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
