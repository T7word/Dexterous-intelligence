"""按键采集 xMate ER7 Pro 当前机器人位姿。

用于眼在手上手眼标定：
  c 或空格键：读取一次当前关节和末端位姿并追加到 CSV
  q 或 Esc 键：退出并断开机器人

位姿单位：
  - joint_pos: rad，同时保存 deg 便于查看
  - flange_pose/end_pose: [x, y, z, rx, ry, rz]
    平移单位 m，旋转单位 rad，同时保存 deg 便于查看
"""

from __future__ import annotations

import argparse
import csv
import math
import msvcrt
import os
import sys
from datetime import datetime
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


def read_robot_sample(robot) -> dict[str, object]:
    """读取一次当前机器人状态和位姿。"""
    ec: dict = {}

    joint_rad = list(robot.jointPos(ec)[:7])
    check_ec(ec, "jointPos")

    ec = {}
    flange_pose = list(
        robot.posture(xCoreSDK_python.CoordinateType.flangeInBase, ec)
    )
    check_ec(ec, "posture(flangeInBase)")

    ec = {}
    end_pose = list(robot.posture(xCoreSDK_python.CoordinateType.endInRef, ec))
    check_ec(ec, "posture(endInRef)")

    ec = {}
    operation_state = robot.operationState(ec)
    check_ec(ec, "operationState")

    state_name = getattr(operation_state, "name", str(operation_state))
    now = datetime.now().astimezone()

    return {
        "timestamp": now.isoformat(timespec="milliseconds"),
        "joint_rad": joint_rad,
        "joint_deg": [math.degrees(value) for value in joint_rad],
        "flange_pose": flange_pose,
        "flange_pose_deg": flange_pose[:3]
        + [math.degrees(value) for value in flange_pose[3:]],
        "end_pose": end_pose,
        "end_pose_deg": end_pose[:3]
        + [math.degrees(value) for value in end_pose[3:]],
        "operation_state": state_name,
    }


def csv_header() -> list[str]:
    fields = ["timestamp", "sample_id"]
    fields += [f"joint_{i}_rad" for i in range(1, 8)]
    fields += [f"joint_{i}_deg" for i in range(1, 8)]
    fields += [f"flange_{axis}_m" for axis in ("x", "y", "z")]
    fields += [f"flange_r{axis}_rad" for axis in ("x", "y", "z")]
    fields += [f"flange_r{axis}_deg" for axis in ("x", "y", "z")]
    fields += [f"end_{axis}_m" for axis in ("x", "y", "z")]
    fields += [f"end_r{axis}_rad" for axis in ("x", "y", "z")]
    fields += [f"end_r{axis}_deg" for axis in ("x", "y", "z")]
    fields += ["operation_state"]
    return fields


def sample_row(sample_id: int, sample: dict[str, object]) -> list[object]:
    joint_rad = sample["joint_rad"]
    joint_deg = sample["joint_deg"]
    flange_pose = sample["flange_pose"]
    flange_pose_deg = sample["flange_pose_deg"]
    end_pose = sample["end_pose"]
    end_pose_deg = sample["end_pose_deg"]

    return (
        [sample["timestamp"], sample_id]
        + list(joint_rad)
        + list(joint_deg)
        + list(flange_pose[:3])
        + list(flange_pose[3:])
        + list(flange_pose_deg[3:])
        + list(end_pose[:3])
        + list(end_pose[3:])
        + list(end_pose_deg[3:])
        + [sample["operation_state"]]
    )


def open_csv(path: Path) -> tuple[object, csv.writer, int]:
    """打开 CSV；已有文件继续追加，并从已有数据估计下一个编号。"""
    exists = path.exists() and path.stat().st_size > 0
    path.parent.mkdir(parents=True, exist_ok=True)
    file = path.open("a", newline="", encoding="utf-8-sig")
    writer = csv.writer(file)

    if not exists:
        writer.writerow(csv_header())
        file.flush()
        return file, writer, 1

    with path.open("r", newline="", encoding="utf-8-sig") as old_file:
        rows = list(csv.DictReader(old_file))
    last_id = 0
    for row in rows:
        try:
            last_id = max(last_id, int(row.get("sample_id", 0)))
        except (TypeError, ValueError):
            pass
    return file, writer, last_id + 1


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
        default=str(HERE / "eye_in_hand_pose_samples.csv"),
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
                sample = read_robot_sample(robot)
                writer.writerow(sample_row(sample_id, sample))
                csv_file.flush()

                print(f"\n[{sample_id}] captured at {sample['timestamp']}")
                print("  joint(deg):", [round(v, 4) for v in sample["joint_deg"]])
                print(
                    "  flangeInBase [m,deg]:",
                    [round(v, 6) for v in sample["flange_pose_deg"]],
                )
                print(
                    "  endInRef [m,deg]:",
                    [round(v, 6) for v in sample["end_pose_deg"]],
                )
                print("  state:", sample["operation_state"])
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
