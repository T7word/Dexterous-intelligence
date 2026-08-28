"""
demo_rokae_drag.py
==================

Offline-friendly version of the SDK's ``drag_example.py`` (drag/teach
pendant flow).  Same API surface as the real SDK.
"""

from __future__ import annotations

import os
import sys
import time

USE_REAL = os.environ.get("ROKAE_SDK_MODE", "mock").lower() == "real"

if USE_REAL:
    SDK_DIR = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..",
        "precompiled_v0.1.6",
        "rokae_SDK_win_v0.1.6_py38",
    )
    sys.path.insert(0, os.path.join(SDK_DIR, "lib"))
    sys.path.insert(0, SDK_DIR)

    from robot import XMateRobot          # noqa: E402
    from convert_tools import message     # noqa: E402
    from env import robot_config          # noqa: E402
    import rokae                         # noqa: E402
    BACKEND = "REAL"
else:
    from rokae_mock import XMateRobot, message, rokae  # noqa: E402

    class _Cfg:
        remoteIP = "127.0.0.1"
    robot_config = _Cfg()
    BACKEND = "MOCK"

print(f"[demo] SDK backend = {BACKEND}")


def waitRobot(robot) -> None:
    while True:
        time.sleep(0.1)
        ec = {}
        st = robot.operationState(ec)
        if st in (rokae.OperationState.idle.value,
                  rokae.OperationState.unknown.value):
            return


def run_drag_enable_disable(robot, ec: dict) -> None:
    robot.enableDrag(
        rokae.DragParameter.Space.cartesianSpace.value,
        rokae.DragParameter.Type.freely.value,
        ec,
    )
    print("drag enabled, dragging?", getattr(robot._state, "dragging", "n/a"))
    robot.disableDrag(ec)


def run_record_path(robot, ec: dict, name: str = "demo_path") -> None:
    robot.startRecordPath(10, ec)        # 10 ms period
    time.sleep(0.2)
    robot.saveRecordPath(name, ec)
    print(message(ec))
    print("available paths:", robot.queryPathLists(ec))


def run_replay_path(robot, ec: dict, name: str = "demo_path") -> None:
    robot.disableDrag(ec)
    robot.setOperateMode(rokae.OperateMode.automatic, ec)
    robot.setPowerState(True, ec)
    robot.replayPath(name, 1, ec)
    waitRobot(robot)


def main() -> None:
    ip = robot_config.remoteIP
    ec: dict = {}

    with XMateRobot(ip) as robot:
        robot.connectToRobot(ec)
        robot.setPowerState(False, ec)
        robot.setOperateMode(rokae.OperateMode.manual, ec)
        robot.moveReset(ec)

        # --- automated run instead of keyboard input ---
        print("\n[step 1] enable drag")
        run_drag_enable_disable(robot, ec)

        print("\n[step 2] record a path")
        run_record_path(robot, ec, "demo_path_1")

        print("\n[step 3] replay the path")
        run_replay_path(robot, ec, "demo_path_1")

        waitRobot(robot)
        robot.stop(ec)
        time.sleep(1)
        robot.setPowerState(False, ec)
        robot.disconnectFromRobot(ec)


if __name__ == "__main__":
    main()