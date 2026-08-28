"""
demo_rokae_threading.py
=======================

Offline-friendly version of the SDK's ``threading_example.py``.
Spawns two background threads that poll ``getPointPos`` and call
``pause/moveStart`` while the main thread executes the motion queue.
"""

from __future__ import annotations

import os
import sys
import threading
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
    from convert_tools import MoveLCommand  # noqa: E402
    import rokae                          # noqa: E402
    BACKEND = "REAL"
else:
    from rokae_mock import XMateRobot, MoveLCommand, rokae  # noqa: E402
    BACKEND = "MOCK"

print(f"[demo] SDK backend = {BACKEND}")


runState = False
ip = "127.0.0.1"
robot = XMateRobot(ip)


def pointPose() -> None:
    global runState
    while True:
        if not runState:
            time.sleep(0.1)
            continue
        ec = {}
        pp = robot.getPointPos(ec)
        # mimic the original logic: slow down near waypoints
        if pp == -1:
            robot.adjustSpeedOnline(1 / 11.5, ec);  print("speed~1.0")
        elif pp == 0:
            robot.adjustSpeedOnline(0.5 / 11.5, ec); print("speed~0.5")
        elif pp == 1:
            robot.adjustSpeedOnline(1 / 11.5, ec);   print("speed~1.0")
        elif pp == 2:
            robot.adjustSpeedOnline(0.5 / 11.5, ec); print("speed~0.5")
        time.sleep(0.2)


def pauser() -> None:
    global runState
    while True:
        if not runState:
            time.sleep(0.1)
            continue
        ec = {}
        time.sleep(2.0)
        robot.pause(ec); print("[thread] pause")
        time.sleep(2.0)
        robot.moveStart(ec); print("[thread] resume")


def main() -> None:
    global runState
    ec: dict = {}

    with robot:
        robot.connectToRobot(ec)

        threading.Thread(target=pointPose, daemon=True).start()
        threading.Thread(target=pauser, daemon=True).start()

        robot.setOperateMode(rokae.OperateMode.automatic, ec)
        robot.setPowerState(True, ec)
        robot.moveReset(ec)

        cmds = [
            MoveLCommand([0.030, 1.164, 0.279, -3.137, 0.206, -2.059], 200, 50),
            MoveLCommand([0.012, 1.175, -0.010, 2.995, 0.072,  3.089], 200, 50),
            MoveLCommand([-0.099, 1.186, -0.012, 2.977, 0.041, 3.046], 200, 50),
            MoveLCommand([-0.080, 1.134, 0.188, 3.110, 0.020, -3.099], 200, 50),
        ]
        robot.executeCommand(cmds, ec)
        robot.moveStart(ec)
        runState = True

        # let the threads do their thing for a few seconds
        time.sleep(8)
        runState = False
        time.sleep(1)


if __name__ == "__main__":
    main()