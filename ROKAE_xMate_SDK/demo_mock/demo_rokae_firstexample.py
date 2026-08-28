"""
demo_rokae_firstexample.py
==========================

Drop-in offline demo based on the SDK's official ``firstexample.py``,
adapted so it can run *without* the real ROKAE xMate SDK and *without*
a physical robot.

Usage (Windows / this venv):

    # (optional) use the real SDK when on a machine that has it
    set ROKAE_SDK_MODE=real     # set to "real" to use the cp38 binaries
    # otherwise leave unset to run against the bundled mock.

    python demo_rokae_firstexample.py

The code is intentionally written in the *exact* style of the SDK's
``firstexample.py``: same function names, same call signatures, same
``ec`` dict pattern - so that swapping the mock for the real SDK is a
one-line change (replace the ``import`` block).
"""

from __future__ import annotations

import os
import sys
import time

# ---------------------------------------------------------------------------
# SDK import: real SDK (cp38-only) vs. pure-Python mock
# ---------------------------------------------------------------------------
USE_REAL = os.environ.get("ROKAE_SDK_MODE", "mock").lower() == "real"

if USE_REAL:
    # Real SDK layout:
    #   ROKAE_xMate_SDK/precompiled_v0.1.6/rokae_SDK_win_v0.1.6_py38/lib
    SDK_DIR = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "..",
        "precompiled_v0.1.6",
        "rokae_SDK_win_v0.1.6_py38",
    )
    sys.path.insert(0, os.path.join(SDK_DIR, "lib"))
    sys.path.insert(0, SDK_DIR)

    from robot import XMateErProRobot  # noqa: E402  (real SDK)
    from convert_tools import MoveLCommand, degree2rad, message  # noqa: E402
    from env import robot_config  # noqa: E402
    import rokae  # noqa: E402  (real SDK exposes rokae module)

    SDK_BACKEND = "REAL"
else:
    # Use the bundled mock that mirrors the public API.
    from rokae_mock import (
        XMateErProRobot,
        MoveLCommand,
        degree2rad,
        message,
        rokae,
    )

    class _FakeCfg:
        remoteIP = "127.0.0.1"
        localIP = "127.0.0.1"

    robot_config = _FakeCfg()
    SDK_BACKEND = "MOCK"

print(f"[demo] SDK backend = {SDK_BACKEND}")


# ---------------------------------------------------------------------------
# Helper: wait until motion finishes
# ---------------------------------------------------------------------------

def waitRobot(robot) -> None:
    while True:
        time.sleep(0.1)
        ec = {}
        st = robot.operationState(ec)
        if st in (rokae.OperationState.idle.value,
                  rokae.OperationState.unknown.value):
            return


# ---------------------------------------------------------------------------
# Main flow (mirrors firstexample.py)
# ---------------------------------------------------------------------------

def main() -> None:
    ip = robot_config.remoteIP
    ec: dict = {}

    with XMateErProRobot(ip) as robot:
        # ---------- 1. connect / power ----------
        robot.connectToRobot(ec)
        robot.setPowerState(True, ec)
        power = robot.powerState(ec)
        print("Power state after power-on:", power)
        time.sleep(2)
        robot.setPowerState(False, ec)
        print("Power state after power-off:", robot.powerState(ec))

        # ---------- 2. query information ----------
        info = robot.robotInfo(ec)
        print("joint_num:", info["joint_num"],
              "type:", info["type"],
              "version:", info["version"])
        print("SDK version:", robot.sdkVersion(ec))
        print("Operate mode:", robot.operateMode(ec))
        print("Operation state:", robot.operationState(ec))

        # ---------- 3. pose / joints / frames ----------
        print("joint pos:", robot.jointPos(ec))
        print("joint vel:", robot.jointVel(ec))
        print("joint torque:", robot.jointTorque(ec))
        print("flange pos:", robot.flangePos(ec))
        print("base frame:", robot.baseFrame(ec))
        print("toolset:", robot.toolset(ec))

        # ---------- 4. FK / IK ----------
        joint_deg = [10, 20, 30, 40, 50, 10]
        joint_rad = degree2rad(joint_deg)
        print("input joints (rad):", joint_rad)
        fk = robot.calcFK(joint_rad, ec)
        print("FK pose:", fk)
        ik = robot.calcIK(fk, ec)
        print("IK back:", ik)

        # ---------- 5. DI / DO ----------
        do = robot.getDO(1, 0, ec)
        print("error container:", message(ec))
        print("DO[1,0]:", do)
        di = robot.getDI(0, 0, ec)
        print("DI[0,0]:", di)
        robot.setDO(0, 0, False, ec)
        print("DO[0,0] after set False:", robot.getDO(0, 0, ec))
        robot.setDO(0, 0, True, ec)

        # ---------- 6. reconnect ----------
        robot.disconnectFromRobot(ec)
        time.sleep(2)
        robot.connectToRobot(ec)

        # ---------- 7. drag (commented by default in original) ----------
        # robot.setPowerState(False, ec)
        # robot.setOperateMode(rokae.OperateMode.manual, ec)
        # robot.enableDrag(
        #     rokae.DragParameter.Space.cartesianSpace.value,
        #     rokae.DragParameter.Type.freely.value, ec)
        # time.sleep(2)
        # robot.disableDrag(ec)

        # ---------- 9. motion ----------
        robot.setOperateMode(rokae.OperateMode.automatic, ec)
        robot.setPowerState(True, ec)
        robot.moveReset(ec)

        # Five-point square path in flange frame
        home = [0.6319677128120011,
                -8.34603520129436e-05,
                0.5079049014875741,
                3.1415841917280183,
                -0.0005208332350316503,
                -3.1415883987024773]

        def shift(p, dx=0.0, dy=0.0, dz=0.0):
            return [p[0] + dx, p[1] + dy, p[2] + dz, *p[3:]]

        p1 = MoveLCommand(home,                          500, 0)
        p2 = MoveLCommand(shift(home, dy=+0.2),          400, 0)
        p3 = MoveLCommand(shift(home, dy=+0.2, dz=-0.2), 300, 0)
        p4 = MoveLCommand(shift(home, dy=+0.2),          100, 300)
        p5 = MoveLCommand(home,                          100, 300)

        cmds = [p1, p2, p3, p4, p5]
        robot.executeCommand(cmds, ec)
        print("queue length:", len(cmds))
        robot.moveStart(ec)

        # poll current point index, exactly like the official demo does
        deadline = time.time() + 10.0
        last_pp = -2
        while time.time() < deadline:
            pp = robot.getPointPos(ec)
            if pp != last_pp:
                print(f"  point_pos -> {pp}")
                last_pp = pp
            if pp >= len(cmds) - 1:
                break
            time.sleep(0.1)

        waitRobot(robot)
        print("motion finished, final joint pos:",
              robot.jointPos(ec))

        # ---------- shutdown ----------
        robot.stop(ec)
        time.sleep(1)
        robot.setPowerState(False, ec)
        robot.disconnectFromRobot(ec)


if __name__ == "__main__":
    main()