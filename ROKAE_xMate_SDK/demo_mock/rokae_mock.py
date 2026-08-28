"""
rokae_mock.py
=============

A pure-Python mock implementation of the ROKAE xMate robot SDK
(v0.1.6, Python 3.8 Windows pre-compiled), so that the SDK usage
example can be exercised on this machine without:

    * needing the cp38 ABI .pyd files,
    * needing a Python 3.8 interpreter,
    * needing a physical ROKAE robot on the LAN.

The mock reproduces the *public* surface used by the official
examples shipped with the SDK:

    from robot     import XMateRobot, XMateErProRobot, rokae
    from convert_tools import MoveLCommand, MoveJCommand, ...

All public methods take the same ``ec`` (error container) dict
argument that the real SDK uses and modify it in-place on failure.

This file is *only* meant for unit/integration style smoke-tests of
control code paths.  It does NOT talk to any real robot.
"""

from __future__ import annotations

import math
import time
import threading
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional


# ---------------------------------------------------------------------------
# Enums mirroring rokae.* (only the ones used by the official examples)
# ---------------------------------------------------------------------------

class OperationState(Enum):
    idle = 0
    running = 1
    pause = 2
    unknown = 3


class OperateMode(Enum):
    manual = 0
    automatic = 1


class DragParameter:
    class Space(Enum):
        cartesianSpace = 0
        jointSpace = 1

    class Type(Enum):
        freely = 0
        constrained = 1


# Re-export the same way the SDK does:
rokae = type("rokae_namespace", (), {})()  # placeholder module-like object
rokae.OperationState = OperationState
rokae.OperateMode = OperateMode
rokae.DragParameter = DragParameter


# ---------------------------------------------------------------------------
# Helper utilities mirroring convert_tools
# ---------------------------------------------------------------------------

def degree2rad(deg_list: List[float]) -> List[float]:
    return [d * math.pi / 180.0 for d in deg_list]


def rad2degree(rad_list: List[float]) -> List[float]:
    return [r * 180.0 / math.pi for r in rad_list]


def message(ec: Dict[str, Any]) -> Dict[str, Any]:
    """Return last error code/message from the container."""
    return ec


def zeroToolset() -> Dict[str, Any]:
    return {
        "end": {"rot": [0.0, 0.0, 0.0], "trans": [0.0, 0.0, 0.0]},
        "load": {"cog": [0.0, 0.0, 0.0],
                 "inertia": [0.0, 0.0, 0.0],
                 "mass": 0.0},
        "ref": {"rot": [0.0, 0.0, 0.0], "trans": [0.0, 0.0, 0.0]},
    }


@dataclass
class MoveLCommand:
    pos: List[float]
    speed: float = 100.0
    zone: float = 0.0
    offset: Optional[List[float]] = None

    def __init__(self, pos, speed=100, zone=0):
        self.pos = pos
        self.speed = speed
        self.zone = zone
        self.offset = None


@dataclass
class MoveJCommand:
    joint_pos: List[float]
    speed: float = 100.0
    zone: float = 0.0


# ---------------------------------------------------------------------------
# Internal state for the simulated robot
# ---------------------------------------------------------------------------

@dataclass
class _SimState:
    connected: bool = False
    powered: bool = False
    operate_mode: OperateMode = OperateMode.manual
    operation_state: OperationState = OperationState.idle
    joint_pos: List[float] = field(default_factory=lambda: [0.0] * 6)
    joint_vel: List[float] = field(default_factory=lambda: [0.0] * 6)
    joint_torque: List[float] = field(default_factory=lambda: [0.0] * 6)
    flange_pos: List[float] = field(default_factory=lambda: [0.5, 0.0, 0.4,
                                                              3.14, 0.0, -3.14])
    base_frame: Dict[str, Any] = field(default_factory=lambda: {
        "rot": [0.0, 0.0, 0.0], "trans": [0.0, 0.0, 0.0]
    })
    toolset: Dict[str, Any] = field(default_factory=zeroToolset)
    do: Dict[str, bool] = field(default_factory=lambda: {"0_0": False, "1_0": False})
    di: Dict[str, bool] = field(default_factory=lambda: {"0_0": False, "1_0": True})
    dragging: bool = False
    point_pos: int = -1   # index of currently executed point in queue
    cmd_queue: List[Any] = field(default_factory=list)
    history: List[Dict[str, Any]] = field(default_factory=list)


# ---------------------------------------------------------------------------
# Base class shared by XMateRobot / XMateErProRobot
# ---------------------------------------------------------------------------

class _BaseRobot:
    """Common implementation of every ROKAE robot class."""

    joint_num = 6

    def __init__(self, ip: str):
        self.ip = ip
        self._state = _SimState()
        self._lock = threading.Lock()
        self._motion_thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()

    # -- lifecycle --------------------------------------------------------

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        try:
            self.disconnectFromRobot({})
        except Exception:
            pass

    def connectToRobot(self, ec: Dict[str, Any]) -> None:
        with self._lock:
            if self._state.connected:
                self._fail(ec, 1001, "already connected")
                return
            time.sleep(0.05)
            self._state.connected = True
        print(f"[mock] connectToRobot({self.ip}) OK")

    def disconnectFromRobot(self, ec: Dict[str, Any]) -> None:
        with self._lock:
            self._state.connected = False
            self._state.powered = False
            self._stop_flag.set()
        print("[mock] disconnectFromRobot OK")

    # -- power / mode / state ---------------------------------------------

    def setPowerState(self, on: bool, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        if not self._state.connected:
            return
        with self._lock:
            self._state.powered = bool(on)
        print(f"[mock] setPowerState({on})")

    def powerState(self, ec: Dict[str, Any]) -> bool:
        self._require_connected(ec)
        return self._state.powered

    def setOperateMode(self, mode: OperateMode, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.operate_mode = mode
        print(f"[mock] setOperateMode({mode})")

    def operateMode(self, ec: Dict[str, Any]) -> OperateMode:
        self._require_connected(ec)
        return self._state.operate_mode

    def operationState(self, ec: Dict[str, Any]) -> int:
        self._require_connected(ec)
        return self._state.operation_state.value

    # -- info -------------------------------------------------------------

    def robotInfo(self, ec: Dict[str, Any]) -> Dict[str, Any]:
        self._require_connected(ec)
        return {
            "joint_num": self.joint_num,
            "type": "xMateMock",
            "version": "mock-0.1.6",
        }

    def sdkVersion(self, ec: Dict[str, Any]) -> str:
        return "0.1.6-mock"

    # -- joint / pose queries ---------------------------------------------

    def jointPos(self, ec: Dict[str, Any]) -> List[float]:
        self._require_connected(ec)
        return list(self._state.joint_pos)

    def jointVel(self, ec: Dict[str, Any]) -> List[float]:
        self._require_connected(ec)
        return list(self._state.joint_vel)

    def jointTorque(self, ec: Dict[str, Any]) -> List[float]:
        self._require_connected(ec)
        return list(self._state.joint_torque)

    def flangePos(self, ec: Dict[str, Any]) -> List[float]:
        self._require_connected(ec)
        return list(self._state.flange_pos)

    def baseFrame(self, ec: Dict[str, Any]) -> Dict[str, Any]:
        self._require_connected(ec)
        return dict(self._state.base_frame)

    def toolset(self, ec: Dict[str, Any]) -> Dict[str, Any]:
        self._require_connected(ec)
        return dict(self._state.toolset)

    def setToolset(self, ts: Dict[str, Any], ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.toolset = dict(ts)
        print("[mock] setToolset")

    # -- FK / IK ----------------------------------------------------------

    def calcFK(self, joint_list: List[float], ec: Dict[str, Any]) -> List[float]:
        self._require_connected(ec)
        if len(joint_list) != self.joint_num:
            self._fail(ec, 2001, "joint number mismatch")
            return []
        # trivial mock: flange = [j0*0.05 + 0.5, j1*0.05, j2*0.05 + 0.4, 0, 0, 0]
        return [0.5 + joint_list[0] * 0.05,
                joint_list[1] * 0.05,
                0.4 + joint_list[2] * 0.05,
                joint_list[3],
                joint_list[4],
                joint_list[5]]

    def calcIK(self, pose: List[float], ec: Dict[str, Any]) -> List[float]:
        self._require_connected(ec)
        if len(pose) != 6:
            self._fail(ec, 2002, "pose must have 6 elements")
            return []
        return [pose[3], pose[4], pose[5], 0.0, 0.0, 0.0]

    # -- DI / DO ----------------------------------------------------------

    def getDO(self, port: int, index: int, ec: Dict[str, Any]) -> bool:
        self._require_connected(ec)
        return self._state.do.get(f"{port}_{index}", False)

    def setDO(self, port: int, index: int, value: bool,
              ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        self._state.do[f"{port}_{index}"] = bool(value)
        print(f"[mock] setDO({port},{index})={value}")

    def getDI(self, port: int, index: int, ec: Dict[str, Any]) -> bool:
        self._require_connected(ec)
        return self._state.di.get(f"{port}_{index}", False)

    # -- drag -------------------------------------------------------------

    def enableDrag(self, space: int, dtype: int,
                   ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.dragging = True
        print(f"[mock] enableDrag space={space} type={dtype}")

    def disableDrag(self, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.dragging = False
        print("[mock] disableDrag")

    # -- motion queue -----------------------------------------------------

    def moveReset(self, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        self._stop_flag.set()
        with self._lock:
            self._state.cmd_queue.clear()
            self._state.point_pos = -1
            self._state.operation_state = OperationState.idle
        time.sleep(0.05)
        self._stop_flag.clear()
        print("[mock] moveReset")

    def executeCommand(self, cmds: List[Any], ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        if not self._state.powered:
            self._fail(ec, 3001, "robot is not powered on")
            return
        if self._state.operate_mode != OperateMode.automatic:
            self._fail(ec, 3002, "robot is not in automatic mode")
            return
        with self._lock:
            self._state.cmd_queue = list(cmds)
            self._state.point_pos = -1
            self._state.history.append({
                "t": time.time(),
                "n": len(cmds),
            })
        print(f"[mock] executeCommand, {len(cmds)} commands queued")

    def moveStart(self, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            if not self._state.cmd_queue:
                self._fail(ec, 3010, "empty command queue")
                return
            self._state.operation_state = OperationState.running

        # run the motion queue in a background thread so that callers
        # can still poll getPointPos() etc., exactly like the real SDK.
        self._stop_flag.clear()

        def _runner():
            for idx, cmd in enumerate(self._state.cmd_queue):
                if self._stop_flag.is_set():
                    return
                with self._lock:
                    self._state.point_pos = idx
                # simulate motion: move joints linearly toward target
                target = getattr(cmd, "pos", None) or getattr(cmd, "joint_pos", None)
                if target is None:
                    continue
                self._animate_to(target, duration=max(0.1, 2.0 / max(1.0, cmd.speed)))
                # every other point we briefly pause, so getPointPos logic works
                time.sleep(0.05)
            with self._lock:
                self._state.point_pos = len(self._state.cmd_queue)
                self._state.operation_state = OperationState.idle

        self._motion_thread = threading.Thread(target=_runner, daemon=True)
        self._motion_thread.start()
        print("[mock] moveStart")

    def _animate_to(self, target: List[float], duration: float = 0.5) -> None:
        steps = 10
        start = list(self._state.joint_pos)
        if len(target) < len(start):
            target = target + [0.0] * (len(start) - len(target))
        for s in range(1, steps + 1):
            if self._stop_flag.is_set():
                return
            a = s / steps
            self._state.joint_pos = [start[i] * (1 - a) + target[i] * a
                                      for i in range(len(start))]
            self._state.flange_pos = list(self._state.joint_pos)
            time.sleep(duration / steps)

    def pause(self, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.operation_state = OperationState.pause
        print("[mock] pause")

    def stop(self, ec: Dict[str, Any]) -> None:
        self._stop_flag.set()
        with self._lock:
            self._state.operation_state = OperationState.idle
        print("[mock] stop")

    def getPointPos(self, ec: Dict[str, Any]) -> int:
        self._require_connected(ec)
        return self._state.point_pos

    def adjustSpeedOnline(self, ratio: float, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        print(f"[mock] adjustSpeedOnline({ratio})")

    # -- recording --------------------------------------------------------

    def startRecordPath(self, period_ms: int, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        print(f"[mock] startRecordPath period={period_ms}ms")

    def saveRecordPath(self, name: str, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        print(f"[mock] saveRecordPath name={name}")

    def queryPathLists(self, ec: Dict[str, Any]) -> List[str]:
        self._require_connected(ec)
        return ["path_demo_1", "path_demo_2"]

    def replayPath(self, name: str, repeat: int, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        print(f"[mock] replayPath name={name} repeat={repeat}")

    # -- helpers ----------------------------------------------------------

    def _require_connected(self, ec: Dict[str, Any]) -> None:
        if not self._state.connected:
            self._fail(ec, 9001, "robot not connected")

    @staticmethod
    def _fail(ec: Dict[str, Any], code: int, msg: str) -> None:
        ec.clear()
        ec["code"] = code
        ec["message"] = msg


# ---------------------------------------------------------------------------
# Concrete robot classes (same names as the real SDK)
# ---------------------------------------------------------------------------

class XMateRobot(_BaseRobot):
    """xMate 6-axis cobot (default mock)."""
    joint_num = 6


class XMateErProRobot(_BaseRobot):
    """xMate ER Pro variant (same API as XMateRobot in mock form)."""
    joint_num = 6


# public API mirrors real SDK --------------------------------------------------------

__all__ = [
    "XMateRobot",
    "XMateErProRobot",
    "rokae",
    "MoveLCommand",
    "MoveJCommand",
    "degree2rad",
    "rad2degree",
    "message",
    "zeroToolset",
]