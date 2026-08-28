"""
xCoreSDK_python.py
===================

A pure-Python mock of the ROKAE xCore SDK-Python package (v0.7.1).
Mirrors the public surface used by every official example in
``xCoreSDK-Python/example/``:

    import xCoreSDK_python                  # module object
    robot = xCoreSDK_python.xMateRobot(ip)  # factory function
    ec = {}
    robot.connectToRobot(ec)
    robot.setPowerState(True, ec)
    ...

Every API in the official demo has a counterpart here.  The mock
matches the *type hints* shipped in ``Release/**/xCoreSDK_python/__init__.pyi``
so it can be drop-in replaced.

Limitations: no real hardware, no real physics; motion is animated
in a background thread that walks joint_pos toward the target.
"""

from __future__ import annotations

import math
import threading
import time
import uuid
from dataclasses import dataclass, field
from datetime import timedelta
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Union


# ---------------------------------------------------------------------------
# PyType* helper classes (tiny stubs that mimic the C++ wrappers)
# ---------------------------------------------------------------------------

class PyTypeDouble:
    def __init__(self, v: float = 0.0):
        self._v = float(v)
    def content(self) -> float: return self._v
    def __repr__(self): return f"PyTypeDouble({self._v})"


class PyTypeFloat:
    def __init__(self, v: float = 0.0):
        self._v = float(v)
    def content(self) -> float: return self._v


class PyTypeInt:
    def __init__(self, v: int = 0):
        self._v = int(v)
    def content(self) -> int: return self._v


class PyTypeBool:
    def __init__(self, v: bool = False):
        self._v = bool(v)
    def content(self) -> bool: return self._v


class PyTypeUInt8:
    def __init__(self, v: int = 0):
        self._v = int(v)
    def content(self) -> int: return self._v


class PyTypeUInt64:
    def __init__(self, v: int = 0):
        self._v = int(v)
    def content(self) -> int: return self._v


class PyTypeVectorBool:
    def __init__(self, data: Optional[List[bool]] = None):
        self._v: List[bool] = list(data or [])
    def content(self) -> List[bool]: return list(self._v)
    def __len__(self): return len(self._v)


class PyTypeVectorInt:
    def __init__(self, data: Optional[List[int]] = None):
        self._v: List[int] = list(data or [])
    def content(self) -> List[int]: return list(self._v)


class PyTypeVectorFloat:
    def __init__(self, data: Optional[List[float]] = None):
        self._v: List[float] = list(data or [])
    def content(self) -> List[float]: return list(self._v)


class PyTypeVectorDouble:
    def __init__(self, data: Optional[List[float]] = None):
        self._v: List[float] = list(data or [])
    def content(self) -> List[float]: return list(self._v)


class PyTypeVectorArrayDouble2:
    def __init__(self, data: Optional[List[List[float]]] = None):
        self._v: List[List[float]] = list(data or [])
    def content(self) -> List[List[float]]: return [list(r) for r in self._v]


class PyTypeVectorString:
    def __init__(self, data: Optional[List[str]] = None):
        self._v: List[str] = list(data or [])
    def content(self) -> List[str]: return list(self._v)


class PyString:
    def __init__(self, s: str = ""):
        self._s = s
    def content(self) -> str: return self._s


# ---------------------------------------------------------------------------
# Enums (mirroring xCoreSDK_python.*)
# ---------------------------------------------------------------------------

class AvoidSingularityMethod(Enum):
    lockAxis4 = 0
    wrist = 1
    jointWay = 2


class CoordinateType(Enum):
    endInRef = 0
    flangeInBase = 1


class DragParameterSpace(Enum):
    jointSpace = 0
    cartesianSpace = 1


class DragParameterType(Enum):
    translationOnly = 0
    rotationOnly = 1
    freely = 2


class Event(Enum):
    moveExecution = 0
    path = 1
    rlExecution = 2
    jog = 3
    jogging = 4
    safety = 5
    rtControlling = 6
    rlProgram = 7
    none = 8
    error = 9
    dynamicIdentify = 10
    loadIdentify = 11
    frictionIdentify = 12
    baseParallelMode = 13
    rail = 14
    base = 15
    estop = 16
    gstop = 17
    logReporter = 18


class FrameType(Enum):
    tool = 0
    wobj = 1
    base = 2


class JogOptSpace(Enum):
    jointSpace = 0
    world = 1
    tool = 2
    singularityAvoidMode = 3


class LogInfoLevel(Enum):
    info = 0
    warning = 1
    error = 2


class MotionControlMode(Enum):
    NrtCommandMode = 0
    RtCommandMode = 1
    RtControllerMode = 2


class MoveCFCommandRotType(Enum):
    constPose = 0
    translationOnly = 1
    rotationOnly = 2


class OperateMode(Enum):
    manual = 0
    automatic = 1


class OperationState(Enum):
    idle = 0
    moving = 1
    jog = 2
    unknown = 3


class PowerState(Enum):
    off = 0
    on = 1


class StopLevel(Enum):
    stop0 = 0
    stop1 = 1
    stop2 = 2
    suppleStop = 3


class CartesianPositionOffsetType(Enum):
    offs = 0
    relTool = 1


class xPanelOptVout(Enum):
    reserve = 0
    supply12v = 1
    supply24v = 2
    off = 3
    on = 4


# ---------------------------------------------------------------------------
# Data classes used as both parameters and return types
# ---------------------------------------------------------------------------

class Finishable:
    """Marker base class."""


@dataclass
class CartesianPosition(Finishable):
    trans: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pos: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    confData: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0, 0.0])
    elbow: float = 0.0
    hasElbow: bool = False
    external: List[float] = field(default_factory=list)

    def __init__(self, vals=None, trans=None, rpy=None):
        if vals is not None:
            assert len(vals) == 6
            self.trans = list(vals[0:3])
            self.rpy = list(vals[3:6])
        else:
            self.trans = list(trans or [0.0, 0.0, 0.0])
            self.rpy = list(rpy or [0.0, 0.0, 0.0])
        self.pos = self.trans + self.rpy
        self.confData = [0.0, 0.0, 0.0, 0.0]
        self.elbow = 0.0
        self.hasElbow = False
        self.external = []

    def isFinished(self) -> bool:
        return True


@dataclass
class JointPosition(Finishable):
    joints: List[float] = field(default_factory=lambda: [0.0] * 6)

    def __init__(self, vals=None):
        if vals is None:
            vals = [0.0] * 6
        assert len(vals) >= 1
        self.joints = [float(v) for v in vals]

    def isFinished(self) -> bool:
        return True

    def __iter__(self): return iter(self.joints)
    def __len__(self): return len(self.joints)
    def __getitem__(self, i): return self.joints[i]


@dataclass
class Load:
    mass: float = 0.0
    cog: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    inertia: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


@dataclass
class Frame:
    trans: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    rpy: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    pos: List[float] = field(default_factory=lambda: [0.0] * 6)


@dataclass
class Toolset:
    end: Frame = field(default_factory=Frame)
    ref: Frame = field(default_factory=Frame)
    load: Load = field(default_factory=Load)


@dataclass
class KeyPadState:
    key1_state: bool = False
    key2_state: bool = False
    key3_state: bool = False
    key4_state: bool = False
    key5_state: bool = False
    key6_state: bool = False
    key7_state: bool = False


@dataclass
class Info:
    id: str = "mock-xMate-7"
    version: str = "mock-3.2.2"
    type: str = "xMateER7 Pro"
    joint_num: int = 7
    mac: str = "00:00:00:00:00:00"


@dataclass
class LogInfo:
    timestamp: str = ""
    content: str = ""
    repair: str = ""


@dataclass
class FrameCalibrationResult:
    frame: Frame = field(default_factory=Frame)
    errors: List[float] = field(default_factory=list)


@dataclass
class CartesianPositionOffset:
    type: CartesianPositionOffsetType = CartesianPositionOffsetType.offs
    pose: CartesianPosition = field(default_factory=CartesianPosition)


@dataclass
class Torque:
    value: float = 0.0


# ---------------------------------------------------------------------------
# Move commands
# ---------------------------------------------------------------------------

class NrtCommand:
    """Base marker class."""


@dataclass
class MoveAbsJCommand(NrtCommand):
    joint_pos: JointPosition
    speed: float = 1000.0
    zone: float = 0.0

    def __init__(self, joint_pos, speed=1000, zone=0):
        if isinstance(joint_pos, JointPosition):
            self.joint_pos = joint_pos
        else:
            self.joint_pos = JointPosition(joint_pos)
        self.speed = speed
        self.zone = zone


@dataclass
class MoveLCommand(NrtCommand):
    cart_pos: CartesianPosition
    speed: float = 1000.0
    zone: float = 0.0

    def __init__(self, cart_pos, speed=1000, zone=0):
        if isinstance(cart_pos, CartesianPosition):
            self.cart_pos = cart_pos
        else:
            self.cart_pos = CartesianPosition(cart_pos)
        self.speed = speed
        self.zone = zone


@dataclass
class MoveJCommand(NrtCommand):
    cart_pos: CartesianPosition
    speed: float = 1000.0
    zone: float = 0.0

    def __init__(self, cart_pos, speed=1000, zone=0):
        if isinstance(cart_pos, CartesianPosition):
            self.cart_pos = cart_pos
        else:
            self.cart_pos = CartesianPosition(cart_pos)
        self.speed = speed
        self.zone = zone


@dataclass
class MoveCCommand(NrtCommand):
    target: CartesianPosition
    aux: CartesianPosition
    speed: float = 1000.0
    zone: float = 0.0

    def __init__(self, target, aux, speed=1000, zone=0):
        self.target = target if isinstance(target, CartesianPosition) else CartesianPosition(target)
        self.aux = aux if isinstance(aux, CartesianPosition) else CartesianPosition(aux)
        self.speed = speed
        self.zone = zone


@dataclass
class MoveCFCommand(NrtCommand):
    target: CartesianPosition
    aux: CartesianPosition
    angle: float = 0.0
    speed: float = 1000.0
    zone: float = 0.0
    rot_type: MoveCFCommandRotType = MoveCFCommandRotType.constPose

    def __init__(self, target, aux, angle=0.0, speed=1000, zone=0,
                 rot_type=MoveCFCommandRotType.constPose):
        self.target = target if isinstance(target, CartesianPosition) else CartesianPosition(target)
        self.aux = aux if isinstance(aux, CartesianPosition) else CartesianPosition(aux)
        self.angle = angle
        self.speed = speed
        self.zone = zone
        self.rot_type = rot_type


@dataclass
class MoveSPCommand(NrtCommand):
    target: CartesianPosition
    r0: float = 0.0
    rstep: float = 0.0
    angle: float = 0.0
    direction: bool = True
    speed: float = 1000.0

    def __init__(self, target, r0, rstep, angle, direction, speed=1000):
        self.target = target if isinstance(target, CartesianPosition) else CartesianPosition(target)
        self.r0 = r0
        self.rstep = rstep
        self.angle = angle
        self.direction = direction
        self.speed = speed


@dataclass
class MoveWaitCommand(NrtCommand):
    wait_time: timedelta

    def __init__(self, t: timedelta):
        self.wait_time = t


# ---------------------------------------------------------------------------
# Robot state
# ---------------------------------------------------------------------------

class _MotionControlModeProxy:
    NrtCommandMode = 0
    RtCommandMode = 1
    RtControllerMode = 2


class RtSupportedFields:
    """Minimal subset: only the strings used by official demos."""
    tcpPose_m = "tcpPose_m"
    jointPos_m = "jointPos_m"
    keypads = "keypads"


# ---------------------------------------------------------------------------
# Base robot implementation
# ---------------------------------------------------------------------------

class _SimState:
    def __init__(self):
        self.connected = False
        self.powered = PowerState.off
        self.operate_mode = OperateMode.manual
        self.op_state = OperationState.idle
        self.joint_pos = [0.0] * 7
        self.joint_vel = [0.0] * 7
        self.joint_torque = [0.0] * 7
        self.flange_pos = [0.5, 0.0, 0.4] + [math.pi, 0.0, -math.pi]
        self.base_frame = Frame()
        self.toolset = Toolset()
        self.do: Dict[str, bool] = {}
        self.di: Dict[str, bool] = {}
        self.ai: Dict[str, float] = {}
        self.registers: Dict[str, List[float]] = {
            "register1": [0.0],
            "register2": [0.0] * 5,
        }
        self.dragging = False
        self.recording = False
        self.recorded_paths: List[str] = ["demo_path_1", "demo_path_2"]
        self.simulation_mode = False
        self.motion_ctrl_mode = _MotionControlModeProxy.NrtCommandMode
        self.collision_detection_enabled = False
        self.collision_sensitivity = [1.0] * 7
        self.acc = PyTypeDouble(0.5)
        self.jerk = PyTypeDouble(0.5)
        self.default_speed = 100.0
        self.default_zone = 0.0
        self.event_watchers: Dict[Event, Optional[Callable[[Dict[str, Any]], None]]] = {}
        self.keypad_state = KeyPadState()
        self.jog_count = 0  # how many times startJog has been called


class BaseRobot:
    """Mirrors the C++ BaseRobot used by all example scripts."""

    def __init__(self, remote_ip: str, local_ip: str = ""):
        self._remote = remote_ip
        self._local = local_ip
        self._state = _SimState()
        self._lock = threading.RLock()
        self._motion_thread: Optional[threading.Thread] = None
        self._stop_flag = threading.Event()

    # -- static helpers ----------------------------------------------------

    @staticmethod
    def sdkVersion() -> str:
        return "0.7.1-mock"

    # -- connection --------------------------------------------------------

    def connectToRobot(self, ec: Dict[str, Any]) -> None:
        with self._lock:
            time.sleep(0.05)
            self._state.connected = True
        self._ok(ec)

    def disconnectFromRobot(self, ec: Dict[str, Any]) -> None:
        with self._lock:
            self._stop_flag.set()
            self._state.connected = False
            self._state.powered = PowerState.off
        self._ok(ec)

    # -- power / mode -----------------------------------------------------

    def setPowerState(self, on: bool, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        if not self._state.connected:
            return
        with self._lock:
            self._state.powered = PowerState.on if on else PowerState.off
        self._ok(ec)

    def powerState(self, ec: Dict[str, Any]) -> PowerState:
        self._require_connected(ec)
        return self._state.powered

    def setOperateMode(self, mode: OperateMode, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.operate_mode = mode
        self._ok(ec)

    def operateMode(self, ec: Dict[str, Any]) -> OperateMode:
        self._require_connected(ec)
        return self._state.operate_mode

    def operationState(self, ec: Dict[str, Any]) -> OperationState:
        self._require_connected(ec)
        return self._state.op_state

    def setMotionControlMode(self, mode: int, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.motion_ctrl_mode = mode
        self._ok(ec)

    # -- info / kinematics ------------------------------------------------

    def robotInfo(self, ec: Dict[str, Any]) -> Info:
        self._require_connected(ec)
        return Info()

    def posture(self, ct: CoordinateType, ec: Dict[str, Any]) -> List[float]:
        self._require_connected(ec)
        return list(self._state.flange_pos)

    def cartPosture(self, ct: CoordinateType, ec: Dict[str, Any]) -> CartesianPosition:
        self._require_connected(ec)
        cp = CartesianPosition()
        cp.trans = list(self._state.flange_pos[:3])
        cp.rpy = list(self._state.flange_pos[3:6])
        cp.pos = cp.trans + cp.rpy
        cp.confData = [0.0, 0.0, 0.0, 0.0]
        cp.elbow = 0.0
        cp.hasElbow = False
        cp.external = []
        return cp

    def jointPos(self, ec: Dict[str, Any]) -> List[float]:
        self._require_connected(ec)
        return list(self._state.joint_pos)

    def jointVel(self, ec: Dict[str, Any]) -> List[float]:
        self._require_connected(ec)
        return list(self._state.joint_vel)

    def jointTorque(self, ec: Dict[str, Any]) -> List[float]:
        self._require_connected(ec)
        return list(self._state.joint_torque)

    def baseFrame(self, ec: Dict[str, Any]) -> List[float]:
        self._require_connected(ec)
        f = self._state.base_frame
        return list(f.trans) + list(f.rpy)

    def setBaseFrame(self, frame: Frame, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.base_frame = frame
        self._ok(ec)

    def toolset(self, ec: Dict[str, Any]) -> Toolset:
        self._require_connected(ec)
        return self._state.toolset

    def setToolset(self, toolset: Union[Toolset, str], wobj_or_ec=None,
                   ec: Optional[Dict[str, Any]] = None) -> Optional[Toolset]:
        # Overload 1: setToolset(Toolset, ec)
        # Overload 2: setToolset(toolName: str, wobjName: str, ec)
        if isinstance(toolset, Toolset):
            ec = wobj_or_ec
            self._require_connected(ec)
            with self._lock:
                self._state.toolset = toolset
            self._ok(ec)
            return None
        else:
            ec = ec if ec is not None else {}
            self._require_connected(ec)
            return Toolset()

    # -- IO --------------------------------------------------------------

    def getDO(self, board: int, port: int, ec: Dict[str, Any]) -> bool:
        self._require_connected(ec)
        return self._state.do.get(f"{board}_{port}", False)

    def setDO(self, board: int, port: int, state: bool,
              ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.do[f"{board}_{port}"] = bool(state)
        self._ok(ec)

    def getDI(self, board: int, port: int, ec: Dict[str, Any]) -> bool:
        self._require_connected(ec)
        return self._state.di.get(f"{board}_{port}", False)

    def setDI(self, board: int, port: int, state: bool,
              ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        if not self._state.simulation_mode:
            self._fail(ec, "setDI requires simulation mode")
            return
        with self._lock:
            self._state.di[f"{board}_{port}"] = bool(state)
        self._ok(ec)

    def getAI(self, board: int, port: int, ec: Dict[str, Any]) -> float:
        self._require_connected(ec)
        return self._state.ai.get(f"{board}_{port}", 0.0)

    def setAO(self, board: int, port: int, value: float,
              ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.ai[f"{board}_{port}"] = float(value)
        self._ok(ec)

    # -- registers --------------------------------------------------------

    def readRegister(self, name: str, index: int,
                     value: Union[PyTypeBool, PyTypeInt, PyTypeFloat,
                                  PyTypeVectorBool, PyTypeVectorInt,
                                  PyTypeVectorFloat],
                     ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        if name not in self._state.registers:
            self._fail(ec, f"register {name} not found")
            return
        data = self._state.registers[name]
        # choose behavior by type of container
        if isinstance(value, (PyTypeVectorBool, PyTypeVectorInt, PyTypeVectorFloat)):
            # return whole array regardless of index
            if isinstance(value, PyTypeVectorBool):
                value._v = [bool(v) for v in data]
            elif isinstance(value, PyTypeVectorInt):
                value._v = [int(v) for v in data]
            else:
                value._v = [float(v) for v in data]
        else:
            if index < 0 or index >= len(data):
                self._fail(ec, f"register index {index} out of range")
                return
            v = data[index]
            if isinstance(value, PyTypeBool):
                value._v = bool(v)
            elif isinstance(value, PyTypeInt):
                value._v = int(v)
            else:
                value._v = float(v)
        self._ok(ec)

    def writeRegister(self, name: str, index: int,
                      value: Union[bool, int, float,
                                   List[bool], List[int], List[float]],
                      ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        if name not in self._state.registers:
            self._state.registers[name] = [0.0] * 5
        data = self._state.registers[name]
        if isinstance(value, list):
            # write whole array
            for i, v in enumerate(value):
                if i < len(data):
                    data[i] = float(v) if not isinstance(v, bool) else float(v)
            # pad if shorter
            for i in range(len(value), len(data)):
                data[i] = 0.0
        else:
            if index < 0 or index >= len(data):
                self._fail(ec, f"register index {index} out of range")
                return
            data[index] = float(value)
        self._ok(ec)

    # -- motion queue -----------------------------------------------------

    def moveReset(self, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        self._stop_flag.set()
        with self._lock:
            self._state.op_state = OperationState.idle
        time.sleep(0.05)
        self._stop_flag.clear()
        self._ok(ec)

    def moveAppend(self, cmds, cmdID: PyString, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        if self._state.powered != PowerState.on:
            self._fail(ec, "robot not powered on")
            return
        if self._state.operate_mode != OperateMode.automatic:
            self._fail(ec, "robot not in automatic mode")
            return
        if self._state.motion_ctrl_mode != _MotionControlModeProxy.NrtCommandMode:
            self._fail(ec, "must be in NrtCommandMode")
            return
        if isinstance(cmds, list):
            new_id = f"cmd-{uuid.uuid4().hex[:8]}"
            cmdID._s = new_id
            with self._lock:
                self._state.cmd_queue = list(cmds)
        else:
            new_id = f"cmd-{uuid.uuid4().hex[:8]}"
            cmdID._s = new_id
            with self._lock:
                self._state.cmd_queue.append(cmds)
        self._ok(ec)

    def moveStart(self, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        self._stop_flag.clear()
        with self._lock:
            if not self._state.cmd_queue:
                self._fail(ec, "empty queue")
                return
            self._state.op_state = OperationState.moving
        # start a background motion thread
        self._motion_thread = threading.Thread(target=self._animate, daemon=True)
        self._motion_thread.start()
        self._ok(ec)

    def _animate(self) -> None:
        for cmd in self._state.cmd_queue:
            if self._stop_flag.is_set():
                return
            if isinstance(cmd, MoveWaitCommand):
                # mimic the wait by sleeping
                secs = cmd.wait_time.total_seconds()
                end_t = time.time() + secs
                while time.time() < end_t and not self._stop_flag.is_set():
                    time.sleep(0.05)
                continue
            target = self._resolve_target(cmd)
            self._walk_joints_toward(target, max(0.2, 1.0))
        with self._lock:
            self._state.op_state = OperationState.idle
        watcher = self._state.event_watchers.get(Event.moveExecution)
        if watcher:
            try:
                watcher({})
            except Exception:
                pass

    def _resolve_target(self, cmd) -> List[float]:
        if isinstance(cmd, MoveAbsJCommand):
            return list(cmd.joint_pos.joints)
        # cartesian commands - map to joint-space pose via simple
        # offset relative to current joints (purely for animation)
        return [self._state.joint_pos[i] + 0.05 for i in range(7)]

    def _walk_joints_toward(self, target: List[float], duration: float) -> None:
        n = len(self._state.joint_pos)
        if len(target) < n:
            target = target + [0.0] * (n - len(target))
        steps = 10
        start = list(self._state.joint_pos)
        for s in range(1, steps + 1):
            if self._stop_flag.is_set():
                return
            a = s / steps
            with self._lock:
                self._state.joint_pos = [start[i] * (1 - a) + target[i] * a
                                          for i in range(n)]
                self._state.flange_pos = list(self._state.joint_pos[:6])
            time.sleep(duration / steps)

    def stop(self, ec: Dict[str, Any]) -> None:
        self._stop_flag.set()
        with self._lock:
            self._state.op_state = OperationState.idle
        self._ok(ec)

    def checkPath(self, *args, **kwargs) -> List[float]:
        # overloads vary; return a fake successful joint list
        ec = args[-1] if args and isinstance(args[-1], dict) else kwargs.get("ec", {})
        self._ok(ec)
        return [0.0, 0.1, 0.2, 0.0, 0.3, 0.0]

    def model(self):
        # returned object has calcFk / calcIk methods
        return _ModelProxy(self)

    # -- drag / record / replay ------------------------------------------

    def enableDrag(self, space: int, dtype: int,
                   ec: Dict[str, Any], enable_drag_button: bool = False) -> None:
        self._require_connected(ec)
        if self._state.operate_mode != OperateMode.manual:
            self._fail(ec, "drag requires manual mode")
            return
        with self._lock:
            self._state.dragging = True
        self._ok(ec)

    def disableDrag(self, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.dragging = False
        self._ok(ec)

    def startRecordPath(self, duration: int, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.recording = True
        self._ok(ec)

    def stopRecordPath(self, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.recording = False
        self._ok(ec)

    def saveRecordPath(self, name: str, ec: Dict[str, Any], saveAs: str = "") -> None:
        self._require_connected(ec)
        with self._lock:
            if saveAs:
                self._state.recorded_paths = [p for p in self._state.recorded_paths
                                              if p != name] + [saveAs]
            else:
                if name not in self._state.recorded_paths:
                    self._state.recorded_paths.append(name)
        self._ok(ec)

    def cancelRecordPath(self, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.recording = False
        self._ok(ec)

    def queryPathLists(self, ec: Dict[str, Any]) -> List[str]:
        self._require_connected(ec)
        return list(self._state.recorded_paths)

    def removePath(self, name: str, ec: Dict[str, Any], removeAll: bool = False) -> None:
        self._require_connected(ec)
        with self._lock:
            if removeAll:
                self._state.recorded_paths.clear()
            else:
                self._state.recorded_paths = [p for p in self._state.recorded_paths
                                              if p != name]
        self._ok(ec)

    def replayPath(self, name: str, rate: float, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        if name not in self._state.recorded_paths:
            self._fail(ec, f"path {name} not found")
            return
        with self._lock:
            # synthesize a couple of MoveAbsJ commands to drive motion
            self._state.cmd_queue = [
                MoveAbsJCommand(JointPosition([0.1, 0.2, 0.3, 0.0, 0.4, 0.0, 0.0])),
                MoveAbsJCommand(JointPosition([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])),
            ]
        self._ok(ec)

    # -- collision / events ---------------------------------------------

    def setEventWatcher(self, event: Event, callback: Callable,
                        ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.event_watchers[event] = callback
        self._ok(ec)

    def setNoneEventWatcher(self, event: Event, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.event_watchers.pop(event, None)
        self._ok(ec)

    def enableCollisionDetection(self, sensitivity: List[float],
                                 behaviour: StopLevel,
                                 fallback_compliance: float,
                                 ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.collision_detection_enabled = True
            self._state.collision_sensitivity = list(sensitivity)
        self._ok(ec)

    def disableCollisionDetection(self, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.collision_detection_enabled = False
        self._ok(ec)

    def recoverState(self, item: int, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        self._ok(ec)

    def clearServoAlarm(self, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        self._ok(ec)

    # -- motion config ---------------------------------------------------

    def setDefaultSpeed(self, speed: float, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.default_speed = float(speed)
        self._ok(ec)

    def setDefaultZone(self, zone: float, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.default_zone = float(zone)
        self._ok(ec)

    def setDefaultConfOpt(self, forced: bool, ec: Dict[str, Any]) -> None:
        self._ok(ec)

    def setMaxCacheSize(self, number: int, ec: Dict[str, Any]) -> None:
        self._ok(ec)

    def getAcceleration(self, acc: PyTypeDouble, jerk: PyTypeDouble,
                        ec: Dict[str, Any]) -> None:
        acc._v = self._state.acc.content()
        jerk._v = self._state.jerk.content()
        self._ok(ec)

    def adjustAcceleration(self, acc: float, jerk: float,
                           ec: Dict[str, Any]) -> None:
        self._state.acc._v = float(acc)
        self._state.jerk._v = float(jerk)
        self._ok(ec)

    def getSoftLimit(self, limits: PyTypeVectorArrayDouble2,
                     ec: Dict[str, Any]) -> bool:
        limits._v = [[-math.pi, math.pi]] * 7
        self._ok(ec)
        return True

    def setSoftLimit(self, enable: bool, ec: Dict[str, Any],
                     limits: Optional[List[List[float]]] = None) -> None:
        self._ok(ec)

    # -- calibrate / jog --------------------------------------------------

    def calibrateFrame(self, frame_type: FrameType, points: List[List[float]],
                       is_held: bool, ec: Dict[str, Any],
                       base_aux: Optional[List[float]] = None) -> FrameCalibrationResult:
        self._require_connected(ec)
        result = FrameCalibrationResult()
        result.frame = Frame()
        result.errors = [0.001, 0.001, 0.001, 0.01, 0.01, 0.01]
        self._ok(ec)
        return result

    def startJog(self, space: JogOptSpace, rate: float, step: float,
                 index: int, direction: bool, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.op_state = OperationState.jog
            self._state.jog_count += 1
        self._ok(ec)

    def startJogWithExt(self, *args, **kwargs) -> None:
        ec = kwargs.get("ec", args[-1] if args and isinstance(args[-1], dict) else {})
        self._ok(ec)

    # -- system ----------------------------------------------------------

    def rebootSystem(self, ec: Dict[str, Any]) -> None:
        self._ok(ec)

    def shutdownSystem(self, ec: Dict[str, Any]) -> None:
        self._ok(ec)

    def setSimulationMode(self, state: bool, ec: Dict[str, Any]) -> None:
        self._require_connected(ec)
        with self._lock:
            self._state.simulation_mode = bool(state)
        self._ok(ec)

    def queryControllerLog(self, count: int,
                           level: set, ec: Dict[str, Any],
                           offset: int = 0) -> List[LogInfo]:
        self._require_connected(ec)
        ts = time.strftime("%Y-%m-%d %H:%M:%S")
        out = []
        for i in range(count):
            out.append(LogInfo(
                timestamp=ts,
                content=f"[mock] log entry #{offset + i} (level {[l.name for l in level]})",
                repair="",
            ))
        self._ok(ec)
        return out

    def getKeypadState(self, ec: Dict[str, Any]) -> KeyPadState:
        self._require_connected(ec)
        return self._state.keypad_state

    def setxPanelRS485(self, opt: int, if_rs485: bool,
                       ec: Dict[str, Any]) -> None:
        self._ok(ec)

    def XPRWModbusRTUReg(self, *args, **kwargs) -> None:
        ec = kwargs.get("ec", args[-1] if args and isinstance(args[-1], dict) else {})
        self._ok(ec)

    def XPRWModbusRTUCoil(self, *args, **kwargs) -> None:
        ec = kwargs.get("ec", args[-1] if args and isinstance(args[-1], dict) else {})
        self._ok(ec)

    def XPRS485SendData(self, *args, **kwargs) -> None:
        ec = kwargs.get("ec", args[-1] if args and isinstance(args[-1], dict) else {})
        self._ok(ec)

    def getRobotCfg_DHparam(self, *_args, **_kwargs) -> List[float]:
        return [0.0, 0.0, 0.4, 0.0, 0.0, 0.0]

    def setConnectionHandler(self, handler: Callable[[bool], None]) -> None:
        # would normally call handler(True) on (re)connect, but here we just store
        self._conn_handler = handler

    def startReceiveRobotState(self, *_args, **_kwargs) -> None:
        pass

    def stopReceiveRobotState(self) -> None:
        pass

    def updateRobotState(self, *_args, **_kwargs) -> int:
        return 1

    def getStateData(self, *_args, **_kwargs) -> int:
        return 0

    def queryEventInfo(self, *_args, **_kwargs) -> Dict[str, Any]:
        return {}

    # -- error helpers ---------------------------------------------------

    def _require_connected(self, ec: Dict[str, Any]) -> None:
        if not self._state.connected:
            self._fail(ec, "robot not connected")

    def _ok(self, ec: Dict[str, Any]) -> None:
        ec.clear()
        ec["ec"] = 0
        ec["message"] = "ok"

    def _fail(self, ec: Dict[str, Any], msg: str) -> None:
        ec.clear()
        ec["ec"] = -1
        ec["message"] = msg


# ---------------------------------------------------------------------------
# Model proxy (returned by robot.model())
# ---------------------------------------------------------------------------

class _ModelProxy:
    def __init__(self, robot: BaseRobot):
        self._robot = robot

    def calcFk(self, joint_list: List[float], toolset: Toolset,
               ec: Dict[str, Any]) -> CartesianPosition:
        self._robot._require_connected(ec)
        cp = CartesianPosition()
        cp.trans = [0.5 + joint_list[0] * 0.05,
                    joint_list[1] * 0.05 if len(joint_list) > 1 else 0.0,
                    0.4 + joint_list[2] * 0.05 if len(joint_list) > 2 else 0.4]
        cp.rpy = [joint_list[3] if len(joint_list) > 3 else 0.0,
                  joint_list[4] if len(joint_list) > 4 else 0.0,
                  joint_list[5] if len(joint_list) > 5 else 0.0]
        cp.pos = cp.trans + cp.rpy
        self._robot._ok(ec)
        return cp

    def calcIk(self, cart_pos: CartesianPosition, toolset: Toolset,
               ec: Dict[str, Any]) -> List[float]:
        self._robot._require_connected(ec)
        self._robot._ok(ec)
        return [cart_pos.rpy[0], cart_pos.rpy[1], cart_pos.rpy[2], 0.0, 0.0, 0.0]

    def calcAllIkSolutions(self, cart_pos: CartesianPosition,
                           confs_out: list, ec: Dict[str, Any]) -> List[List[float]]:
        self._robot._require_connected(ec)
        self._robot._ok(ec)
        joints1 = [cart_pos.rpy[0], cart_pos.rpy[1], cart_pos.rpy[2], 0.0, 0.0, 0.0]
        joints2 = [-cart_pos.rpy[0], -cart_pos.rpy[1], -cart_pos.rpy[2], 0.0, 0.0, 0.0]
        confs_out.append([1, 0, 0, 0])
        confs_out.append([0, 1, 0, 0])
        return [joints1, joints2]


# ---------------------------------------------------------------------------
# Factory functions matching xCoreSDK_python.xMateRobot, etc.
# ---------------------------------------------------------------------------

def xMateRobot(ip: str, local_ip: str = "") -> BaseRobot:
    return BaseRobot(ip, local_ip)


def xMateErProRobot(ip: str, local_ip: str = "") -> BaseRobot:
    return BaseRobot(ip, local_ip)


def StandardRobot(ip: str, local_ip: str = "") -> BaseRobot:
    return BaseRobot(ip, local_ip)


def PCB3Robot(ip: str, local_ip: str = "") -> BaseRobot:
    return BaseRobot(ip, local_ip)


def PCB4Robot(ip: str, local_ip: str = "") -> BaseRobot:
    return BaseRobot(ip, local_ip)


def Cobot_5(ip: str, local_ip: str = "") -> BaseRobot:
    return BaseRobot(ip, local_ip)


def Cobot_6(ip: str, local_ip: str = "") -> BaseRobot:
    return BaseRobot(ip, local_ip)


def Cobot_7(ip: str, local_ip: str = "") -> BaseRobot:
    return BaseRobot(ip, local_ip)


def IndustrialRobot_3(ip: str, local_ip: str = "") -> BaseRobot:
    return BaseRobot(ip, local_ip)


def IndustrialRobot_4(ip: str, local_ip: str = "") -> BaseRobot:
    return BaseRobot(ip, local_ip)


def IndustrialRobot_6(ip: str, local_ip: str = "") -> BaseRobot:
    return BaseRobot(ip, local_ip)


# ---------------------------------------------------------------------------
# EventInfoKey sub-module (mirrors the C++ .pyi for this subpackage).
# Both ``from xCoreSDK_python.EventInfoKey import MoveExecution`` and
# ``from Release.windows.xCoreSDK_python.EventInfoKey import MoveExecution``
# should work.
# ---------------------------------------------------------------------------

class _EventInfoKeyPackage:
    """Lightweight stand-in for ``xCoreSDK_python.EventInfoKey``.

    Exposes MoveExecution/LogReporter/RlExecution/Safety sub-objects
    whose attributes are the field-key strings from the official pyi.
    """
    class MoveExecution:
        CustomInfo = "customInfo"
        Error = "error"
        ID = "cmdID"
        ReachTarget = "reachTarget"
        Remark = "remark"
        WaypointIndex = "wayPointIndex"

    class LogReporter:
        Report = "report"

    class RlExecution:
        ID = "cmdID"
        State = "state"

    class Safety:
        Collided = "collided"
        Stopped = "stopped"


import sys as _sys, types as _types
_eik_mod = _types.ModuleType("xCoreSDK_python.EventInfoKey")
_eik_mod.MoveExecution = _EventInfoKeyPackage.MoveExecution
_eik_mod.LogReporter = _EventInfoKeyPackage.LogReporter
_eik_mod.RlExecution = _EventInfoKeyPackage.RlExecution
_eik_mod.Safety = _EventInfoKeyPackage.Safety
_sys.modules["xCoreSDK_python.EventInfoKey"] = _eik_mod
# also expose the inner classes at top-level for convenience
MoveExecution = _EventInfoKeyPackage.MoveExecution
LogReporter = _EventInfoKeyPackage.LogReporter
RlExecution = _EventInfoKeyPackage.RlExecution
Safety = _EventInfoKeyPackage.Safety


# ---------------------------------------------------------------------------
# Module-level helpers (used by some demos)
# ---------------------------------------------------------------------------

def message(ec: Dict[str, Any]) -> str:
    return ec.get("message", "")


def createErrorCode(code: int = 0, msg: str = "") -> Dict[str, Any]:
    return {"ec": code, "message": msg}


def demo() -> None:
    print("xCoreSDK_python mock demo mode")