"""
robot_control_gui.py
====================

PySide6 desktop GUI for the ROKAE xMate ER7 Pro robot using the
official xCore SDK-Python (v0.7.1).

Features
--------
* Connect / disconnect to the robot (default 192.168.0.160)
* Power on / off, switch between manual and automatic modes
* Real-time display of joint positions (J1-J7) and cartesian pose
* Sliders to set target joint angles (deg), then ``Move!`` to execute
* Spin boxes for cartesian targets (X/Y/Z + Rx/Ry/Rz), then ``MoveL!``
* Live end-effector keypad state (7 buttons)
* Drag teaching, path recording, path management and replay
* Auto-poll loop runs on a QTimer so the UI never blocks
* All SDK calls are wrapped to surface errors via the ``ec`` dict

Running
-------
    cd xCoreSDK-Python/example            # so setup_path resolves correctly
    python ../gui/robot_control_gui.py
"""

from __future__ import annotations

import math
import os
import sys
import time
from collections.abc import Callable
from typing import Optional

# Make sure the real SDK is found (mirrors official setup_path.py)
_HERE = os.path.dirname(os.path.abspath(__file__))
_EXAMPLE_DIR = os.path.abspath(os.path.join(_HERE, "..", "example"))
sys.path.insert(0, _EXAMPLE_DIR)
import setup_path  # noqa: E402,F401

import platform  # noqa: E402

if platform.system() == "Windows":
    from Release.windows import xCoreSDK_python  # noqa: E402
elif platform.system() == "Linux":
    from Release.linux import xCoreSDK_python  # noqa: E402
else:
    raise SystemExit("Unsupported OS")

# Real SDK now imported (PySide6 + .pyd)
from PySide6.QtCore import Qt, QTimer, Slot  # noqa: E402
from PySide6.QtGui import QFont, QPalette, QColor  # noqa: E402
from PySide6.QtWidgets import (  # noqa: E402
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGridLayout, QFormLayout, QGroupBox, QLabel, QLineEdit, QPushButton,
    QSlider, QDoubleSpinBox, QSpinBox, QComboBox, QPlainTextEdit,
    QStatusBar, QTabWidget, QSizePolicy, QMessageBox, QCheckBox,
    QDialog, QDialogButtonBox,
)

try:  # 支持作为 gui 包导入，也支持直接运行本文件
    from .dexhand_panel import DexHandPanel
except ImportError:  # pragma: no cover - direct script import path
    from dexhand_panel import DexHandPanel  # type: ignore


# ---------------------------------------------------------------------------
# Backend wrapper: hides the C-style ec dict and translates to Python errors
# ---------------------------------------------------------------------------

class SDKError(RuntimeError):
    """Raised when a SDK call sets a non-zero error code in ``ec``."""


class RobotBackend:
    """Thin facade around ``xCoreSDK_python`` with a friendlier Python API."""

    def __init__(self, log_fn: Optional[Callable[[str], None]] = None) -> None:
        self.robot: Optional[xCoreSDK_python.BaseRobot] = None
        self.connected = False
        self.robot_type_name = ""
        self._log_fn = log_fn or (lambda _s: None)

    def _log(self, msg: str) -> None:
        self._log_fn(msg)

    # -- connection ------------------------------------------------------

    def connect(self, ip: str, local_ip: str = "") -> None:
        # Try several robot types until one connects cleanly. The SDK
        # refuses to connect if the type doesn't match the controller.
        last_err: Optional[SDKError] = None
        factory_names = (
            "xMateErProRobot",
            "xMateRobot",
            "Cobot_7",
            "Cobot_6",
            "StandardRobot",
            "xMateCr5Robot",
        )
        # Some offline/mock distributions do not expose every robot factory;
        # skip those names while keeping compatibility with the full SDK.
        factories = [
            getattr(xCoreSDK_python, name)
            for name in factory_names
            if hasattr(xCoreSDK_python, name)
        ]
        for factory in factories:
            try:
                self.robot = factory(ip, local_ip)
                ec: dict = {}
                self.robot.connectToRobot(ec)
                self._check(ec, "connectToRobot")
                self.connected = True
                self.robot_type_name = str(
                    getattr(factory, "__name__", type(self.robot).__name__)
                )
                return
            except SDKError as e:
                last_err = e
                try:
                    self.robot.disconnectFromRobot({})
                except Exception:
                    pass
                self.robot = None
                self.robot_type_name = ""
        raise last_err or SDKError("unknown connect failure")

    def disconnect(self) -> None:
        if not self.connected:
            return
        ec: dict = {}
        self.robot.disconnectFromRobot(ec)
        self._check(ec, "disconnectFromRobot")
        self.connected = False
        self.robot_type_name = ""

    # -- power / mode ---------------------------------------------------

    def power_on(self) -> None:
        ec: dict = {}
        self.robot.setPowerState(True, ec)
        self._check(ec, "setPowerState(True)")

    def power_off(self) -> None:
        ec: dict = {}
        self.robot.setPowerState(False, ec)
        self._check(ec, "setPowerState(False)")

    def is_powered_on(self) -> bool:
        ec: dict = {}
        ps = self.robot.powerState(ec)
        self._check(ec, "powerState")
        return ps == xCoreSDK_python.PowerState.on

    def set_mode(self, mode: xCoreSDK_python.OperateMode) -> None:
        ec: dict = {}
        self.robot.setOperateMode(mode, ec)
        self._check(ec, f"setOperateMode({mode.name})")

    def get_mode(self) -> xCoreSDK_python.OperateMode:
        ec: dict = {}
        m = self.robot.operateMode(ec)
        self._check(ec, "operateMode")
        return m

    def set_motion_ctrl(self, mode: xCoreSDK_python.MotionControlMode) -> None:
        ec: dict = {}
        self.robot.setMotionControlMode(mode, ec)
        self._check(ec, f"setMotionControlMode({mode.name})")

    # -- queries ---------------------------------------------------------

    def joint_pos(self) -> list[float]:
        ec: dict = {}
        jp = self.robot.jointPos(ec)
        self._check(ec, "jointPos")
        # jointPos returns [J1..Jn, ExJ1..ExJ6].  We only care about the
        # first 7 (the robot's native DoF).  External axes are ignored.
        return list(jp[:7])

    def cart_pos(self) -> list[float]:
        ec: dict = {}
        cp = self.robot.posture(xCoreSDK_python.CoordinateType.endInRef, ec)
        self._check(ec, "posture")
        return list(cp)

    def operation_state(self) -> xCoreSDK_python.OperationState:
        ec: dict = {}
        st = self.robot.operationState(ec)
        self._check(ec, "operationState")
        return st

    def sdk_version(self) -> str:
        return self.robot.sdkVersion() if self.robot else "n/a"

    def robot_info(self) -> str:
        ec: dict = {}
        info = self.robot.robotInfo(ec)
        self._check(ec, "robotInfo")
        return (f"id={info.id} | ver={info.version} | "
                f"type={info.type} | joints={info.joint_num} | "
                f"mac={info.mac}")

    def keypad_state(self) -> xCoreSDK_python.KeyPadState:
        ec: dict = {}
        ks = self.robot.getKeypadState(ec)
        self._check(ec, "getKeypadState")
        return ks

    # -- motion commands -------------------------------------------------

    def move_abs_j(self, joints_deg: list[float],
                   speed: float = 500.0, zone: float = 10.0) -> None:
        """Send ``MoveAbsJCommand`` then ``moveStart``.  Blocks until done."""
        ec: dict = {}
        # ensure correct motion control mode
        self.set_motion_ctrl(xCoreSDK_python.MotionControlMode.NrtCommandMode)
        joints_rad = [math.radians(d) for d in joints_deg]
        jp = xCoreSDK_python.JointPosition(joints_rad)
        cmd = xCoreSDK_python.MoveAbsJCommand(jp, speed, zone)
        cmd_id = xCoreSDK_python.PyString()
        self.robot.moveAppend([cmd], cmd_id, ec)
        self._check(ec, "moveAppend(MoveAbsJCommand)")
        ec = {}
        self.robot.moveStart(ec)
        self._check(ec, "moveStart")
        self._wait_until_idle()

    def moveL_cart(self, pose: list[float],
                   speed: float = 500.0, zone: float = 10.0) -> None:
        """Send ``MoveLCommand`` for a Cartesian pose."""
        ec: dict = {}
        self.set_motion_ctrl(xCoreSDK_python.MotionControlMode.NrtCommandMode)
        cart = xCoreSDK_python.CartesianPosition(pose)
        cmd = xCoreSDK_python.MoveLCommand(cart, speed, zone)
        cmd_id = xCoreSDK_python.PyString()
        self.robot.moveAppend([cmd], cmd_id, ec)
        self._check(ec, "moveAppend(MoveLCommand)")
        ec = {}
        self.robot.moveStart(ec)
        self._check(ec, "moveStart")
        self._wait_until_idle()

    def stop_motion(self) -> None:
        ec: dict = {}
        self.robot.stop(ec)
        # ``stop`` can race with an in-flight error; don't raise.

    def move_jog(self, joint_index: int, delta_deg: float,
                 speed: float = 30.0, zone: float = 10.0) -> None:
        """Relative jog: move J[1+joint_index] by ``delta_deg`` degrees
        from the current pose.  The pose is re-read right before sending
        so it stays correct even if the GUI's ``last_joint_pos`` is stale.
        """
        # always read the *current* pose - don't trust last_joint_pos
        ec: dict = {}
        jp_cur = self.robot.jointPos(ec)
        self._check(ec, "jointPos")
        cur = list(jp_cur[:7])

        target = list(cur)
        target[joint_index] = target[joint_index] + math.radians(delta_deg)
        self._log(f"Jog J{joint_index + 1} {delta_deg:+.2f}°  "
                  f"({math.degrees(cur[joint_index]):+.2f}° → "
                  f"{math.degrees(target[joint_index]):+.2f}°)")

        # ensure motion control mode
        self.set_motion_ctrl(xCoreSDK_python.MotionControlMode.NrtCommandMode)
        jp_obj = xCoreSDK_python.JointPosition(target)
        cmd = xCoreSDK_python.MoveAbsJCommand(jp_obj, speed, zone)
        cmd_id = xCoreSDK_python.PyString()
        ec = {}
        self.robot.moveAppend([cmd], cmd_id, ec)
        self._check(ec, "moveAppend(MoveAbsJCommand)")
        ec = {}
        self.robot.moveStart(ec)
        self._check(ec, "moveStart")
        self._wait_until_idle()

        # re-read to refresh cache
        ec = {}
        new_jp = self.robot.jointPos(ec)
        if ec.get("ec", 0) == 0:
            self.last_joint_pos = list(new_jp[:7])

    # -- drag teaching / path recording ---------------------------------

    def enable_drag(self) -> None:
        """Prepare the robot for hand-guiding and enable drag mode.

        The SDK example requires the robot to be powered off and in manual
        mode before ``enableDrag`` is called.  Keep that sequence here so the
        GUI cannot accidentally enable drag from automatic mode.
        """
        ec: dict = {}
        self.robot.setPowerState(False, ec)
        self._check(ec, "setPowerState(False) for drag")
        ec = {}
        self.robot.setOperateMode(xCoreSDK_python.OperateMode.manual, ec)
        self._check(ec, "setOperateMode(manual) for drag")
        ec = {}
        self.robot.moveReset(ec)
        self._check(ec, "moveReset before drag")
        ec = {}
        # 1: base-frame drag, 2: freely drag.  This is the combination used
        # by the official drag_example.py and is supported by v0.7.1.
        self.robot.enableDrag(1, 2, ec, False)
        self._check(ec, "enableDrag")

    def disable_drag(self) -> None:
        ec: dict = {}
        self.robot.disableDrag(ec)
        self._check(ec, "disableDrag")

    def start_record_path(self, duration_s: int) -> None:
        ec: dict = {}
        self.robot.startRecordPath(duration_s, ec)
        self._check(ec, "startRecordPath")

    def stop_record_path(self) -> None:
        ec: dict = {}
        self.robot.stopRecordPath(ec)
        self._check(ec, "stopRecordPath")

    def save_record_path(self, name: str) -> None:
        ec: dict = {}
        self.robot.saveRecordPath(name, ec)
        self._check(ec, "saveRecordPath")

    def cancel_record_path(self) -> None:
        ec: dict = {}
        self.robot.cancelRecordPath(ec)
        self._check(ec, "cancelRecordPath")

    def query_path_lists(self) -> list[str]:
        ec: dict = {}
        paths = self.robot.queryPathLists(ec)
        self._check(ec, "queryPathLists")
        return list(paths or [])

    def remove_path(self, name: str) -> None:
        ec: dict = {}
        self.robot.removePath(name, ec)
        self._check(ec, "removePath")

    def replay_path(self, name: str, rate: float = 1.0) -> None:
        """Switch to automatic mode and start replaying a saved path.

        ``moveStart`` is asynchronous in the SDK.  The GUI's existing poll
        timer therefore remains responsive while the controller executes the
        path, and the operation state is used to detect completion.
        """
        ec: dict = {}
        self.robot.disableDrag(ec)
        self._check(ec, "disableDrag before replay")
        ec = {}
        self.robot.setOperateMode(xCoreSDK_python.OperateMode.automatic, ec)
        self._check(ec, "setOperateMode(automatic) for replay")
        ec = {}
        self.robot.setPowerState(True, ec)
        self._check(ec, "setPowerState(True) for replay")
        ec = {}
        self.robot.replayPath(name, rate, ec)
        self._check(ec, "replayPath")
        ec = {}
        self.robot.moveStart(ec)
        self._check(ec, "moveStart for replay")

    # -- helpers ---------------------------------------------------------

    def _wait_until_idle(self, timeout_s: float = 30.0) -> None:
        t0 = time.time()
        ec: dict = {}
        while time.time() - t0 < timeout_s:
            st = self.robot.operationState(ec)
            if st in (xCoreSDK_python.OperationState.idle,
                      xCoreSDK_python.OperationState.unknown):
                return
            time.sleep(0.05)
        raise SDKError(f"motion timed out after {timeout_s}s")

    @staticmethod
    def _check(ec: dict, name: str) -> None:
        # The C++ SDK writes ``ec`` entries with at least these keys:
        #   ec["ec"]: int error code (0 == success)
        #   ec["message"]: str human description
        code = ec.get("ec", 0)
        if code != 0:
            msg = ec.get("message", "<no message>")
            raise SDKError(f"{name} failed (code={code}): {msg}")


# ---------------------------------------------------------------------------
# Helper widgets
# ---------------------------------------------------------------------------

class JointSlider(QWidget):
    """One joint's slider + spinbox + current-value label.

    Soft-limits (lo/hi) default to the safe range of any 7-DoF ROKAE arm.
    """

    def __init__(self, name: str,
                 lo: float = -170.0, hi: float = 170.0,
                 parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._lo, self._hi = lo, hi

        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)

        self.label = QLabel(f"{name}:")
        self.label.setMinimumWidth(40)
        layout.addWidget(self.label)

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(int(lo * 10), int(hi * 10))
        self.slider.setSingleStep(1)
        self.slider.setPageStep(50)
        self.slider.setMinimumWidth(220)
        layout.addWidget(self.slider, 1)

        self.spin = QDoubleSpinBox()
        self.spin.setRange(lo, hi)
        self.spin.setDecimals(2)
        self.spin.setSingleStep(1.0)
        self.spin.setSuffix(" °")
        self.spin.setFixedWidth(90)
        layout.addWidget(self.spin)

        self.current = QLabel("0.00 °")
        self.current.setMinimumWidth(70)
        self.current.setStyleSheet("color:#0066cc;")
        layout.addWidget(self.current)

        # sync slider <-> spinbox
        self.slider.valueChanged.connect(self._on_slider_changed)
        self.spin.valueChanged.connect(self._on_spin_changed)

    def _on_slider_changed(self, v: int) -> None:
        deg = v / 10.0
        if abs(self.spin.value() - deg) > 1e-6:
            self.spin.blockSignals(True)
            self.spin.setValue(deg)
            self.spin.blockSignals(False)

    def _on_spin_changed(self, v: float) -> None:
        slider_v = int(round(v * 10))
        if self.slider.value() != slider_v:
            self.slider.setValue(slider_v)

    def set_value_deg(self, deg: float) -> None:
        # Clamp into the soft-limit range before propagating.
        deg = max(self._lo, min(self._hi, deg))
        self.spin.setValue(deg)
        self.current.setText(f"{deg:+7.2f} °")

    def set_current_display(self, deg: float) -> None:
        """Update the *read-only* "current" readout without touching
        the slider/spinbox (which the user is dragging)."""
        self.current.setText(f"{deg:+7.2f} °")

    def get_value_deg(self) -> float:
        return self.spin.value()

    def reset_to_current(self, current_deg: float) -> None:
        """Snap slider/spinbox back to the *actual* robot pose.

        This is the safety hatch: if the user dragged to a wrong value
        or the robot ended up somewhere unexpected after a move, this
        puts both controls back in sync with reality.
        """
        self.set_value_deg(current_deg)
        self.set_current_display(current_deg)


# ---------------------------------------------------------------------------
# Main window
# ---------------------------------------------------------------------------

class MainWindow(QMainWindow):

    POLL_INTERVAL_MS = 200

    def __init__(self) -> None:
        super().__init__()
        self.backend = RobotBackend(log_fn=lambda msg: self._log(msg))
        self.last_joint_pos: list[float] = [0.0] * 7
        self.last_cart_pos: list[float] = [0.0] * 6
        self.drag_enabled = False
        self.recording = False
        self.record_ready = False
        self.replaying = False

        self.setWindowTitle("ROKAE xMate ER7 Pro – 关节控制面板 (PySide6)")
        self.resize(960, 760)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        root.addWidget(self._build_connection_box())
        root.addWidget(self._build_status_box())

        tabs = QTabWidget()
        tabs.addTab(self._build_drag_tab(),    "拖动录制与回放")
        self.dexhand_panel = DexHandPanel(
            self.backend,
            xCoreSDK_python,
            log_fn=self._log,
        )
        tabs.addTab(self.dexhand_panel, "DexHand021 S 三指灵巧手")
        tabs.addTab(self._build_joint_tab(),   "关节控制 (J1–J7)")
        tabs.addTab(self._build_cart_tab(),    "笛卡尔控制")
        tabs.addTab(self._build_keypad_tab(),  "末端按键")
        tabs.addTab(self._build_log_tab(),     "日志")
        root.addWidget(tabs, 1)

        self.setStatusBar(QStatusBar(self))
        self._log("就绪。请在右上角填写 IP 并点击「连接」。")

        self.poll_timer = QTimer(self)
        self.poll_timer.setInterval(self.POLL_INTERVAL_MS)
        self.poll_timer.timeout.connect(self._poll_state)

    # -- UI sections -----------------------------------------------------

    def _build_connection_box(self) -> QGroupBox:
        box = QGroupBox("机器人连接")
        layout = QHBoxLayout(box)

        layout.addWidget(QLabel("机器人 IP:"))
        self.ip_edit = QLineEdit("192.168.0.160")
        self.ip_edit.setFixedWidth(140)
        layout.addWidget(self.ip_edit)

        layout.addWidget(QLabel("本机 IP:"))
        self.local_ip_edit = QLineEdit("192.168.0.11")
        self.local_ip_edit.setFixedWidth(140)
        layout.addWidget(self.local_ip_edit)

        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        layout.addWidget(self.connect_btn)

        self.disconnect_btn = QPushButton("断开")
        self.disconnect_btn.clicked.connect(self._on_disconnect_clicked)
        self.disconnect_btn.setEnabled(False)
        layout.addWidget(self.disconnect_btn)

        layout.addStretch(1)

        self.status_label = QLabel("● 未连接")
        self.status_label.setStyleSheet("color:#cc0000; font-weight:bold;")
        layout.addWidget(self.status_label)

        return box

    def _build_status_box(self) -> QGroupBox:
        box = QGroupBox("实时状态")
        grid = QGridLayout(box)

        self.power_state_lbl  = QLabel("—")
        self.mode_lbl         = QLabel("—")
        self.op_state_lbl     = QLabel("—")
        self.info_lbl         = QLabel("—")
        self.sdk_ver_lbl      = QLabel("—")

        grid.addWidget(QLabel("上下电:"),    0, 0); grid.addWidget(self.power_state_lbl,  0, 1)
        grid.addWidget(QLabel("操作模式:"),  0, 2); grid.addWidget(self.mode_lbl,         0, 3)
        grid.addWidget(QLabel("运行状态:"),  0, 4); grid.addWidget(self.op_state_lbl,     0, 5)
        grid.addWidget(QLabel("SDK 版本:"),  1, 0); grid.addWidget(self.sdk_ver_lbl,      1, 1, 1, 2)
        grid.addWidget(QLabel("机器人:"),    1, 2); grid.addWidget(self.info_lbl,         1, 3, 1, 3)

        grid.setColumnStretch(5, 1)
        return box

    def _build_joint_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        # current pose row
        layout.addWidget(QLabel("当前关节角度 (实时):"))
        self.joint_display = QPlainTextEdit()
        self.joint_display.setReadOnly(True)
        self.joint_display.setMaximumHeight(90)
        font = QFont("Consolas")
        font.setStyleHint(QFont.Monospace)
        self.joint_display.setFont(font)
        layout.addWidget(self.joint_display)

        # sliders
        sliders_box = QGroupBox("目标关节角度 (°) — 调整后点击「运动到目标」")
        sliders_layout = QVBoxLayout(sliders_box)
        self.joint_sliders: list[JointSlider] = []
        for i in range(1, 8):
            js = JointSlider(f"J{i}")
            sliders_layout.addWidget(js)
            self.joint_sliders.append(js)
        layout.addWidget(sliders_box)

        # speed / zone
        params_box = QGroupBox("运动参数")
        params = QFormLayout(params_box)
        self.speed_spin = QDoubleSpinBox()
        self.speed_spin.setRange(1.0, 5000.0)
        self.speed_spin.setValue(100.0)        # 100 mm/s or 100 deg/s = safe default
        self.speed_spin.setSuffix(" (mm/s 或 deg/s)")
        params.addRow("速度", self.speed_spin)

        self.zone_spin = QDoubleSpinBox()
        self.zone_spin.setRange(0.0, 100.0)
        self.zone_spin.setValue(10.0)
        self.zone_spin.setSuffix(" mm")
        params.addRow("转弯区", self.zone_spin)
        layout.addWidget(params_box)

        # ---- Jog (relative motion) section ----
        jog_box = QGroupBox("Jog 增量运动 (相对当前位置)")
        jog_form = QFormLayout(jog_box)
        self.jog_step = QDoubleSpinBox()
        self.jog_step.setRange(0.1, 30.0)
        self.jog_step.setDecimals(2)
        self.jog_step.setSingleStep(1.0)
        self.jog_step.setValue(5.0)
        self.jog_step.setSuffix(" °")
        jog_form.addRow("每步增量", self.jog_step)
        layout.addWidget(jog_box)

        jog_grid = QGridLayout()
        for i in range(7):
            minus = QPushButton(f"J{i+1} −")
            plus  = QPushButton(f"J{i+1} +")
            minus.setFixedWidth(70)
            plus.setFixedWidth(70)
            minus.clicked.connect(lambda _checked=False, idx=i: self._on_jog_clicked(idx, -1.0))
            plus.clicked.connect(lambda _checked=False, idx=i: self._on_jog_clicked(idx, +1.0))
            minus.setEnabled(False); plus.setEnabled(False)
            jog_grid.addWidget(minus, i, 0)
            jog_grid.addWidget(plus,  i, 1)
            # remember them so _after_connect can enable
            if not hasattr(self, "jog_minus_btns"):
                self.jog_minus_btns = []
                self.jog_plus_btns = []
            self.jog_minus_btns.append(minus)
            self.jog_plus_btns.append(plus)
        layout.addLayout(jog_grid)

        # ---- Absolute-move buttons ----
        btn_row = QHBoxLayout()
        self.move_joint_btn = QPushButton("运动到目标 (MoveAbsJ)")
        self.move_joint_btn.setStyleSheet("font-weight:bold; padding:8px;")
        self.move_joint_btn.clicked.connect(self._on_move_joint_clicked)
        self.move_joint_btn.setEnabled(False)
        btn_row.addWidget(self.move_joint_btn)

        self.get_current_joint_btn = QPushButton("用当前关节角度填充滑块")
        self.get_current_joint_btn.clicked.connect(self._on_fill_joints_clicked)
        self.get_current_joint_btn.setEnabled(False)
        btn_row.addWidget(self.get_current_joint_btn)

        self.zero_btn = QPushButton("全部归零")
        self.zero_btn.clicked.connect(self._on_zero_joints_clicked)
        self.zero_btn.setEnabled(False)
        btn_row.addWidget(self.zero_btn)

        btn_row.addStretch(1)
        layout.addLayout(btn_row)
        layout.addStretch(1)
        return tab

    def _build_cart_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        layout.addWidget(QLabel("当前笛卡尔位姿 (实时, endInRef):"))
        self.cart_display = QPlainTextEdit()
        self.cart_display.setReadOnly(True)
        self.cart_display.setMaximumHeight(70)
        font = QFont("Consolas"); font.setStyleHint(QFont.Monospace)
        self.cart_display.setFont(font)
        layout.addWidget(self.cart_display)

        target_box = QGroupBox("目标笛卡尔位姿")
        form = QFormLayout(target_box)
        self.cart_spins: list[QDoubleSpinBox] = []
        labels = ["X (m)", "Y (m)", "Z (m)", "Rx (rad)", "Ry (rad)", "Rz (rad)"]
        ranges = [(-2.0, 2.0), (-2.0, 2.0), (-2.0, 2.0),
                  (-math.pi, math.pi), (-math.pi, math.pi), (-math.pi, math.pi)]
        for lbl, (lo, hi) in zip(labels, ranges):
            sp = QDoubleSpinBox()
            sp.setRange(lo, hi)
            sp.setDecimals(4)
            sp.setSingleStep(0.01)
            self.cart_spins.append(sp)
            form.addRow(lbl, sp)
        layout.addWidget(target_box)

        btn_row = QHBoxLayout()
        self.moveL_btn = QPushButton("MoveL 到目标")
        self.moveL_btn.setStyleSheet("font-weight:bold; padding:8px;")
        self.moveL_btn.clicked.connect(self._on_moveL_clicked)
        self.moveL_btn.setEnabled(False)
        btn_row.addWidget(self.moveL_btn)

        self.get_current_cart_btn = QPushButton("用当前位姿填充")
        self.get_current_cart_btn.clicked.connect(self._on_fill_cart_clicked)
        self.get_current_cart_btn.setEnabled(False)
        btn_row.addWidget(self.get_current_cart_btn)

        btn_row.addStretch(1)
        layout.addLayout(btn_row)
        layout.addStretch(1)
        return tab

    def _build_drag_tab(self) -> QWidget:
        """Build the hand-guiding, path-recording and replay controls."""
        tab = QWidget()
        layout = QVBoxLayout(tab)

        safety = QLabel(
            "拖动模式会让机器人下电并切换到手动模式。"
            "开启前请确认机器人工作区域安全。"
        )
        safety.setWordWrap(True)
        safety.setStyleSheet("color:#aa5500; font-weight:bold;")
        layout.addWidget(safety)

        state_box = QGroupBox("拖动状态")
        state_layout = QHBoxLayout(state_box)
        state_layout.addWidget(QLabel("状态："))
        self.drag_state_lbl = QLabel("未开启")
        self.drag_state_lbl.setStyleSheet("color:#444; font-weight:bold;")
        state_layout.addWidget(self.drag_state_lbl)
        state_layout.addStretch(1)

        self.enable_drag_btn = QPushButton("开启拖动")
        self.enable_drag_btn.clicked.connect(self._on_enable_drag_clicked)
        self.enable_drag_btn.setEnabled(False)
        state_layout.addWidget(self.enable_drag_btn)

        self.disable_drag_btn = QPushButton("关闭拖动")
        self.disable_drag_btn.clicked.connect(self._on_disable_drag_clicked)
        self.disable_drag_btn.setEnabled(False)
        state_layout.addWidget(self.disable_drag_btn)
        layout.addWidget(state_box)

        record_box = QGroupBox("路径录制")
        record_form = QFormLayout(record_box)
        self.record_duration_spin = QSpinBox()
        self.record_duration_spin.setRange(1, 1800)
        self.record_duration_spin.setValue(60)
        self.record_duration_spin.setSuffix(" 秒")
        record_form.addRow("录制时长", self.record_duration_spin)

        record_buttons = QHBoxLayout()
        self.start_record_btn = QPushButton("开始录制")
        self.start_record_btn.clicked.connect(self._on_start_record_clicked)
        self.start_record_btn.setEnabled(False)
        record_buttons.addWidget(self.start_record_btn)

        self.stop_record_btn = QPushButton("停止录制")
        self.stop_record_btn.clicked.connect(self._on_stop_record_clicked)
        self.stop_record_btn.setEnabled(False)
        record_buttons.addWidget(self.stop_record_btn)

        self.cancel_record_btn = QPushButton("取消录制")
        self.cancel_record_btn.clicked.connect(self._on_cancel_record_clicked)
        self.cancel_record_btn.setEnabled(False)
        record_buttons.addWidget(self.cancel_record_btn)
        record_buttons.addStretch(1)
        record_form.addRow(record_buttons)
        layout.addWidget(record_box)

        path_box = QGroupBox("已保存路径")
        path_form = QFormLayout(path_box)
        self.path_name_edit = QLineEdit("path_1")
        self.path_name_edit.setPlaceholderText("路径名称（保存在机器人控制器中）")
        self.path_name_edit.textChanged.connect(self._update_drag_controls)
        path_form.addRow("保存名称", self.path_name_edit)

        path_buttons = QHBoxLayout()
        self.save_path_btn = QPushButton("保存录制路径")
        self.save_path_btn.clicked.connect(self._on_save_path_clicked)
        self.save_path_btn.setEnabled(False)
        path_buttons.addWidget(self.save_path_btn)

        self.refresh_paths_btn = QPushButton("刷新路径列表")
        self.refresh_paths_btn.clicked.connect(self._on_refresh_paths_clicked)
        self.refresh_paths_btn.setEnabled(False)
        path_buttons.addWidget(self.refresh_paths_btn)
        path_buttons.addStretch(1)
        path_form.addRow(path_buttons)

        self.path_combo = QComboBox()
        self.path_combo.setMinimumWidth(260)
        path_form.addRow("路径", self.path_combo)

        replay_row = QHBoxLayout()
        self.replay_rate_spin = QDoubleSpinBox()
        self.replay_rate_spin.setRange(0.1, 3.0)
        self.replay_rate_spin.setDecimals(2)
        self.replay_rate_spin.setSingleStep(0.1)
        self.replay_rate_spin.setValue(1.0)
        self.replay_rate_spin.setSuffix(" 倍")
        replay_row.addWidget(QLabel("回放速度"))
        replay_row.addWidget(self.replay_rate_spin)

        self.replay_path_btn = QPushButton("回放选中路径")
        self.replay_path_btn.clicked.connect(self._on_replay_path_clicked)
        self.replay_path_btn.setEnabled(False)
        replay_row.addWidget(self.replay_path_btn)

        self.stop_replay_btn = QPushButton("停止回放")
        self.stop_replay_btn.clicked.connect(self._on_stop_replay_clicked)
        self.stop_replay_btn.setEnabled(False)
        replay_row.addWidget(self.stop_replay_btn)

        self.delete_path_btn = QPushButton("删除选中路径")
        self.delete_path_btn.clicked.connect(self._on_delete_path_clicked)
        self.delete_path_btn.setEnabled(False)
        replay_row.addWidget(self.delete_path_btn)
        replay_row.addStretch(1)
        path_form.addRow(replay_row)
        layout.addWidget(path_box)

        layout.addStretch(1)
        return tab

    def _build_keypad_tab(self) -> QWidget:
        tab = QWidget()
        layout = QVBoxLayout(tab)

        layout.addWidget(QLabel("末端按键 (key1 ~ key7):"))
        self.keypad_grid = QGridLayout()
        self.keypad_leds: list[QLabel] = []
        for i in range(1, 8):
            led = QLabel("OFF")
            led.setAlignment(Qt.AlignCenter)
            led.setFixedHeight(40)
            led.setStyleSheet("background:#444; color:#fff; "
                              "border-radius:8px; font-weight:bold;")
            self.keypad_leds.append(led)
            self.keypad_grid.addWidget(QLabel(f"key {i}:"), i - 1, 0)
            self.keypad_grid.addWidget(led, i - 1, 1)
        layout.addLayout(self.keypad_grid)

        self.read_keypad_btn = QPushButton("刷新按键状态")
        self.read_keypad_btn.clicked.connect(self._on_read_keypad_clicked)
        self.read_keypad_btn.setEnabled(False)
        layout.addWidget(self.read_keypad_btn)

        layout.addStretch(1)
        return tab

    def _build_log_tab(self) -> QGroupBox:
        box = QGroupBox("日志")
        v = QVBoxLayout(box)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(2000)
        v.addWidget(self.log_view)
        clear_btn = QPushButton("清空日志")
        clear_btn.clicked.connect(lambda: self.log_view.clear())
        v.addWidget(clear_btn)
        return box

    # -- slots: connection ----------------------------------------------

    @Slot()
    def _on_connect_clicked(self) -> None:
        ip = self.ip_edit.text().strip()
        local_ip = self.local_ip_edit.text().strip()
        self._log(f"正在连接 {ip} (本机 {local_ip}) …")
        try:
            self.backend.connect(ip, local_ip)
        except SDKError as e:
            self._log(f"连接失败: {e}")
            return
        self._log(f"已连接机器人 SDK 类型：{self.backend.robot_type_name}")
        self._log("连接成功")
        self._after_connect()

    @Slot()
    def _on_disconnect_clicked(self) -> None:
        self._cleanup_drag_state()
        self.dexhand_panel.set_robot_connected(False)
        try:
            self.backend.disconnect()
        except SDKError as e:
            self._log(f"断开出错: {e}")
        self._log("已断开")
        self._after_disconnect()

    def _cleanup_drag_state(self) -> None:
        """Stop active path work before disconnecting or closing the GUI."""
        if not self.backend.connected:
            self.drag_enabled = False
            self.recording = False
            self.record_ready = False
            self.replaying = False
            return
        try:
            if self.replaying:
                self.backend.stop_motion()
            if self.recording:
                self.backend.cancel_record_path()
            if self.drag_enabled:
                self.backend.disable_drag()
        except SDKError as e:
            self._log(f"Cleanup before disconnect failed: {e}")
        finally:
            self.drag_enabled = False
            self.recording = False
            self.record_ready = False
            self.replaying = False

    def _after_connect(self) -> None:
        self.dexhand_panel.set_robot_connected(True)
        self.connect_btn.setEnabled(False)
        self.disconnect_btn.setEnabled(True)
        self.ip_edit.setEnabled(False)
        self.local_ip_edit.setEnabled(False)
        self.status_label.setText("● 已连接")
        self.status_label.setStyleSheet("color:#00aa00; font-weight:bold;")
        for w in (self.move_joint_btn, self.get_current_joint_btn,
                  self.zero_btn, self.moveL_btn, self.get_current_cart_btn,
                  self.read_keypad_btn):
            w.setEnabled(True)
        # also enable the Jog +/- buttons
        if hasattr(self, "jog_minus_btns"):
            for b in self.jog_minus_btns:
                b.setEnabled(True)
        if hasattr(self, "jog_plus_btns"):
            for b in self.jog_plus_btns:
                b.setEnabled(True)
        self._update_drag_controls()
        self._on_refresh_paths_clicked()

        # Fetch static info
        try:
            self.sdk_ver_lbl.setText(self.backend.sdk_version())
            self.info_lbl.setText(self.backend.robot_info())
        except SDKError as e:
            self._log(f"读取机器人信息失败: {e}")

        self.poll_timer.start()

    def _after_disconnect(self) -> None:
        self.dexhand_panel.set_robot_connected(False)
        self.poll_timer.stop()
        self.connect_btn.setEnabled(True)
        self.disconnect_btn.setEnabled(False)
        self.ip_edit.setEnabled(True)
        self.local_ip_edit.setEnabled(True)
        self.status_label.setText("● 未连接")
        self.status_label.setStyleSheet("color:#cc0000; font-weight:bold;")
        for w in (self.move_joint_btn, self.get_current_joint_btn,
                  self.zero_btn, self.moveL_btn, self.get_current_cart_btn,
                  self.read_keypad_btn):
            w.setEnabled(False)
        # also disable jog buttons
        if hasattr(self, "jog_minus_btns"):
            for b in self.jog_minus_btns:
                b.setEnabled(False)
        if hasattr(self, "jog_plus_btns"):
            for b in self.jog_plus_btns:
                b.setEnabled(False)
        self.drag_enabled = False
        self.recording = False
        self.record_ready = False
        self.replaying = False
        self._update_drag_controls()
        for lbl in (self.power_state_lbl, self.mode_lbl, self.op_state_lbl,
                    self.sdk_ver_lbl, self.info_lbl):
            lbl.setText("—")
        self.joint_display.clear()
        self.cart_display.clear()

    # -- slots: power / mode --------------------------------------------

    def _build_power_mode_widgets(self):
        # inline section under "实时状态" — not separate; use toolbar instead
        pass

    def _ensure_powered_and_mode(self, mode: xCoreSDK_python.OperateMode) -> None:
        """Make sure the robot is powered on and in the desired mode.

        Raises SDKError if powering on fails (e.g. e-stop pressed) so the
        caller can abort the motion instead of silently enqueueing a
        command that the controller will reject.
        """
        if not self.backend.is_powered_on():
            self._log("上电中…")
            self.backend.power_on()              # raises on -3 / e-stop etc.
            self._log("已上电")
        cur_mode = self.backend.get_mode()
        if cur_mode != mode:
            self._log(f"切换到 {mode.name} 模式…")
            self.backend.set_mode(mode)
            self._log(f"已切换到 {mode.name} 模式")

    # -- slots: joint control -------------------------------------------

    @Slot()
    def _on_fill_joints_clicked(self) -> None:
        jp = self.last_joint_pos
        for slider, rad in zip(self.joint_sliders, jp):
            slider.set_value_deg(math.degrees(rad))

    @Slot()
    def _on_zero_joints_clicked(self) -> None:
        for slider in self.joint_sliders:
            slider.set_value_deg(0.0)

    @Slot()
    def _on_jog_clicked(self, joint_index: int, sign: float) -> None:
        """Incremental jog: move joint by sign * self.jog_step degrees."""
        step = self.jog_step.value()
        delta = sign * step
        try:
            self._ensure_powered_and_mode(xCoreSDK_python.OperateMode.automatic)
            self.backend.move_jog(joint_index, delta,
                                  speed=self.speed_spin.value(),
                                  zone=self.zone_spin.value())
        except SDKError as e:
            self._log(f"Jog J{joint_index + 1} 失败: {e}")
            QMessageBox.warning(self, "Jog 失败", str(e))
            return
        # sync slider display to the new pose
        try:
            self._poll_state()
            for slider, rad in zip(self.joint_sliders, self.last_joint_pos):
                slider.reset_to_current(math.degrees(rad))
        except SDKError:
            pass

    @Slot()
    def _on_move_joint_clicked(self) -> None:
        targets = [s.get_value_deg() for s in self.joint_sliders]
        current = [math.degrees(r) for r in self.last_joint_pos]
        deltas = [t - c for t, c in zip(targets, current)]
        speed = self.speed_spin.value()
        zone = self.zone_spin.value()

        # Safety: refuse to send a move if any delta exceeds the absolute
        # safety threshold (likely the user dragged the wrong slider or
        # there is a unit confusion somewhere).
        ABS_LIMIT_DEG = 170.0
        for i, d in enumerate(deltas):
            if abs(d) > ABS_LIMIT_DEG:
                QMessageBox.critical(
                    self, "运动幅度过大",
                    f"J{i+1} 目标角度 {targets[i]:+.2f}° 与当前 {current[i]:+.2f}° "
                    f"相差 {d:+.2f}°，超过安全阈值 ±{ABS_LIMIT_DEG}°。\n\n"
                    "可能原因：\n"
                    "  • 拖错了关节的滑块\n"
                    "  • 滑块值与实际传给 SDK 的角度不一致\n"
                    "  • 单位错误（应是「度」不是「弧度」）\n\n"
                    "建议先点「用当前关节填充」把滑块归位。")
                return

        # Safety: confirm with the user (default No).  This prevents
        # accidental "click the wrong button" catastrophes.
        if not self._confirm_move(
                title="确认运动 (MoveAbsJ)",
                body=("即将运动到以下关节目标:\n\n"
                      + "\n".join(
                          f"  J{i+1}: {current[i]:+7.2f}° → {targets[i]:+7.2f}°  "
                          f"(Δ {deltas[i]:+6.2f}°)"
                          for i in range(7))
                      + f"\n\n速度 speed={speed}\n转弯区 zone={zone}")):
            return

        self._log(f"MoveAbsJ → {[f'{t:+6.2f}' for t in targets]} "
                  f"speed={speed} zone={zone}")
        try:
            self._ensure_powered_and_mode(xCoreSDK_python.OperateMode.automatic)
            self.backend.move_abs_j(targets, speed=speed, zone=zone)
        except SDKError as e:
            self._log(f"MoveAbsJ 失败: {e}")
            QMessageBox.warning(self, "MoveAbsJ 失败", str(e))
            return
        self._log("MoveAbsJ 完成")

        # CRITICAL: snap sliders back to the actual pose so the user can
        # see exactly where the robot ended up.  This is the fix for the
        # "J2 keeps going positive" report: after each move, the GUI
        # resyncs to reality instead of trusting stale targets.
        try:
            self._poll_state()
            for slider, rad in zip(self.joint_sliders, self.last_joint_pos):
                slider.reset_to_current(math.degrees(rad))
        except SDKError:
            pass

    def _confirm_move(self, title: str, body: str) -> bool:
        """Show a Yes/No dialog.  Returns True iff the user picked Yes."""
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Question)
        box.setWindowTitle(title)
        box.setText(body)
        box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        box.setDefaultButton(QMessageBox.No)
        return box.exec() == QMessageBox.Yes

    # -- slots: cartesian control ---------------------------------------

    @Slot()
    def _on_fill_cart_clicked(self) -> None:
        for sp, val in zip(self.cart_spins, self.last_cart_pos):
            sp.setValue(val)

    @Slot()
    def _on_moveL_clicked(self) -> None:
        target = [sp.value() for sp in self.cart_spins]
        speed = self.speed_spin.value()
        zone = self.zone_spin.value()

        # Safety: confirm with the user (default No).
        body = ("即将执行 MoveL 到:\n\n"
                + "  X = {:+.4f} m   Y = {:+.4f} m   Z = {:+.4f} m\n".format(*target[:3])
                + "  Rx = {:+.4f} rad   Ry = {:+.4f} rad   Rz = {:+.4f} rad\n\n".format(*target[3:])
                + f"速度 speed={speed}\n转弯区 zone={zone}")
        if not self._confirm_move("确认运动 (MoveL)", body):
            return

        self._log(f"MoveL → {[f'{v:+.4f}' for v in target]} "
                  f"speed={speed} zone={zone}")
        try:
            self._ensure_powered_and_mode(xCoreSDK_python.OperateMode.automatic)
            self.backend.moveL_cart(target, speed=speed, zone=zone)
        except SDKError as e:
            self._log(f"MoveL 失败: {e}")
            QMessageBox.warning(self, "MoveL 失败", str(e))
            return
        self._log("MoveL 完成")

        # resync GUI to actual robot pose
        try:
            self._poll_state()
            for sp, v in zip(self.cart_spins, self.last_cart_pos):
                sp.blockSignals(True)
                sp.setValue(v)
                sp.blockSignals(False)
        except SDKError:
            pass

    # -- slots: drag teaching / path recording ---------------------------

    def _update_drag_controls(self, *_args) -> None:
        """Keep drag/path buttons consistent with the current operation."""
        if not hasattr(self, "enable_drag_btn"):
            return
        connected = self.backend.connected
        busy = self.recording or self.replaying
        has_path = self.path_combo.count() > 0
        has_name = bool(self.path_name_edit.text().strip())

        self.enable_drag_btn.setEnabled(
            connected and not self.drag_enabled and not busy
        )
        self.disable_drag_btn.setEnabled(
            connected and self.drag_enabled and not busy
        )
        self.start_record_btn.setEnabled(
            connected and self.drag_enabled and not self.recording
            and not self.replaying
        )
        self.stop_record_btn.setEnabled(connected and self.recording)
        self.cancel_record_btn.setEnabled(connected and self.recording)
        self.save_path_btn.setEnabled(
            connected and self.record_ready and not busy and has_name
        )
        self.refresh_paths_btn.setEnabled(connected and not busy)
        self.replay_path_btn.setEnabled(connected and has_path and not busy)
        self.delete_path_btn.setEnabled(connected and has_path and not busy)
        self.stop_replay_btn.setEnabled(connected and self.replaying)

        if self.replaying:
            text, color = "回放中", "#aa5500"
        elif self.recording:
            text, color = "录制中", "#cc0000"
        elif self.drag_enabled:
            text, color = "拖动已开启", "#0066cc"
        else:
            text, color = "未开启", "#444"
        self.drag_state_lbl.setText(text)
        self.drag_state_lbl.setStyleSheet(
            f"color:{color}; font-weight:bold;"
        )

    @Slot()
    def _on_enable_drag_clicked(self) -> None:
        if not self._confirm_move(
                "确认开启拖动",
                "机器人将下电并切换到手动模式。\n\n"
                "请确认机器人工作区域安全，是否开启拖动？"):
            return
        try:
            self.backend.enable_drag()
        except SDKError as e:
            self._log(f"开启拖动失败：{e}")
            QMessageBox.warning(self, "开启拖动失败", str(e))
            return
        self.drag_enabled = True
        self._log("拖动已开启，可以手动移动机器人")
        self._update_drag_controls()

    @Slot()
    def _on_disable_drag_clicked(self) -> None:
        try:
            self.backend.disable_drag()
        except SDKError as e:
            self._log(f"关闭拖动失败：{e}")
            QMessageBox.warning(self, "关闭拖动失败", str(e))
            return
        self.drag_enabled = False
        self._log("拖动已关闭")
        self._update_drag_controls()

    @Slot()
    def _on_start_record_clicked(self) -> None:
        try:
            self.backend.start_record_path(self.record_duration_spin.value())
        except SDKError as e:
            self._log(f"开始录制失败：{e}")
            QMessageBox.warning(self, "开始录制失败", str(e))
            return
        self.recording = True
        self.record_ready = False
        self._log(
            f"路径录制已开始（预留时长="
            f"{self.record_duration_spin.value()} 秒）"
        )
        self._update_drag_controls()

    @Slot()
    def _on_stop_record_clicked(self) -> None:
        try:
            self.backend.stop_record_path()
        except SDKError as e:
            self._log(f"停止录制失败：{e}")
            QMessageBox.warning(self, "停止录制失败", str(e))
            return
        self.recording = False
        self.record_ready = True
        self._log("路径录制已停止，请输入名称后保存")
        self._update_drag_controls()

    @Slot()
    def _on_cancel_record_clicked(self) -> None:
        try:
            self.backend.cancel_record_path()
        except SDKError as e:
            self._log(f"取消录制失败：{e}")
            QMessageBox.warning(self, "取消录制失败", str(e))
            return
        self.recording = False
        self.record_ready = False
        self._log("路径录制已取消")
        self._update_drag_controls()

    @Slot()
    def _on_save_path_clicked(self) -> None:
        name = self.path_name_edit.text().strip()
        if not name:
            QMessageBox.warning(self, "路径名称无效", "请先输入路径名称。")
            return
        try:
            self.backend.save_record_path(name)
        except SDKError as e:
            self._log(f"保存路径失败：{e}")
            QMessageBox.warning(self, "保存路径失败", str(e))
            return
        self.record_ready = False
        self._log(f"路径已保存：{name}")
        self._on_refresh_paths_clicked()
        self._update_drag_controls()

    @Slot()
    def _on_refresh_paths_clicked(self) -> None:
        if not self.backend.connected:
            return
        try:
            paths = self.backend.query_path_lists()
        except SDKError as e:
            self._log(f"查询路径失败：{e}")
            return
        current = self.path_combo.currentText()
        self.path_combo.blockSignals(True)
        self.path_combo.clear()
        self.path_combo.addItems(paths)
        if current in paths:
            self.path_combo.setCurrentText(current)
        self.path_combo.blockSignals(False)
        self._log(f"已加载 {len(paths)} 条已保存路径")
        self._update_drag_controls()

    @Slot()
    def _on_delete_path_clicked(self) -> None:
        name = self.path_combo.currentText().strip()
        if not name:
            return
        if not self._confirm_move(
                "删除已保存路径",
                f"确定从机器人控制器中删除路径“{name}”吗？"):
            return
        try:
            self.backend.remove_path(name)
        except SDKError as e:
            self._log(f"删除路径失败：{e}")
            QMessageBox.warning(self, "删除路径失败", str(e))
            return
        self._log(f"路径已删除：{name}")
        self._on_refresh_paths_clicked()

    @Slot()
    def _on_replay_path_clicked(self) -> None:
        name = self.path_combo.currentText().strip()
        if not name:
            return
        rate = self.replay_rate_spin.value()
        if not self._confirm_move(
                "确认路径回放",
                f"确定以 {rate:.2f} 倍速度回放路径“{name}”吗？\n\n"
                "机器人将切换到自动模式并上电。"):
            return
        try:
            self.backend.replay_path(name, rate)
        except SDKError as e:
            self._log(f"路径回放失败：{e}")
            QMessageBox.warning(self, "路径回放失败", str(e))
            return
        self.drag_enabled = False
        self.recording = False
        self.record_ready = False
        self.replaying = True
        self._log(f"路径回放已开始：{name}（{rate:.2f} 倍）")
        self._update_drag_controls()

    @Slot()
    def _on_stop_replay_clicked(self) -> None:
        try:
            self.backend.stop_motion()
        except SDKError as e:
            self._log(f"停止回放失败：{e}")
            QMessageBox.warning(self, "停止回放失败", str(e))
            return
        self.replaying = False
        self._log("路径回放已停止")
        self._update_drag_controls()

    # -- slots: keypad ---------------------------------------------------

    @Slot()
    def _on_read_keypad_clicked(self) -> None:
        try:
            self._update_keypad_display()
        except SDKError as e:
            self._log(f"读取按键失败: {e}")

    # -- polling ---------------------------------------------------------

    def _poll_state(self) -> None:
        if not self.backend.connected:
            return
        try:
            self.last_joint_pos = self.backend.joint_pos()
            self.last_cart_pos = self.backend.cart_pos()
            op_state = self.backend.operation_state()
        except SDKError as e:
            self._log(f"轮询失败: {e}")
            return

        # update displays
        jp_str = "  ".join(f"J{i+1}={math.degrees(r):+7.2f}°"
                            for i, r in enumerate(self.last_joint_pos))
        self.joint_display.setPlainText(jp_str)
        cp_str = "  ".join(
            f"{name}={val:+.4f}" for name, val in
            zip(("X", "Y", "Z", "Rx", "Ry", "Rz"), self.last_cart_pos))
        self.cart_display.setPlainText(cp_str)

        # update sliders' "current" readout without changing their value
        for slider, rad in zip(self.joint_sliders, self.last_joint_pos):
            slider.set_current_display(math.degrees(rad))

        # status labels
        self.power_state_lbl.setText("已上电" if self.backend.is_powered_on() else "已下电")
        mode_name = self.backend.get_mode().name
        self.mode_lbl.setText({
            "manual": "手动",
            "automatic": "自动",
            "unknown": "未知",
        }.get(mode_name, mode_name))
        state_name = op_state.name
        self.op_state_lbl.setText({
            "idle": "空闲",
            "jog": "点动待机",
            "rtControlling": "实时控制中",
            "drag": "拖动已开启",
            "rlProgram": "RL 工程运行中",
            "demo": "Demo 演示中",
            "dynamicIdentify": "动力学辨识中",
            "frictionIdentify": "摩擦力辨识中",
            "loadIdentify": "负载辨识中",
            "moving": "运动中",
            "jogging": "Jog 中",
            "unknown": "未知",
        }.get(state_name, state_name))

        if self.replaying and op_state in (
                xCoreSDK_python.OperationState.idle,
                xCoreSDK_python.OperationState.unknown):
            self.replaying = False
            self._log("路径回放已完成")
            self._update_drag_controls()

        # keypad
        self._update_keypad_display()

    def _update_keypad_display(self) -> None:
        ks = self.backend.keypad_state()
        states = [ks.key1_state, ks.key2_state, ks.key3_state,
                  ks.key4_state, ks.key5_state, ks.key6_state, ks.key7_state]
        for led, on in zip(self.keypad_leds, states):
            led.setText("开" if on else "关")
            color = "#00aa00" if on else "#444444"
            led.setStyleSheet(f"background:{color}; color:#fff; "
                              "border-radius:8px; font-weight:bold;")

    # -- helpers ---------------------------------------------------------

    def _log(self, msg: str) -> None:
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {msg}"
        self.log_view.appendPlainText(line)
        self.statusBar().showMessage(msg, 5000)

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self.dexhand_panel.close_hand()
        if self.backend.connected:
            self._cleanup_drag_state()
            try:
                self.backend.disconnect()
            except Exception:
                pass
        super().closeEvent(event)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main() -> None:
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
