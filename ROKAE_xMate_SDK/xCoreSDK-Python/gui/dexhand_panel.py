"""DexHand021 S 中文控制页。"""

from __future__ import annotations

import json
import time
from typing import Any, Callable, Optional

from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFrame,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHeaderView,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QScrollArea,
    QSlider,
    QSpinBox,
    QSizePolicy,
    QTableWidget,
    QTableWidgetItem,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

try:  # 支持作为 gui 包导入，也支持官方示例目录下直接运行脚本。
    from .dexhand_backend import (
        AXIS_IDS,
        AXIS_LABELS,
        FINGER_IDS,
        DeviceInfo,
        DexHand021SBackend,
        DexHandError,
        HandStatus,
        SDK_MODE_TO_RTU_MOTOR_MODE,
    )
except ImportError:  # pragma: no cover - direct script import path
    from dexhand_backend import (  # type: ignore
        AXIS_IDS,
        AXIS_LABELS,
        FINGER_IDS,
        DeviceInfo,
        DexHand021SBackend,
        DexHandError,
        HandStatus,
        SDK_MODE_TO_RTU_MOTOR_MODE,
    )


class DexHandPanel(QWidget):
    """三指 DexHand021 S 控制界面。

    支持：
    * P1/P2/P3 三个手指轴和 R 旋转轴；
    * USB 转 485 直连、支持 xPanel 的珞石机型末端 485、离线协议测试；
    * 角度、带限制 Hall、力矩三种控制模式；
    * 实时状态、压力/触觉、舵机诊断、设备信息；
    * 反馈模式、安全参数、寄存器读写、动作序列保存与回放。
    """

    # 机器人末端 485 的一笔读取会同步经过控制器。自动状态页使用轻量
    # 状态读取，并留出足够的总线空闲时间，让按钮点击不会被轮询淹没。
    STATUS_INTERVAL_MS = 1500
    TRACE_STATUS_INTERVAL_MS = 2500
    # 动作序列不再只依赖固定延时，而是通过 485 反馈确认四个轴已经到位
    # 并稳定，再开始本步骤的“完成后延时”。
    SEQUENCE_MOTION_POLL_MS = 300
    SEQUENCE_MOTION_TIMEOUT_MS = 30000
    SEQUENCE_SETTLE_SAMPLES = 3
    SEQUENCE_SPEED_SETTLE_THRESHOLD = 5
    SEQUENCE_ANGLE_TOLERANCE_DEG = 1.5
    SEQUENCE_HALL_TOLERANCE = 20
    MOTION_VERIFY_DELAY_MS = 1200
    MOTION_VERIFY_POLL_MS = 500
    MOTION_VERIFY_TIMEOUT_MS = 30000
    MOTION_VERIFY_SETTLE_SAMPLES = 2
    # USB-RS485 适配器偶尔会在打开后保持收发方向状态；连接阶段可以
    # 重新打开串口，但状态轮询期间不能擅自关闭链路或切断机器人供电。
    USB_PROBE_REOPEN_ROUNDS = 4
    USB_PROBE_ATTEMPTS_PER_ROUND = 3
    USB_PROBE_REOPEN_DELAY_S = 0.50
    AXIS_IDS = AXIS_IDS
    FINGER_IDS = FINGER_IDS
    AXIS_LABELS = AXIS_LABELS

    def __init__(self, robot_backend, sdk_module,
                 log_fn: Optional[Callable[[str], None]] = None,
                 parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._log_fn = log_fn or (lambda _msg: None)
        self._robot_backend = robot_backend
        self._sdk_module = sdk_module
        self.backend = DexHand021SBackend(
            robot_backend=robot_backend,
            sdk_module=sdk_module,
            log_fn=self._log,
        )
        self._last_poll_error = ""
        self._robot_connected = False
        self._rebooting = False
        self._sequence_rows: list[dict] = []
        self._sequence_index = 0
        self._sequence_playing = False
        self._sequence_waiting_for_motion = False
        self._sequence_wait_started_at = 0.0
        self._sequence_settle_count = 0
        self._sequence_pending_delay_ms = 0
        self._sequence_phase = ""
        self._status_io_busy = False
        self._status_error_count = 0
        # 记录目标输入框当前代表的物理量。经本程序直发的 0x31 RTU 帧中，
        # 只有 0x44 是角度；0x55/0x66 都是 Hall。两类数值不能安全地自动
        # 换算。
        self._target_display_mode: Optional[int] = None
        # 力矩模式的默认值应独立于 Hall 速度。否则从 0x55 切换到 0x66 时，
        # 速度下限 50 会被误当成 50 PWM，容易因静摩擦触发堵转保护。
        # 用户在 0x66 下手动设置过的 PWM 则应在下次切回时保留。
        self._last_torque_pwm = 200
        self._pending_motion_checks: dict[int, tuple[int, int]] = {}
        self._motion_verify_started_at = 0.0
        self._motion_verify_settle_count = 0
        self._motion_verify_last_log_at = 0.0
        self._motion_verify_unavailable_logged = False
        self.motion_verify_timer = QTimer(self)
        self.motion_verify_timer.setSingleShot(True)
        self.motion_verify_timer.timeout.connect(self._verify_motion_feedback)
        # 官方 C++ 示例在开始动作前会清除一次固件保护状态。这个标记
        # 防止每个轴都重复发送清错帧；断开或重新连接后重新初始化。
        self._hand_control_initialized = False
        self.sequence_timer = QTimer(self)
        self.sequence_timer.setSingleShot(True)
        self.sequence_timer.timeout.connect(self._play_sequence_step)
        self.sequence_motion_timer = QTimer(self)
        self.sequence_motion_timer.setInterval(self.SEQUENCE_MOTION_POLL_MS)
        self.sequence_motion_timer.timeout.connect(self._check_sequence_motion)

        self._build_ui()

        self.status_timer = QTimer(self)
        self.status_timer.setInterval(self._status_poll_interval_ms())
        self.status_timer.timeout.connect(self._poll_status)

    # -- UI -------------------------------------------------------------

    def _build_ui(self) -> None:
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        root = QVBoxLayout(self)
        root.setContentsMargins(6, 6, 6, 6)

        root.addWidget(self._build_connection_box())

        self.tabs = QTabWidget()
        # 每个功能页独立滚动，避免“高级设置”页的高度把“轴控制”页
        # 一起撑成很高的外层页面；同时彻底去掉不必要的横向滚动。
        self.tabs.addTab(self._scroll_tab(self._build_control_tab()), "轴控制")
        self.tabs.addTab(self._scroll_tab(self._build_status_tab()), "状态监测")
        self.tabs.addTab(
            self._scroll_tab(self._build_advanced_tab()),
            "高级设置与寄存器",
        )
        self.tabs.addTab(self._scroll_tab(self._build_sequence_tab()), "动作序列")
        root.addWidget(self.tabs, 1)

        self._on_mode_changed()
        self._set_hand_controls_enabled(False)
        self._set_link_status(False)
        self._on_transport_changed()
        self._refresh_ports()

    @staticmethod
    def _scroll_tab(widget: QWidget) -> QScrollArea:
        """给灵巧手的每个子页面提供独立的纵向滚动区域。"""
        widget.setMinimumWidth(0)
        widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QFrame.NoFrame)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        scroll.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        scroll.setWidget(widget)
        return scroll

    def _build_connection_box(self) -> QGroupBox:
        connection = QGroupBox("DexHand021 S 通信与设备")
        grid = QGridLayout(connection)

        grid.addWidget(QLabel("通信方式:"), 0, 0)
        self.transport_combo = QComboBox()
        self.transport_combo.addItem("USB 转 485（电脑直连）", "usb")
        self.transport_combo.addItem(
            "机器人末端 485（珞石控制器透传）", "robot"
        )
        self.transport_combo.addItem("离线协议测试（不连接硬件）", "mock")
        self.transport_combo.currentIndexChanged.connect(self._on_transport_changed)
        grid.addWidget(self.transport_combo, 0, 1, 1, 2)

        self.port_label = QLabel("串口:")
        grid.addWidget(self.port_label, 0, 3)
        self.port_edit = QLineEdit("COM7")
        self.port_edit.setPlaceholderText("例如 COM7")
        self.port_edit.setMinimumWidth(85)
        self.port_edit.setMaximumWidth(220)
        grid.addWidget(self.port_edit, 0, 4)

        self.refresh_port_btn = QPushButton("刷新串口")
        self.refresh_port_btn.clicked.connect(self._refresh_ports)
        grid.addWidget(self.refresh_port_btn, 0, 5)

        grid.addWidget(QLabel("设备 ID:"), 0, 6)
        self.device_id_spin = QSpinBox()
        self.device_id_spin.setRange(1, 127)
        self.device_id_spin.setValue(1)
        self.device_id_spin.setToolTip("021S 板子 ID 范围为 1~127")
        self.device_id_spin.setMinimumWidth(60)
        self.device_id_spin.setMaximumWidth(120)
        grid.addWidget(self.device_id_spin, 0, 7)

        self.voltage_label = QLabel("末端供电:")
        grid.addWidget(self.voltage_label, 1, 0)
        self.voltage_combo = QComboBox()
        self.voltage_combo.addItem("24 V（021S 推荐）", 3)
        self.voltage_combo.addItem("12 V", 2)
        self.voltage_combo.addItem("保持控制器当前设置", 1)
        self.voltage_combo.setToolTip("只在机器人末端 485 模式下生效")
        grid.addWidget(self.voltage_combo, 1, 1, 1, 2)

        self.connect_btn = QPushButton("连接灵巧手")
        self.connect_btn.clicked.connect(self._on_connect_clicked)
        grid.addWidget(self.connect_btn, 1, 3)

        self.disconnect_btn = QPushButton("断开灵巧手")
        self.disconnect_btn.clicked.connect(self._on_disconnect_clicked)
        self.disconnect_btn.setEnabled(False)
        grid.addWidget(self.disconnect_btn, 1, 4)

        self.link_status_lbl = QLabel("● 未连接")
        self.link_status_lbl.setStyleSheet("color:#cc0000; font-weight:bold;")
        grid.addWidget(self.link_status_lbl, 1, 5, 1, 3)

        self.port_info_lbl = QLabel("串口设备：未扫描")
        # 不让完整的串口描述列表把整个页面的 sizeHint 撑到几千像素；
        # 页面缩小时显示摘要，完整列表放在悬停提示中。
        self.port_info_lbl.setWordWrap(True)
        self.port_info_lbl.setMinimumWidth(0)
        self.port_info_lbl.setSizePolicy(
            QSizePolicy.Ignored, QSizePolicy.Preferred
        )
        self.port_info_lbl.setStyleSheet("color:#555;")
        grid.addWidget(self.port_info_lbl, 2, 0, 1, 8)

        self.robot_hint_lbl = QLabel(
            "USB 直连：电脑作为 485 主站，参数固定为 115200、8N1；"
            "设备 ID 默认 1；请给灵巧手另接 24 V 电源。"
        )
        self.robot_hint_lbl.setWordWrap(True)
        self.robot_hint_lbl.setStyleSheet("color:#765500;")
        grid.addWidget(self.robot_hint_lbl, 3, 0, 1, 8)

        self.trace_log_check = QCheckBox("详细通信日志（记录 485 请求/回复帧）")
        # 详细帧日志每次状态轮询都会产生多条 UI 更新。默认关闭，排障时
        # 用户仍可手动打开；打开后同步降低轮询频率。
        self.trace_log_check.setChecked(False)
        self.trace_log_check.setToolTip(
            "开启后在“日志”页显示发送帧、期望回复长度、实际回复帧、"
            "xPanel SDK 返回码和异常原因；开启时状态轮询会自动放慢，"
            "排查结束后建议关闭以保持界面流畅。"
        )
        self.trace_log_check.toggled.connect(self._on_trace_logging_changed)
        grid.addWidget(self.trace_log_check, 4, 0, 1, 4)
        return connection

    def _build_control_tab(self) -> QWidget:
        tab = QWidget()
        root = QVBoxLayout(tab)

        control = QGroupBox("P1 / P2 / P3 / R 轴控制")
        control_layout = QVBoxLayout(control)
        form = QFormLayout()

        self.mode_combo = QComboBox()
        self.mode_combo.addItem(
            "角度位置模式（SDK 0x44 / RTU 0x04，目标角度×10）", 0x44
        )
        self.mode_combo.addItem(
            "带限制 Hall 位置（SDK 0x55 / RTU 0x05，目标 Hall）", 0x55
        )
        self.mode_combo.addItem(
            "级联 MIT 力矩模式（SDK 0x66 / RTU 0x06，目标 Hall，控制量 50~800）",
            0x66,
        )
        # 官方 C++ SDK 的 021S 示例使用 0x55 + 速度 1000。
        self.mode_combo.setCurrentIndex(1)
        self.mode_combo.currentIndexChanged.connect(self._on_mode_changed)
        form.addRow("控制模式:", self.mode_combo)

        self.control_value_spin = QSpinBox()
        self.control_value_spin.setRange(0, 32767)
        self.control_value_spin.setValue(1000)
        self.control_value_spin.setSuffix(" °/s")
        self.control_value_spin.valueChanged.connect(self._on_control_value_changed)
        self.control_value_label = QLabel("速度（°/s ×100）:")
        form.addRow(self.control_value_label, self.control_value_spin)

        self.force_check = QCheckBox("读取压力/触觉（轮询较慢）")
        self.force_check.setChecked(False)
        form.addRow("扩展状态:", self.force_check)

        self.diagnostics_check = QCheckBox("读取舵机诊断与保护参数（轮询最慢）")
        self.diagnostics_check.setChecked(False)
        form.addRow("", self.diagnostics_check)
        control_layout.addLayout(form)

        self.mode_help_lbl = QLabel()
        self.mode_help_lbl.setWordWrap(True)
        self.mode_help_lbl.setStyleSheet("color:#555; padding:4px;")
        control_layout.addWidget(self.mode_help_lbl)

        axis_grid = QGridLayout()
        headers = ("轴", "目标滑块", "目标值（Hall）", "发送")
        for col, text in enumerate(headers):
            label = QLabel(text)
            if col == 2:
                self.axis_target_header_lbl = label
            axis_grid.addWidget(label, 0, col)
        axis_grid.setColumnStretch(1, 1)

        self.finger_controls: dict[int, dict[str, QWidget]] = {}
        self.axis_controls = self.finger_controls
        for row, axis_id in enumerate(AXIS_IDS, start=1):
            axis_grid.addWidget(QLabel(AXIS_LABELS[axis_id]), row, 0)

            slider = QSlider(Qt.Horizontal)
            slider.setSingleStep(1)
            slider.setPageStep(50)
            slider.setMinimumWidth(80)
            slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
            axis_grid.addWidget(slider, row, 1)

            # 滑块始终保留协议原始分辨率（角度模式为 0.1°），输入框
            # 则按当前模式显示“度”或“Hall”，避免让用户手算 ×10。
            value_spin = QDoubleSpinBox()
            value_spin.setDecimals(0)
            value_spin.setKeyboardTracking(False)
            value_spin.setMinimumWidth(80)
            value_spin.setMaximumWidth(140)
            value_spin.valueChanged.connect(
                lambda value, aid=axis_id: self._on_axis_spin_changed(aid, value)
            )
            slider.valueChanged.connect(
                lambda value, aid=axis_id: self._on_axis_slider_changed(aid, value)
            )
            axis_grid.addWidget(value_spin, row, 2)

            send_btn = QPushButton("发送")
            send_btn.setMinimumWidth(60)
            send_btn.setMaximumWidth(110)
            send_btn.clicked.connect(
                lambda _checked=False, aid=axis_id: self._send_axis(aid)
            )
            axis_grid.addWidget(send_btn, row, 3)
            self.finger_controls[axis_id] = {
                "slider": slider,
                "spin": value_spin,
                "send": send_btn,
            }
        control_layout.addLayout(axis_grid)

        buttons = QHBoxLayout()
        self.open_btn = QPushButton("三指张开")
        self.open_btn.clicked.connect(lambda: self._move_fingers(0, "张开"))
        buttons.addWidget(self.open_btn)

        self.close_btn = QPushButton("三指闭合")
        self.close_btn.clicked.connect(self._on_close_all_clicked)
        buttons.addWidget(self.close_btn)

        self.rotation_zero_btn = QPushButton("旋转轴归零")
        self.rotation_zero_btn.clicked.connect(self._on_rotation_zero_clicked)
        buttons.addWidget(self.rotation_zero_btn)

        self.send_all_btn = QPushButton("发送全部当前目标")
        self.send_all_btn.clicked.connect(self._on_send_all_clicked)
        buttons.addWidget(self.send_all_btn)

        self.clear_error_btn = QPushButton("清除错误")
        self.clear_error_btn.clicked.connect(self._on_clear_error_clicked)
        buttons.addWidget(self.clear_error_btn)

        self.reset_joints_btn = QPushButton("官方复位初始化")
        self.reset_joints_btn.clicked.connect(self._on_reset_joints_clicked)
        self.reset_joints_btn.setToolTip(
            "按官方 SDK 顺序执行：清错 → P1/P2/P3 Hall=0、R=280 复位 → 再清错；会驱动机械手"
        )
        buttons.addWidget(self.reset_joints_btn)

        self.auto_clear_on_connect_check = QCheckBox("连接后自动清除保护（不自动复位）")
        self.auto_clear_on_connect_check.setChecked(True)
        self.auto_clear_on_connect_check.setToolTip(
            "连接成功后只发送官方清错指令，不会自动驱动复位；"
            "需要复位时请明确点击“官方复位初始化”。"
        )
        buttons.addWidget(self.auto_clear_on_connect_check)

        self.read_status_btn = QPushButton("立即读取状态")
        self.read_status_btn.clicked.connect(self._on_read_status_clicked)
        buttons.addWidget(self.read_status_btn)
        buttons.addStretch(1)
        control_layout.addLayout(buttons)

        warning = QLabel(
            "提示：P1/P2/P3 是三个手指轴，R 是旋转轴。所有带“发送”的按钮都会实际驱动灵巧手，"
            "请确认工作区安全；角度位置模式更适合三指同步控制。"
        )
        warning.setWordWrap(True)
        warning.setStyleSheet("color:#aa5500;")
        control_layout.addWidget(warning)

        root.addWidget(control)
        root.addStretch(1)
        return tab

    def _build_status_tab(self) -> QWidget:
        tab = QWidget()
        root = QVBoxLayout(tab)

        status_box = QGroupBox("实时轴状态")
        status_layout = QVBoxLayout(status_box)
        self.status_table = QTableWidget(4, 8)
        self.status_table.setHorizontalHeaderLabels(
            ["轴", "角度 (°)", "Hall 位置", "速度 (°/s)",
             "电流 (mA)", "扭矩 PWM", "温度 (°C)", "电压 (V)"]
        )
        self._configure_table(self.status_table)
        for row, axis_id in enumerate(AXIS_IDS):
            self.status_table.setItem(row, 0, QTableWidgetItem(AXIS_LABELS[axis_id]))
        status_layout.addWidget(self.status_table)
        self.status_extra_lbl = QLabel("尚未读取状态")
        self.status_extra_lbl.setStyleSheet("color:#555;")
        status_layout.addWidget(self.status_extra_lbl)
        root.addWidget(status_box)

        diagnostic_box = QGroupBox("舵机诊断与保护参数")
        diagnostic_layout = QVBoxLayout(diagnostic_box)
        self.diagnostic_table = QTableWidget(4, 7)
        self.diagnostic_table.setHorizontalHeaderLabels(
            ["轴", "最大扭矩", "最大电流", "最大速度",
             "保护温度", "堵转触发时间", "堵转保护电流"]
        )
        self._configure_table(self.diagnostic_table)
        for row, axis_id in enumerate(AXIS_IDS):
            self.diagnostic_table.setItem(
                row, 0, QTableWidgetItem(AXIS_LABELS[axis_id])
            )
        diagnostic_layout.addWidget(self.diagnostic_table)
        root.addWidget(diagnostic_box)

        force_box = QGroupBox("压力/触觉状态（P1~P3）")
        force_layout = QVBoxLayout(force_box)
        self.force_table = QTableWidget(3, 7)
        self.force_table.setHorizontalHeaderLabels(
            ["手指", "法向力 (N)", "法向变化", "切向力 (N)",
             "切向变化", "切向方向 (°)", "接近觉"]
        )
        self._configure_table(self.force_table)
        for row, finger_id in enumerate(FINGER_IDS):
            self.force_table.setItem(
                row, 0, QTableWidgetItem(f"P{finger_id}")
            )
        force_layout.addWidget(self.force_table)
        root.addWidget(force_box)
        root.addStretch(1)
        return tab

    def _build_advanced_tab(self) -> QWidget:
        tab = QWidget()
        root = QVBoxLayout(tab)

        feedback_box = QGroupBox("反馈模式")
        feedback_form = QFormLayout(feedback_box)
        self.feedback_mode_combo = QComboBox()
        self.feedback_mode_combo.addItem("关闭自动反馈", 0)
        self.feedback_mode_combo.addItem("CANFD 自动回传", 1)
        self.feedback_mode_combo.addItem("Modbus RTU 问询回传", 2)
        self.feedback_mode_combo.setCurrentIndex(2)
        feedback_form.addRow("反馈模式:", self.feedback_mode_combo)

        self.feedback_interval_spin = QSpinBox()
        self.feedback_interval_spin.setRange(10, 20)
        self.feedback_interval_spin.setValue(20)
        self.feedback_interval_spin.setSuffix(" ms")
        feedback_form.addRow("反馈间隔:", self.feedback_interval_spin)

        self.apply_feedback_btn = QPushButton("应用反馈设置")
        self.apply_feedback_btn.clicked.connect(self._on_apply_feedback_clicked)
        feedback_form.addRow("", self.apply_feedback_btn)
        root.addWidget(feedback_box)

        safety_box = QGroupBox("输出与温度保护")
        safety_form = QFormLayout(safety_box)
        self.max_current_spin = QSpinBox()
        self.max_current_spin.setRange(200, 500)
        self.max_current_spin.setValue(250)
        self.max_current_spin.setSuffix(" mA")
        safety_form.addRow("最大输出电流:", self.max_current_spin)

        self.protection_temperature_spin = QSpinBox()
        self.protection_temperature_spin.setRange(40, 90)
        self.protection_temperature_spin.setValue(90)
        self.protection_temperature_spin.setSuffix(" ℃")
        safety_form.addRow("舵机保护温度:", self.protection_temperature_spin)

        self.cooldown_temperature_spin = QSpinBox()
        self.cooldown_temperature_spin.setRange(3, 20)
        self.cooldown_temperature_spin.setValue(10)
        self.cooldown_temperature_spin.setSuffix(" ℃")
        safety_form.addRow("降温幅度:", self.cooldown_temperature_spin)

        self.apply_safety_btn = QPushButton("应用保护参数")
        self.apply_safety_btn.clicked.connect(self._on_apply_safety_clicked)
        safety_form.addRow("", self.apply_safety_btn)
        root.addWidget(safety_box)

        device_box = QGroupBox("设备信息与维护")
        device_layout = QVBoxLayout(device_box)
        device_buttons = QHBoxLayout()
        self.read_device_info_btn = QPushButton("读取设备信息")
        self.read_device_info_btn.clicked.connect(self._on_read_device_info_clicked)
        device_buttons.addWidget(self.read_device_info_btn)
        self.advanced_clear_error_btn = QPushButton("清除错误")
        self.advanced_clear_error_btn.clicked.connect(self._on_clear_error_clicked)
        device_buttons.addWidget(self.advanced_clear_error_btn)
        self.reboot_btn = QPushButton("重启灵巧手")
        self.reboot_btn.clicked.connect(self._on_reboot_clicked)
        device_buttons.addWidget(self.reboot_btn)
        device_buttons.addStretch(1)
        device_layout.addLayout(device_buttons)
        self.device_info_lbl = QLabel("尚未读取设备信息")
        self.device_info_lbl.setWordWrap(True)
        device_layout.addWidget(self.device_info_lbl)
        device_note = QLabel(
            "重启会中断当前通信，只有在设备状态允许时使用；不提供 IAP 升级按钮，"
            "避免误触导致固件风险。"
        )
        device_note.setWordWrap(True)
        device_note.setStyleSheet("color:#aa5500;")
        device_layout.addWidget(device_note)
        root.addWidget(device_box)

        raw_box = QGroupBox("Modbus 寄存器工具")
        raw_layout = QVBoxLayout(raw_box)
        raw_form = QFormLayout()
        self.raw_function_combo = QComboBox()
        self.raw_function_combo.addItem("读取保持寄存器（0x03）", 0x03)
        self.raw_function_combo.addItem("读取输入寄存器（0x04）", 0x04)
        raw_form.addRow("读取类型:", self.raw_function_combo)
        self.raw_start_spin = QSpinBox()
        self.raw_start_spin.setRange(0, 0xFFFF)
        self.raw_start_spin.setDisplayIntegerBase(16)
        self.raw_start_spin.setPrefix("0x")
        raw_form.addRow("起始地址:", self.raw_start_spin)
        self.raw_count_spin = QSpinBox()
        self.raw_count_spin.setRange(1, 125)
        self.raw_count_spin.setValue(4)
        raw_form.addRow("寄存器数量:", self.raw_count_spin)
        self.raw_read_btn = QPushButton("读取寄存器")
        self.raw_read_btn.clicked.connect(self._on_raw_read_clicked)
        raw_form.addRow("", self.raw_read_btn)
        raw_layout.addLayout(raw_form)

        write_form = QFormLayout()
        self.raw_write_address_spin = QSpinBox()
        self.raw_write_address_spin.setRange(0, 0xFFFF)
        self.raw_write_address_spin.setDisplayIntegerBase(16)
        self.raw_write_address_spin.setPrefix("0x")
        write_form.addRow("写入地址:", self.raw_write_address_spin)
        self.raw_write_value_spin = QSpinBox()
        self.raw_write_value_spin.setRange(0, 0xFFFF)
        self.raw_write_value_spin.setDisplayIntegerBase(16)
        self.raw_write_value_spin.setPrefix("0x")
        write_form.addRow("写入数值:", self.raw_write_value_spin)
        self.raw_write_btn = QPushButton("写入单个保持寄存器")
        self.raw_write_btn.clicked.connect(self._on_raw_write_clicked)
        write_form.addRow("", self.raw_write_btn)
        raw_layout.addLayout(write_form)

        self.raw_result = QPlainTextEdit()
        self.raw_result.setReadOnly(True)
        self.raw_result.setMaximumHeight(105)
        self.raw_result.setPlaceholderText("寄存器读取结果将显示在这里")
        raw_layout.addWidget(self.raw_result)
        raw_note = QLabel(
            "寄存器写入会直接改变设备参数，仅建议按说明书地址表使用；管理员寄存器和 IAP 相关地址不在这里开放。"
        )
        raw_note.setWordWrap(True)
        raw_note.setStyleSheet("color:#aa5500;")
        raw_layout.addWidget(raw_note)
        root.addWidget(raw_box)
        root.addStretch(1)
        return tab

    def _build_sequence_tab(self) -> QWidget:
        tab = QWidget()
        root = QVBoxLayout(tab)

        intro = QLabel(
            "动作序列用于保存 P1/P2/P3/R 的目标值和完成后延时，可保存为 JSON 后重复回放。"
            "每一步都会先发送动作、读取反馈确认动作完成，再等待设定延时后执行下一步。"
        )
        intro.setWordWrap(True)
        intro.setStyleSheet("color:#555;")
        root.addWidget(intro)

        self.sequence_table = QTableWidget(0, 8)
        self.sequence_table.setHorizontalHeaderLabels(
            ["步骤", "模式", "控制量", "完成后延时 (ms)", "P1", "P2", "P3", "R"]
        )
        self._configure_table(self.sequence_table)
        root.addWidget(self.sequence_table, 1)

        row_form = QHBoxLayout()
        row_form.addWidget(QLabel("新增步骤延时:"))
        self.sequence_delay_spin = QSpinBox()
        self.sequence_delay_spin.setRange(20, 60000)
        self.sequence_delay_spin.setValue(500)
        self.sequence_delay_spin.setSuffix(" ms")
        row_form.addWidget(self.sequence_delay_spin)
        self.add_sequence_btn = QPushButton("记录当前目标")
        self.add_sequence_btn.clicked.connect(self._on_add_sequence_clicked)
        row_form.addWidget(self.add_sequence_btn)
        self.delete_sequence_btn = QPushButton("删除选中")
        self.delete_sequence_btn.clicked.connect(self._on_delete_sequence_clicked)
        row_form.addWidget(self.delete_sequence_btn)
        self.clear_sequence_btn = QPushButton("清空序列")
        self.clear_sequence_btn.clicked.connect(self._on_clear_sequence_clicked)
        row_form.addWidget(self.clear_sequence_btn)
        row_form.addStretch(1)
        root.addLayout(row_form)

        action_row = QHBoxLayout()
        self.save_sequence_btn = QPushButton("保存 JSON")
        self.save_sequence_btn.clicked.connect(self._on_save_sequence_clicked)
        action_row.addWidget(self.save_sequence_btn)
        self.load_sequence_btn = QPushButton("加载 JSON")
        self.load_sequence_btn.clicked.connect(self._on_load_sequence_clicked)
        action_row.addWidget(self.load_sequence_btn)
        self.play_sequence_btn = QPushButton("回放动作序列")
        self.play_sequence_btn.clicked.connect(self._on_play_sequence_clicked)
        action_row.addWidget(self.play_sequence_btn)
        self.stop_sequence_btn = QPushButton("停止回放")
        self.stop_sequence_btn.clicked.connect(self._on_stop_sequence_clicked)
        action_row.addWidget(self.stop_sequence_btn)
        action_row.addStretch(1)
        root.addLayout(action_row)

        self.sequence_status_lbl = QLabel("序列为空")
        self.sequence_status_lbl.setStyleSheet("color:#555;")
        root.addWidget(self.sequence_status_lbl)
        return tab

    @staticmethod
    def _configure_table(table: QTableWidget) -> None:
        table.verticalHeader().setVisible(False)
        table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        table.setSelectionBehavior(QAbstractItemView.SelectRows)
        table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

    # -- state and helpers ---------------------------------------------

    def _log(self, message: str) -> None:
        self._log_fn(message)

    def _transport_key(self) -> str:
        return str(self.transport_combo.currentData())

    def _is_er_pro_robot(self) -> bool:
        robot_type = str(getattr(self._robot_backend, "robot_type_name", ""))
        normalized_type = robot_type.replace("_", "").lower()
        return "xmateerpro" in normalized_type or "erpro" in normalized_type

    def _mode(self) -> int:
        return int(self.mode_combo.currentData())

    @staticmethod
    def _wire_mode(mode: int) -> int:
        """返回 0x31 RTU 帧内的 Motor_Mode 字段。"""
        return SDK_MODE_TO_RTU_MOTOR_MODE.get(int(mode), int(mode))

    @staticmethod
    def _is_angle_target_mode(mode: int) -> bool:
        """裸 0x31 RTU 中只有 Motor_Mode=0x04 的目标是角度×10。"""
        return int(mode) == 0x44

    @staticmethod
    def _axis_limit_for_mode(axis_id: int, mode: int) -> int:
        """返回直发 0x31 RTU 帧的原始目标上限。"""
        if int(mode) == 0x44:
            return 1600 if int(axis_id) == 4 else 750
        # 《使用说明书》0x05/0x06 的原始帧示例均使用三指 Hall=1200；
        # R 轴为 Hall=0~1600。这里是裸 RTU 语义，不是高层 SDK 参数。
        return 1600 if int(axis_id) == 4 else 1200

    def _axis_limit(self, axis_id: int) -> int:
        return self._axis_limit_for_mode(axis_id, self._mode())

    def _target_display_scale(self, mode: Optional[int] = None) -> float:
        return 10.0 if self._is_angle_target_mode(
            self._mode() if mode is None else int(mode)
        ) else 1.0

    def _target_display_suffix(self, mode: Optional[int] = None) -> str:
        return " °" if self._is_angle_target_mode(
            self._mode() if mode is None else int(mode)
        ) else " Hall"

    def _format_target_value(self, axis_id: int, target: int,
                             mode: Optional[int] = None) -> str:
        actual_mode = self._mode() if mode is None else int(mode)
        limit = self._axis_limit_for_mode(axis_id, actual_mode)
        raw = max(0, min(limit, int(target)))
        if self._is_angle_target_mode(actual_mode):
            return f"{raw / 10.0:.1f}°"
        return f"Hall {raw}"

    def _set_axis_target(self, axis_id: int, target: int) -> None:
        controls = self.finger_controls[axis_id]
        slider = controls["slider"]
        spin = controls["spin"]
        assert isinstance(slider, QSlider)
        assert isinstance(spin, QDoubleSpinBox)
        raw = max(0, min(self._axis_limit(axis_id), int(target)))
        scale = self._target_display_scale()
        slider_signals = slider.blockSignals(True)
        spin_signals = spin.blockSignals(True)
        try:
            slider.setValue(raw)
            spin.setValue(raw / scale)
        finally:
            slider.blockSignals(slider_signals)
            spin.blockSignals(spin_signals)

    def _axis_target(self, axis_id: int) -> int:
        slider = self.finger_controls[axis_id]["slider"]
        assert isinstance(slider, QSlider)
        return int(slider.value())

    def _on_axis_slider_changed(self, axis_id: int, raw_value: int) -> None:
        """将协议原始滑块值同步为当前模式的可读单位。"""
        spin = self.finger_controls[axis_id]["spin"]
        assert isinstance(spin, QDoubleSpinBox)
        raw = max(0, min(self._axis_limit(axis_id), int(raw_value)))
        signals = spin.blockSignals(True)
        try:
            spin.setValue(raw / self._target_display_scale())
        finally:
            spin.blockSignals(signals)

    def _on_axis_spin_changed(self, axis_id: int, display_value: float) -> None:
        """将用户输入的“度/Hall”转换为 0x31 帧所需的原始目标值。"""
        slider = self.finger_controls[axis_id]["slider"]
        assert isinstance(slider, QSlider)
        raw = int(round(float(display_value) * self._target_display_scale()))
        raw = max(0, min(self._axis_limit(axis_id), raw))
        signals = slider.blockSignals(True)
        try:
            slider.setValue(raw)
        finally:
            slider.blockSignals(signals)

    def _configure_axis_target_widget(self, axis_id: int, raw_target: int) -> None:
        controls = self.finger_controls[axis_id]
        slider = controls["slider"]
        spin = controls["spin"]
        assert isinstance(slider, QSlider)
        assert isinstance(spin, QDoubleSpinBox)
        mode = self._mode()
        limit = self._axis_limit_for_mode(axis_id, mode)
        scale = self._target_display_scale(mode)
        slider_signals = slider.blockSignals(True)
        spin_signals = spin.blockSignals(True)
        try:
            slider.setRange(0, limit)
            slider.setSingleStep(1)
            slider.setPageStep(50 if self._is_angle_target_mode(mode) else 100)
            spin.setRange(0.0, limit / scale)
            spin.setDecimals(1 if self._is_angle_target_mode(mode) else 0)
            spin.setSingleStep(0.1 if self._is_angle_target_mode(mode) else 1.0)
            spin.setSuffix(self._target_display_suffix(mode))
            slider.setValue(max(0, min(limit, int(raw_target))))
            spin.setValue(slider.value() / scale)
        finally:
            slider.blockSignals(slider_signals)
            spin.blockSignals(spin_signals)

    def _update_mode_dependent_labels(self) -> None:
        """把按钮和表头的单位与当前控制模式保持一致。"""
        mode = self._mode()
        if self._is_angle_target_mode(mode):
            self.axis_target_header_lbl.setText("目标值（°）")
            self.open_btn.setText("三指张开（0.0°）")
            self.close_btn.setText("三指闭合（75.0°）")
        else:
            self.axis_target_header_lbl.setText("目标值（Hall）")
            self.open_btn.setText("三指张开（Hall 0）")
            if mode == 0x66:
                self.close_btn.setText(
                    f"三指闭合（Hall 1200，{self.control_value_spin.value()} PWM）"
                )
            else:
                self.close_btn.setText("三指闭合（Hall 1200）")

    @Slot(int)
    def _on_control_value_changed(self, value: int) -> None:
        """仅记住用户在力矩模式下显式设定的 PWM。"""
        if self._mode() == 0x66:
            self._last_torque_pwm = min(
                max(int(value), self.backend.TORQUE_MIN),
                self.backend.TORQUE_MAX,
            )
        self._update_mode_dependent_labels()

    @Slot()
    def _refresh_ports(self) -> None:
        try:
            from serial.tools import list_ports  # type: ignore
            ports = list(list_ports.comports())
        except ImportError:
            self.port_info_lbl.setText("串口设备：缺少 pyserial")
            self.port_info_lbl.setToolTip("")
            return
        if not ports:
            self.port_info_lbl.setText("串口设备：未发现")
            self.port_info_lbl.setToolTip("")
            return
        names = [str(info.device) for info in ports]
        descriptions = [
            f"{info.device}（{info.description or '未知设备'}）"
            for info in ports
        ]
        current = self.port_edit.text().strip().upper()
        if current not in {name.upper() for name in names}:
            for name in names:
                if name.upper() == "COM7":
                    self.port_edit.setText(name)
                    break
        selected = self.port_edit.text().strip().upper()
        selected_desc = next(
            (description for name, description in zip(names, descriptions)
             if name.upper() == selected),
            "未选择",
        )
        self.port_info_lbl.setText(
            f"串口设备：当前 {selected_desc}；共检测到 {len(ports)} 个"
        )
        self.port_info_lbl.setToolTip(
            "完整串口列表：\n" + "\n".join(descriptions)
        )

    @Slot()
    def _on_transport_changed(self) -> None:
        is_usb = self._transport_key() == "usb"
        is_robot = self._transport_key() == "robot"
        is_er_pro = is_robot and self._is_er_pro_robot()
        # ER7Pro-M 不使用电脑串口，也不通过本程序写入“末端供电”选项。
        # 隐藏这两组仅适用于其它链路的控件，避免把其它机型的菜单项
        # 误当成本机型的必需配置。
        self.port_label.setVisible(is_usb)
        self.port_edit.setVisible(is_usb)
        self.refresh_port_btn.setVisible(is_usb)
        self.port_edit.setEnabled(is_usb and not self.backend.connected)
        self.refresh_port_btn.setEnabled(is_usb and not self.backend.connected)
        self.voltage_label.setVisible(not is_er_pro)
        self.voltage_combo.setVisible(not is_er_pro)
        self.voltage_combo.setEnabled(
            is_robot and not is_er_pro and not self.backend.connected
        )
        if is_robot:
            self.port_info_lbl.setText(
                "机器人末端 485：本模式不使用电脑串口。"
            )
            self.port_info_lbl.setToolTip("")
        if is_robot:
            if is_er_pro:
                self.robot_hint_lbl.setText(
                    "xMate ER7Pro-M（XME7p-R850）：机器人末端模式不使用电脑 COM 口，"
                    "不创建或运行 RL 工程，直接调用 xCore SDK 的 XPRS485SendData。"
                    "程序不写入控制器末端工具/供电配置；请在控制器现场确认末端 RS485 和供电。"
                    "ID=1、115200、8N1 仅为 DexHand021 S 协议参数。"
                )
            else:
                self.robot_hint_lbl.setText(
                    "机器人末端模式（仅适用于手册所述支持 xPanel 的机型）：先连接机器人；"
                    "请在控制器【通信 → xPanel 设置】确认“输出 24V”和“RS485”。"
                    "本程序连接时也会调用同样的 xPanel 配置；021S 协议为设备 ID=1、115200、8N1。"
                )
        elif is_usb:
            self.robot_hint_lbl.setText(
                "USB 模式：电脑直接作为 485 主站；串口固定为 115200、8N1，"
                "设备 ID 默认 1。CH340 当前检测为 COM7，灵巧手仍需独立 24 V 电源。"
            )
        else:
            self.robot_hint_lbl.setText("离线模式只验证协议帧和界面流程，不连接硬件。")

    @Slot(bool)
    def _on_trace_logging_changed(self, enabled: bool) -> None:
        self.backend.set_trace_enabled(enabled)
        timer = getattr(self, "status_timer", None)
        if timer is not None:
            timer.setInterval(self._status_poll_interval_ms())
        if self.backend.connected:
            interval = self._status_poll_interval_ms() / 1000.0
            self._log(
                "详细通信日志已%s；自动状态轮询间隔为 %.1f 秒"
                % ("开启" if enabled else "关闭", interval)
            )

    def _status_poll_interval_ms(self) -> int:
        if getattr(self, "trace_log_check", None) is not None and self.trace_log_check.isChecked():
            return self.TRACE_STATUS_INTERVAL_MS
        return self.STATUS_INTERVAL_MS

    @Slot()
    def _on_mode_changed(self) -> None:
        mode = self._mode()
        previous_mode = self._target_display_mode
        previous_control_value = int(self.control_value_spin.value())
        old_targets = {
            axis_id: self._axis_target(axis_id) for axis_id in AXIS_IDS
        }
        unit_changed = (
            previous_mode is not None
            and self._is_angle_target_mode(previous_mode)
            != self._is_angle_target_mode(mode)
        )
        # Hall 与关节角度没有一一对应关系。切换这两类模式时保留原数字
        # 会使下一次“发送”产生不可预期的动作，因此显式安全归零。
        if unit_changed:
            old_targets = {axis_id: 0 for axis_id in AXIS_IDS}
        for axis_id in AXIS_IDS:
            self._configure_axis_target_widget(axis_id, old_targets[axis_id])
        self._target_display_mode = mode

        # 程序切换量程时不应触发 valueChanged，否则 Hall 速度的 50 会被
        # 记录成“用户选择的 50 PWM”。
        spin_signals = self.control_value_spin.blockSignals(True)
        try:
            if mode == 0x44:
                self.control_value_label.setText("控制量：")
                self.control_value_spin.setRange(0, 0)
                self.control_value_spin.setValue(0)
                self.control_value_spin.setSuffix("（此模式无效）")
                help_text = (
                    "目标值按角度显示：P1/P2/P3 为 0.0~75.0°，R 旋转轴为 0.0~160.0°。"
                )
            elif mode == 0x55:
                self.control_value_label.setText("速度（原始值）：")
                self.control_value_spin.setRange(
                    self.backend.HALL_SPEED_MIN, self.backend.HALL_SPEED_MAX
                )
                self.control_value_spin.setValue(
                    min(
                        max(previous_control_value, self.backend.HALL_SPEED_MIN),
                        self.backend.HALL_SPEED_MAX,
                    )
                )
                self.control_value_spin.setSuffix("（50~300）")
                help_text = (
                    "目标值按 Hall 显示：P1/P2/P3 为 0~1200，R 为 0~1600；"
                    "当前链路直发 0x31/0x05，速度原始值范围 50~300。"
                )
            else:
                self.control_value_label.setText("力矩（PWM）：")
                # 连续停留在 0x66 时保留当前值；从其它模式切入时恢复上次
                # 手动设置的力矩（首次为安全且实用的 200 PWM）。
                torque_value = (
                    previous_control_value
                    if previous_mode == 0x66
                    else self._last_torque_pwm
                )
                torque_value = min(
                    max(torque_value, self.backend.TORQUE_MIN),
                    self.backend.TORQUE_MAX,
                )
                self.control_value_spin.setRange(
                    self.backend.TORQUE_MIN, self.backend.TORQUE_MAX
                )
                self.control_value_spin.setValue(torque_value)
                self.control_value_spin.setSuffix(" PWM")
                self._last_torque_pwm = torque_value
                help_text = (
                    "目标值按 Hall 显示：P1/P2/P3 为 0~1200，R 为 0~1600；"
                    "力矩范围 50~800 PWM，首次切入力矩模式默认 200 PWM。"
                    "当前链路直发 0x31/0x06，说明书规定其目标字段是 Hall，"
                    "不是角度×10：把 75.0° 直接写为 750 会停在 Hall 750（约 60°）。"
                    "空载闭合到机械行程末端请用 Hall 1200，并确认工作区安全。"
                )
        finally:
            self.control_value_spin.blockSignals(spin_signals)
        self.mode_help_lbl.setText(help_text)
        self._update_mode_dependent_labels()
        if unit_changed:
            self._log(
                "控制模式的目标单位已从%s切换为%s；Hall 与角度不能自动换算，"
                "四轴目标已安全归零，请重新输入目标。"
                % (
                    "角度" if self._is_angle_target_mode(previous_mode) else "Hall",
                    "角度" if self._is_angle_target_mode(mode) else "Hall",
                )
            )

    def _set_hand_controls_enabled(self, enabled: bool) -> None:
        for controls in self.finger_controls.values():
            for key in ("slider", "spin", "send"):
                controls[key].setEnabled(enabled)
        for button in (
            self.open_btn,
            self.close_btn,
            self.rotation_zero_btn,
            self.send_all_btn,
            self.clear_error_btn,
            self.reset_joints_btn,
            self.auto_clear_on_connect_check,
            self.read_status_btn,
            self.apply_feedback_btn,
            self.apply_safety_btn,
            self.read_device_info_btn,
            self.advanced_clear_error_btn,
            self.reboot_btn,
            self.raw_read_btn,
            self.raw_write_btn,
        ):
            button.setEnabled(enabled)
        self._update_sequence_controls()

    def _set_link_status(self, connected: bool) -> None:
        if connected:
            self.link_status_lbl.setText(
                f"● 已连接：{self.backend.transport_label()}，ID={self.backend.device_id}"
            )
            self.link_status_lbl.setStyleSheet("color:#008800; font-weight:bold;")
        else:
            self.link_status_lbl.setText("● 未连接")
            self.link_status_lbl.setStyleSheet("color:#cc0000; font-weight:bold;")
        self.connect_btn.setEnabled(not connected)
        self.disconnect_btn.setEnabled(connected)
        self.transport_combo.setEnabled(not connected)
        self.device_id_spin.setEnabled(not connected)
        self._set_hand_controls_enabled(connected)
        self._on_transport_changed()

    # -- connection -----------------------------------------------------

    @Slot()
    def _on_connect_clicked(self) -> None:
        key = self._transport_key()
        if key == "robot" and not getattr(self._robot_backend, "connected", False):
            QMessageBox.warning(self, "无法连接灵巧手", "请先连接珞石机器人控制器。")
            return
        open_args = {
            "port": self.port_edit.text().strip(),
            "device_id": self.device_id_spin.value(),
            "voltage_option": int(self.voltage_combo.currentData()),
        }
        self._log(
            f"灵巧手连接参数：方式={self.transport_combo.currentText()}；"
            f"设备 ID={open_args['device_id']}；波特率=115200；"
            "数据位=8；校验=None；停止位=1"
        )
        if key == "robot":
            if self._is_er_pro_robot():
                self._log(
                    "检测到 xMate ER7Pro-M（XME7p-R850，SDK 类型 xMateErProRobot）："
                    "不创建或运行 RL 工程，不调用 XPRS485Init、setxPanelRS485 或供电设置接口；"
                    "每一笔末端 485 请求直接调用 xCore SDK 的 XPRS485SendData。"
                )
            else:
                self._log(
                    "xPanel 目标配置：对外供电=输出24V；模拟输入/RS485=RS485；"
                    "程序会按已连接机器人型号调用官方 xPanel 配置接口。"
                )
        try:
            self.backend.open(key, **open_args)
            # 打开串口本身不代表灵巧手已经在线。先做一次只读的 P1~P3
            # 角度探测，避免界面显示“已连接”后用户点击发送才发现总线无回复。
            if key == "robot":
                self._log("末端 485 已打开，等待灵巧手上线并进行只读探测……")
                function, values = self.backend.probe_connection(
                    attempts=1 if self._is_er_pro_robot() else 6,
                    retry_delay_s=0.35,
                )
            else:
                # USB-RS485 的第一帧可能落在 CH340 打开后的初始化窗口内。
                # 多次只读重试不会驱动电机；如果整轮仍然是 0 字节，关闭并
                # 重新打开 CH340，再做一轮探测，避免用户必须反复点击“连接”。
                last_probe_error: Optional[DexHandError] = None
                for reopen_round in range(self.USB_PROBE_REOPEN_ROUNDS):
                    if reopen_round:
                        self._log(
                            f"USB 485 第 {reopen_round} 轮探测仍无回复，"
                            "正在重新打开串口……"
                        )
                        self.backend.close()
                        time.sleep(self.USB_PROBE_REOPEN_DELAY_S)
                        self.backend.open(key, **open_args)
                    self._log(
                        f"USB 485 已打开，进行只读探测（第 {reopen_round + 1}/"
                        f"{self.USB_PROBE_REOPEN_ROUNDS} 轮，每轮最多 "
                        f"{self.USB_PROBE_ATTEMPTS_PER_ROUND} 次）……"
                    )
                    try:
                        function, values = self.backend.probe_connection(
                            attempts=self.USB_PROBE_ATTEMPTS_PER_ROUND,
                            retry_delay_s=0.20,
                        )
                        break
                    except DexHandError as exc:
                        last_probe_error = exc
                else:
                    if last_probe_error is not None:
                        raise last_probe_error
                    raise DexHandError("USB 485 只读探测失败")
        except DexHandError as exc:
            self.backend.close()
            self._log(f"连接 DexHand021 S 失败：{exc}")
            if key == "robot":
                if self._is_er_pro_robot():
                    checks = (
                        "xMate ER7Pro-M（XME7p-R850）诊断结论：\n"
                        "1. 本程序未创建、上传或运行 RL 工程，也未调用电脑 COM7；\n"
                        "2. 本程序通过 xCore SDK XPRS485SendData 直连末端 485；\n"
                        "3. 本程序不写入控制器末端工具/供电配置，请以控制器现场实际可见配置为准；\n"
                        "4. 检查末端 485 的 A、B、GND 是否接对，必要时互换 A/B；\n"
                        "5. 灵巧手设备 ID 必须为 1，通信参数为 115200、8N1；\n"
                        "6. 如果日志为 XPRS485SendData 错误码 -1，说明当前 ER7 控制器/SDK"
                        "拒绝直连透传，需要珞石提供支持该机型的直接末端 485 SDK 接口。"
                    )
                else:
                    checks = (
                        "请依次检查：\n"
                        "1. 珞石控制器末端接口是否已接通并保持输出 24 V；\n"
                        "2. 控制器【通信 → xPanel 设置】是否选择 RS485（若页面可用）；\n"
                        "3. 末端 485 的 A、B、GND 是否接对，必要时互换 A/B；\n"
                        "4. 灵巧手设备 ID 是否与界面一致（当前为 "
                        f"{self.device_id_spin.value()}）；\n"
                        "5. 不要同时把电脑 USB 转 485 接到同一条总线。"
                    )
            else:
                checks = (
                    "请依次检查：\n"
                    "1. 灵巧手是否已单独接通 24 V 电源；\n"
                    "2. USB 转 485 的 A、B、GND 是否接对，必要时互换 A/B；\n"
                    f"3. {self.port_edit.text().strip() or '当前串口'} 是否被其他串口工具占用；\n"
                    "4. 设备 ID 是否与界面一致（当前为 "
                    f"{self.device_id_spin.value()}）。"
                )
            QMessageBox.warning(
                self,
                "连接灵巧手失败",
                f"{exc}\n\n{checks}",
            )
            return
        self._set_link_status(True)
        self._log(
            f"已完成只读探测（功能码 0x{function:02X}，P1~P3={values}），"
            "灵巧手响应正常"
        )
        self._hand_control_initialized = False
        self._status_error_count = 0
        if not self._initialize_hand_for_motion():
            self.backend.close()
            self._set_link_status(False)
            QMessageBox.warning(
                self,
                "灵巧手初始化失败",
                "串口只读探测已成功，但清除保护状态失败；已安全断开。"
                "请确认 24 V 电源稳定后重新连接。",
            )
            return
        self.status_timer.start()
        self._poll_status()

    @Slot()
    def _on_disconnect_clicked(self) -> None:
        self.close_hand()

    @Slot()
    def _on_read_status_clicked(self) -> None:
        # R 轴状态探测在失败后会自动暂停，避免影响 P1~P3。用户明确点击
        # 此按钮时，才重置该保护并重新发起一次纯只读探测。
        self.backend.retry_rotation_status_probe()
        self._poll_status(full=True)

    def close_hand(self) -> None:
        self.status_timer.stop()
        self._rebooting = False
        self._on_stop_sequence_clicked(silent=True)
        self._reset_motion_verification()
        self.backend.close()
        self._hand_control_initialized = False
        self._status_error_count = 0
        self.status_extra_lbl.setText("尚未读取状态")
        self._set_link_status(False)

    def set_robot_connected(self, connected: bool) -> None:
        """由主窗口在机器人连接状态变化后调用。"""
        was_connected = self._robot_connected
        self._robot_connected = connected
        if (
            connected
            and not was_connected
            and not self.backend.connected
            and self._is_er_pro_robot()
            and self._transport_key() == "usb"
        ):
            index = self.transport_combo.findData("robot")
            if index >= 0:
                self.transport_combo.setCurrentIndex(index)
                self._log(
                    "已检测到 xMate ER7Pro-M（XME7p-R850）：灵巧手通信方式已自动切换为"
                    "“机器人末端 485（珞石 SDK 裸透传）”；如需电脑直连，"
                    "可手动切回 USB 转 485。"
                )
        if not connected and self._transport_key() == "robot" and self.backend.connected:
            self._log("机器人已断开，自动关闭灵巧手末端 485")
            self.close_hand()

    # -- movement -------------------------------------------------------

    def _confirm(self, title: str, message: str) -> bool:
        box = QMessageBox(self)
        box.setIcon(QMessageBox.Question)
        box.setWindowTitle(title)
        box.setText(message)
        box.setStandardButtons(QMessageBox.Yes | QMessageBox.No)
        box.setDefaultButton(QMessageBox.No)
        return box.exec() == QMessageBox.Yes

    def _run_official_initialization(self) -> None:
        """执行官方 021S 的清错、四轴复位、再清错顺序。"""
        self.backend.clear_error()
        time.sleep(0.05)
        self.backend.reset_joints()
        time.sleep(0.05)
        self.backend.clear_error()
        time.sleep(0.05)

    def _clear_protection_before_motion(self) -> None:
        """在动作前清除一次可能锁存的保护状态。"""
        self.backend.clear_error()
        time.sleep(0.08)

    def _initialize_hand_for_motion(self) -> bool:
        """连接后只清除保护；官方复位改为用户明确点击后执行。"""
        if self._hand_control_initialized:
            return True
        if not self.auto_clear_on_connect_check.isChecked():
            self._hand_control_initialized = True
            self._log("已跳过连接后的自动清错；首次动作前仍会为本次动作清除保护")
            return True
        try:
            self._clear_protection_before_motion()
        except DexHandError as exc:
            self._log(f"连接初始化（清除保护）失败：{exc}")
            return False
        self._hand_control_initialized = True
        self._log("已完成连接初始化：清除保护（未发送复位/运动指令）")
        return True

    def _ensure_hand_ready_for_motion(self) -> bool:
        """首次动作前执行官方初始化；失败时阻止继续发送运动帧。"""
        if self._hand_control_initialized:
            return True
        return self._initialize_hand_for_motion()

    @Slot()
    def _on_reset_joints_clicked(self) -> None:
        if not self._confirm(
            "确认执行官方复位初始化",
            "将执行清错 → P1/P2/P3 Hall=0、R=280 复位 → 再清错，"
            "复位过程会驱动机械手。请确认工作区安全后继续。",
        ):
            return
        try:
            self._run_official_initialization()
        except DexHandError as exc:
            self._hand_control_initialized = False
            self._log(f"官方复位初始化失败：{exc}")
            QMessageBox.warning(self, "官方复位初始化失败", str(exc))
            return
        self._hand_control_initialized = True
        self._log(
            "已手动完成官方动作初始化：清错 → P1/P2/P3 Hall=0、R=280 复位 → 再清错"
        )

    def _reset_motion_verification(self) -> None:
        self.motion_verify_timer.stop()
        self._pending_motion_checks.clear()
        self._motion_verify_started_at = 0.0
        self._motion_verify_settle_count = 0
        self._motion_verify_last_log_at = 0.0
        self._motion_verify_unavailable_logged = False

    def _schedule_motion_feedback(self, axis_id: int, mode: int,
                                  target: int) -> None:
        """登记动作目标，并在初始等待后持续回读直到到位或超时。"""
        if not self.backend.connected or self._sequence_playing:
            return
        if int(mode) == 0x66:
            # 0x66 是限矩模式。控制确认后，机构可能在接触工件、达到 PWM
            # 上限或机械行程前停止，不能把“未到达角度目标”误报成控制失败；
            # 状态页仍会按较低频率显示实际位置。
            return
        # 新动作到来时，以最后一次发送为本次等待的起点；只登记目标，
        # 不自动重发运动帧，避免把总线抖动放大成重复动作。
        self._motion_verify_started_at = time.monotonic()
        self._motion_verify_settle_count = 0
        self._motion_verify_last_log_at = 0.0
        self._motion_verify_unavailable_logged = False
        self._pending_motion_checks[int(axis_id)] = (int(mode), int(target))
        self.motion_verify_timer.start(self.MOTION_VERIFY_DELAY_MS)

    @Slot()
    def _verify_motion_feedback(self) -> None:
        pending = dict(self._pending_motion_checks)
        if not pending or not self.backend.connected or self._rebooting:
            self._reset_motion_verification()
            return
        if self._status_io_busy:
            # 状态轮询正在占用半双工总线，稍后再做这次只读回读。
            self.motion_verify_timer.start(250)
            return
        self._status_io_busy = True
        try:
            include_angles = any(
                mode == 0x44 for mode, _target in pending.values()
            )
            status = self.backend.read_motion_status(
                include_angles=include_angles
            )
        except DexHandError as exc:
            # 485 偶发丢帧时继续等并重试；不要因为一次读失败就把已发送
            # 的动作标成失败，也不要重新发送运动帧。
            timed_out = (
                self._motion_verify_started_at > 0
                and (time.monotonic() - self._motion_verify_started_at)
                >= self.MOTION_VERIFY_TIMEOUT_MS / 1000.0
            )
            if timed_out:
                self._reset_motion_verification()
                self._log(
                    f"动作超过 {self.MOTION_VERIFY_TIMEOUT_MS / 1000:.1f} 秒仍未能完成到位检查；"
                    "请检查 485 通信、保护错误、机械限位、堵转和 24 V 供电"
                )
                return
            if str(exc) != self._last_poll_error:
                self._log(f"动作到位检查暂时读失败，将重试：{exc}")
                self._last_poll_error = str(exc)
            self.motion_verify_timer.start(self.MOTION_VERIFY_POLL_MS)
            return
        finally:
            self._status_io_busy = False
        self._last_poll_error = ""
        now = time.monotonic()
        timed_out = (
            self._motion_verify_started_at > 0
            and (now - self._motion_verify_started_at)
            >= self.MOTION_VERIFY_TIMEOUT_MS / 1000.0
        )
        all_reached = True
        available_count = 0
        status_lines: list[str] = []
        unavailable_axes: list[str] = []
        missing_axes: list[str] = []
        for axis_id, (mode, target) in pending.items():
            axis = status.fingers.get(axis_id)
            if axis is None:
                all_reached = False
                missing_axes.append(AXIS_LABELS[axis_id])
                continue
            if not getattr(axis, "available", True):
                unavailable_axes.append(AXIS_LABELS[axis_id])
                continue
            available_count += 1
            if mode == 0x44:
                desired = target / 10.0
                actual = float(axis.angle_deg)
                delta = abs(actual - desired)
                reached = delta <= self.SEQUENCE_ANGLE_TOLERANCE_DEG
                status_lines.append(
                    f"{AXIS_LABELS[axis_id]} 实际角度={actual:.2f}°/目标={desired:.2f}°，"
                    f"速度={axis.speed_dps}，电流={axis.current_ma} mA"
                )
            else:
                delta = abs(int(axis.hall_position) - target)
                reached = delta <= self.SEQUENCE_HALL_TOLERANCE
                status_lines.append(
                    f"{AXIS_LABELS[axis_id]} Hall={axis.hall_position}/目标={target}，"
                    f"速度={axis.speed_dps}，电流={axis.current_ma} mA"
                )
            if not reached:
                all_reached = False

        if unavailable_axes and not self._motion_verify_unavailable_logged:
            self._log(
                "；".join(unavailable_axes)
                + " 状态寄存器暂不可读，跳过这些轴的位置一致性判断"
            )
            self._motion_verify_unavailable_logged = True
        if status_lines and (
            self._motion_verify_last_log_at <= 0
            or now - self._motion_verify_last_log_at >= 1.0
            or all_reached
            or timed_out
        ):
            suffix = "；继续等待" if not all_reached and not timed_out else ""
            self._log(f"动作到位检查：{'；'.join(status_lines)}{suffix}")
            self._motion_verify_last_log_at = now

        # “没有可读轴”只有在状态明确把所有待检查轴标记为 unavailable 时才算
        # 控制帧已被接受；如果是状态响应缺少某个轴，必须继续等待，不能误判为完成。
        if (available_count == 0 and not missing_axes) or all_reached:
            self._motion_verify_settle_count += 1
        else:
            self._motion_verify_settle_count = 0

        if self._motion_verify_settle_count >= self.MOTION_VERIFY_SETTLE_SAMPLES:
            self._reset_motion_verification()
            if unavailable_axes and available_count == 0:
                self._log("动作控制已确认；所有待检查轴均无可用位置寄存器")
            else:
                self._log("动作已到位，停止到位检查，不重复发送运动指令")
            return

        if timed_out:
            self._reset_motion_verification()
            self._log(
                f"动作超过 {self.MOTION_VERIFY_TIMEOUT_MS / 1000:.1f} 秒仍未到位；"
                "请检查保护错误、机械限位、堵转和 24 V 供电"
            )
            return

        self.motion_verify_timer.start(self.MOTION_VERIFY_POLL_MS)

    def _send_axis(self, axis_id: int, target: Optional[int] = None,
                   mode: Optional[int] = None,
                   control_value: Optional[int] = None) -> None:
        target = self._axis_target(axis_id) if target is None else int(target)
        mode = self._mode() if mode is None else int(mode)
        control_value = (
            self.control_value_spin.value()
            if control_value is None else int(control_value)
        )
        try:
            if not self._ensure_hand_ready_for_motion():
                return
            self.backend.move_finger(axis_id, target, control_value, mode)
        except DexHandError as exc:
            message = self._movement_error_message(axis_id, exc)
            self._log(f"发送 {AXIS_LABELS[axis_id]} 控制失败：{message}")
            QMessageBox.warning(self, "发送控制失败", message)
            return
        self._log(
            f"{AXIS_LABELS[axis_id]} 已发送：C++ SDK 模式=0x{mode:02X}，"
            f"RTU 0x31 Motor_Mode=0x{self._wire_mode(mode):02X}，"
            f"目标={self._format_target_value(axis_id, target, mode)}"
            f"（原始值={target}），控制量={control_value}"
        )
        self._schedule_motion_feedback(axis_id, mode, target)

    def _movement_error_message(self, axis_id: int,
                                error: DexHandError) -> str:
        """为电机拒绝动作补充现场可执行的排查步骤。"""
        message = str(error)
        if "MOT_BLOCK" not in message and "0x08" not in message:
            return message
        axis_label = AXIS_LABELS.get(axis_id, f"电机 {axis_id}")
        return (
            f"{message}\n\n"
            f"{axis_label} 报告堵转/运动受阻。请先停止继续发送动作，检查：\n"
            "1. 手指是否碰到机械限位、桌面或工件；\n"
            "2. 夹持物是否过紧、手指连杆/线缆是否卡住；\n"
            "3. 目标值是否距离当前实际位置过大；\n"
            "4. 电机温度、电压和供电是否正常。\n"
            "排除阻挡后点击“清除错误”，再用较小目标或较低速度单轴测试。"
        )

    def _send_all_targets(self, targets: Optional[dict[int, int]] = None,
                          mode: Optional[int] = None,
                          control_value: Optional[int] = None) -> None:
        mode = self._mode() if mode is None else int(mode)
        control_value = (
            self.control_value_spin.value()
            if control_value is None else int(control_value)
        )
        if targets is None:
            targets = {axis_id: self._axis_target(axis_id) for axis_id in AXIS_IDS}
        if not self._ensure_hand_ready_for_motion():
            raise DexHandError("官方动作初始化失败，已阻止发送运动帧")
        for axis_id in AXIS_IDS:
            try:
                self.backend.move_finger(
                    axis_id, int(targets[axis_id]), control_value, mode
                )
            except DexHandError as exc:
                # 给“发送全部”和动作序列复用与单轴/三指快捷动作相同的
                # 设备错误诊断，并保留当前失败轴，便于现场排查。
                raise DexHandError(
                    f"{self._movement_error_message(axis_id, exc)}"
                ) from exc
            self._set_axis_target(axis_id, int(targets[axis_id]))
            self._schedule_motion_feedback(
                axis_id, mode, int(targets[axis_id])
            )
            if axis_id != AXIS_IDS[-1]:
                time.sleep(0.003)

    def _move_fingers(self, target: int, label: str) -> None:
        mode = self._mode()
        target_text = self._format_target_value(1, target, mode)
        if not self._confirm(
            f"确认三指{label}",
            f"将向 P1/P2/P3 发送{label}目标 {target_text}，请确认工作区安全。",
        ):
            return
        control = 0 if mode == 0x44 else self.control_value_spin.value()
        targets = {axis_id: target for axis_id in FINGER_IDS}
        failed_axis_id = 0
        try:
            if not self._ensure_hand_ready_for_motion():
                return
            for axis_id in FINGER_IDS:
                failed_axis_id = axis_id
                self.backend.move_finger(axis_id, target, control, mode)
                self._set_axis_target(axis_id, target)
                self._schedule_motion_feedback(axis_id, mode, target)
                if axis_id != FINGER_IDS[-1]:
                    time.sleep(0.003)
        except DexHandError as exc:
            message = self._movement_error_message(failed_axis_id, exc)
            self._log(f"三指{label}失败：{message}")
            QMessageBox.warning(self, "三指动作失败", message)
            return
        if mode == 0x66:
            self._log(
                f"三指{label}指令已发送：目标 {target_text}，力矩 {control} PWM；"
                "限矩模式接触物体或达到力矩上限时会提前停止。"
            )
        else:
            self._log(f"三指{label}指令已发送：目标 {target_text}")

    @Slot()
    def _on_close_all_clicked(self) -> None:
        mode = self._mode()
        # 0x44 的原始目标是 75.0°×10；而直发的 0x05/0x06 目标是
        # Hall。说明书的 0x06 力矩控制示例以 Hall=1200 作为闭合目标。
        target = 750 if mode == 0x44 else 1200
        self._move_fingers(target, "闭合")

    @Slot()
    def _on_rotation_zero_clicked(self) -> None:
        if not self._confirm("确认旋转轴归零", "将向 R 旋转轴发送 0 目标，是否继续？"):
            return
        mode = self._mode()
        control = 0 if mode == 0x44 else self.control_value_spin.value()
        try:
            if not self._ensure_hand_ready_for_motion():
                return
            self.backend.move_finger(4, 0, control, mode)
            self._set_axis_target(4, 0)
            self._schedule_motion_feedback(4, mode, 0)
        except DexHandError as exc:
            message = self._movement_error_message(4, exc)
            self._log(f"旋转轴归零失败：{message}")
            QMessageBox.warning(self, "旋转轴归零失败", message)
            return
        self._log("R 旋转轴归零指令已发送")

    @Slot()
    def _on_send_all_clicked(self) -> None:
        if not self._confirm(
            "确认发送全部轴目标",
            "将向 P1/P2/P3/R 四个轴发送当前目标，请确认工作区安全。",
        ):
            return
        try:
            self._send_all_targets()
        except DexHandError as exc:
            self._log(f"发送全部轴目标失败：{exc}")
            QMessageBox.warning(self, "发送全部目标失败", str(exc))
            return
        self._log("P1/P2/P3/R 全部目标已发送")

    @Slot()
    def _on_clear_error_clicked(self) -> None:
        try:
            self._clear_protection_before_motion()
        except DexHandError as exc:
            self._log(f"清除灵巧手错误失败：{exc}")
            QMessageBox.warning(self, "清错失败", str(exc))
            return
        self._hand_control_initialized = True
        self._log("已发送灵巧手清错指令（未发送复位）；可以重新测试单轴动作")

    # -- status ---------------------------------------------------------

    @Slot()
    def _poll_status(self, full: bool = False) -> None:
        if (
            not self.backend.connected
            or self._rebooting
            or self._status_io_busy
            or bool(self._pending_motion_checks)
            or self._sequence_waiting_for_motion
        ):
            return
        self._status_io_busy = True
        status: Optional[HandStatus] = None
        status_detail = ""
        try:
            if full or self.force_check.isChecked() or self.diagnostics_check.isChecked():
                status = self.backend.read_status(
                    self.force_check.isChecked(),
                    self.diagnostics_check.isChecked(),
                )
                status_detail = "完整状态"
            else:
                status = self.backend.read_poll_status()
                status_detail = "快速状态（扭矩/温度/电压为最近完整读取值）"
        except DexHandError as exc:
            message = str(exc)
            if message != self._last_poll_error:
                self._log(f"读取灵巧手状态失败：{message}")
                self._last_poll_error = message
        finally:
            self._status_io_busy = False
        if status is None:
            # 状态轮询失败时保持当前链路和供电不变。自动 close/open 会在
            # 运动过程中清空 USB 缓冲，机器人模式还可能切断末端 24 V，
            # 造成“必须断电重启才能再次连接”的二次故障。用户可用顶部
            # 的断开/连接按钮在确认安全后手动重连。
            return
        self._status_error_count = 0
        self._last_poll_error = ""
        self._update_status_table(status, status_detail)

    def _update_status_table(self, status: HandStatus,
                             status_detail: str = "基础状态") -> None:
        for row, axis_id in enumerate(AXIS_IDS):
            axis = status.fingers.get(axis_id)
            if axis is None:
                continue
            if not getattr(axis, "available", True):
                values = (
                    AXIS_LABELS[axis_id],
                    "不可读",
                    "不可读",
                    "不可读",
                    "不可读",
                    "不可读",
                    "不可读",
                    "不可读",
                )
            else:
                values = (
                    AXIS_LABELS[axis_id],
                    f"{axis.angle_deg:.2f}",
                    str(axis.hall_position),
                    str(axis.speed_dps),
                    str(axis.current_ma),
                    str(axis.torque_pwm),
                    f"{axis.temperature_c:.1f}",
                    f"{axis.voltage_v:.3f}",
                )
            for col, value in enumerate(values):
                self.status_table.setItem(row, col, QTableWidgetItem(value))

            diagnostic_values = (
                AXIS_LABELS[axis_id],
                str(axis.max_output_torque or ""),
                str(axis.max_output_current or ""),
                str(axis.max_speed_dps or ""),
                str(axis.protection_temperature_c or ""),
                str(axis.stall_trigger_ms or ""),
                str(axis.stall_protection_current_ma or ""),
            )
            for col, value in enumerate(diagnostic_values):
                self.diagnostic_table.setItem(row, col, QTableWidgetItem(value))

        for row, finger_id in enumerate(FINGER_IDS):
            finger = status.fingers.get(finger_id)
            if finger is None:
                continue
            force_values = (
                f"P{finger_id}",
                f"{finger.normal_force_n:.3f}",
                str(finger.normal_force_delta),
                f"{finger.tangent_force_n:.3f}",
                str(finger.tangent_force_delta),
                str(finger.tangent_force_angle_deg),
                str(finger.proximity),
            )
            for col, value in enumerate(force_values):
                self.force_table.setItem(row, col, QTableWidgetItem(value))

        import datetime
        stamp = datetime.datetime.fromtimestamp(status.received_at).strftime("%H:%M:%S")
        enabled = []
        if self.force_check.isChecked():
            enabled.append("已读取压力/触觉")
        if self.diagnostics_check.isChecked():
            enabled.append("已读取诊断")
        suffix = "、".join(enabled) if enabled else status_detail
        self.status_extra_lbl.setText(f"更新时间：{stamp}；{suffix}")

    # -- advanced settings ---------------------------------------------

    @Slot()
    def _on_apply_feedback_clicked(self) -> None:
        mode = int(self.feedback_mode_combo.currentData())
        interval = self.feedback_interval_spin.value()
        try:
            self.backend.set_feedback_mode(mode, interval)
        except DexHandError as exc:
            self._log(f"应用反馈设置失败：{exc}")
            QMessageBox.warning(self, "反馈设置失败", str(exc))
            return
        self._log(f"反馈模式已设置为 {mode}，间隔 {interval} ms")

    @Slot()
    def _on_apply_safety_clicked(self) -> None:
        if not self._confirm(
            "确认应用保护参数",
            "将修改灵巧手最大输出电流和舵机保护温度，是否继续？",
        ):
            return
        try:
            self.backend.set_max_current(self.max_current_spin.value())
            self.backend.set_protection_temperature(
                self.protection_temperature_spin.value(),
                self.cooldown_temperature_spin.value(),
            )
        except DexHandError as exc:
            self._log(f"应用保护参数失败：{exc}")
            QMessageBox.warning(self, "保护参数失败", str(exc))
            return
        self._log("最大输出电流与舵机保护温度已应用")

    @Slot()
    def _on_read_device_info_clicked(self) -> None:
        try:
            info = self.backend.read_device_info()
        except DexHandError as exc:
            self._log(f"读取设备信息失败：{exc}")
            QMessageBox.warning(self, "读取设备信息失败", str(exc))
            return
        self._update_device_info(info)
        expected_id = self.device_id_spin.value()
        self._log(
            f"设备信息读取成功：设备 ID={info.device_id}；"
            f"固件版本=0x{info.firmware_version:04X}；"
            f"管理员模式={info.admin_mode}；升级状态={info.upgrade_status}；"
            f"界面当前目标 ID={expected_id}"
        )
        if info.device_id != expected_id:
            self._log(
                f"设备 ID 不一致：设备返回 {info.device_id}，"
                f"界面设置 {expected_id}；请把界面 ID 改为设备返回值后重新连接。"
            )
            QMessageBox.warning(
                self,
                "设备 ID 不一致",
                f"灵巧手返回设备 ID={info.device_id}，"
                f"当前界面设置为 ID={expected_id}。\n"
                "请断开灵巧手，将界面设备 ID 改为返回值后重新连接。",
            )

    def _update_device_info(self, info: DeviceInfo) -> None:
        self.device_info_lbl.setText(
            f"管理员模式：{info.admin_mode}；设备 ID：{info.device_id}；"
            f"固件版本：0x{info.firmware_version:04X}；"
            f"升级状态：{info.upgrade_status}；IAP 标志：{info.iap_upgrade_flag}"
        )

    @Slot()
    def _on_reboot_clicked(self) -> None:
        if not self._confirm(
            "确认重启灵巧手",
            "重启会中断通信并停止当前动作，确认设备处于安全状态后再继续。",
        ):
            return
        # rebootDevice() 的固件流程是非阻塞的；后端会先校验 0x25/0x75
        # 接收确认，随后设备重启串口，不应由后台轮询同时读取这段窗口。
        self.status_timer.stop()
        self._rebooting = True
        self._set_hand_controls_enabled(False)
        self.status_extra_lbl.setText("设备重启中，请等待约 1 秒……")
        try:
            self.backend.reboot()
        except DexHandError as exc:
            self._rebooting = False
            self._set_hand_controls_enabled(True)
            self.status_timer.start()
            self._log(f"重启灵巧手失败：{exc}")
            QMessageBox.warning(self, "重启失败", str(exc))
            return
        self._log("已发送灵巧手重启指令；暂停状态轮询，等待设备重新上线")
        QTimer.singleShot(1200, self._finish_reboot)

    @Slot()
    def _finish_reboot(self) -> None:
        if not self.backend.connected:
            self._rebooting = False
            return
        self._rebooting = False
        self._set_hand_controls_enabled(True)
        self.status_timer.start()
        self._poll_status()
        self._log("灵巧手重启等待结束，已恢复状态轮询")

    @Slot()
    def _on_raw_read_clicked(self) -> None:
        function = int(self.raw_function_combo.currentData())
        start = self.raw_start_spin.value()
        count = self.raw_count_spin.value()
        try:
            values = (
                self.backend.read_holding_registers(start, count)
                if function == 0x03
                else self.backend.read_input_registers(start, count)
            )
        except DexHandError as exc:
            self._log(f"读取寄存器失败：{exc}")
            QMessageBox.warning(self, "读取寄存器失败", str(exc))
            return
        self.raw_result.setPlainText(
            f"功能码：0x{function:02X}\n"
            f"起始地址：0x{start:04X}\n"
            f"数量：{count}\n"
            f"十进制：{values}\n"
            f"十六进制：{' '.join(f'{value:04X}' for value in values)}"
        )
        self._log(f"寄存器读取成功：0x{start:04X}，数量={count}")

    @Slot()
    def _on_raw_write_clicked(self) -> None:
        address = self.raw_write_address_spin.value()
        value = self.raw_write_value_spin.value()
        if not self._confirm(
            "确认写入寄存器",
            f"将写入保持寄存器 0x{address:04X}=0x{value:04X}，请确认地址正确。",
        ):
            return
        try:
            self.backend.write_holding_register(address, value)
        except DexHandError as exc:
            self._log(f"写入寄存器失败：{exc}")
            QMessageBox.warning(self, "写入寄存器失败", str(exc))
            return
        self._log(f"保持寄存器写入成功：0x{address:04X}=0x{value:04X}")

    # -- action sequence ------------------------------------------------

    def _update_sequence_controls(self) -> None:
        connected = self.backend.connected and not self._rebooting
        has_rows = bool(self._sequence_rows)
        self.add_sequence_btn.setEnabled(connected and not self._sequence_playing)
        self.delete_sequence_btn.setEnabled(has_rows and not self._sequence_playing)
        self.clear_sequence_btn.setEnabled(has_rows and not self._sequence_playing)
        self.save_sequence_btn.setEnabled(has_rows and not self._sequence_playing)
        self.load_sequence_btn.setEnabled(not self._sequence_playing)
        self.play_sequence_btn.setEnabled(
            connected and has_rows and not self._sequence_playing
        )
        self.stop_sequence_btn.setEnabled(self._sequence_playing)

    def _refresh_sequence_table(self) -> None:
        self.sequence_table.setRowCount(len(self._sequence_rows))
        for row, step in enumerate(self._sequence_rows):
            mode = int(step["mode"])
            values = [
                str(row + 1),
                f"0x{mode:02X}",
                str(int(step["control"])),
                str(int(step["delay_ms"])),
                *(self._format_target_value(
                    axis_id, int(step["targets"][str(axis_id)]), mode
                )
                  for axis_id in AXIS_IDS),
            ]
            for col, value in enumerate(values):
                self.sequence_table.setItem(row, col, QTableWidgetItem(value))
        status_text = f"共 {len(self._sequence_rows)} 步"
        if self._sequence_playing:
            phase = {
                "sending": "正在发送",
                "waiting": "等待动作完成",
                "delaying": "动作已完成，等待下一步",
            }.get(self._sequence_phase, "正在回放")
            status_text += f"，第 {self._sequence_index + 1} 步：{phase}"
        elif self._sequence_index >= len(self._sequence_rows) and self._sequence_rows:
            status_text += "，回放完成"
        self.sequence_status_lbl.setText(status_text)
        self._update_sequence_controls()

    @Slot()
    def _on_add_sequence_clicked(self) -> None:
        self._sequence_rows.append({
            "mode": self._mode(),
            "control": self.control_value_spin.value(),
            "delay_ms": self.sequence_delay_spin.value(),
            "targets": {
                str(axis_id): self._axis_target(axis_id) for axis_id in AXIS_IDS
            },
        })
        self._refresh_sequence_table()
        self._log("已把当前 P1/P2/P3/R 目标记录为一个动作步骤")

    @Slot()
    def _on_delete_sequence_clicked(self) -> None:
        row = self.sequence_table.currentRow()
        if row < 0 or row >= len(self._sequence_rows):
            return
        self._sequence_rows.pop(row)
        self._refresh_sequence_table()

    @Slot()
    def _on_clear_sequence_clicked(self) -> None:
        if not self._confirm("确认清空序列", "动作序列将被清空，是否继续？"):
            return
        self._sequence_rows.clear()
        self._refresh_sequence_table()

    @Slot()
    def _on_save_sequence_clicked(self) -> None:
        if not self._sequence_rows:
            return
        path, _ = QFileDialog.getSaveFileName(
            self, "保存 DexHand 动作序列", "", "JSON 文件 (*.json)"
        )
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as file:
                json.dump(
                    {
                        "format": "DexHand021S 动作序列",
                        "version": 1,
                        "steps": self._sequence_rows,
                    },
                    file,
                    ensure_ascii=False,
                    indent=2,
                )
        except OSError as exc:
            QMessageBox.warning(self, "保存序列失败", str(exc))
            return
        self._log(f"动作序列已保存：{path}")

    @Slot()
    def _on_load_sequence_clicked(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "加载 DexHand 动作序列", "", "JSON 文件 (*.json)"
        )
        if not path:
            return
        try:
            with open(path, "r", encoding="utf-8") as file:
                payload = json.load(file)
            steps = payload.get("steps", payload) if isinstance(payload, dict) else payload
            if not isinstance(steps, list):
                raise ValueError("JSON 中没有 steps 数组")
            normalized = []
            compatibility_notes: list[str] = []
            legacy_modes = {0x04: 0x44, 0x05: 0x55, 0x06: 0x66}
            for index, step in enumerate(steps, start=1):
                if not isinstance(step, dict):
                    raise ValueError(f"第 {index} 步不是对象")
                mode = int(step["mode"])
                if mode in legacy_modes:
                    compatibility_notes.append(
                        f"第 {index} 步模式 0x{mode:02X} 已兼容转换为 0x{legacy_modes[mode]:02X}"
                    )
                    mode = legacy_modes[mode]
                if mode not in (0x44, 0x55, 0x66):
                    raise ValueError(
                        f"第 {index} 步控制模式 0x{mode:02X} 不支持，必须是 0x44、0x55 或 0x66"
                    )
                raw_control = int(step.get("control", 0))
                if mode == 0x44:
                    control = 0
                    limits = {1: 750, 2: 750, 3: 750, 4: 1600}
                elif mode == 0x55:
                    control = min(
                        max(raw_control, self.backend.HALL_SPEED_MIN),
                        self.backend.HALL_SPEED_MAX,
                    )
                    limits = {1: 1200, 2: 1200, 3: 1200, 4: 1600}
                else:
                    control = min(
                        max(raw_control, self.backend.TORQUE_MIN),
                        self.backend.TORQUE_MAX,
                    )
                    # 直发的 RTU 0x06 目标字段为 Hall，而不是角度×10。
                    limits = {1: 1200, 2: 1200, 3: 1200, 4: 1600}
                if control != raw_control:
                    compatibility_notes.append(
                        f"第 {index} 步控制量已调整为 {control}（符合 021S 范围）"
                    )
                delay_ms = int(step.get("delay_ms", 500))
                targets = step["targets"]
                target_map = {}
                for axis_id in AXIS_IDS:
                    raw_target = int(targets[str(axis_id)])
                    target = min(max(raw_target, 0), limits[axis_id])
                    if target != raw_target:
                        compatibility_notes.append(
                            f"第 {index} 步 {AXIS_LABELS[axis_id]} 目标已限制为 {target}"
                        )
                    target_map[str(axis_id)] = target
                normalized.append({
                    "mode": mode,
                    "control": control,
                    "delay_ms": max(20, delay_ms),
                    "targets": target_map,
                })
            self._sequence_rows = normalized
            self._refresh_sequence_table()
        except (OSError, ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
            QMessageBox.warning(self, "加载序列失败", str(exc))
            return
        self._log(f"动作序列已加载：{path}")
        if compatibility_notes:
            self._log("旧动作序列已按 DexHand021 S 官方模式/范围兼容：" + "；".join(compatibility_notes))

    @Slot()
    def _on_play_sequence_clicked(self) -> None:
        if not self._sequence_rows or not self.backend.connected:
            return
        if not self._confirm(
            "确认回放动作序列",
            f"将连续发送 {len(self._sequence_rows)} 步 P1/P2/P3/R 动作，请确认工作区安全。",
        ):
            return
        self._sequence_index = 0
        self._sequence_playing = True
        self._sequence_waiting_for_motion = False
        self._sequence_phase = "sending"
        self._refresh_sequence_table()
        self._play_sequence_step()

    @Slot()
    def _on_stop_sequence_clicked(self, silent: bool = False) -> None:
        was_playing = self._sequence_playing
        self.sequence_timer.stop()
        self.sequence_motion_timer.stop()
        self._sequence_playing = False
        self._sequence_waiting_for_motion = False
        self._sequence_phase = ""
        self._sequence_index = 0
        self._refresh_sequence_table()
        if was_playing and not silent:
            self._log("动作序列回放已停止")

    def _play_sequence_step(self) -> None:
        if not self._sequence_playing:
            return
        if self._sequence_index >= len(self._sequence_rows):
            self._sequence_playing = False
            self.sequence_motion_timer.stop()
            self._sequence_waiting_for_motion = False
            self._sequence_phase = ""
            self._refresh_sequence_table()
            self._log("动作序列回放完成")
            return
        step = self._sequence_rows[self._sequence_index]
        self._sequence_phase = "sending"
        self._refresh_sequence_table()
        try:
            self._send_all_targets(
                targets={
                    axis_id: int(step["targets"][str(axis_id)])
                    for axis_id in AXIS_IDS
                },
                mode=int(step["mode"]),
                control_value=int(step["control"]),
            )
        except DexHandError as exc:
            self._sequence_playing = False
            self.sequence_timer.stop()
            self.sequence_motion_timer.stop()
            self._sequence_waiting_for_motion = False
            self._sequence_phase = ""
            self._refresh_sequence_table()
            message = str(exc)
            self._log(f"动作序列第 {self._sequence_index + 1} 步失败：{message}")
            QMessageBox.warning(self, "动作序列失败", message)
            return
        self._sequence_waiting_for_motion = True
        self._sequence_wait_started_at = time.monotonic()
        self._sequence_settle_count = 0
        self._sequence_pending_delay_ms = max(20, int(step.get("delay_ms", 500)))
        self._sequence_phase = "waiting"
        self._refresh_sequence_table()
        self.sequence_motion_timer.start()
        # 发送后立即检查一次，目标本来就已到位时不必等待完整轮询周期。
        self._check_sequence_motion()

    def _sequence_target_reached(self, axis_id: int, mode: int,
                                 target: int, status: Any) -> bool:
        """根据控制模式判断某个轴是否已达到本步目标。"""
        if mode == 0x44:
            # C++ SDK 0x44 的控制帧目标为角度×10，状态角度按度返回。
            return (
                abs(float(status.angle_deg) - target / 10.0)
                <= self.SEQUENCE_ANGLE_TOLERANCE_DEG
                and abs(int(status.speed_dps))
                <= self.SEQUENCE_SPEED_SETTLE_THRESHOLD
            )
        # 直发 RTU 的 0x05/0x06 目标字段都是 Hall 值，需要同时满足位置
        # 误差和速度稳定。
        return (
            abs(int(status.hall_position) - target)
            <= self.SEQUENCE_HALL_TOLERANCE
            and abs(int(status.speed_dps))
            <= self.SEQUENCE_SPEED_SETTLE_THRESHOLD
        )

    def _sequence_motion_complete(self, status: HandStatus, mode: int,
                                  targets: dict[int, int]) -> bool:
        """判断可读状态的轴是否均已完成，并要求连续多次满足条件。

        某些 021S 固件不提供 R 轴状态寄存器，但 R 轴控制帧仍能正常
        下发。不可读轴不能把动作序列永久卡在“等待动作完成”。
        """
        available_count = 0
        for axis_id in AXIS_IDS:
            axis = status.fingers.get(axis_id)
            if axis is None:
                return False
            if not getattr(axis, "available", True):
                continue
            available_count += 1
            if not self._sequence_target_reached(
                axis_id, mode, int(targets[axis_id]), axis
            ):
                return False
        return available_count > 0

    def _sequence_motion_failed(self, message: str) -> None:
        step_number = self._sequence_index + 1
        self.sequence_timer.stop()
        self.sequence_motion_timer.stop()
        self._sequence_playing = False
        self._sequence_waiting_for_motion = False
        self._sequence_phase = ""
        self._refresh_sequence_table()
        full_message = f"动作序列第 {step_number} 步等待动作完成失败：{message}"
        self._log(full_message)
        QMessageBox.warning(self, "动作序列等待失败", full_message)

    @Slot()
    def _check_sequence_motion(self) -> None:
        if not self._sequence_playing or not self._sequence_waiting_for_motion:
            self.sequence_motion_timer.stop()
            return
        if not self.backend.connected or self._rebooting:
            self._sequence_motion_failed("灵巧手已断开或正在重启")
            return
        elapsed_ms = int((time.monotonic() - self._sequence_wait_started_at) * 1000)
        if elapsed_ms >= self.SEQUENCE_MOTION_TIMEOUT_MS:
            self._sequence_motion_failed(
                f"超过 {self.SEQUENCE_MOTION_TIMEOUT_MS / 1000:.1f} 秒；"
                "请检查目标值、机械限位、堵转错误和状态反馈"
            )
            return
        if self._status_io_busy:
            return
        step = self._sequence_rows[self._sequence_index]
        mode = int(step["mode"])
        targets = {
            axis_id: int(step["targets"][str(axis_id)])
            for axis_id in AXIS_IDS
        }
        self._status_io_busy = True
        try:
            status = self.backend.read_motion_status(include_angles=(mode == 0x44))
        except DexHandError as exc:
            # 485 是半双工链路，偶发读失败时继续等待并重试；只有达到
            # 动作超时才停止序列，避免一次总线抖动直接跳到下一步。
            if str(exc) != self._last_poll_error:
                self._log(f"动作序列等待反馈失败，将重试：{exc}")
                self._last_poll_error = str(exc)
            return
        finally:
            self._status_io_busy = False
        self._last_poll_error = ""
        if self._sequence_motion_complete(status, mode, targets):
            self._sequence_settle_count += 1
        else:
            self._sequence_settle_count = 0
        if self._sequence_settle_count < self.SEQUENCE_SETTLE_SAMPLES:
            return

        self.sequence_motion_timer.stop()
        self._sequence_waiting_for_motion = False
        completed_step = self._sequence_index + 1
        self._sequence_index += 1
        if self._sequence_index >= len(self._sequence_rows):
            self._sequence_playing = False
            self._sequence_phase = ""
            self._refresh_sequence_table()
            self._log(f"动作序列第 {completed_step} 步动作完成，动作序列回放完成")
            return
        self._sequence_phase = "delaying"
        self._refresh_sequence_table()
        self._log(
            f"动作序列第 {completed_step} 步动作已完成，"
            f"{self._sequence_pending_delay_ms} ms 后执行第 {self._sequence_index + 1} 步"
        )
        self.sequence_timer.start(self._sequence_pending_delay_ms)
