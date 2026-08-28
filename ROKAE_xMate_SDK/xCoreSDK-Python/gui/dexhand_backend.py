"""DexHand021 S 的 Modbus RTU 后端。

021S 的 USB 转 485 与珞石机器人末端 485 都使用同一套裸 Modbus RTU 帧。
机器人路径直接调用 xCore SDK 暴露的 XPRS485SendData；本模块不创建、上传
或运行机器人 RL 工程，也不依赖工程变量和 Print 回传。这个模块只负责协议
编码/解码和传输层，界面层不需要知道帧的具体格式。

协议依据：DexHand021 S《使用说明书》V1.7 第六章：
* 115200、8N1；
* 0x03 读取保持寄存器，0x04 读取输入寄存器，0x06 写单寄存器；
 * 0x25 全局设置，0x31 电机控制；
 * CRC16-Modbus，低字节在前。

021S 的 0x31 控制帧中，电机/轴 ID 为：
* 1：P1/手指 1；2：P2/手指 2；3：P3/手指 3；4：旋转轴。

控制模式在 C++ SDK API 中使用 0x44/0x55/0x66，编码到 0x31 帧的
Motor_Mode 字段时分别为 0x04/0x05/0x06。

注意：本模块经 USB-485 或珞石末端透传直接发送的是 *原始* 0x31
RTU 帧，目标字段必须遵循《DexHand021 S 使用说明书》6.6：

* Motor_Mode 0x04：目标位置为关节角度 × 10；
* Motor_Mode 0x05、0x06：目标位置为 Hall 位置。

不能把 C++ SDK 高层 API 文档中的参数语义直接套到裸 RTU 帧。尤其是
0x06 力矩控制若把 75.0° 直接写成 750，设备会把它当作 Hall=750，而
不是 75.0°。
"""

from __future__ import annotations

import os
import json
import re
import struct
import tempfile
import threading
import time
import uuid
import zipfile
from dataclasses import dataclass, field
from typing import Any, Callable, Optional


class DexHandError(RuntimeError):
    """灵巧手通信或协议错误。"""


def format_frame(data: bytes | bytearray) -> str:
    """Format an RTU frame for the diagnostic log."""
    # Python 3.7 的 bytes.hex() 不支持分隔符参数；逐字节格式化同时
    # 兼容 Python 3.7 及新版 Python，避免详细日志本身遮蔽通信错误。
    return " ".join(f"{int(value) & 0xFF:02X}" for value in data) or "空"


def crc16_modbus(data: bytes | bytearray) -> int:
    """返回 Modbus CRC16 数值。"""
    crc = 0xFFFF
    for value in data:
        crc ^= int(value)
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 0x0001 else crc >> 1
    return crc & 0xFFFF


def with_crc(payload: bytes | bytearray) -> bytes:
    crc = crc16_modbus(payload)
    return bytes(payload) + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def verify_crc(frame: bytes | bytearray) -> None:
    if len(frame) < 4:
        raise DexHandError(f"485 回复帧过短：{len(frame)} 字节")
    expected = crc16_modbus(frame[:-2])
    actual = int(frame[-2]) | (int(frame[-1]) << 8)
    if expected != actual:
        raise DexHandError(
            f"CRC 校验失败：期望 0x{expected:04X}，收到 0x{actual:04X}"
        )


def _u16(value: int) -> bytes:
    return bytes(((value >> 8) & 0xFF, value & 0xFF))


def _u16_le(value: int) -> bytes:
    return bytes((value & 0xFF, (value >> 8) & 0xFF))


def _s16(value: int) -> bytes:
    return _u16(value & 0xFFFF)


def read_register_frame(device_id: int, function: int,
                        start: int, count: int) -> bytes:
    if function not in (0x03, 0x04):
        raise ValueError("读取寄存器功能码必须是 0x03 或 0x04")
    if not 1 <= device_id <= 0x7F:
        raise ValueError("021S 设备 ID 范围为 1~127")
    if not 0 <= start <= 0xFFFF or not 1 <= count <= 0x7D:
        raise ValueError("寄存器地址或数量超出范围")
    return with_crc(bytes((device_id, function)) + _u16(start) + _u16(count))


def read_input_frame(device_id: int, start: int, count: int) -> bytes:
    return read_register_frame(device_id, 0x04, start, count)


def write_single_register_frame(device_id: int, address: int, value: int) -> bytes:
    if not 0 <= address <= 0xFFFF or not 0 <= value <= 0xFFFF:
        raise ValueError("寄存器地址或数值超出范围")
    return with_crc(bytes((device_id, 0x06)) + _u16(address) + _u16(value))


def global_setting_frame(device_id: int, command: int, motor_id: int = 0,
                         valid_length: int = 0, data0: int = 0,
                         data1: int = 0) -> bytes:
    """构造 DexHand021 S 官方 0x25 全局设置帧。

    RTU 手册规定 0x25 后依次为 command、motor_id、有效数据长度和
    两个 Uint16 数据槽位。有效长度只描述数据槽位中有多少字节有效，
    四个数据字节本身仍必须全部放在帧中；0x25 后不能额外插入 0x03。
    """
    if not 1 <= device_id <= 0x7F:
        raise ValueError("021S 设备 ID 范围为 1~127")
    if not 0 <= command <= 0xFF or not 0 <= motor_id <= 0xFF:
        raise ValueError("全局设置命令或电机 ID 超出范围")
    if not 0 <= valid_length <= 4:
        raise ValueError("全局设置有效数据长度超出范围")
    data = _u16_le(data0 & 0xFFFF) + _u16_le(data1 & 0xFFFF)
    payload = bytes((
        device_id,
        0x25,
        command,
        motor_id,
        valid_length,
    )) + data
    # 0x25 是 021S 自定义功能码；四个数据字节按小端序完整发送。
    return with_crc(payload)


def feedback_mode_frame(device_id: int, mode: int = 2,
                        interval_ms: int = 20) -> bytes:
    """设置为 Modbus RTU 问询反馈模式（0x25/0x23）。"""
    if mode not in (0, 1, 2):
        raise ValueError("反馈模式只能是 0、1 或 2")
    if not 10 <= interval_ms <= 20:
        raise ValueError("反馈间隔应在 10~20 ms")
    # 手册规定全局设置数据为小端：低字节是模式，高字节是间隔。
    return global_setting_frame(
        device_id, 0x23, motor_id=0, valid_length=3,
        data0=(mode & 0xFF) | ((interval_ms & 0xFF) << 8), data1=0x0403,
    )


def clear_error_frame(device_id: int) -> bytes:
    return global_setting_frame(device_id, 0xA4)


def global_setting_command_frame(device_id: int, command: int,
                                 data0: int = 0, data1: int = 0,
                                 valid_length: int = 0,
                                 motor_id: int = 0) -> bytes:
    """构造一个带命令语义的 0x25 全局设置帧。"""
    return global_setting_frame(
        device_id,
        command,
        motor_id=motor_id,
        valid_length=valid_length,
        data0=data0,
        data1=data1,
    )


def set_max_current_frame(device_id: int, current_ma: int) -> bytes:
    if not 200 <= current_ma <= 500:
        raise ValueError("最大输出电流范围为 200~500 mA")
    return global_setting_command_frame(
        device_id, 0x65, data0=current_ma, valid_length=2
    )


def set_protection_temperature_frame(device_id: int, temperature_c: int,
                                     cooldown_c: int = 10) -> bytes:
    if not 40 <= temperature_c <= 90:
        raise ValueError("舵机保护温度范围为 40~90 ℃")
    if not 3 <= cooldown_c <= 20:
        raise ValueError("降温幅度范围为 3~20 ℃")
    # 0x6D 的两个 Uint16 数据槽位分别携带保护温度和降温幅度。
    return global_setting_command_frame(
        device_id, 0x6D, data0=temperature_c, data1=cooldown_c,
        valid_length=3
    )


def reboot_frame(device_id: int) -> bytes:
    return global_setting_command_frame(device_id, 0x75)


# C++ SDK 对外暴露的 MotorControlMode 与 0x31 RTU 帧内 Motor_Mode
# 不是同一层的数值。SDK 在串口发送前会做下面这层转换：
#   CASCADED_PID_CONTROL_MODE  (0x44) -> Motor_Mode 0x04
#   HALL_POSLIMIT_CONTROL_MODE (0x55) -> Motor_Mode 0x05
#   CASCADED_MIT_CONTROL_MODE  (0x66) -> Motor_Mode 0x06
# 0x04/0x05/0x06 只是 0x31 自定义控制帧里的模式字段；它们不能替代
# Modbus 读写请求本身的功能码 0x03/0x04/0x06。
SDK_MODE_TO_RTU_MOTOR_MODE = {
    0x44: 0x04,
    0x55: 0x05,
    0x66: 0x06,
}
RTU_MOTOR_MODE_TO_SDK_MODE = {
    raw_mode: sdk_mode
    for sdk_mode, raw_mode in SDK_MODE_TO_RTU_MOTOR_MODE.items()
}


def move_frame(device_id: int, finger_id: int, mode: int,
              target: int, control_value: int = 0) -> bytes:
    """构造 0x31 电机控制帧。

    0x44：级联 PID 角度位置模式，目标为角度×10；
    0x55：带限制霍尔位置模式，目标为 Hall 值，控制量为速度；
    0x66：级联 MIT/力矩模式，目标为 Hall 值，控制量为力矩。
    ``mode`` 参数严格使用 C++ SDK ``MotorControlMode`` 的枚举值
    0x44/0x55/0x66；发送到 RTU 时，本模块把它编码为 0x31 帧内的
    Motor_Mode=0x04/0x05/0x06。三种模式的数据字段均为小端序。

    本函数不经 dexhand.dll 的高层参数换算，因此 0x05/0x06 必须传入
    原始 Hall 目标，不能传入“角度×10”。
    """
    if not 1 <= device_id <= 0x7F:
        raise ValueError("021S 设备 ID 范围为 1~127")
    if not 1 <= finger_id <= 4:
        raise ValueError("DexHand021 S 电机/轴 ID 范围为 1~4")
    if mode not in (0x44, 0x55, 0x66):
        raise ValueError("DexHand021 S 控制模式必须是 0x44、0x55 或 0x66")
    if not 0 <= target <= 0xFFFF or not 0 <= control_value <= 0xFFFF:
        raise ValueError("控制目标超出 16 位无符号范围")
    rtu_motor_mode = SDK_MODE_TO_RTU_MOTOR_MODE[mode]
    payload = bytes((device_id, 0x31, rtu_motor_mode, finger_id))
    # C++ SDK 的 0x31 动作参数按小端序编码。
    payload += bytes((target & 0xFF, (target >> 8) & 0xFF))
    payload += bytes((control_value & 0xFF, (control_value >> 8) & 0xFF))
    return with_crc(payload)


def _signed16(value: int) -> int:
    return value - 0x10000 if value & 0x8000 else value


def parse_register_response(frame: bytes, device_id: int,
                            function: int, count: int) -> list[int]:
    verify_crc(frame)
    if len(frame) < 5:
        raise DexHandError("寄存器回复帧过短")
    if frame[0] != device_id:
        raise DexHandError(
            f"设备 ID 不匹配：期望 {device_id}，收到 {frame[0]}"
        )
    if frame[1] == (function | 0x80):
        code = frame[2] if len(frame) > 2 else -1
        names = {
            0x01: "非法功能",
            0x02: "非法数据地址",
            0x03: "非法数据值",
            0x04: "设备执行失败",
        }
        raise DexHandError(
            f"灵巧手返回 Modbus 异常：功能码 0x{function:02X}，"
            f"异常码 0x{code:02X}（{names.get(code, '未知异常')}），"
            f"回复帧={frame.hex(' ')}"
        )
    if frame[1] != function:
        raise DexHandError(
            f"功能码不匹配：期望 0x{function:02X}，收到 0x{frame[1]:02X}"
        )
    byte_count = frame[2]
    if byte_count != count * 2 or len(frame) != 5 + byte_count:
        raise DexHandError(
            f"寄存器回复长度不匹配：声明 {byte_count} 字节，实际 {len(frame)} 字节"
        )
    data = frame[3:-2]
    return [int.from_bytes(data[i:i + 2], "big")
            for i in range(0, len(data), 2)]


def validate_ack(frame: bytes, device_id: int, function: int,
                 expected_len: Optional[int] = None) -> None:
    verify_crc(frame)
    # Modbus 异常响应固定为 5 字节（ID、功能码|0x80、异常码、CRC）。
    # 必须先解析它，再检查正常响应长度，否则现场只会看到“收到 5 字节”，
    # 看不到设备实际返回的异常原因。
    if len(frame) >= 2 and frame[1] == (function | 0x80):
        code = frame[2] if len(frame) > 2 else -1
        names = {
            0x01: "非法功能",
            0x02: "非法数据地址",
            0x03: "非法数据值",
            0x04: "设备执行失败",
        }
        detail = names.get(code, "未知异常")
        raise DexHandError(
            f"灵巧手返回 Modbus 异常：功能码 0x{function:02X}，"
            f"异常码 0x{code:02X}（{detail}），回复帧={frame.hex(' ')}"
        )
    if expected_len is not None and len(frame) != expected_len:
        raise DexHandError(
            f"应答长度不匹配：期望 {expected_len}，收到 {len(frame)}；"
            f"回复帧={frame.hex(' ')}"
        )
    if frame[0] != device_id:
        raise DexHandError(
            f"设备 ID 不匹配：期望 {device_id}，收到 {frame[0]}"
        )
    if frame[1] == (function | 0x80):
        code = frame[2] if len(frame) > 2 else -1
        raise DexHandError(f"灵巧手返回异常，功能码 0x{function:02X}，错误码 0x{code:02X}")
    if frame[1] != function:
        raise DexHandError(
            f"功能码不匹配：期望 0x{function:02X}，收到 0x{frame[1]:02X}"
        )


def validate_global_ack(frame: bytes, device_id: int, command: int) -> None:
    """校验 0x25 全局设置反馈。

    021S 手册规定该反馈为 6 字节：ID、0x25、命令、结果、CRC16。
    结果字节为 1 表示成功，0 表示失败。
    """
    validate_ack(frame, device_id, 0x25, expected_len=6)
    if frame[2] != command:
        raise DexHandError(
            f"全局设置命令不匹配：期望 0x{command:02X}，收到 0x{frame[2]:02X}"
        )
    if frame[3] != 1:
        raise DexHandError(
            f"全局设置命令 0x{command:02X} 执行失败，结果码 0x{frame[3]:02X}"
        )


MOTOR_ERROR_FLAGS_21S = {
    # Names and values follow MotorErrorCode_21S in the supplied C++ SDK.
    0x01: "CRT_ERROR（电气/电流异常）",
    0x02: "VTG_ERROR（电压异常）",
    0x04: "OVER_HEAT（过热）",
    0x08: "MOT_BLOCK（电机堵转/运动受阻）",
    0x10: "MOT_ERROR（电机内部错误）",
}


def describe_motor_error_21s(error_flags: int) -> str:
    """将 DexHand021 S 的电机错误位翻译为可操作的中文提示。"""
    flags = int(error_flags) & 0xFF
    if flags == 0:
        return "无错误"
    names = [
        name for bit, name in MOTOR_ERROR_FLAGS_21S.items()
        if flags & bit
    ]
    known_mask = 0
    for bit in MOTOR_ERROR_FLAGS_21S:
        known_mask |= bit
    unknown = flags & ~known_mask
    if unknown:
        names.append(f"未定义错误位 0x{unknown:02X}")
    return "；".join(names)


def validate_control_ack(frame: bytes, device_id: int, motor_id: int) -> None:
    """校验 0x31 电机控制反馈。

    反馈格式为 ID、0x31、Motor_id、错误标志、CRC16；错误标志为 0 才是成功。
    """
    validate_ack(frame, device_id, 0x31, expected_len=6)
    if frame[2] != motor_id:
        raise DexHandError(
            f"电机 ID 不匹配：期望 {motor_id}，收到 {frame[2]}"
        )
    if frame[3] != 0:
        raise DexHandError(
            f"电机 {motor_id} 控制被灵巧手拒绝，错误标志 0x{frame[3]:02X}："
            f"{describe_motor_error_21s(frame[3])}"
        )


class Transport:
    """021S 请求-应答传输层接口。"""

    max_frame_bytes: Optional[int] = None

    def open(self) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    def request(self, frame: bytes, response_len: int) -> bytes:
        raise NotImplementedError


class SerialTransport(Transport):
    """电脑 USB 转 485 传输层，使用 pyserial。"""

    # CH340 类 USB-RS485 适配器打开后需要等待收发方向和驱动缓冲区稳定；
    # 如果立刻发送第一帧，第一轮只读探测经常收到 0 字节。
    OPEN_SETTLE_S = 0.50
    TURNAROUND_S = 0.006
    REQUEST_GAP_S = 0.025
    READ_RETRY_COUNT = 1
    READ_RETRY_DELAY_S = 0.035

    def __init__(self, port: str, baudrate: int = 115200,
                 timeout_s: float = 0.35,
                 log_fn: Optional[Callable[[str], None]] = None,
                 trace_fn: Optional[Callable[[str], None]] = None) -> None:
        self.port = port.strip()
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self._serial: Any = None
        self._last_request_at = 0.0
        self._log_fn = log_fn or (lambda _message: None)
        self._trace_fn = trace_fn or (lambda _message: None)

    def _trace(self, message: str) -> None:
        self._trace_fn(f"USB 485 {message}")

    def open(self) -> None:
        if not self.port:
            raise DexHandError("USB 转 485 模式需要填写串口，例如 COM7")
        try:
            import serial  # type: ignore
        except ImportError as exc:
            raise DexHandError(
                "当前 Python 环境缺少 pyserial，请执行："
                "python -m pip install pyserial"
            ) from exc
        try:
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout_s,
                write_timeout=self.timeout_s,
                # 021S 是半双工 RS485；不使用硬件流控，避免 CH340 的
                # RTS/CTS 状态把收发方向或数据发送卡住。
                rtscts=False,
                dsrdtr=False,
            )
            self._serial.reset_input_buffer()
            self._last_request_at = 0.0
            time.sleep(self.OPEN_SETTLE_S)
            self._log_fn(
                f"USB 485 串口参数：端口={self.port}，波特率={self.baudrate}，"
                "数据位=8，校验=None，停止位=1，硬件流控=关闭"
            )
        except Exception as exc:  # serial.SerialException varies by backend
            self._serial = None
            raise DexHandError(f"打开串口 {self.port} 失败：{exc}") from exc

    def close(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            finally:
                self._serial = None
        self._last_request_at = 0.0

    def _wait_request_gap(self) -> None:
        remaining = self.REQUEST_GAP_S - (
            time.monotonic() - self._last_request_at
        )
        if remaining > 0:
            time.sleep(remaining)

    def _request_once(self, frame: bytes, response_len: int) -> bytes:
        if self._serial is None:
            raise DexHandError("USB 转 485 尚未连接")
        self._trace(
            f"发送：长度={len(frame)}，期望回复={response_len} 字节，"
            f"帧={format_frame(frame)}"
        )
        try:
            self._serial.reset_input_buffer()
            self._serial.write(frame)
            self._serial.flush()
            # 给 USB-RS485 适配器留出收发方向切换和总线帧间隔时间。
            # 021S 手册要求帧间隔不小于 4 个字符；CH340 在 Windows 下
            # 有时还需要额外的 USB 调度时间，6 ms 比 2 ms 更稳妥。
            time.sleep(self.TURNAROUND_S)
            # 0x75 重启和部分维护指令是“只发送、不等待应答”的命令。
            # 若继续 read(6)，设备重启期间必然会被误报成超时。
            if response_len == 0:
                self._trace("回复：不等待（非阻塞指令）")
                return b""

            # 不要一次性按“正常响应长度”读取。设备返回 Modbus 异常时，
            # 响应只有 5 字节；一次性 read(11) 会把真正的异常码隐藏成超时。
            # 先读取地址、功能码和第三个字段，再按帧类型确定剩余长度。
            header = bytes(self._serial.read(min(3, response_len)))
            if len(header) < min(3, response_len):
                data = header
            elif len(header) >= 2 and header[1] == (frame[1] | 0x80):
                data = header + bytes(self._serial.read(2))
            elif frame[1] in (0x03, 0x04):
                byte_count = header[2]
                actual_len = 5 + byte_count
                remaining = max(0, actual_len - len(header))
                data = header + bytes(self._serial.read(remaining))
            else:
                remaining = max(0, response_len - len(header))
                data = header + bytes(self._serial.read(remaining))
        except Exception as exc:
            self._trace(f"底层异常：{exc}")
            raise DexHandError(f"串口收发失败：{exc}") from exc
        self._trace(
            f"接收：长度={len(data)}，帧={format_frame(data)}"
        )
        if len(data) != response_len:
            # 对寄存器读取来说，异常帧的合法长度是 5；交给上层解析，
            # 这样用户能看到“非法地址/设备执行失败”等真实原因。
            is_exception = (
                len(data) == 5 and len(data) >= 2
                and data[1] == (frame[1] | 0x80)
            )
            if is_exception:
                return data
            raise DexHandError(
                f"串口回复超时：期望 {response_len} 字节，收到 {len(data)} 字节；"
                f"请求帧={frame.hex(' ')}；收到帧={data.hex(' ') or '空'}"
            )
        return data

    def request(self, frame: bytes, response_len: int) -> bytes:
        """执行一笔事务；只对读请求做一次安全重试。

        运动帧不能盲目重发，否则一次 USB 抖动可能把同一个动作发送两次。
        0x03/0x04 是幂等读请求，收到 0 字节或短帧时可以在清空输入缓冲后
        重试一次，这正好覆盖 CH340 首帧丢失和 RS485 收发方向切换的情况。
        """
        is_read = (
            response_len > 0 and len(frame) >= 2 and frame[1] in (0x03, 0x04)
        )
        attempts = 1 + self.READ_RETRY_COUNT if is_read else 1
        last_error: Optional[DexHandError] = None
        for attempt in range(attempts):
            self._wait_request_gap()
            try:
                result = self._request_once(frame, response_len)
                self._last_request_at = time.monotonic()
                return result
            except DexHandError as exc:
                last_error = exc
                self._last_request_at = time.monotonic()
                if attempt + 1 >= attempts:
                    break
                time.sleep(self.READ_RETRY_DELAY_S)
        if last_error is not None:
            raise last_error
        raise DexHandError("USB 转 485 请求失败")


class RobotPanelTransport(Transport):
    """珞石机器人末端 xPanel 485 传输层。

    xCore SDK 的 XPRS485SendData 限制单次收发各 16 字节，因此协议层会
    将寄存器读取拆成能够在 16 字节内完成的请求。
    """

    max_frame_bytes = 16
    # xPanel 在上一次会话异常结束后可能仍保持 RS485 状态。连接前做一次
    # 关闭/打开复位，并给末端电源和 021S 固件留出启动时间，避免必须手动
    # 拔插灵巧手电源才能重新上线。
    POWER_OFF_SETTLE_S = 0.35
    POWER_ON_SETTLE_S = 0.80
    REQUEST_GAP_S = 0.035

    def __init__(self, robot_backend: Any, sdk_module: Any,
                 voltage_option: int = 3,
                 log_fn: Optional[Callable[[str], None]] = None,
                 trace_fn: Optional[Callable[[str], None]] = None) -> None:
        self.robot_backend = robot_backend
        self.sdk = sdk_module
        self.voltage_option = voltage_option
        self._log_fn = log_fn or (lambda _msg: None)
        self._trace_fn = trace_fn or (lambda _msg: None)
        self._opened = False
        self._last_request_at = 0.0
        # xMate ER7Pro-M 的末端工具配置由控制器现场配置负责；
        # ER Pro 的 open/close 不调用其它 xPanel 机型的配置接口，避免
        # 把不存在的供电选项写入控制器或因错误码 -290 影响末端链路。
        self._er_pro_mode = False

    def _trace(self, message: str) -> None:
        channel = "末端工具 485" if self._er_pro_mode else "xPanel 485"
        self._trace_fn(f"珞石 {channel} {message}")

    def _check(self, ec: dict, name: str) -> None:
        code = ec.get("ec", 0)
        if code != 0:
            if code == -1 and name == "XPRS485SendData" and self._er_pro_mode:
                raise DexHandError(
                    "珞石控制器拒绝 xCore SDK 的 XPRS485SendData（错误码 -1）。"
                    "当前程序未上传、生成或运行 RL 工程；这表示 ER7Pro-M 当前固件/"
                    "控制器没有接受 SDK 直连的末端 485 裸透传请求。请确认控制器已启用"
                    "末端工具 RS485、末端供电已接通，并向珞石索取支持 ER7Pro-M 的"
                    "直接末端 485 SDK 接口；当前 Python SDK 没有 XPRS485RWData 方法。"
                )
            raise DexHandError(
                f"{name} 失败（错误码 {code}）：{ec.get('message', '无详细信息')}"
            )

    def _is_er_pro_robot(self, robot: Any) -> bool:
        """识别 xMateErProRobot SDK 类型，使用 ER Pro 的直连透传路径。"""
        backend_name = str(getattr(self.robot_backend, "robot_type_name", ""))
        robot_name = str(type(robot).__name__)
        names = f"{backend_name} {robot_name}".replace("_", "").lower()
        return "xmateerpro" in names or "erpro" in names

    def open(self) -> None:
        if not getattr(self.robot_backend, "connected", False):
            raise DexHandError("机器人末端 485 模式需要先连接珞石机器人")
        robot = getattr(self.robot_backend, "robot", None)
        if robot is None:
            raise DexHandError("机器人对象不可用")
        self._er_pro_mode = self._is_er_pro_robot(robot)
        if self._er_pro_mode:
            # xMate ER7Pro-M 的末端工具链路和供电由控制器现场配置负责。
            # 本程序不创建/上传/运行 RL 工程，也不调用 RL 语言里的
            # XPRS485RWData；只使用 xCore SDK 实际暴露的 XPRS485SendData。
            self._log_fn(
                "检测到 xMate ER7Pro-M（XME7p-R850）："
                "不创建或运行 RL 工程；不调用 XPRS485Init、setxPanelRS485 或 "
                "setxPanelVout，直接使用 xCore SDK 的 XPRS485SendData。"
            )
            self._opened = True
            self._last_request_at = 0.0
            self._log_fn(
                "xMate ER7Pro-M 已打开 SDK 直连末端 485 通道；"
                "程序不写入控制器末端工具/供电配置，马上进行一次只读能力验证"
            )
            return

        option = getattr(self.sdk, "xPanelOptVout", None)
        if option is not None:
            voltage = {
                0: getattr(option, "off", 0),
                1: getattr(option, "reserve", 1),
                2: getattr(option, "supply12v", 2),
                3: getattr(option, "supply24v", 3),
            }.get(self.voltage_option, getattr(option, "supply24v", 3))
        else:
            voltage = self.voltage_option

        off = getattr(option, "off", 0) if option is not None else 0
        voltage_names = {
            0: "关闭输出",
            1: "保留",
            2: "输出12V",
            3: "输出24V",
        }

        self._log_fn(
            f"xPanel 配置目标：对外供电={voltage_names.get(voltage, '未知')}"
            f"（枚举值={voltage}），RS485=启用（True）；"
            "对应控制器菜单【通信 → xPanel 设置】"
        )

        # 先清理上一次连接留下的 xPanel 状态。即使关闭调用返回了提示，
        # 也继续执行正式打开；真正的能力错误会由下面的打开调用报告。
        try:
            ec: dict = {}
            robot.setxPanelRS485(off, False, ec)
            self._log_fn(
                f"xPanel 复位请求返回：ec={ec.get('ec', 0)}；"
                f"message={ec.get('message', '无')}"
            )
            if ec.get("ec", 0) != 0:
                self._log_fn(
                    f"连接前复位 xPanel 485 返回提示：{ec.get('message', ec)}；继续尝试打开"
                )
        except Exception as exc:
            self._log_fn(f"连接前关闭 xPanel 485 返回提示：{exc}；继续尝试打开")
        time.sleep(self.POWER_OFF_SETTLE_S)

        ec = {}
        robot.setxPanelRS485(voltage, True, ec)
        self._log_fn(
            f"xPanel 打开请求返回：ec={ec.get('ec', 0)}；"
            f"message={ec.get('message', '无')}"
        )
        self._check(ec, "打开 xPanel 24V/485")
        self._opened = True
        self._last_request_at = 0.0
        time.sleep(self.POWER_ON_SETTLE_S)
        self._log_fn(
            "已复位并打开珞石末端 485，已给 xPanel 输出所选电压；"
            "RS485 模式已请求启用；正在等待灵巧手完成上电启动"
        )

    def close(self) -> None:
        if not self._opened:
            return
        if self._er_pro_mode:
            # ER Pro 上不修改控制器现场配置，也不生成/停止 RL 工程。
            self._log_fn(
                "xMate ER7Pro-M 断开 SDK 直连末端 485：不修改控制器当前末端工具配置"
            )
            self._opened = False
            self._last_request_at = 0.0
            return
        robot = getattr(self.robot_backend, "robot", None)
        if robot is not None and getattr(self.robot_backend, "connected", False):
            try:
                ec: dict = {}
                option = getattr(self.sdk, "xPanelOptVout", None)
                off = getattr(option, "off", 0) if option is not None else 0
                robot.setxPanelRS485(off, False, ec)
                self._log_fn(
                    f"xPanel 关闭请求返回：ec={ec.get('ec', 0)}；"
                    f"message={ec.get('message', '无')}"
                )
                self._check(ec, "关闭 xPanel 24V/485")
            except Exception as exc:
                self._log_fn(f"关闭 xPanel 485 时出现提示：{exc}")
        self._opened = False
        self._last_request_at = 0.0

    def request(self, frame: bytes, response_len: int) -> bytes:
        if not self._opened:
            raise DexHandError("机器人末端 485 尚未连接")
        if not 1 <= len(frame) <= 16 or not 0 <= response_len <= 16:
            raise DexHandError("珞石 xPanel 单次收发限制为 0~16 字节")
        robot = getattr(self.robot_backend, "robot", None)
        if robot is None:
            raise DexHandError("机器人对象不可用")
        remaining = self.REQUEST_GAP_S - (
            time.monotonic() - self._last_request_at
        )
        if remaining > 0:
            time.sleep(remaining)
        self._trace(
            f"发送：长度={len(frame)}，期望回复={response_len} 字节，"
            f"帧={format_frame(frame)}"
        )
        try:
            import numpy as np  # type: ignore
            send_data = np.asarray(list(frame), dtype=np.uint8)
            # xCore SDK 官方 Python 示例使用空的 PyTypeVectorInt 作为输出
            # 容器，由 rev_byte 告知 SDK 期望的回复长度。RL 指令才要求在
            # 工程源码中声明固定长度的 byte 数组；这里不生成 RL 工程，不能
            # 把那套规则套到 XPRS485SendData 的 Python 绑定上。
            rev_data = self.sdk.PyTypeVectorInt()
            self._trace(
                f"手册参数校验：send_byte={len(frame)}，"
                f"send_data_len={len(send_data)}，rev_byte={response_len}，"
                "rev_data 初始长度=0（SDK 输出容器）"
            )
            ec: dict = {}
            robot.XPRS485SendData(len(frame), response_len,
                                  send_data, rev_data, ec)
            self._trace(
                f"SDK XPRS485SendData 返回：ec={ec.get('ec', 0)}；"
                f"message={ec.get('message', '无')}"
            )
            self._check(ec, "XPRS485SendData")
            raw_result = [int(v) & 0xFF for v in rev_data.content()]
            if response_len and len(raw_result) > response_len:
                # 当前 Windows pybind 会在输出 vector 中保留 rev_byte 个
                # 预留 0，再把真实回复追加到尾部；有效回复是最后
                # response_len 个字节。接收容器本身仍必须从空 vector 开始。
                result = bytes(raw_result[-response_len:])
            else:
                result = bytes(raw_result)
            self._trace(
                f"接收：原始长度={len(raw_result)}，有效长度={len(result)}，"
                f"帧={format_frame(result)}"
            )
        except DexHandError:
            raise
        except ImportError as exc:
            raise DexHandError("机器人末端模式需要 numpy，请安装 numpy") from exc
        except Exception as exc:
            self._trace(f"底层异常：{exc}")
            raise DexHandError(f"珞石末端 485 收发失败：{exc}") from exc
        finally:
            self._last_request_at = time.monotonic()
        if response_len == 0:
            # rebootDevice()/clearFirmwareError() are non-blocking commands in
            # the official SDK.  Ignore any optional echo/late response here;
            # the next request starts with a clean transaction.
            return b""
        if len(result) != response_len:
            # xPanel 也可能把 Modbus 异常响应按实际长度返回。保留该 5
            # 字节帧，让上层给出明确的异常码，而不是泛化成长度错误。
            if (
                len(result) == 5 and len(result) >= 2
                and result[1] == (frame[1] | 0x80)
            ):
                return result
            raise DexHandError(
                f"珞石末端 485 回复长度不符：期望 {response_len}，收到 {len(result)}"
            )
        return result





class RobotRLTransport(Transport):
    """已停用的旧 RL 工程传输层。

    这是早期实验代码的保留区。当前程序不会实例化它，也不会上传、生成
    或运行机器人工程；所有机器人末端通信统一使用 RobotPanelTransport 的
    xCore SDK XPRS485SendData。保留类名只是避免旧配置/导入立即失效。
    """

    max_frame_bytes = 16
    PROJECT_NAME = "DexHand021SBridge"
    REQUEST_GAP_S = 0.05
    LOG_POLL_INTERVAL_S = 0.06
    LOG_POLL_TIMEOUT_S = 8.0

    _PROJECT_INFO = """{
    "create_time": "2026-08-14 00:00:00",
    "description": "DexHand021 S ER7 Pro RL XPRS485RWData bridge",
    "rl_version": "2.0"
}
"""
    _TASK_XML = """<?xml version="1.0" encoding="UTF-8"?>
<m>
    <l>
        <c name="auto_start" type="1" value="true"/>
        <c name="autoboot_start" type="1" value="true"/>
        <c name="check" type="1" value="true"/>
        <c name="description" type="10" value="DexHand021 S RL透传"/>
        <c name="files">
            <l>
                <c name="description" type="10" value="DexHand021 S通信"/>
                <c name="name" type="10" value="main"/>
                <c name="type" type="10" value="MOD"/>
            </l>
        </c>
        <c name="id" type="4" value="0"/>
        <c name="name" type="10" value="task0"/>
        <c name="pre_task" type="10" value=""/>
        <c name="priority" type="2" value="2"/>
        <c name="safety_level" type="10" value="nosafety"/>
        <c name="type" type="10" value="motion"/>
    </l>
</m>
"""
    _EMPTY_XML = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n<m/>\n"

    # xCore importProject does not only look at task.xml/task0/main.mod.
    # A project created by RobotAssist also contains the compiled-project
    # metadata under _build.  Without these files importProject may return
    # ec=0, but the next importFile/loadProject reports -60005 because the
    # task is not registered in the imported project.
    _BUILD_FRAME_JSON = """{
    \"SYSTEM_FRAME_LIST\": [
        {
            \"id\": 0,
            \"name\": \"userframe0\",
            \"param\": {
                \"ori\": {
                    \"euler\": {\"a\": 0, \"b\": 0, \"c\": 0},
                    \"quaternion\": {\"q1\": 1, \"q2\": 0, \"q3\": 0, \"q4\": 0}
                },
                \"pos\": {\"x\": 0, \"y\": 0, \"z\": 0}
            }
        }
    ]
}
"""
    _BUILD_IO_MAP_JSON = "{\n    \"IO_MAPPING\": null\n}\n"
    _BUILD_PATH_LIST = "MODULE _PATHLIST\nENDMODULE\n"
    _BUILD_TOOLS = (
        "MODULE _TOOLLIST\n"
        "GLOBAL VAR tool tool0 = {true,{{0,0,0},{1,0,0,0}},"
        "{0,{0,0,0},{1,0,0,0},0,0,0}};\n"
        "ENDMODULE\n"
    )
    _BUILD_WOBJS = (
        "MODULE _WOBJLIST\n"
        "GLOBAL VAR wobj wobj0 = {false,true,\"robot\",{0,0,0},"
        "{1,0,0,0},0,{0,{0,0,0},{1,0,0,0},0,0,0}};\n"
        "ENDMODULE\n"
    )
    _BUILD_PREDEFINE = """MODULE _PREDEFINE
GLOBAL VAR speed v1000 = v:{100,1000,200,1000,500};
GLOBAL VAR speed vmax = v:{100,1000000,200,5000,1000};
GLOBAL VAR zone fine = s:{0,0};
GLOBAL VAR zone z50 = s:{50,25};
ENDMODULE
"""

    @classmethod
    def _build_project_descriptor(cls) -> str:
        return json.dumps(
            {
                "load": {
                    "IO": {"path": "io_map.json"},
                    "frame": {"path": "frame.json"},
                    "path": {"path": "path_list.sys"},
                    "point": {"path": "point_list.sys"},
                    "tool": {"path": "tools.sys"},
                    "wobj": {"path": "wobjs.sys"},
                },
                "program": {"task0": {"enable": True, "path": "task0"}},
                "updateID": str(uuid.uuid4()),
            },
            indent=4,
        ) + "\n"

    @classmethod
    def _build_task_descriptor(cls) -> str:
        return json.dumps(
            {
                "auto_start": True,
                "load": [
                    "predefine",
                    "point_list",
                    "tools",
                    "wobjs",
                    "path_list",
                ],
                "pre_task_name": "",
                "priority": 2,
                "project_name": cls.PROJECT_NAME,
                "safety_level": "nosafety",
                "task_name": "task0",
                "task_type": "motion",
            },
            indent=4,
        ) + "\n"

    def __init__(self, robot_backend: Any, sdk_module: Any,
                 voltage_option: int = 3,
                 log_fn: Optional[Callable[[str], None]] = None,
                 trace_fn: Optional[Callable[[str], None]] = None) -> None:
        del robot_backend, sdk_module, voltage_option, log_fn, trace_fn
        raise DexHandError(
            "旧 RL 工程透传已停用；机器人末端模式只允许使用 "
            "xCore SDK 的 XPRS485SendData，不会生成机器人工程。"
        )

    @staticmethod
    def is_er_pro_robot(robot_backend: Any) -> bool:
        robot = getattr(robot_backend, "robot", None)
        backend_name = str(getattr(robot_backend, "robot_type_name", ""))
        robot_name = str(type(robot).__name__) if robot is not None else ""
        names = f"{backend_name} {robot_name}".replace("_", "").lower()
        return "xmateerpro" in names or "erpro" in names

    def _trace(self, message: str) -> None:
        self._trace_fn(f"珞石 RL XPRS485RWData {message}")

    def _robot(self) -> Any:
        robot = getattr(self.robot_backend, "robot", None)
        if robot is None:
            raise DexHandError("珞石机器人对象不可用")
        return robot

    def _controller_log_lines(self, count: int = 40) -> list[str]:
        """读取控制器日志；失败时返回空列表，不遮蔽原始 SDK 错误。"""
        robot = getattr(self.robot_backend, "robot", None)
        query = getattr(robot, "queryControllerLog", None)
        if not callable(query):
            return []
        ec: dict = {}
        try:
            infos = query(int(count), set(), ec)
        except Exception:
            return []
        if ec.get("ec", 0) != 0:
            return []
        lines: list[str] = []
        for info in infos or []:
            content = str(getattr(info, "content", "")).strip()
            if content:
                lines.append(content)
        return lines

    def _recent_controller_logs(self) -> str:
        lines = self._controller_log_lines(20)
        if not lines:
            return ""
        return "；".join(line[-300:] for line in lines[-8:])

    def _check_ec(self, ec: dict, name: str) -> None:
        code = int(ec.get("ec", 0) or 0)
        if code == 0:
            return
        message = str(ec.get("message", "无详细信息"))
        logs = self._recent_controller_logs()
        suffix = f"；控制器日志：{logs}" if logs else ""
        raise DexHandError(
            f"珞石 RL {name} 失败（错误码 {code}）：{message}{suffix}"
        )

    def _set_rl_mode(self) -> None:
        robot = self._robot()
        motion_mode = getattr(getattr(self.sdk, "MotionControlMode", None),
                              "NrtRLTask", None)
        operate_mode = getattr(getattr(self.sdk, "OperateMode", None),
                               "automatic", None)
        if motion_mode is None or operate_mode is None:
            raise DexHandError(
                "当前珞石 Python SDK 缺少 NrtRLTask 或 automatic 枚举，"
                "无法运行 XPRS485RWData 的 RL 工程"
            )
        ec: dict = {}
        robot.setMotionControlMode(motion_mode, ec)
        self._trace(
            f"setMotionControlMode(NrtRLTask)：ec={ec.get('ec', 0)}；"
            f"message={ec.get('message', '无')}"
        )
        self._check_ec(ec, "setMotionControlMode(NrtRLTask)")
        ec = {}
        robot.setOperateMode(operate_mode, ec)
        self._trace(
            f"setOperateMode(automatic)：ec={ec.get('ec', 0)}；"
            f"message={ec.get('message', '无')}"
        )
        self._check_ec(ec, "setOperateMode(automatic)")

    def _create_project_zip(self, main_source: str) -> str:
        if self._project_temp is None:
            self._project_temp = tempfile.TemporaryDirectory(
                prefix="dexhand021s_rl_"
            )
        archive_path = os.path.join(
            self._project_temp.name, f"{self.PROJECT_NAME}.zip"
        )
        with zipfile.ZipFile(archive_path, "w", zipfile.ZIP_DEFLATED) as archive:
            archive.writestr("projectinfo.json", self._PROJECT_INFO)
            archive.writestr("task.xml", self._TASK_XML)
            archive.writestr("io_signal.xml", self._EMPTY_XML)
            archive.writestr("user_frame.xml", self._EMPTY_XML)
            archive.writestr("task0/main.mod", main_source)
            # Keep the same project layout as an RL project exported by
            # RobotAssist.  In particular, the .prj filename and the
            # project_name in .rlprog must match the name returned by
            # importProject; otherwise importFile(project/.../task0/main.mod)
            # fails with controller error -60005.
            archive.writestr(
                f"_build/{self.PROJECT_NAME}.prj",
                self._build_project_descriptor(),
            )
            archive.writestr("_build/frame.json", self._BUILD_FRAME_JSON)
            archive.writestr("_build/io_map.json", self._BUILD_IO_MAP_JSON)
            archive.writestr("_build/path_list.sys", self._BUILD_PATH_LIST)
            archive.writestr("_build/predefine.sys", self._BUILD_PREDEFINE)
            archive.writestr("_build/tools.sys", self._BUILD_TOOLS)
            archive.writestr("_build/wobjs.sys", self._BUILD_WOBJS)
            archive.writestr(
                "_build/task0/.rlprog",
                self._build_task_descriptor(),
            )
            archive.writestr(
                "_build/task0/main.mod",
                "MODULE MOD_MAIN\n"
                "GLOBAL PROC main()\n"
                "ENDPROC\n"
                "ENDMODULE\n",
            )
        return archive_path

    def _write_main_file(self, source: str) -> str:
        if self._project_temp is None:
            raise DexHandError("RL 临时工程尚未初始化")
        path = os.path.join(
            self._project_temp.name,
            f"request_{self._request_number:06d}.mod",
        )
        with open(path, "w", encoding="ascii", newline="\n") as stream:
            stream.write(source)
        return path

    @staticmethod
    def _wire_response_len(frame: bytes, response_len: int) -> int:
        """将后端的“不等待回复”映射到 RL 可验证的接收数组。"""
        if response_len:
            return int(response_len)
        # 021S 的 0x25/0xA4 清错在现场 RL 工程中返回 6 字节确认，因此
        # 仍按 6 字节数组调用并在 Python 层丢弃该确认。重启 0x75 才使用
        # rev_byte=0；RL 语法用 1 字节占位数组承载该零长度参数。
        if len(frame) >= 3 and frame[1] == 0x25 and frame[2] == 0xA4:
            return 6
        return 0

    @staticmethod
    def _render_source(requests: list[tuple[bytes, int]], token: str) -> str:
        lines = [
            "// DexHand021 S ER7 Pro RL bridge; XPRS485Init is intentionally not used.",
            "// The controller keeps the end-tool physical configuration; this task only transmits bytes.",
        ]
        for index, (frame, wire_response_len) in enumerate(requests):
            tx_values = ",".join(str(int(value) & 0xFF) for value in frame)
            rx_size = max(1, int(wire_response_len))
            rx_values = ",".join("0" for _ in range(rx_size))
            lines.append(
                f"VAR byte tx{index}[{len(frame)}] = {{{tx_values}}};"
            )
            lines.append(
                f"VAR byte rx{index}[{rx_size}] = {{{rx_values}}};"
            )
            lines.append(f"VAR int ret{index} = -1;")
        lines.append("")
        lines.append("GLOBAL PROC main()")
        for index, (frame, wire_response_len) in enumerate(requests):
            lines.append(
                f"    XPRS485RWData({len(frame)}, tx{index}, "
                f"{wire_response_len}, rx{index}, ret{index});"
            )
            lines.append(
                f'    Print("DHB|{token}|{index}|RET=", ret{index});'
            )
            if wire_response_len:
                values = ", ".join(
                    f"rx{index}[{position}]"
                    for position in range(1, wire_response_len + 1)
                )
                lines.append(
                    f'    Print("DHB|{token}|{index}|RX=", {values});'
                )
            if index != len(requests) - 1:
                lines.append("    Wait(0.05);")
        lines.append(f'    Print("DHB|{token}|DONE");')
        lines.append("ENDPROC")
        lines.append("")
        return "\n".join(lines)

    @staticmethod
    def _marker_values(contents: list[str], marker: str) -> list[int]:
        for content in contents:
            position = content.find(marker)
            if position >= 0:
                tail = content[position + len(marker):]
                return [int(value) for value in re.findall(r"-?\d+", tail)]
        return []

    def _wait_for_result(self, token: str,
                         requests: list[tuple[bytes, int]],
                         wire_lengths: list[int]) -> list[bytes]:
        done_marker = f"DHB|{token}|DONE"
        deadline = time.monotonic() + self.LOG_POLL_TIMEOUT_S
        contents: list[str] = []
        while time.monotonic() < deadline:
            contents = self._controller_log_lines(100)
            if any(done_marker in content for content in contents):
                responses: list[bytes] = []
                for index, (_frame, requested_len) in enumerate(requests):
                    ret_marker = f"DHB|{token}|{index}|RET="
                    ret_values = self._marker_values(contents, ret_marker)
                    if not ret_values:
                        raise DexHandError(
                            f"RL 工程已结束但未找到第 {index + 1} 笔 ret；"
                            f"控制器日志={self._recent_controller_logs()}"
                        )
                    ret = ret_values[0]
                    rx_marker = f"DHB|{token}|{index}|RX="
                    rx_values = self._marker_values(contents, rx_marker)
                    wire_len = wire_lengths[index]
                    if ret != 0:
                        raise DexHandError(
                            f"XPRS485RWData 第 {index + 1} 笔发送失败："
                            f"ret={ret}；请求帧={format_frame(_frame)}；"
                            f"控制器日志={self._recent_controller_logs()}"
                        )
                    if wire_len and len(rx_values) < wire_len:
                        raise DexHandError(
                            f"XPRS485RWData 第 {index + 1} 笔回复长度不足："
                            f"期望至少 {wire_len} 字节，收到 {len(rx_values)}；"
                            f"请求帧={format_frame(_frame)}"
                        )
                    actual = bytes(int(value) & 0xFF for value in rx_values[:wire_len])
                    responses.append(actual if requested_len else b"")
                return responses
            time.sleep(self.LOG_POLL_INTERVAL_S)
        log_text = "；".join(contents[-8:]) if contents else "无"
        raise DexHandError(
            f"RL 工程未在 {self.LOG_POLL_TIMEOUT_S:.1f} 秒内返回完成标记；"
            f"请查询控制器 RL 日志，最近日志={log_text}"
        )

    def _run_requests(self, requests: list[tuple[bytes, int]]) -> list[bytes]:
        if not requests:
            return []
        self._request_number += 1
        token = f"{self._request_number:06d}_{uuid.uuid4().hex[:8]}"
        normalized: list[tuple[bytes, int]] = []
        wire_lengths: list[int] = []
        for frame, response_len in requests:
            raw_frame = bytes(frame)
            expected = int(response_len)
            if not 1 <= len(raw_frame) <= self.max_frame_bytes:
                raise DexHandError("珞石 RL 单次发送长度必须为 1~16 字节")
            if not 0 <= expected <= self.max_frame_bytes:
                raise DexHandError("珞石 RL 单次接收长度必须为 0~16 字节")
            normalized.append((raw_frame, expected))
            wire_lengths.append(self._wire_response_len(raw_frame, expected))

        remaining = self.REQUEST_GAP_S - (
            time.monotonic() - self._last_request_at
        )
        if remaining > 0:
            time.sleep(remaining)
        source_requests = [
            (frame, wire_lengths[index])
            for index, (frame, _expected) in enumerate(normalized)
        ]
        source = self._render_source(source_requests, token)
        source_path = self._write_main_file(source)
        robot = self._robot()
        # xCore SDK 的 importFile 目标必须包含 .mod 文件名；task.xml
        # 中登记的主程序文件名是 main，因此不能只传到 task0 目录。
        destination = f"project/{self._project_name}/task0/main.mod"
        self._trace(
            f"上传 RL 任务：工程={self._project_name}；任务=task0；"
            f"请求数={len(normalized)}；token={token}"
        )
        ec: dict = {}
        imported_file = robot.importFile(source_path, destination, True, ec)
        self._trace(
            f"importFile 返回：file={imported_file}；ec={ec.get('ec', 0)}；"
            f"message={ec.get('message', '无')}"
        )
        self._check_ec(ec, "importFile")
        ec = {}
        robot.loadProject(self._project_name, ["task0"], ec)
        self._trace(
            f"loadProject 返回：ec={ec.get('ec', 0)}；"
            f"message={ec.get('message', '无')}"
        )
        self._check_ec(ec, "loadProject")
        ec = {}
        robot.ppToMain(ec)
        self._trace(
            f"ppToMain 返回：ec={ec.get('ec', 0)}；"
            f"message={ec.get('message', '无')}"
        )
        self._check_ec(ec, "ppToMain")
        ec = {}
        robot.setProjectRunningOpt(1.0, False, ec)
        self._trace(
            f"setProjectRunningOpt 返回：ec={ec.get('ec', 0)}；"
            f"message={ec.get('message', '无')}"
        )
        self._check_ec(ec, "setProjectRunningOpt")
        ec = {}
        self._project_running = True
        robot.runProject(ec)
        self._trace(
            f"runProject 返回：ec={ec.get('ec', 0)}；"
            f"message={ec.get('message', '无')}"
        )
        try:
            self._check_ec(ec, "runProject")
            responses = self._wait_for_result(token, normalized, wire_lengths)
        except Exception:
            # 超时或控制器日志中出现 ret 错误时，工程可能仍停在
            # XPRS485RWData；保留 running 标记，让 close() 尝试 pauseProject。
            raise
        else:
            self._project_running = False
        finally:
            self._last_request_at = time.monotonic()
        for index, ((frame, requested_len), response) in enumerate(
                zip(normalized, responses)):
            self._trace(
                f"第 {index + 1} 笔完成：请求={format_frame(frame)}；"
                f"期望回复={requested_len}；有效回复={format_frame(response)}"
            )
        return responses

    def open(self) -> None:
        with self._lock:
            if self._opened:
                return
            if not getattr(self.robot_backend, "connected", False):
                raise DexHandError(
                    "xMate ER7Pro-M 末端 485 模式需要先连接珞石机器人"
                )
            if not callable(getattr(self._robot(), "importProject", None)):
                raise DexHandError(
                    "当前珞石 Python SDK 没有 importProject，无法使用 RL 末端透传"
                )
            try:
                self._project_temp = tempfile.TemporaryDirectory(
                    prefix="dexhand021s_rl_"
                )
                self._set_rl_mode()
                self._project_zip = self._create_project_zip(
                    "GLOBAL PROC main()\nENDPROC\n"
                )
                ec: dict = {}
                project_name = self._robot().importProject(
                    self._project_zip, True, ec
                )
                self._trace(
                    f"importProject 返回：name={project_name}；"
                    f"ec={ec.get('ec', 0)}；message={ec.get('message', '无')}"
                )
                self._check_ec(ec, "importProject")
                self._project_name = str(project_name or self.PROJECT_NAME).strip()
                if not self._project_name:
                    self._project_name = self.PROJECT_NAME
                # Validate the imported task before declaring the hand link
                # open.  This makes a bad project package fail here with the
                # controller's real loadProject error instead of later during
                # the first Modbus request.
                ec = {}
                self._robot().loadProject(self._project_name, ["task0"], ec)
                self._trace(
                    f"importProject 后 loadProject：工程={self._project_name}；"
                    f"任务=task0；ec={ec.get('ec', 0)}；"
                    f"message={ec.get('message', '无')}"
                )
                self._check_ec(ec, "loadProject(task0)")
                self._opened = True
                self._last_request_at = 0.0
                self._log_fn(
                    "xMate ER7Pro-M（XME7p-R850）已进入 RL 末端 485 透传："
                    "控制器任务只调用 XPRS485RWData；未调用 XPRS485Init、"
                    "XPRS485SendData 或 setxPanelRS485。程序不写入控制器末端"
                    "工具/供电配置；DexHand021 S 协议参数为 ID=1、115200、8N1。"
                )
            except Exception:
                self._opened = False
                self._project_name = ""
                if self._project_temp is not None:
                    self._project_temp.cleanup()
                    self._project_temp = None
                self._project_zip = ""
                raise

    def close(self) -> None:
        with self._lock:
            if self._project_running:
                try:
                    ec: dict = {}
                    self._robot().pauseProject(ec)
                    self._trace(
                        f"pauseProject 返回：ec={ec.get('ec', 0)}；"
                        f"message={ec.get('message', '无')}"
                    )
                except Exception as exc:
                    self._log_fn(f"停止 xMate ER7Pro-M RL 透传任务时出现提示：{exc}")
            self._project_running = False
            self._opened = False
            self._project_name = ""
            self._last_request_at = 0.0
            if self._project_temp is not None:
                self._project_temp.cleanup()
                self._project_temp = None
            self._project_zip = ""

    def request_many(self, requests: list[tuple[bytes, int]]) -> list[bytes]:
        with self._lock:
            if not self._opened:
                raise DexHandError("xMate ER7Pro-M RL 末端 485 尚未连接")
            return self._run_requests(requests)

    def request(self, frame: bytes, response_len: int) -> bytes:
        return self.request_many([(frame, response_len)])[0]


@dataclass
class FingerStatus:
    finger_id: int
    label: str = ""
    available: bool = True
    angle_deg: float = 0.0
    hall_position: int = 0
    speed_dps: int = 0
    current_ma: int = 0
    torque_pwm: int = 0
    temperature_c: float = 0.0
    voltage_v: float = 0.0
    max_output_torque: int = 0
    max_output_current: int = 0
    max_speed_dps: int = 0
    protection_temperature_c: int = 0
    stall_trigger_ms: int = 0
    stall_protection_current_ma: int = 0
    normal_force_n: float = 0.0
    tangent_force_n: float = 0.0
    normal_force_delta: int = 0
    tangent_force_delta: int = 0
    tangent_force_angle_deg: int = 0
    proximity: int = 0


@dataclass
class DeviceInfo:
    admin_mode: int = 0
    device_id: int = 0
    firmware_version: int = 0
    upgrade_status: int = 0
    iap_upgrade_flag: int = 0


AXIS_IDS = (1, 2, 3, 4)
FINGER_IDS = (1, 2, 3)
ROTATION_AXIS_ID = 4
AXIS_LABELS = {
    1: "P1（手指1）",
    2: "P2（手指2）",
    3: "P3（手指3）",
    4: "R（旋转轴）",
}
SERVO_BASES = {axis_id: 0x40 + (axis_id - 1) * 0x30
               for axis_id in AXIS_IDS}
PRESSURE_BASES = {finger_id: 0x10 + (finger_id - 1) * 0x10
                  for finger_id in FINGER_IDS}


@dataclass
class HandStatus:
    fingers: dict[int, FingerStatus] = field(default_factory=dict)
    device_info: Optional[DeviceInfo] = None
    received_at: float = field(default_factory=time.time)


class DexHand021SBackend:
    """三指 DexHand021 S 的统一高层后端。"""

    BAUDRATE = 115200
    # 官方 DexHand C++ SDK 的 MotorControlMode 枚举值。
    # 发送到 0x31 RTU 帧时由 SDK_MODE_TO_RTU_MOTOR_MODE 转成 0x04/0x05/0x06。
    POSITION_MODE = 0x44       # CASCADED_PID_CONTROL_MODE
    HALL_LIMIT_MODE = 0x55     # HALL_POSLIMIT_CONTROL_MODE
    TORQUE_MODE = 0x66         # CASCADED_MIT_CONTROL_MODE
    AXIS_IDS = AXIS_IDS
    FINGER_IDS = FINGER_IDS
    ROTATION_AXIS_ID = ROTATION_AXIS_ID
    AXIS_LABELS = AXIS_LABELS
    # 官方 dexhand.dll 的 resetJoints(021S) 会按 0x55 模式发送这四个
    # 目标：P1/P2/P3 回到 Hall=0，R 轴回到 Hall=0x118（280）。
    RESET_TARGETS = {1: 0, 2: 0, 3: 0, 4: 0x118}
    # 以下三个控制量是直发 0x31 原始 RTU 帧的取值，遵循说明书 6.6.2、
    # 6.6.3；不能采用 dexhand.dll 高层 moveFinger() 的换算后参数范围。
    RESET_CONTROL_VALUE = 300
    HALL_SPEED_MIN = 50
    HALL_SPEED_MAX = 300
    TORQUE_MIN = 50
    TORQUE_MAX = 800
    # 《DexHand021 S 使用说明书》0x31 原始帧的 0x05/0x06 示例均以
    # Hall=1200 作为三指闭合目标；R 轴 Hall 范围为 0~1600。虽然 C++
    # 头文件的高层 API 注释写有 1000，上述值才是裸 RTU 透传需要遵循的
    # 实际帧语义，因此在这里单独定义，避免将 75.0° 错发为 Hall=750。
    FINGER_HALL_TARGET_MAX = 1200
    ROTATION_HALL_TARGET_MAX = 1600
    RESET_MODE = HALL_LIMIT_MODE
    RESET_INTER_AXIS_DELAY_S = 0.01

    def __init__(self, robot_backend: Any = None, sdk_module: Any = None,
                 log_fn: Optional[Callable[[str], None]] = None) -> None:
        self.robot_backend = robot_backend
        self.sdk_module = sdk_module
        self._log_fn = log_fn or (lambda _msg: None)
        self.transport: Optional[Transport] = None
        self.transport_name = ""
        self.device_id = 1
        # 详细帧日志会显著增加机器人末端 485 轮询期间的 UI 更新量。
        # 默认关闭，用户可在界面中按需开启排障。
        self._trace_enabled = False
        self.connected = False
        # R 轴在部分 021S 固件中没有开放完整的状态寄存器。缓存最后一
        # 次已知角度/状态，让 R 轴不可读时不再把 P1~P3 的状态轮询拖死。
        self._last_angles = [0, 0, 0, 0]
        self._last_servo_status: dict[int, FingerStatus] = {}
        # None 表示尚未探测；False 表示本次连接中 R 轴状态读失败，暂停
        # 后续自动轮询；True 表示已确认 R 轴状态寄存器可读。
        self._rotation_status_available: Optional[bool] = None
        self._rotation_status_error = ""

    def _log(self, message: str) -> None:
        self._log_fn(message)

    def _trace(self, message: str) -> None:
        if self._trace_enabled:
            self._log_fn(f"[通信] {message}")

    def set_trace_enabled(self, enabled: bool) -> None:
        self._trace_enabled = bool(enabled)
        self._log(
            "详细通信日志：已开启（记录请求/回复帧）"
            if self._trace_enabled else
            "详细通信日志：已关闭（仅保留关键事件和错误）"
        )

    def open(self, transport_name: str, port: str = "", device_id: int = 1,
             voltage_option: int = 3) -> None:
        if self.connected:
            self.close()
        self.device_id = int(device_id)
        self._last_angles = [0, 0, 0, 0]
        self._last_servo_status = {}
        self._rotation_status_available = None
        self._rotation_status_error = ""
        if not 1 <= self.device_id <= 0x7F:
            raise DexHandError("021S 设备 ID 范围为 1~127")
        self._log(
            f"DexHand021 S 协议参数：设备 ID={self.device_id}；"
            f"波特率={self.BAUDRATE}；数据位=8；校验=None；停止位=1；"
            "CRC=Modbus RTU，低字节在前"
        )
        if transport_name == "usb":
            transport: Transport = SerialTransport(
                port,
                self.BAUDRATE,
                log_fn=self._log,
                trace_fn=self._trace,
            )
        elif transport_name == "robot":
            if self.robot_backend is None or self.sdk_module is None:
                raise DexHandError("机器人末端 485 后端未初始化")
            # 所有机器人型号统一走 xCore SDK 实际暴露的裸透传接口。
            # 特别是 ER7Pro-M：不上传、生成或运行临时 RL 工程，也不依赖
            # RL 工程变量/Print 回传；每一笔请求直接调用 XPRS485SendData。
            transport = RobotPanelTransport(
                self.robot_backend,
                self.sdk_module,
                voltage_option,
                self._log,
                self._trace,
            )
        elif transport_name == "mock":
            transport = MockTransport()
        else:
            raise DexHandError(f"不支持的灵巧手通信方式：{transport_name}")
        transport.open()
        self.transport = transport
        self.transport_name = transport_name
        self.connected = True
        # 不在打开串口时自动发送 0x23。手册要求设置反馈模式后等待 40 ms，
        # 且不同固件对该可选设置的应答兼容性不同；连接阶段只打开链路，
        # 由界面的“立即读取状态”或“应用反馈模式”显式发起请求。
        self._log(
            f"DexHand021 S 已打开链路（{self.transport_label()}，设备 ID={self.device_id}）；"
            "请先点击“立即读取状态”验证设备响应"
        )

    def close(self) -> None:
        if self.transport is not None:
            try:
                self.transport.close()
            finally:
                self.transport = None
        if self.connected:
            self._log("DexHand021 S 已断开")
        self.connected = False
        self.transport_name = ""

    def transport_label(self) -> str:
        return {
            "usb": "USB 转 485 直连",
            "robot": "珞石机器人末端 485",
            "mock": "模拟协议",
        }.get(self.transport_name, self.transport_name or "未连接")

    def _request(self, frame: bytes, response_len: int) -> bytes:
        if not self.connected or self.transport is None:
            raise DexHandError("灵巧手尚未连接")
        if self.transport.max_frame_bytes and response_len > self.transport.max_frame_bytes:
            raise DexHandError(
                f"当前传输层单次最多接收 {self.transport.max_frame_bytes} 字节，"
                f"本次需要 {response_len} 字节"
            )
        return self.transport.request(frame, response_len)

    def _request_many(self, requests: list[tuple[bytes, int]]) -> list[bytes]:
        if not self.connected or self.transport is None:
            raise DexHandError("灵巧手尚未连接")
        request_many = getattr(self.transport, "request_many", None)
        if callable(request_many):
            return list(request_many(requests))
        return [self._request(frame, response_len)
                for frame, response_len in requests]

    def set_feedback_mode(self, mode: int = 2, interval_ms: int = 20) -> None:
        frame = feedback_mode_frame(self.device_id, mode, interval_ms)
        response = self._request(frame, 6)
        validate_global_ack(response, self.device_id, 0x23)
        # 说明书明确要求反馈模式设置后与下一条控制帧至少间隔 40 ms。
        time.sleep(0.04)

    def clear_error(self) -> None:
        # clearFirmwareError() 是“固件不等待清错流程结束”的非阻塞调用，
        # 不是“RTU 没有确认帧”。021S 手册的 0x25/A4 仍返回 6 字节全局
        # 设置确认。更重要的是，ER7Pro-M 的 XPRS485SendData 不接受
        # rev_byte=0；使用 6 字节确认才能把清错帧实际送到末端。
        frame = clear_error_frame(self.device_id)
        self._trace(
            f"清除保护：设备 ID={self.device_id}；发送帧={format_frame(frame)}；"
            "期望回复=6（0x25/A4 全局设置确认）"
        )
        response = self._request(frame, 6)
        validate_global_ack(response, self.device_id, 0xA4)
        # 官方 SDK 明确说明清错流程是非阻塞的；确认帧只表示指令被接收，
        # 留出固件处理窗口，避免下一控制帧紧跟在 0x25 后面而被保护吞掉。
        time.sleep(0.08)

    def reset_joints(self) -> None:
        """按官方 021S resetJoints() 流程发送四轴初始位置指令。

        C++ SDK 的 resetJoints() 本身是非阻塞接口，并以 10 ms 的队列延时
        依次发送 P1/P2/P3/R。这里通过统一的 0x31 控制帧逐轴发送并校验
        0x31 确认，保证 USB 直连和珞石 xPanel 两种链路行为一致。
        """
        for axis_id in AXIS_IDS:
            target = self.RESET_TARGETS[axis_id]
            self.move_finger(
                axis_id,
                target,
                self.RESET_CONTROL_VALUE,
                self.RESET_MODE,
            )
            if axis_id != AXIS_IDS[-1]:
                time.sleep(self.RESET_INTER_AXIS_DELAY_S)

    def set_max_current(self, current_ma: int) -> None:
        response = self._request(set_max_current_frame(self.device_id, current_ma), 6)
        validate_global_ack(response, self.device_id, 0x65)

    def set_protection_temperature(self, temperature_c: int,
                                   cooldown_c: int = 10) -> None:
        response = self._request(
            set_protection_temperature_frame(
                self.device_id, temperature_c, cooldown_c
            ),
            6,
        )
        validate_global_ack(response, self.device_id, 0x6D)

    def reboot(self) -> None:
        # 与清错相同，xCore SDK 的裸透传必须带接收长度；设备通常先返回
        # 0x25/0x75 确认，再进入重启。只有确认成功后才开始等待上电窗口。
        response = self._request(reboot_frame(self.device_id), 6)
        validate_global_ack(response, self.device_id, 0x75)
        time.sleep(0.05)

    def read_holding_registers(self, start: int, count: int) -> list[int]:
        return self._read_registers(0x03, start, count)

    def write_holding_register(self, address: int, value: int) -> None:
        response = self._request(
            write_single_register_frame(self.device_id, address, value), 8
        )
        validate_ack(response, self.device_id, 0x06, expected_len=8)

    def read_device_info(self) -> DeviceInfo:
        values = self.read_holding_registers(0x00, 0x06)
        return DeviceInfo(
            admin_mode=values[0],
            device_id=values[1],
            firmware_version=values[2],
            upgrade_status=values[4],
            iap_upgrade_flag=values[5],
        )

    def move_finger(self, finger_id: int, target: int, control_value: int,
                    mode: int = POSITION_MODE) -> None:
        if not 1 <= finger_id <= 4:
            raise DexHandError("021S 电机/轴 ID 范围为 1~4")
        if mode == self.POSITION_MODE:
            limit = 1600 if finger_id == self.ROTATION_AXIS_ID else 750
            if not 0 <= target <= limit:
                raise DexHandError(
                    f"角度位置模式轴 {finger_id} 目标应为 0~{limit}（角度×10）"
                )
            control_value = 0
        elif mode == self.HALL_LIMIT_MODE:
            limit = (
                self.ROTATION_HALL_TARGET_MAX
                if finger_id == self.ROTATION_AXIS_ID
                else self.FINGER_HALL_TARGET_MAX
            )
            if not 0 <= target <= limit:
                raise DexHandError(
                    f"带限制霍尔模式轴 {finger_id} 目标应为 0~{limit}"
                )
            if not self.HALL_SPEED_MIN <= control_value <= self.HALL_SPEED_MAX:
                raise DexHandError(
                    "带限制霍尔模式速度控制量应为 "
                    f"{self.HALL_SPEED_MIN}~{self.HALL_SPEED_MAX}"
                )
        elif mode == self.TORQUE_MODE:
            # 这是直接发送的 RTU Motor_Mode=0x06 帧。按 021S 说明书
            # 6.6.3，目标字段为 Hall 位置，不能把角度×10 写入这里。
            limit = (
                self.ROTATION_HALL_TARGET_MAX
                if finger_id == self.ROTATION_AXIS_ID
                else self.FINGER_HALL_TARGET_MAX
            )
            if not 0 <= target <= limit:
                raise DexHandError(
                    f"力矩模式轴 {finger_id} Hall 目标应为 0~{limit}"
                )
            if not self.TORQUE_MIN <= control_value <= self.TORQUE_MAX:
                raise DexHandError(
                    f"力矩模式控制量应为 {self.TORQUE_MIN}~{self.TORQUE_MAX}"
                )
        else:
            raise DexHandError(
                f"不支持的 DexHand021 S 控制模式：0x{mode:02X}；"
                "必须是 0x44、0x55 或 0x66"
            )
        frame = move_frame(
            self.device_id, finger_id, mode, target, control_value
        )
        target_unit = "角度×10" if mode == self.POSITION_MODE else "Hall"
        self._trace(
            f"运动控制：轴 ID={finger_id}；SDK 模式=0x{mode:02X}；"
            f"RTU Motor_Mode=0x{SDK_MODE_TO_RTU_MOTOR_MODE[mode]:02X}；"
            f"目标={target}（{target_unit}）；控制量={control_value}；"
            f"发送帧={format_frame(frame)}；期望回复=6"
        )
        response = self._request(frame, 6)
        validate_control_ack(response, self.device_id, finger_id)
        self._trace(
            f"运动确认：轴 ID={finger_id}；回复帧={format_frame(response)}；"
            "错误标志=0"
        )

    def move_all_position(self, angle_deg: float) -> None:
        target = int(round(max(0.0, min(75.0, angle_deg)) * 10.0))
        for finger_id in (1, 2, 3):
            self.move_finger(finger_id, target, 0, self.POSITION_MODE)
            time.sleep(0.003)

    def _read_registers(self, function: int, start: int, count: int) -> list[int]:
        if function not in (0x03, 0x04):
            raise DexHandError("仅支持 0x03/0x04 寄存器读取")
        if count < 1:
            return []
        max_count = count
        if self.transport is not None and self.transport.max_frame_bytes:
            max_count = min(
                max_count,
                max(1, (self.transport.max_frame_bytes - 5) // 2),
            )
        values: list[int] = []
        remaining = count
        offset = 0
        while remaining:
            chunk = min(remaining, max_count)
            response = self._request(
                read_register_frame(self.device_id, function, start + offset, chunk),
                5 + chunk * 2,
            )
            values.extend(
                parse_register_response(response, self.device_id, function, chunk)
            )
            offset += chunk
            remaining -= chunk
        return values

    def read_input_registers(self, start: int, count: int) -> list[int]:
        return self._read_registers(0x04, start, count)

    def probe_connection(self, attempts: int = 2,
                         retry_delay_s: float = 0.05,
                         allow_holding_fallback: bool = False
                         ) -> tuple[int, list[int]]:
        """执行安全的只读在线探测，返回实际响应的功能码和寄存器值。

        连接串口成功并不代表 021S 已经在线。默认严格按手册使用 0x04
        读取 P1~P3 角度。不能把 0x03 读到的设备信息当成角度，否则会
        出现“探测成功但控制仍无效”的假在线状态。只有明确知道是旧固件
        时，调用方才可以显式打开 ``allow_holding_fallback``。
        """
        attempts = max(1, int(attempts))
        retry_delay_s = max(0.0, float(retry_delay_s))
        errors: list[str] = []
        functions = (0x04, 0x03) if allow_holding_fallback else (0x04,)
        for function in functions:
            for attempt in range(attempts):
                self._trace(
                    f"只读探测：功能码=0x{function:02X}；"
                    f"起始地址=0x0000；数量=3；第 {attempt + 1}/{attempts} 次"
                )
                try:
                    if function == 0x04:
                        values = self.read_input_registers(0x00, 3)
                    else:
                        values = self.read_holding_registers(0x00, 3)
                    self._trace(
                        f"只读探测成功：功能码=0x{function:02X}；寄存器值={values}"
                    )
                    return function, values
                except DexHandError as exc:
                    self._trace(
                        f"只读探测失败：功能码=0x{function:02X}；原因={exc}"
                    )
                    errors.append(f"0x{function:02X} 第 {attempt + 1} 次：{exc}")
                    if attempt + 1 < attempts:
                        # 给 RS485 收发器、xPanel 转发链路和 USB 驱动一个
                        # 完整的帧间隔。机器人末端模式会传入更长的等待值，
                        # 以覆盖 xPanel 首次打开后的启动窗口。
                        time.sleep(retry_delay_s)
        detail = "；".join(errors)
        summary = (
            "0x04/0x03 均未收到有效回复"
            if allow_holding_fallback else "0x04 输入寄存器未收到有效回复"
        )
        raise DexHandError(f"灵巧手只读探测失败：{summary}。详细信息：{detail}")

    def _read_servo_status(self, axis_id: int,
                           include_diagnostics: bool = False) -> FingerStatus:
        base = SERVO_BASES[axis_id]
        # 021S 的标准状态表是 Hall(2)、速度、电流四个寄存器；扭矩
        # 和温度/电压位于后续地址。不能把它们合并成 base 起始的 5
        # 个寄存器，否则部分固件会直接丢弃 0x04/base/5 请求。
        values = self.read_input_registers(base, 4)
        torque_values = self.read_input_registers(base + 0x04, 1)
        temp_voltage = self.read_input_registers(base + 0x05, 2)
        low_word, high_word = values[0], values[1]
        hall = (high_word << 16) | low_word
        if hall & 0x80000000:
            hall -= 0x100000000
        status = FingerStatus(
            finger_id=axis_id,
            label=AXIS_LABELS[axis_id],
            hall_position=hall,
            speed_dps=_signed16(values[2]),
            current_ma=_signed16(values[3]),
            torque_pwm=_signed16(torque_values[0]),
            temperature_c=float(_signed16(temp_voltage[0])),
            voltage_v=_signed16(temp_voltage[1]) / 1000.0,
        )
        if include_diagnostics:
            limits = self.read_input_registers(base + 0x07, 3)
            protection = self.read_input_registers(base + 0x0F, 3)
            status.max_output_torque = limits[0]
            status.max_output_current = limits[1]
            status.max_speed_dps = limits[2]
            status.protection_temperature_c = protection[0]
            status.stall_trigger_ms = protection[1]
            status.stall_protection_current_ma = protection[2]
        return status

    @staticmethod
    def _servo_status_from_values(axis_id: int, values: list[int],
                                  torque_values: Optional[list[int]] = None,
                                  temp_voltage: Optional[list[int]] = None
                                  ) -> FingerStatus:
        if len(values) < 4:
            raise DexHandError(
                f"电机 {axis_id} 标准状态寄存器回复不足 4 个寄存器"
            )
        torque_values = torque_values or [0]
        temp_voltage = temp_voltage or [0, 0]
        low_word, high_word = values[0], values[1]
        hall = (high_word << 16) | low_word
        if hall & 0x80000000:
            hall -= 0x100000000
        return FingerStatus(
            finger_id=axis_id,
            label=AXIS_LABELS[axis_id],
            hall_position=hall,
            speed_dps=_signed16(values[2]),
            current_ma=_signed16(values[3]),
            torque_pwm=_signed16(torque_values[0]) if torque_values else 0,
            temperature_c=(
                float(_signed16(temp_voltage[0])) if temp_voltage else 0.0
            ),
            voltage_v=(
                _signed16(temp_voltage[1]) / 1000.0
                if len(temp_voltage) > 1 else 0.0
            ),
        )

    @staticmethod
    def _servo_motion_from_values(axis_id: int,
                                  values: list[int]) -> FingerStatus:
        status = DexHand021SBackend._servo_status_from_values(axis_id, values)
        status.torque_pwm = 0
        status.temperature_c = 0.0
        status.voltage_v = 0.0
        return status

    def _read_servo_motion_status(self, axis_id: int) -> FingerStatus:
        """只读取动作完成判定所需的 Hall 和速度反馈。

        动作序列会以较高频率等待电机停止。完整状态还包含温度、电压、
        压力和保护参数，读取次数较多，不适合在等待循环中重复读取；这里
        读取标准 Hall、速度和电流寄存器，保证界面显示的电流是实测值；
        对应说明书的 0x04/base/4 请求。
        """
        base = SERVO_BASES[axis_id]
        values = self.read_input_registers(base, 4)
        low_word, high_word = values[0], values[1]
        hall = (high_word << 16) | low_word
        if hall & 0x80000000:
            hall -= 0x100000000
        return FingerStatus(
            finger_id=axis_id,
            label=AXIS_LABELS[axis_id],
            hall_position=hall,
            speed_dps=_signed16(values[2]),
            current_ma=_signed16(values[3]),
        )

    def _merge_cached_extended_status(self, status: FingerStatus) -> FingerStatus:
        """保留最近一次完整状态中的低频字段。

        快速轮询只读取 Hall、速度和电流，避免每次都读取扭矩、温度和
        电压寄存器。这里把最近一次完整读取的低频字段带回状态表，既不
        伪造实时数据，也不会因为轻量轮询把界面已有数据重置成 0。
        """
        cached = self._last_servo_status.get(status.finger_id)
        if cached is None:
            return status
        for field_name in (
            "torque_pwm",
            "temperature_c",
            "voltage_v",
            "max_output_torque",
            "max_output_current",
            "max_speed_dps",
            "protection_temperature_c",
            "stall_trigger_ms",
            "stall_protection_current_ma",
            "normal_force_n",
            "tangent_force_n",
            "normal_force_delta",
            "tangent_force_delta",
            "tangent_force_angle_deg",
            "proximity",
        ):
            setattr(status, field_name, getattr(cached, field_name))
        return status

    def _read_pressure_status(self, finger_id: int,
                              status: FingerStatus) -> None:
        values = self.read_input_registers(PRESSURE_BASES[finger_id], 0x0B)
        status.normal_force_n = _decode_float(values[0:2], 20.0)
        status.normal_force_delta = _decode_u32_words(values[2:4])
        status.tangent_force_n = _decode_float(values[4:6], 20.0)
        status.tangent_force_delta = _decode_u32_words(values[6:8])
        status.tangent_force_angle_deg = values[8]
        status.proximity = _decode_u32_words(values[9:11])

    def _read_angle_values(self) -> list[int]:
        """读取 P1~P3 的角度寄存器。

        R 轴（Motor_4）按说明书位于独立的输入寄存器 ``0x0003``。为了让
        某个固件对该寄存器的异常回复不会影响 P1~P3，本方法只读取三个
        手指；R 轴由 ``_read_rotation_status`` 在本轮最后独立探测。
        """
        values = self.read_input_registers(0x00, 3)
        self._last_angles[:3] = values[:3]
        return list(self._last_angles)

    def retry_rotation_status_probe(self) -> None:
        """允许用户通过“立即读取状态”重新发起一次 R 轴只读探测。"""
        self._rotation_status_available = None
        self._rotation_status_error = ""

    def _mark_rotation_status_unavailable(self, exc: DexHandError) -> FingerStatus:
        """隔离 R 轴状态读取异常，避免拖慢后续 P1~P3 状态轮询。"""
        first_failure = self._rotation_status_available is not False
        self._rotation_status_available = False
        self._rotation_status_error = str(exc)
        if first_failure:
            self._log(
                "R（旋转轴）状态读取失败："
                f"{exc}；本次连接将暂停 R 轴自动轮询，P1/P2/P3 不受影响。"
                "可点击“立即读取状态”重新探测。"
            )
        return self._unavailable_rotation_status()

    def _read_rotation_status(self, include_diagnostics: bool = False) -> FingerStatus:
        """读取 Motor_4 的完整状态，失败时仅降级 R 轴本身。

        《DexHand021 S 使用说明书》6.7.3 指定：Motor_4 角度为输入寄存器
        0x0003，舵机04状态基址为 0x00D0。所有请求都是 0x04 只读请求。
        """
        if self._rotation_status_available is False:
            return self._unavailable_rotation_status()
        try:
            if self._rotation_status_available is None:
                self._trace(
                    "R 轴状态探测：角度寄存器=0x0003；"
                    "舵机04状态基址=0x00D0"
                )
            angle_value = self.read_input_registers(0x03, 1)[0]
            status = self._read_servo_status(
                self.ROTATION_AXIS_ID, include_diagnostics
            )
        except DexHandError as exc:
            return self._mark_rotation_status_unavailable(exc)

        was_unprobed = self._rotation_status_available is not True
        self._rotation_status_available = True
        self._rotation_status_error = ""
        self._last_angles[3] = angle_value
        status.angle_deg = _signed16(angle_value) / 100.0
        self._last_servo_status[self.ROTATION_AXIS_ID] = status
        if was_unprobed:
            self._log(
                "R（旋转轴）状态寄存器探测成功："
                "角度=0x0003，Hall/速度/电流/扭矩/温度/电压=0x00D0 起"
            )
        return status

    def _read_rotation_motion_status(self,
                                     include_angle: bool = False) -> FingerStatus:
        """读取 R 轴动作完成判定所需的 Hall/速度，按需附带角度。"""
        if self._rotation_status_available is False:
            return self._unavailable_rotation_status()
        try:
            angle_value: Optional[int] = None
            if include_angle:
                angle_value = self.read_input_registers(0x03, 1)[0]
            status = self._read_servo_motion_status(self.ROTATION_AXIS_ID)
        except DexHandError as exc:
            return self._mark_rotation_status_unavailable(exc)

        status = self._merge_cached_extended_status(status)
        self._rotation_status_available = True
        self._rotation_status_error = ""
        if angle_value is not None:
            self._last_angles[3] = angle_value
            status.angle_deg = _signed16(angle_value) / 100.0
        else:
            status.angle_deg = _signed16(self._last_angles[3]) / 100.0
        self._last_servo_status[self.ROTATION_AXIS_ID] = status
        return status

    def _unavailable_rotation_status(self) -> FingerStatus:
        cached = self._last_servo_status.get(self.ROTATION_AXIS_ID)
        if cached is None:
            status = FingerStatus(
                finger_id=self.ROTATION_AXIS_ID,
                label=AXIS_LABELS[self.ROTATION_AXIS_ID],
                available=False,
            )
        else:
            status = FingerStatus(
                finger_id=cached.finger_id,
                label=cached.label,
                available=False,
                angle_deg=cached.angle_deg,
                hall_position=cached.hall_position,
                speed_dps=cached.speed_dps,
                current_ma=cached.current_ma,
                torque_pwm=cached.torque_pwm,
                temperature_c=cached.temperature_c,
                voltage_v=cached.voltage_v,
            )
        status.angle_deg = _signed16(self._last_angles[3]) / 100.0
        return status

    def read_poll_status(self) -> HandStatus:
        """读取自动界面轮询所需的轻量状态。

        机器人末端 485 每笔请求均会经过控制器，完整状态需要 14 笔
        Modbus 读取，容易让 PySide 主线程在按钮点击时显得迟滞。该方法
        只读取 4 轴的角度、Hall、速度和电流：P1~P3 共四笔，R 轴独立
        两笔；扭矩/温度/电压保留最后一次完整读取值。点击“立即读取状态”
        仍会调用 ``read_status`` 完整刷新。
        """
        angles = self._read_angle_values()
        statuses: dict[int, FingerStatus] = {}
        for axis_id in FINGER_IDS:
            status = self._read_servo_motion_status(axis_id)
            status = self._merge_cached_extended_status(status)
            status.angle_deg = _signed16(angles[axis_id - 1]) / 100.0
            self._last_servo_status[axis_id] = status
            statuses[axis_id] = status
        statuses[self.ROTATION_AXIS_ID] = self._read_rotation_motion_status(
            include_angle=True
        )
        return HandStatus(statuses)

    def read_status(self, include_force: bool = False,
                    include_diagnostics: bool = False) -> HandStatus:
        # RobotRLTransport 仅为兼容旧版本而保留，当前 ER7Pro-M 不会实例化
        # 它，也不会上传或运行 RL 工程。下面的批量分支只服务于旧对象。
        if (
            isinstance(self.transport, RobotRLTransport)
            and not include_force
            and not include_diagnostics
        ):
            requests: list[tuple[bytes, int]] = [
                (read_input_frame(self.device_id, 0x00, 3), 11)
            ]
            for axis_id in FINGER_IDS:
                base = SERVO_BASES[axis_id]
                requests.extend((
                    (read_input_frame(self.device_id, base, 4), 13),
                    (read_input_frame(self.device_id, base + 0x04, 1), 7),
                    (read_input_frame(self.device_id, base + 0x05, 2), 9),
                ))
            responses = self._request_many(requests)
            angles = parse_register_response(
                responses[0], self.device_id, 0x04, 3
            )
            self._last_angles[:3] = angles[:3]
            statuses: dict[int, FingerStatus] = {}
            response_index = 1
            for axis_id in FINGER_IDS:
                values = parse_register_response(
                    responses[response_index], self.device_id, 0x04, 4
                )
                torque_values = parse_register_response(
                    responses[response_index + 1], self.device_id, 0x04, 1
                )
                temp_voltage = parse_register_response(
                    responses[response_index + 2], self.device_id, 0x04, 2
                )
                status = self._servo_status_from_values(
                    axis_id, values, torque_values, temp_voltage
                )
                status.angle_deg = _signed16(angles[axis_id - 1]) / 100.0
                self._last_servo_status[axis_id] = status
                statuses[axis_id] = status
                response_index += 3
            statuses[self.ROTATION_AXIS_ID] = self._unavailable_rotation_status()
            return HandStatus(statuses)

        angles = self._read_angle_values()
        statuses: dict[int, FingerStatus] = {}
        for axis_id in FINGER_IDS:
            status = self._read_servo_status(axis_id, include_diagnostics)
            status.angle_deg = _signed16(angles[axis_id - 1]) / 100.0
            if include_force and axis_id in FINGER_IDS:
                self._read_pressure_status(axis_id, status)
            self._last_servo_status[axis_id] = status
            statuses[axis_id] = status
        # R 轴在 P1~P3 完成后独立读取。若个别固件不支持它，异常只会
        # 降级 R 轴本身，不会再连锁影响下一轮手指状态轮询。
        statuses[self.ROTATION_AXIS_ID] = self._read_rotation_status(
            include_diagnostics
        )
        return HandStatus(statuses)

    def read_motion_status(self, include_angles: bool = False) -> HandStatus:
        """读取动作序列等待所需的四轴角度、Hall 和速度反馈。

        这是 ``read_status(False, False)`` 的轻量版本，避免动作序列每次
        轮询都额外读取温度、电压和扭矩寄存器。读取失败仍向上抛出，调用方
        可以在动作完成超时前继续重试。
        """
        # 裸 RTU 的 0x44 以角度目标判定；0x05/0x06 只需 Hall+速度，
        # 少读一帧能降低 485 轮询占用。R 轴状态在部分固件中不可读，
        # 不应阻塞 P1~P3。
        if isinstance(self.transport, RobotRLTransport):
            requests: list[tuple[bytes, int]] = []
            if include_angles:
                requests.append((read_input_frame(self.device_id, 0x00, 3), 11))
            for axis_id in FINGER_IDS:
                requests.append((
                    read_input_frame(self.device_id, SERVO_BASES[axis_id], 4),
                    13,
                ))
            responses = self._request_many(requests)
            response_index = 0
            angles: Optional[list[int]] = None
            if include_angles:
                angles = parse_register_response(
                    responses[0], self.device_id, 0x04, 3
                )
                self._last_angles[:3] = angles[:3]
                response_index = 1
            statuses: dict[int, FingerStatus] = {}
            for axis_id in FINGER_IDS:
                values = parse_register_response(
                    responses[response_index], self.device_id, 0x04, 4
                )
                status = self._servo_motion_from_values(axis_id, values)
                status = self._merge_cached_extended_status(status)
                if angles is not None:
                    status.angle_deg = _signed16(angles[axis_id - 1]) / 100.0
                self._last_servo_status[axis_id] = status
                statuses[axis_id] = status
                response_index += 1
            statuses[self.ROTATION_AXIS_ID] = self._unavailable_rotation_status()
            return HandStatus(statuses)

        angles = self._read_angle_values() if include_angles else None
        statuses: dict[int, FingerStatus] = {}
        for axis_id in FINGER_IDS:
            status = self._read_servo_motion_status(axis_id)
            status = self._merge_cached_extended_status(status)
            if angles is not None:
                status.angle_deg = _signed16(angles[axis_id - 1]) / 100.0
            self._last_servo_status[axis_id] = status
            statuses[axis_id] = status
        statuses[self.ROTATION_AXIS_ID] = self._read_rotation_motion_status(
            include_angles
        )
        return HandStatus(statuses)


def _decode_u32_words(registers: list[int]) -> int:
    if len(registers) < 2:
        return 0
    return (int(registers[1]) << 16) | int(registers[0])


def _decode_float(registers: list[int], max_value: float = 100.0) -> float:
    if len(registers) < 2:
        return 0.0
    raw = b"".join(value.to_bytes(2, "big") for value in registers)
    candidates = [
        struct.unpack(">f", raw[:4])[0],
        struct.unpack("<f", raw[:4])[0],
        struct.unpack(">f", raw[2:4] + raw[0:2])[0],
        struct.unpack("<f", raw[2:4] + raw[0:2])[0],
    ]
    for value in candidates:
        if value == value and -0.01 <= value <= max_value:
            return float(value)
    return 0.0


class MockTransport(Transport):
    """离线协议测试用传输层，不连接任何硬件。"""

    def __init__(self) -> None:
        self.opened = False
        self.requests: list[bytes] = []
        # 角度寄存器单位为 degree*100，舵机状态寄存器是 Hall 位置；两者
        # 在真实设备中并非同一个物理量，离线模拟也必须分开保存。
        self.angles = [0, 0, 0, 0]
        self.hall_positions = [0, 0, 0, 0]

    def open(self) -> None:
        self.opened = True

    def close(self) -> None:
        self.opened = False

    def request(self, frame: bytes, response_len: int) -> bytes:
        if not self.opened:
            raise DexHandError("模拟传输层尚未连接")
        self.requests.append(bytes(frame))
        device_id, function = frame[0], frame[1]
        if function in (0x03, 0x04):
            start = int.from_bytes(frame[2:4], "big")
            count = int.from_bytes(frame[4:6], "big")
            regs = [0] * count
            if function == 0x03:
                if start == 0:
                    values = [0, device_id, 0x0107, 0, 0, 0]
                    regs[:min(count, len(values))] = values[:count]
            elif function == 0x04:
                if start == 0x00:
                    regs[:min(count, 4)] = self.angles[:min(count, 4)]
                elif start == 0x03 and count == 1:
                    # _read_angle_values() 会单独探测 R 轴；模拟器也应返回
                    # 第四个角度寄存器，便于离线测试覆盖四轴。
                    regs[0] = self.angles[3]
                elif start >= 0x40:
                    axis_id = ((start - 0x40) // 0x30) + 1
                    offset = (start - 0x40) % 0x30
                    if not 1 <= axis_id <= 4:
                        axis_id = 1
                    if offset == 0 and count >= 3:
                        hall = self.hall_positions[axis_id - 1]
                        values = [hall & 0xFFFF,
                                  (hall >> 16) & 0xFFFF, 0, 0, 0, 250, 7200]
                        regs[:min(count, len(values))] = values[:count]
                    elif offset == 7 and count >= 3:
                        regs[:3] = [1000, 250, 400][:count]
                    elif offset == 5 and count >= 2:
                        regs[:2] = [250, 7200][:count]
                    elif offset == 0x0F and count >= 3:
                        regs[:3] = [90, 500, 200][:count]
                elif start in (0x10, 0x20, 0x30):
                    # 模拟压力/触觉寄存器均为安全的零值。
                    pass
            data = b"".join(value.to_bytes(2, "big") for value in regs)
            return with_crc(bytes((device_id, function, len(data))) + data)
        if function == 0x25:
            command = frame[2] if len(frame) > 2 else 0
            if command == 0x75 and response_len == 0:
                return b""
            return with_crc(bytes((device_id, function, command, 1)))
        if function == 0x31:
            axis_id = frame[3]
            # 模拟器接收的是原始 0x31 RTU Motor_Mode 字段：0x04 的目标是
            # 角度×10，0x05/0x06 的目标都是 Hall 位置。
            rtu_mode = frame[2]
            target = int.from_bytes(frame[4:6], "little")
            if 1 <= axis_id <= 4 and rtu_mode == 0x04:
                self.angles[axis_id - 1] = target * 10
                self.hall_positions[axis_id - 1] = target
            elif 1 <= axis_id <= 4 and rtu_mode in (0x05, 0x06):
                self.hall_positions[axis_id - 1] = target
                angle_limit = 10800 if axis_id == 4 else 7500
                hall_limit = 1600 if axis_id == 4 else 1200
                self.angles[axis_id - 1] = int(
                    round(target * angle_limit / hall_limit)
                )
            return with_crc(bytes((device_id, function, frame[3], 0)))
        if function == 0x06:
            return bytes(frame)
        raise DexHandError(f"模拟传输层不支持功能码 0x{function:02X}")
