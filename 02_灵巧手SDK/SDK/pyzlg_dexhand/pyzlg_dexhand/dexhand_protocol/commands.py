"""Host to board command generation and encoding"""

from enum import IntEnum
from dataclasses import dataclass
from typing import Union, List
from . import MessageType

class CommandType(IntEnum):
    """Command type identifiers"""
    MOTOR_COMMAND = 0x00      # Motor control command
    CLEAR_ERROR = 0xA4        # Clear error state
    CONFIG_FEEDBACK = 0x74    # Configure feedback settings

class ControlMode(IntEnum):
    """Motor control modes"""
    ZERO_TORQUE = 0x00
    CURRENT = 0x11
    SPEED = 0x22
    HALL_POSITION = 0x33
    CASCADED_PID = 0x44
    PROTECT_HALL_POSITION = 0x55
    MIT_TORQUE = 0x66
    IMPEDANCE_GRASP = 0x77

class FeedbackMode(IntEnum):
    """Feedback operation modes"""
    PERIODIC = 0x01   # Send feedback at regular intervals
    QUERY = 0x02      # Send feedback only when requested
    ON_CHANGE = 0x03  # Send feedback when values change

class GlobalFunctionCode(IntEnum):
    """Global function codes for broadcast commands"""
    # Global function codes
    CAN_CANFD_SWITCH = 0x70        # CAN/CANFD switch(CAN：0x00 CANFD：0x01)
    FACTORY_RESET = 0x71           # reset to factory settings
    SENSOR_POWER = 0x72            # on/off sensor power
    LOW_POWER_MODE = 0x73          # low power mode
    SENSOR_RESPONSE_MODE = 0x74    # sensor response mode
    REBOOT = 0x75                  # reboot
    IAP_UPGRADE = 0x76             # IAP upgrade
    PID_RESET = 0xA2               # PID reset
    CLEAR_ERROR = 0xA4             # clear error
    PRESSURE_LIMIT_ENABLE = 0xA6   # pressure limit enable

@dataclass(frozen=True)
class MotorCommand:
    """Command for a pair of motors"""
    control_mode: ControlMode  # Control mode (e.g., IMPEDANCE_GRASP)
    motor_enable: int         # Motor enable flags (0x01, 0x02, 0x03)
    motor1_pos: int          # Position command for motor 1 (-32768 to 32767)
    motor2_pos: int          # Position command for motor 2 (-32768 to 32767)
    motor1_speed: int  # Speed command for motor 1 (0-32767)
    motor2_speed: int  # Speed command for motor 2 (0-32767)
    motor1_current: int  # Current command for motor 1 (10-599mA)
    motor2_current: int  # Current command for motor 2 (10-599mA)

@dataclass(frozen=True)
class ClearErrorCommand:
    """Command to clear error state"""
    pass  # No additional data needed

@dataclass(frozen=True)
class FeedbackConfigCommand:
    """Command to configure feedback behavior"""
    mode: FeedbackMode     # Feedback mode
    period_ms: int        # Period in milliseconds (if periodic)
    enable: bool          # Enable flag

@dataclass(frozen=True)
class BroadcastCommand:
    """Command for broadcasting to all motors at once"""
    control_mode: ControlMode               # Control mode
    enable_motors: List[bool]               # 12 boolean flags for motor enable
    clear_error: bool                       # Flag to clear errors
    request_feedback: bool                  # Flag to request feedback
    is_right_hand: bool                     # Whether this is for right hand
    positions: List[int]                    # 12 positions for each motor
    speeds: List[int]                       # 12 speeds for each motor
    currents: List[int]                     # 12 currents for each motor

@dataclass(frozen=True)
class GlobalCommand:
    """Command for global broadcast operations"""
    function_code: GlobalFunctionCode  # global function code
    data: bytes                # additional data (if any)


Command = Union[MotorCommand, ClearErrorCommand, FeedbackConfigCommand, BroadcastCommand, GlobalCommand]

def encode_command(command: Command) -> tuple[MessageType, bytes]:
    """Encode a command for transmission

    Args:
        command: Command to encode

    Returns:
        Tuple of (message_type, encoded_bytes)

    Raises:
        ValueError: If command parameters are invalid
    """
    if isinstance(command, MotorCommand):
        return (MessageType.MOTION_COMMAND, _encode_motor_command(command))
    elif isinstance(command, ClearErrorCommand):
        return (MessageType.CONFIG_COMMAND, bytes([0x03, CommandType.CLEAR_ERROR]))
    elif isinstance(command, FeedbackConfigCommand):
        return (MessageType.CONFIG_COMMAND, _encode_feedback_config(command))
    elif isinstance(command, GlobalCommand):
        return (MessageType.GLOBAL_COMMAND, encode_global_command(command))
    else:
        raise ValueError(f"Unknown command type: {type(command)}")


def _encode_motor_command(command: MotorCommand) -> bytes:
    """Encode a motor control command (internal helper)"""
    if not 0 <= command.motor_enable <= 0x03:
        raise ValueError(f"Invalid motor enable flags: {command.motor_enable}")

    if not (-32768 <= command.motor1_pos <= 32767):
        raise ValueError(f"Motor 1 position out of range: {command.motor1_pos}")

    if not (-32768 <= command.motor2_pos <= 32767):
        raise ValueError(f"Motor 2 position out of range: {command.motor2_pos}")
    
    if not (0 <= command.motor1_speed <= 32767):
        raise ValueError(f"Motor 1 speed out of range: {command.motor1_speed}")
    
    if not (0 <= command.motor2_speed <= 32767):
        raise ValueError(f"Motor 2 speed out of range: {command.motor2_speed}")
    
    if not (10 <= command.motor1_current <= 599):
        raise ValueError(f"Motor 1 current out of range: {command.motor1_current}")
    
    if not (10 <= command.motor2_current <= 599):
        raise ValueError(f"Motor 2 current out of range: {command.motor2_current}")

    return bytes([
        command.control_mode & 0xFF,
        command.motor_enable & 0xFF,
        command.motor1_pos & 0xFF,
        (command.motor1_pos >> 8) & 0xFF,
        command.motor2_pos & 0xFF,
        (command.motor2_pos >> 8) & 0xFF,
        command.motor1_speed & 0xFF,
        (command.motor1_speed >> 8) & 0xFF,
        command.motor2_speed & 0xFF,
        (command.motor2_speed >> 8) & 0xFF,
        command.motor1_current & 0xFF,
        command.motor2_current & 0xFF,
    ])

def _encode_feedback_config(command: FeedbackConfigCommand) -> bytes:
    """Encode a feedback configuration command (internal helper)"""
    if command.mode not in FeedbackMode:
        raise ValueError(f"Invalid feedback mode: {command.mode}")
    if not 0 <= command.period_ms <= 2550:  # Max 255 * 10ms
        raise ValueError(f"Period must be 0-2550ms, got {command.period_ms}")

    period_units = command.period_ms // 10  # Convert to 10ms units
    return bytes([
        0x03,                    # Command prefix
        CommandType.CONFIG_FEEDBACK,
        command.mode,
        period_units,
        0x01 if command.enable else 0x00
    ])

def encode_broadcast_command(command: BroadcastCommand) -> bytes:
    """Encode a broadcast command for transmission with CAN ID 0x100

    Args:
        command: BroadcastCommand to encode

    Returns:
        Encoded bytes ready for transmission

    Raises:
        ValueError: If command parameters are invalid
    """
    # Validate positions, speeds, currents
    if len(command.enable_motors) != 12:
        raise ValueError(f"Invalid enable_motors length, must be 12")
        
    if len(command.positions) != 12:
        raise ValueError(f"Invalid positions length, must be 12")
        
    if len(command.speeds) != 12:
        raise ValueError(f"Invalid speeds length, must be 12")
        
    if len(command.currents) != 12:
        raise ValueError(f"Invalid currents length, must be 12")
    
    # Validate position ranges
    for i, pos in enumerate(command.positions):
        if not (-32768 <= pos <= 32767):
            raise ValueError(f"Position {i} out of range: {pos}, must be -32768 to 32767")
            
    # Validate speed ranges
    for i, speed in enumerate(command.speeds):
        if not (0 <= speed <= 32767):
            raise ValueError(f"Speed {i} out of range: {speed}, must be 0 to 32767")
            
    # Validate current ranges
    for i, current in enumerate(command.currents):
        if not (10 <= current <= 599):
            raise ValueError(f"Current {i} out of range: {current}, must be 10 to 599 mA")
    
    # Build control field
    byte0 = command.control_mode.value  # Use the enum value
    
    # Build enable bits (Byte 1-2)
    byte1 = 0
    byte2 = 0
    
    # Byte 1 (MSB to LSB): th_dip, th_mcp, th_rot, ff_spr, ff_dip, ff_mcp, mf_dip, mf_mcp
    byte1 |= (1 << 7) if command.enable_motors[0] else 0  # th_dip
    byte1 |= (1 << 6) if command.enable_motors[1] else 0  # th_mcp
    byte1 |= (1 << 5) if command.enable_motors[2] else 0  # th_rot
    byte1 |= (1 << 4) if command.enable_motors[3] else 0  # ff_spr
    byte1 |= (1 << 3) if command.enable_motors[4] else 0  # ff_dip
    byte1 |= (1 << 2) if command.enable_motors[5] else 0  # ff_mcp
    byte1 |= (1 << 1) if command.enable_motors[6] else 0  # mf_dip
    byte1 |= (1 << 0) if command.enable_motors[7] else 0  # mf_mcp
    
    # Byte 2 (MSB to LSB): rf_dip, rf_mcp, lf_dip, lf_mcp, unused...
    byte2 |= (1 << 7) if command.enable_motors[8] else 0   # rf_dip
    byte2 |= (1 << 6) if command.enable_motors[9] else 0   # rf_mcp
    byte2 |= (1 << 5) if command.enable_motors[10] else 0  # lf_dip
    byte2 |= (1 << 4) if command.enable_motors[11] else 0  # lf_mcp
    
    # Build settings byte
    byte3 = 0
    byte3 |= (1 << 2) if command.clear_error else 0
    byte3 |= (1 << 1) if command.request_feedback else 0
    byte3 |= (1 << 0) if command.is_right_hand else 0  # 0 for left hand, 1 for right hand
    
    # Build control field
    control_field = bytes([byte0, byte1, byte2, byte3])
    
    # Build data field (60 bytes total, 6 fingers × 10 bytes each)
    data_field = bytearray(60)  # Initialize with zeros
    
    # Define finger groups that map motor indices to finger data fields
    finger_groups = [
        (0, 1),    # Thumb: th_dip, th_mcp
        (2, 3),    # Rotation & spread: th_rot, ff_spr
        (4, 5),    # First finger: ff_dip, ff_mcp
        (6, 7),    # Middle finger: mf_dip, mf_mcp
        (8, 9),    # Ring finger: rf_dip, rf_mcp
        (10, 11)   # Little finger: lf_dip, lf_mcp
    ]
    
    # Fill data field for each finger group
    for i, (idx1, idx2) in enumerate(finger_groups):
        offset = i * 10  # Each finger data is 10 bytes
        
        # Pack position data (2 bytes each, little-endian)
        data_field[offset:offset+2] = command.positions[idx1].to_bytes(2, byteorder='little', signed=True)
        data_field[offset+2:offset+4] = command.positions[idx2].to_bytes(2, byteorder='little', signed=True)
        
        # Pack speed data (2 bytes each, little-endian)
        data_field[offset+4:offset+6] = command.speeds[idx1].to_bytes(2, byteorder='little', signed=False)
        data_field[offset+6:offset+8] = command.speeds[idx2].to_bytes(2, byteorder='little', signed=False)
        
        # Pack current data (1 byte each)
        data_field[offset+8] = command.currents[idx1]
        data_field[offset+9] = command.currents[idx2]
    
    # Combine control field and data field
    return control_field + data_field

def encode_global_command(command: GlobalCommand) -> bytes:
    """Encode a global broadcast command

    Args:
        command: GlobalCommand to encode

    Returns:
        Encoded bytes (8 bytes total)

    Raises:
        ValueError: If command data is invalid
    """
    if len(command.data) > 6:
        raise ValueError(f"Global command data too long: {len(command.data)} bytes (max 6)")
        
    # Build command data
    cmd_data = bytearray([0x03, command.function_code])
    cmd_data.extend(command.data)
    
    # Pad to 8 bytes if necessary
    if len(cmd_data) < 8:
        cmd_data.extend([0] * (8 - len(cmd_data)))
        
    return bytes(cmd_data)
