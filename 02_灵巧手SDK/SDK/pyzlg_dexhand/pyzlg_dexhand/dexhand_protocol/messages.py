from dataclasses import dataclass, asdict
import struct
from enum import IntEnum
from typing import Optional, Tuple
import logging
import numpy as np
from . import MessageType, get_message_type,FlashStorageTable
from .commands import CommandType

logger = logging.getLogger(__name__)

class LogLevel(IntEnum):
    """Logging levels"""
    INFO = 0
    DEBUG = 1
    ERROR = 2

class WarningError(Exception):
    """Warning error message"""
    HALL_LIMIT_MODE = 0x01 # Hall limit mode active

class BoardError(IntEnum):
    """Motor error status"""
    MOTOR1_ERROR = 0x01
    MOTOR2_ERROR = 0x02
    BOTH_MOTORS_ERROR = 0x03

class ErrorCode(IntEnum):
    """Error codes from protocol"""
    CURRENT_OVERLOAD = 0x01
    HALL_ERROR = 0x02
    STALL_ERROR = 0x03
    PARAM_ERROR = 0x04

class ErrorMessageType(IntEnum):  
    """Specific error message types"""  
    MOTOR_ERROR = 0xEE        # Detailed motor errors  
    WARNING = 0xEF            # System warnings  
    OVERHEATING = 0xFF        # Overheating condition

@dataclass(frozen=True)
class MotorFeedback:
    """Feedback data for a single motor"""
    current: int        # Current in mA
    velocity: int       # Speed in rpm
    position: int       # Position in encoder counts
    angle: float        # Angle in degrees
    encoder_value: Optional[int] = None   # Raw encoder value (0-4095), angle = encoder/11.38
    error_code: Optional[int] = None      # Motor error code
    impedance: Optional[float] = None     # Motor impedance reading (float value)

@dataclass(frozen=True)
class TouchFeedback:
    """Feedback from touch sensor"""
    normal_force: float           # Normal force in N
    normal_force_delta: int       # Change in normal force (raw units)
    tangential_force: float       # Tangential force in N
    tangential_force_delta: int   # Change in tangential force (raw units)
    direction: int                # Force direction (0-359 degrees, fingertip is 0)
    proximity: int                # Proximity value (raw units)
    temperature: int              # Temperature in Celsius

@dataclass(frozen=True)
class BoardFeedback:
    """Complete feedback from a motor control board"""
    motor1: MotorFeedback
    motor2: MotorFeedback
    position_sensor1: float  # First position sensor reading
    position_sensor2: float  # Second position sensor reading
    touch: Optional[TouchFeedback] = None

@dataclass(frozen=True)
class ErrorInfo:
    """Error information from a board"""
    error_type: BoardError    # Which motors have errors
    error_data: bytes # Error information dictionary
    description: str        # Human-readable error description
    

@dataclass(frozen=True)
class ProcessedMessage:
    """Processed message from board"""
    sender_id: int
    msg_type: MessageType
    feedback: Optional[BoardFeedback] = None
    error_info: Optional[ErrorInfo] = None

@dataclass
class WriteMessage:
    """Message to be written to the board"""
    sender_id: int
    msg_type: MessageType
    feedback: bool

def process_message(can_id: int, data: bytes) -> ProcessedMessage:
    """Process a received CAN message

    Args:
        can_id: CAN ID of the message
        data: Raw message bytes

    Returns:
        ProcessedMessage containing decoded information

    Raises:
        ValueError: If message cannot be decoded
    """
    msg_type = get_message_type(can_id)
    if msg_type is None:
        raise ValueError(f"Invalid message ID: {can_id:x} (mask: {can_id & 0xF80:x})")

    if msg_type == MessageType.MOTION_FEEDBACK:
        feedback = _decode_feedback(data)
        return ProcessedMessage(
            sender_id=can_id,
            msg_type=MessageType.MOTION_FEEDBACK,
            feedback=feedback
        )

    elif msg_type == MessageType.ERROR_MESSAGE:
        try:
            error = _decode_error(data)
            return ProcessedMessage(
                sender_id=can_id,
                msg_type=MessageType.ERROR_MESSAGE,
                error_info=error
            )
        except ValueError as e:
            logger.warning(f"Failed to decode error message: {e}")
            return ProcessedMessage(
                sender_id=can_id,
                msg_type=MessageType.INVALID
            )

    else:
        # Other message types just pass through
        return ProcessedMessage(
            sender_id=can_id,
            msg_type=msg_type
        )

def _decode_feedback(data: bytes) -> BoardFeedback:
    """Decode motor feedback message (internal helper)"""
    if len(data) < 16:
        raise ValueError("Message too short for basic feedback")

    try:
        # Motor 1 data
        current1 = int.from_bytes(data[0:2], 'little', signed=True)
        velocity1 = int.from_bytes(data[2:4], 'little', signed=True)
        position1 = int.from_bytes(data[4:6], 'little', signed=True)

        # Motor 2 data
        current2 = int.from_bytes(data[6:8], 'little', signed=True)
        velocity2 = int.from_bytes(data[8:10], 'little', signed=True)
        position2 = int.from_bytes(data[10:12], 'little', signed=True)

        # Position sensors (0.01 degree units)
        pos1 = int.from_bytes(data[12:14], 'little', signed=True) / 100.0
        pos2 = int.from_bytes(data[14:16], 'little', signed=True) / 100.0

        # Create basic motor feedback objects
        motor1 = MotorFeedback(current1, velocity1, position1, pos1)
        motor2 = MotorFeedback(current2, velocity2, position2, pos2)

        # Optional data
        touch = None
        
        # Adaptively decode based on message length
        data_len = len(data)
        
        # Handle different message lengths (16, 42, or 56 bytes)
        # Add extended motor data (D42-D55) if available
        if data_len >= 42:
            # Get encoder values (D42-D45)
            encoder1 = int.from_bytes(data[42:44], 'little') if data_len >= 44 else None
            encoder2 = int.from_bytes(data[44:46], 'little') if data_len >= 46 else None
            
            # Get error codes (D46-D47)
            motor1_error = data[46] if data_len >= 47 else None
            motor2_error = data[47] if data_len >= 48 else None
            
            # Get impedance values (D48-D55)
            motor1_impedance = None
            motor2_impedance = None
            if data_len >= 52:
                try:
                    motor1_impedance = struct.unpack('<f', data[48:52])[0]
                except struct.error:
                    logger.warning("Failed to decode motor1 impedance")
            
            if data_len >= 56:
                try:
                    motor2_impedance = struct.unpack('<f', data[52:56])[0]
                except struct.error:
                    logger.warning("Failed to decode motor2 impedance")
            
            # Update motor objects with extended data
            motor1 = MotorFeedback(
                current=current1, 
                velocity=velocity1, 
                position=position1, 
                angle=pos1,
                encoder_value=encoder1,
                error_code=motor1_error,
                impedance=motor1_impedance
            )
            
            motor2 = MotorFeedback(
                current=current2, 
                velocity=velocity2, 
                position=position2, 
                angle=pos2,
                encoder_value=encoder2,
                error_code=motor2_error,
                impedance=motor2_impedance
            )

        # Decode touch sensor data if present
        # Touch data starts at index 16, need at least 42 bytes for complete touch sensor data
        if data_len >= 42:  # Full touch sensor data available
            try:
                touch = TouchFeedback(
                    normal_force=struct.unpack('<f', data[16:20])[0],
                    normal_force_delta=int.from_bytes(data[20:24], 'little'),
                    tangential_force=struct.unpack('<f', data[24:28])[0],
                    tangential_force_delta=int.from_bytes(data[28:32], 'little'),
                    direction=int.from_bytes(data[32:34], 'little'),
                    proximity=int.from_bytes(data[34:38], 'little'),
                    temperature=int.from_bytes(data[38:42], 'little'),
                )
                if np.isnan(touch.normal_force) or np.isnan(touch.tangential_force):
                    logger.warning("Invalid touch sensor data: normal/tangential force is NaN")
            except Exception as e:
                logger.warning(f"Failed to decode touch data: {e}")
                touch = None

        # Create BoardFeedback instance with all available data
        return BoardFeedback(
            motor1=motor1,
            motor2=motor2,
            position_sensor1=pos1,
            position_sensor2=pos2,
            touch=touch,
        )

    except (struct.error, TypeError) as e:
        raise ValueError(f"Failed to decode feedback: {str(e)}")

def _decode_error(data: bytes) -> ErrorInfo:
    """Decode an error message (internal helper)"""
    if len(data) < 2:
        raise ValueError("Invalid error message format: too short")

    # Initialize default values
    error_type = BoardError.BOTH_MOTORS_ERROR
    error_data = b''
    description = "Unknown error"

    try:
        # Standard error message format (0xEE)
        if data[0] == ErrorMessageType.MOTOR_ERROR:
            if len(data) < 3:
                raise ValueError("Invalid standard error message format")
                
            error_type = BoardError(data[1])
            error_data = data[2:4] if len(data) >= 4 else bytes([data[2], 0])
            
            # Build error descriptions
            descriptions = []
            
            # Check motor 1 error
            if error_type in [BoardError.MOTOR1_ERROR, BoardError.BOTH_MOTORS_ERROR] and len(data) >= 3:
                try:
                    motor1_code = ErrorCode(data[2])
                    descriptions.append(f"Motor 1: {motor1_code.name}")
                except ValueError:
                    descriptions.append(f"Motor 1: unknown error code 0x{data[2]:02x}")
            
            # Check motor 2 error
            if error_type in [BoardError.MOTOR2_ERROR, BoardError.BOTH_MOTORS_ERROR] and len(data) >= 4:
                try:
                    motor2_code = ErrorCode(data[3])
                    descriptions.append(f"Motor 2: {motor2_code.name}")
                except ValueError:
                    descriptions.append(f"Motor 2: unknown error code 0x{data[3]:02x}")
            
            description = "; ".join(descriptions) if descriptions else "Unknown motor error"
            
        # Warning message format (0xEF)
        elif data[0] == ErrorMessageType.WARNING:
            if len(data) < 3:
                raise ValueError("Invalid warning message format")
                
            error_type = BoardError(data[1])
            error_data = bytes([data[2]])
            
            if data[2] == 0x01:
                description = f"Warning: Motor temperature high"
            else:
                description = f"Warning: Unknown code 0x{data[2]:02x}"
            
        # Overheat error format (0xFF sequence)
        elif data[0] == ErrorMessageType.OVERHEATING and len(data) >= 6 and all(b == 0xFF for b in data[0:6]):
            logger.error(f"Warning: Overheating detected. Please power off the device and let it rest.")
            error_type = BoardError.BOTH_MOTORS_ERROR
            error_data = b'\xFF\xFF'
            description = "Critical error: System overheated"
            
        else:
            error_data = data[1:] if len(data) > 1 else b''
            description = f"Unknown error message type: 0x{data[0]:02x}"
            
    except Exception as e:
        # Fallback for any errors in parsing
        error_data = data[1:] if len(data) > 1 else b''
        description = f"Error parsing message: {e}"

    return ErrorInfo(
        error_type=error_type,
        error_data=error_data,
        description=description,
    )

def decode_write(can_id: int, data: bytes) -> WriteMessage:
    """Decode a write command response

    Args:
        can_id: CAN ID of the response
        data: Raw response bytes

    Returns:
        Tuple of boolean indicating success and command type (if applicable)
    """
    sender_id = can_id - 0x80
    if data[0] != MessageType.COMMAND_WRITE:
        raise ValueError("Invalid write command response: {data[0]}")
    elif data[0] == MessageType.COMMAND_WRITE and data[4] == 0x01:
        return WriteMessage(
            sender_id=sender_id,
            msg_type=MessageType.COMMAND_WRITE,
            feedback=True  # Success response for write commands
        )
    elif data[0] == MessageType.COMMAND_WRITE and data[4] == 0x00:
        return WriteMessage(
            sender_id=sender_id,
            msg_type=MessageType.COMMAND_WRITE,
            feedback=False  # Failure response for write commands
        )
    else:
        raise ValueError(f"Invalid write type/code: {data[1]}")

def verify_config_response(msg_id: int, data: bytes) -> Tuple[bool, Optional[CommandType]]:
    """Verify a command response

    Args:
        command: Original command
        msg_id: Response CAN ID
        data: Response data

    Returns:
        boolean of whether response indicates success, and command type if applicable
    """
    if len(data) < 4:
        return False, None
    elif data[1] in (CommandType.CONFIG_FEEDBACK, CommandType.CLEAR_ERROR):
        return (data[0] == 0x03 and
                data[2] == 0x00 and
                data[3] == 0x01), data[1]
    else:
        return False, None