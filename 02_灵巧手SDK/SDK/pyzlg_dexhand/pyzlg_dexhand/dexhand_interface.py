from dataclasses import dataclass, asdict
from typing import Dict, List, Optional, Union, Tuple, NamedTuple
import numpy as np
import os
import yaml
import logging
from pathlib import Path
from typing import List, Dict
import time
from .zcan_wrapper import ZCANWrapper
from . import dexhand_protocol as protocol
from .dexhand_protocol import BoardID
from .dexhand_protocol.commands import (
    ControlMode,
    MotorCommand,
    ClearErrorCommand,
    FeedbackConfigCommand,
    BroadcastCommand,
    encode_broadcast_command,
    FeedbackMode,
    encode_command,
    GlobalCommand,
    GlobalFunctionCode,
)
from .dexhand_protocol.messages import (
    BoardFeedback,
    ErrorInfo,
    MessageType,
    ProcessedMessage,
    FlashStorageTable,
    LogLevel,
    process_message,
)
from .dexhand_protocol.constants import (
    NUM_MOTORS,
    NUM_BOARDS,
    MOTORS_PER_BOARD,
    MIN_FIRMWARE_VERSION,
    DEFAULT_MOTOR_SPEED,
    DEFAULT_MOTOR_CURRENT,
    HARDWARE_COUNTS_PER_REV,
    GEAR_RATIO,
    RESOLUTION_FACTOR,
    DEG_PER_REV,
    CASCADE_PID_SCALE,
    BROADCAST_FRAME_ID,
    PRESSURE_LIMIT_SCALE,
    MOTOR1_ENABLE,
    MOTOR2_ENABLE,
    BOTH_MOTORS_ENABLE,
    MAX_STALL_TIME,
    MAX_PRESSURE_LIMIT,
    MIN_MOTOR_CURRENT,
    MAX_MOTOR_CURRENT,
)
from .dexhand_protocol.validation import ValidationHelper


logger = logging.getLogger(__name__)


class JointCommand(NamedTuple):
    """Command for a single joint with position, current, and velocity parameters"""
    position: float
    current: Optional[int] = None  # Current in mA
    velocity: Optional[int] = None  # Velocity in hardware units


@dataclass
class HandConfig:
    """Configuration for hand hardware"""

    channel: int  # CAN channel number
    hall_scale: List[float]  # Scale coefficients for hall position modes


@dataclass
class BoardState:
    """Feedback and Error collected for a single board."""

    feedback_timestamp: float  # Timestamp of last feedback
    status_timestamp: float  # Timestamp of last status (normal / error) update
    is_normal: bool  # True if board is in normal state
    feedback: Optional[BoardFeedback] = None  # Last feedback received
    error_info: Optional[ErrorInfo] = None  # Error information if board is in error state



@dataclass
class JointFeedback:
    """Feedback for a specific joint command"""

    timestamp: float  # When feedback was received
    angle: float  # Joint angle in degrees
    encoder_position: Optional[int] = None  # Encoder position in raw units
    current: Optional[int] = None  # Current in mA
    velocity: Optional[int] = None  # Speed in rpm
    error_code: Optional[int] = None  # Motor error code
    impedance: Optional[float] = None  # Motor impedance value


@dataclass
class StampedTouchFeedback:
    """Timestamped touch sensor feedback for a fingertip"""

    timestamp: float  # When feedback was received
    normal_force: float  # Normal force in N
    normal_force_delta: int  # Change in normal force (raw units)
    tangential_force: float  # Tangential force in N
    tangential_force_delta: int  # Change in tangential force (raw units)
    direction: int  # Force direction (0-359 degrees, fingertip is 0)
    proximity: int  # Proximity value (raw units)
    temperature: int  # Temperature in Celsius


@dataclass
class HandFeedback:
    """Feedback data for whole hand"""

    query_timestamp: float  # When feedback was requested
    joints: Dict[str, JointFeedback]  # Feedback per joint
    touch: Dict[str, StampedTouchFeedback]  # Touch sensor data per fingertip


class DexHandBase:
    """Base class for dexterous hand control"""
    # Constants are now imported from dexhand_protocol.constants
    NUM_MOTORS = NUM_MOTORS  # Make NUM_MOTORS accessible as class attribute

    joint_names = [
        "th_dip",
        "th_mcp",  # Board 0: Thumb
        "th_rot",
        "ff_spr",  # Board 1: Thumb rotation & spread
        "ff_dip",
        "ff_mcp",  # Board 2: First finger
        "mf_dip",
        "mf_mcp",  # Board 3: Middle finger
        "rf_dip",
        "rf_mcp",  # Board 4: Ring finger
        "lf_dip",
        "lf_mcp",  # Board 5: Little finger
    ]
    finger_map = {
        0: "th",
        2: "ff",
        3: "mf",
        4: "rf",
        5: "lf",
    }  # Map from board index to finger name

    def __init__(
            self,
            config: dict, 
            base_id: int, 
            zcan: Optional[ZCANWrapper] = None, 
            log_level: Optional[LogLevel] = LogLevel.INFO,
            device_index: int = 0,
            auto_init: bool = True):
        """Initialize dexterous hand interface

        Args:
            config: Configuration dictionary for the hand
            base_id: Base board ID (0x01 for left, 0x07 for right)
            zcan: Optional existing ZCANWrapper instance to share between hands
            log_level: Logging level for this hand instance
            device_index: Device index for ZCAN device if creating new ZCAN instance
            auto_init: Whether to automatically initialize CAN communication
        """
        self.config = HandConfig(
            channel=config["channel"], hall_scale=config["hall_scale"]
        )
        self.base_id = base_id
        self.zcan = zcan if zcan else ZCANWrapper()
        self._owns_zcan = zcan is None
        self.log_level = log_level
        self._initialized = False

        # Hall position scaling factors
        self._init_hall_scaling()

        # Maintain state for each board
        self.board_states: Dict[int, BoardState] = {
            i: BoardState(
                feedback_timestamp=0,
                feedback=None,
                status_timestamp=0,
                is_normal=True,
                error_info=None,
            )
            for i in range(NUM_BOARDS)
        }
        
        # Automatically initialize if requested
        if auto_init:
            if not self.init(device_index):
                logger.error("Failed to automatically initialize DexHand")

    def _init_hall_scaling(self):
        """Initialize scaling factors for hall position modes"""
        # Conversion factor for hall position modes (from protocol spec)
        factor = (HARDWARE_COUNTS_PER_REV * 
                  GEAR_RATIO * 
                  RESOLUTION_FACTOR / 
                  DEG_PER_REV)  # Converts degrees to hardware units
        self._hall_scale = np.array(self.config.hall_scale) * factor

    def init(self, device_index: int = 0) -> bool:
        """Initialize CAN communication

        Args:
            device_index: Device index for ZCAN device

        Returns:
            bool: True if initialization successful
        """
        # Skip if already initialized
        if self._initialized:
            logger.info("DexHand already initialized")
            return True
            
        # Open ZCAN device if we own it
        if self._owns_zcan:
            if not self.zcan.open(device_index=device_index):
                logger.error("Failed to open CAN device")
                return False

        # Configure channel
        if not self.zcan.configure_channel(self.config.channel):
            logger.error(f"Failed to configure channel {self.config.channel}")
            return False
            
        self._initialized = True
        logger.info(f"Successfully initialized DexHand with base ID {self.base_id}")
        return True
        
    def read_flash_memory(self, board_idx: int, address: int, timeout: float = 0.1) -> Optional[bytes]:
        """Read raw data from flash memory
        
        Args:
            board_idx: Board index (0-5)
            address: Memory address to read from
            timeout: Timeout in seconds
            
        Returns:
            Raw bytes read from memory, or None if read failed
        """
        if not 0 <= board_idx < NUM_BOARDS:
            raise ValueError(f"Invalid board index: {board_idx}")
            
        if not 0 <= address <= 0xFF:
            raise ValueError(f"Invalid memory address: {address}")
            
        # Create read command 
        board_id = self.base_id + board_idx
        can_id = board_id
        data = bytes([MessageType.COMMAND_READ, address])  # Command read, address
        
        # Send command and wait for response
        response_id = MessageType.CONFIG_RESPONSE + board_id  # Response ID is base + config response base
        self.zcan.send_fd_message(self.config.channel, can_id, data)
        
        # Wait for response
        start_time = time.time()
        while time.time() - start_time < timeout:
            messages = self.zcan.receive_fd_messages(self.config.channel)
            for msg_id, msg_data, _ in messages:
                if msg_id != response_id:
                    continue
                    
                # Check if this is a valid response for our address
                if len(msg_data) >= 4 and msg_data[0] == MessageType.COMMAND_READ and msg_data[1] == address:
                    # Return the raw data (first two bytes are command and address)
                    return msg_data[2:]
                    
            time.sleep(0.001)
            
        logger.error(f"Failed to read from board {board_idx} address 0x{address:02x}")
        return None
        
    def write_flash_memory(self, board_idx: int, address: int, value: bytes, timeout: float = 0.1) -> bool:
        """Write data to flash memory
        
        Args:
            board_idx: Board index (0-5)
            address: Memory address to write to
            value: Data to write (up to 6 bytes)
            timeout: Timeout in seconds
            
        Returns:
            True if write succeeded, False otherwise
        """
        if not 0 <= board_idx < NUM_BOARDS:
            raise ValueError(f"Invalid board index: {board_idx}")
            
        if not 0 <= address <= 0xFF:
            raise ValueError(f"Invalid memory address: {address}")
            
        if len(value) > 6:
            raise ValueError(f"Data too large: maximum 6 bytes, got {len(value)}")
            
        # Create write command
        board_id = self.base_id + board_idx
        can_id = board_id
        
        # Command structure: [write command, address, data...]
        data = bytearray([MessageType.COMMAND_WRITE, address]) + value
        # Pad to standard CAN message size using the constant
        data.extend([0] * (CAN_MESSAGE_PADDING_SIZE - len(data)))
        
        # Send command and wait for response
        response_id = MessageType.CONFIG_RESPONSE + board_id
        self.zcan.send_fd_message(self.config.channel, can_id, bytes(data))
        
        # Wait for response
        start_time = time.time()
        while time.time() - start_time < timeout:
            messages = self.zcan.receive_fd_messages(self.config.channel)
            for msg_id, msg_data, _ in messages:
                if msg_id != response_id:
                    continue
                    
                # Check if this is a valid response
                if len(msg_data) >= 5 and msg_data[0] == MessageType.COMMAND_WRITE and msg_data[1] == address:
                    # Check for success indicator at byte 4
                    success = msg_data[4] == 0x01
                    if not success:
                        logger.error(f"Write to board {board_idx} address 0x{address:02x} failed")
                    return success
                    
            time.sleep(0.001)
            
        logger.error(f"No response to write to board {board_idx} address 0x{address:02x}")
        return False
        
    def get_board_firmware_version(self, board_idx: int = 0) -> Optional[int]:
        """Get the firmware version from a specific board
        
        Reads the firmware version value from memory address 0x02 of the specified board.
        
        Args:
            board_idx: Board index (0-5)
            
        Returns:
            Firmware version number, or None if read failed
        """
        data = self.read_flash_memory(board_idx, FlashStorageTable.MEMORY_ADDRESS_FIRMWARE_VERSION)
        if not data or len(data) < 2:
            return None
            
        # Extract firmware version (little-endian)
        return int.from_bytes(data[:2], 'little')
        
    def save_to_flash(self, board_idx: int) -> bool:
        """Save current configuration to flash memory
        
        This command persists any configuration changes to flash memory so they
        will survive power cycles. Without this, configuration changes are temporary.
        
        Args:
            board_idx: Board index (0-5)
            
        Returns:
            True if save succeeded, False otherwise
        """
        # The save-to-flash command is a write to address 0x04 with no data
        return self.write_flash_memory(board_idx, FlashStorageTable.MEMORY_ADDRESS_SAVE_TO_FLASH, b'')
        
    def get_firmware_versions(self) -> Dict[str, Optional[int]]:
        """Get firmware versions from all boards
        
        Returns a dictionary mapping joint names to their firmware versions.
        Boards controlling multiple joints (like the thumb board) will have the same
        firmware version reported for all joints controlled by that board.
        
        Returns:
            Dictionary mapping joint names to firmware versions, or None if read failed
        """
        # Map from board index to joint names
        board_to_joints = {
            0: ["th_dip", "th_mcp"],           # Thumb board
            1: ["th_rot", "ff_spr"],           # Thumb rotation & spread board
            2: ["ff_dip", "ff_mcp"],           # First finger board  
            3: ["mf_dip", "mf_mcp"],           # Middle finger board
            4: ["rf_dip", "rf_mcp"],           # Ring finger board
            5: ["lf_dip", "lf_mcp"],           # Little finger board
        }
        
        versions = {}
        
        # For each board, fetch its firmware version and assign to all joints on that board
        for board_idx in range(NUM_BOARDS):
            version = self.get_board_firmware_version(board_idx)
            for joint_name in board_to_joints.get(board_idx, []):
                versions[joint_name] = version
                
        return versions

    def _get_command_id(self, msg_type: MessageType, board_idx: int) -> int:
        """Get command CAN ID for a board index"""
        if not 0 <= board_idx < NUM_BOARDS:
            raise ValueError(f"Invalid board index: {board_idx}")
        return msg_type + self.base_id + board_idx

    def set_feedback_mode(
        self, mode: FeedbackMode, period_ms: int, enable: bool
    ) -> bool:
        """Configure feedback mode for all boards

        Args:
            mode: Feedback mode
            period_ms: Period in milliseconds (if periodic)
            enable: Enable flag

        Returns:
            bool: True if command sent successfully
        """
        # Create and encode command
        command = FeedbackConfigCommand(mode=mode, period_ms=period_ms, enable=enable)

        try:
            msg_type, data = encode_command(command)
        except ValueError as e:
            logger.error(f"Failed to encode feedback config command: {e}")
            return False

        # Send command to all boards
        for board_idx in range(NUM_BOARDS):
            command_id = self._get_command_id(msg_type, board_idx)
            if not self.zcan.send_fd_message(self.config.channel, command_id, data):
                logger.error(
                    f"Failed to send feedback config command to board {board_idx}"
                )
                return False

        return True

    def set_safe_temperature(
            self, 
            safe_temperature: int,
            log_level: Optional[LogLevel] = None) -> bool:
        """
        Set the security temperature to prevent overheating (default value is 55℃)

        Args:
            safe_temperature: Safety temperature value, ranging from 0 to 255
            log_level: Optional logging level for this operation

        Returns:
            bool: True if command sent successfully, False otherwise
        """
        try:
            if not (0 <= safe_temperature <= 255):
                logger.error(f"Invalid safe temperature: {safe_temperature}")
                return False
            
            if log_level not in {LogLevel.INFO, LogLevel.DEBUG, LogLevel.ERROR, None}:
                logger.error(f"Invalid log level: {log_level}")
                return False
            
        except ValueError as e:
            logger.error(f"Invalid safe temperature: {e}")
            return False
        
        # Construct write command
        data = safe_temperature.to_bytes(1, byteorder='little')
        command = bytes([MessageType.COMMAND_WRITE, FlashStorageTable.MEMORY_ADDRESS_SAFE_TEMPERATURE]) + data
        
        # Send command
        success = self._send_command(command, log_level)
        
        if success:
            self._log_at_level(f"Command sent successfully for set safe temperature: {safe_temperature}", 
                              LogLevel.DEBUG, log_level)
        
        return success


    def set_stall_time(
            self, 
            motor_type: str,
            stall_time: int,
            log_level: Optional[LogLevel] = None) -> bool:
        """
        Set the stall time for motor protection.

        Args:
            motor_type: Motor type, can be "motor1", "motor2", or "motor" (for both motors)
            stall_time: The stall time in milliseconds (0-65535)
            log_level: Optional logging level for this operation

        Returns:
            bool: True if set successfully, False otherwise
        """
        # Validate parameters using the ValidationHelper
        validation_params = {
            'motor_type': {
                'value': motor_type,
                'valid_values': {"motor1", "motor2", "motor"},
                'instance_type': str
            },
            'stall_time': {
                'value': stall_time,
                'valid_range': (0, MAX_STALL_TIME),
                'instance_type': int
            },
            'log_level': {
                'value': log_level,
                'valid_values': {LogLevel.INFO, LogLevel.DEBUG, LogLevel.ERROR, None}
            }
        }
        
        is_valid, error_msg = ValidationHelper.validate_parameters(validation_params, log_level)
        if not is_valid:
            return False

        # Convert stall time to bytes (common for all commands)
        data = stall_time.to_bytes(2, byteorder='little')
        
        # Handle each motor type case
        if motor_type == "motor1":
            # Motor 1 only
            command = bytes([MessageType.COMMAND_WRITE, FlashStorageTable.MEMORY_ADDRESS_STALL_TIME_MOTOR1]) + data
            success = self._send_command(command, log_level)
        elif motor_type == "motor2":
            # Motor 2 only
            command = bytes([MessageType.COMMAND_WRITE, FlashStorageTable.MEMORY_ADDRESS_STALL_TIME_MOTOR2]) + data
            success = self._send_command(command, log_level)
        else:
            # Both motors
            command1 = bytes([MessageType.COMMAND_WRITE, FlashStorageTable.MEMORY_ADDRESS_STALL_TIME_MOTOR1]) + data
            command2 = bytes([MessageType.COMMAND_WRITE, FlashStorageTable.MEMORY_ADDRESS_STALL_TIME_MOTOR2]) + data
            success = self._send_command(command1, log_level) and self._send_command(command2, log_level)
        
        # Log success message if command succeeded
        if success:
            self._log_at_level(f"Command sent successfully for {motor_type} stall time: {stall_time}", 
                             LogLevel.DEBUG, log_level)
        
        return success
    
    def set_pressure_limit_value(
            self, 
            pressure_limit: int, 
            log_level: Optional[LogLevel] = None) -> bool:
        """
        Set the pressure limit value.

        Args:
            pressure_limit: Pressure limit value, ranging from 0 to 20 N
            log_level: Optional logging level for this operation

        Returns:
            bool: True if set successfully, False otherwise
        """
        # Validate parameters using the ValidationHelper
        validation_params = {
            'pressure_limit': {
                'value': pressure_limit,
                'valid_range': (0, MAX_PRESSURE_LIMIT),
                'instance_type': int
            },
            'log_level': {
                'value': log_level,
                'valid_values': {LogLevel.INFO, LogLevel.DEBUG, LogLevel.ERROR, None}
            }
        }
        
        is_valid, error_msg = ValidationHelper.validate_parameters(validation_params, log_level)
        if not is_valid:
            return False

        # Construct write command (convert to hardware units using the constant)
        value_hw = pressure_limit * PRESSURE_LIMIT_SCALE
        data = value_hw.to_bytes(2, byteorder='little')
        command = bytes([MessageType.COMMAND_WRITE, FlashStorageTable.MEMORY_ADDRESS_PRESSURE_LIMIT_VALUE]) + data

        # Send command
        success = self._send_command(command, log_level)
        
        if success:
            self._log_at_level(f"Command sent successfully for pressure limit value: {pressure_limit} N", 
                             LogLevel.DEBUG, log_level)
        
        return success
    
    def set_pressure_limit_enable(
        self, 
        pressure_limit_enable: bool, 
        log_level: Optional[LogLevel] = None
    ) -> bool:
        """
        Enable/disable the pressure limit function.

        Args:
            pressure_limit_enable: True to enable pressure limit, False to disable
            log_level: Optional logging level for this operation

        Returns:
            bool: True if command executed successfully, False otherwise
        """
        # Validate parameters using the ValidationHelper
        validation_params = {
            'pressure_limit_enable': {
                'value': pressure_limit_enable,
                'instance_type': bool
            },
            'log_level': {
                'value': log_level,
                'valid_values': {LogLevel.INFO, LogLevel.DEBUG, LogLevel.ERROR, None}
            }
        }
        
        is_valid, error_msg = ValidationHelper.validate_parameters(validation_params, log_level)
        if not is_valid:
            return False
        
        # Convert boolean to byte
        value = 1 if pressure_limit_enable else 0
        data = value.to_bytes(1, byteorder='little')
        
        # Construct and send command
        command = bytes([MessageType.COMMAND_WRITE, FlashStorageTable.MEMORY_ADDRESS_PRESSURE_LIMIT_ENABLE]) + data
        success = self._send_command(command, log_level)
        
        if success:
            self._log_at_level(f"Command sent successfully for pressure limit enable: {pressure_limit_enable}", 
                             LogLevel.DEBUG, log_level)
        
        return success

    def _log_at_level(self, message: str, level: LogLevel = LogLevel.DEBUG, override_level: Optional[LogLevel] = None):
        """Helper method for consistent logging with level override
        
        Args:
            message: The message to log
            level: Default log level to use
            override_level: Optional override log level from method parameter
        """
        log_level = override_level if override_level is not None else self.log_level
        if log_level <= level:
            # Map log levels to logging functions
            log_methods = {
                LogLevel.DEBUG: logger.debug,
                LogLevel.INFO: logger.info,
                LogLevel.ERROR: logger.error
            }
            # Call the appropriate logging method
            log_method = log_methods.get(level, logger.info)
            log_method(message)

    def _send_command(self, command: bytes, log_level: Optional[LogLevel] = None) -> bool:
        """
        Send a command to the control board.

        Args:
            command (bytes): The command data to be sent.
            log_level: Optional override for logging level

        Returns:
            bool: Returns True if set successfully, False otherwise
        """
        try:
            self._log_at_level(f"Sending command: {command.hex()}", LogLevel.DEBUG, log_level)
            
            # Send commands to all boards
            for board_idx in range(NUM_BOARDS):
                command_id = self._get_command_id(MessageType.CONFIG_COMMAND, board_idx)
                if not self.zcan.send_fd_message(self.config.channel, command_id, command):
                    logger.error(f"Failed to send command to board {board_idx}")
                    return False
            return True
        except Exception as e:
            logger.error(f"Error sending command: {e}")
            return False

    def send_motion_command(
        self,
        board_idx: int,
        motor1_pos: int,
        motor2_pos: int,
        motor_enable: int = BOTH_MOTORS_ENABLE,
        control_mode: ControlMode = ControlMode.IMPEDANCE_GRASP,
        motor1_speed: Optional[int] = None,
        motor2_speed: Optional[int] = None,
        motor1_current: Optional[int] = None,
        motor2_current: Optional[int] = None,
        log_level: Optional[LogLevel] = None
    ) -> bool:
        """Send a motion command to a specific board

        Args:
            board_idx: Board index to command (0-5)
            motor1_pos: Position command for motor 1, in hardware units
            motor2_pos: Position command for motor 2, in hardware units
            motor_enable: Motor enable flags, 0x01 for motor 1, 0x02 for motor 2, 0x03 for both
            control_mode: Control mode
            motor1_speed: Optional speed for motor 1 (0-32767)
            motor2_speed: Optional speed for motor 2 (0-32767)
            motor1_current: Optional current for motor 1 (10-599 mA)
            motor2_current: Optional current for motor 2 (10-599 mA)
            log_level: Optional logging level for this operation

        Returns:
            bool: True if command sent successfully
        """
        # Create and encode command
        command = MotorCommand(
            control_mode=control_mode,
            motor_enable=motor_enable,
            motor1_pos=motor1_pos,
            motor2_pos=motor2_pos,
            motor1_speed=motor1_speed,
            motor2_speed=motor2_speed,
            motor1_current=motor1_current,
            motor2_current=motor2_current,
        )

        try:
            msg_type, data = encode_command(command)
        except ValueError as e:
            logger.error(f"Failed to encode command: {e}")
            return False

        # Send command
        command_id = self._get_command_id(MessageType.MOTION_COMMAND, board_idx)
        if not self.zcan.send_fd_message(self.config.channel, command_id, data):
            logger.error(f"Failed to send command to ID {command_id}")
            return False
            
        self._log_at_level(f"Successfully sent motion command to board {board_idx}", 
                         LogLevel.DEBUG, log_level)
        return True
        
    def send_broadcast_command(
        self,
        control_mode: ControlMode,
        enable_motors: List[bool] = None,  # 12 booleans for each motor
        clear_error: bool = False,
        request_feedback: bool = True,
        is_right_hand: bool = False,
        positions: List[int] = None,  # 12 positions corresponding to each motor
        speeds: List[int] = None,     # 12 speeds corresponding to each motor
        currents: List[int] = None,   # 12 currents corresponding to each motor
        log_level: Optional[LogLevel] = None
    ) -> bool:
        """Send a broadcast command to control all motors at once with CAN ID 0x100.
        
        This method sends a single CAN frame that affects all motors simultaneously,
        which is more efficient than sending individual commands to each board.
        
        Args:
            control_mode: Control mode enum
            enable_motors: List of 12 booleans indicating which motors to enable, in order:
                [th_dip, th_mcp, th_rot, ff_spr, ff_dip, ff_mcp, mf_dip, mf_mcp, rf_dip, rf_mcp, lf_dip, lf_mcp]
            clear_error: Whether to clear errors
            request_feedback: Whether to request feedback
            is_right_hand: True for right hand, False for left hand
            positions: List of 12 position values corresponding to each motor (-32768 to 32767)
            speeds: List of 12 speed values corresponding to each motor (0 to 32767)
            currents: List of 12 current values corresponding to each motor (10 to 599 mA)
            log_level: Optional logging level for this operation
                    
        Returns:
            bool: True if command sent successfully
        """
        # Default to all motors enabled if not specified
        if enable_motors is None:
            enable_motors = [True] * NUM_MOTORS
        
        # Default values for positions, speeds, currents
        if positions is None:
            positions = [0] * NUM_MOTORS
        if speeds is None:
            speeds = [DEFAULT_MOTOR_SPEED] * NUM_MOTORS
        if currents is None:
            currents = [DEFAULT_MOTOR_CURRENT] * NUM_MOTORS
        
        # Create broadcast command
        command = BroadcastCommand(
            control_mode=control_mode,
            enable_motors=enable_motors,
            clear_error=clear_error,
            request_feedback=request_feedback,
            is_right_hand=is_right_hand,
            positions=positions,
            speeds=speeds,
            currents=currents
        )
        
        try:
            # Encode command using the function from commands.py
            frame_data = encode_broadcast_command(command)
            
            self._log_at_level(f"Sending broadcast control frame: {frame_data.hex()}", 
                             LogLevel.DEBUG, log_level)
            
            # Send the frame with the broadcast ID
            if not self.zcan.send_fd_message(self.config.channel, BROADCAST_FRAME_ID, frame_data):
                logger.error("Failed to send broadcast control frame")
                return False
            
            self._log_at_level("Successfully sent broadcast control frame", 
                             LogLevel.INFO, log_level)
                
            return True
            
        except ValueError as e:
            logger.error(f"Failed to encode broadcast command: {e}")
            return False
            
        
    def send_global_command(
        self,
        function_code: GlobalFunctionCode, 
        data: bytes = b'', 
        log_level: Optional[LogLevel] = None
    ) -> bool:
        """Send a global command that affects all boards simultaneously.
        
        Global commands use a special command type that allows a single message
        to be processed by all boards in the system, such as clearing errors
        from all boards at once.
        
        Args:
            function_code: The global function code (defined in GlobalFunctionCode enum)
            data: Optional data for the command (max 6 bytes)
            log_level: Logging level for this operation
                    
        Returns:
            bool: True if command sent successfully
        """
        # 
        command = GlobalCommand(function_code=function_code, data=data)
        
        try:
            # Encode command using the function from commands.py
            msg_type, cmd_data = encode_command(command)
            
            # Get human-readable name for function code
            function_name = function_code.name if hasattr(function_code, 'name') else f"0x{function_code:x}"
            
            # Log debug information if requested
            if (log_level is not None and log_level <= LogLevel.DEBUG) or self.log_level <= LogLevel.DEBUG:
                logger.debug(f"Sending global command: {function_name} (0x{function_code:x}), data={data.hex() if data else 'None'}")
            
            # Send the command
            if not self.zcan.send_fd_message(self.config.channel, msg_type, cmd_data):
                logger.error(f"Failed to send global command: {function_name} (0x{function_code:x})")
                return False
            
            # Log success information if requested
            if (log_level is not None and log_level <= LogLevel.INFO) or self.log_level <= LogLevel.INFO:
                logger.info(f"Successfully sent global command: {function_name} (0x{function_code:x})")
                
            return True
            
        except ValueError as e:
            logger.error(f"Failed to encode global command: {e}")
            return False

    def _refresh_board_states(self,log_level: Optional[LogLevel] = None):
        """Receive CANFD frames to update the states for all boards."""
        # Get all messages
        messages = self.zcan.receive_fd_messages(self.config.channel)

        # Record the received original feedback data
        if log_level is not None and log_level <= LogLevel.DEBUG:
            for msg_id, data, timestamp in messages:
                logger.debug(f"Received original feedback: msg_id={msg_id}, data={data.hex()}, timestamp={timestamp}")

        # Process all received messages
        elif self.log_level <= LogLevel.DEBUG:
            for msg_id, data, timestamp in messages:
                logger.debug(f"Received feedback: msg_id={msg_id}, data={data.hex()}, timestamp={timestamp}")

        # Process all received messages
        for msg_id, data, timestamp in messages:
            try:
                result = process_message(msg_id, data)
                board_idx = msg_id - result.msg_type - self.base_id

                if result.msg_type == MessageType.MOTION_FEEDBACK:
                    self.board_states[board_idx].feedback_timestamp = timestamp
                    self.board_states[board_idx].feedback = result.feedback
                elif result.msg_type == MessageType.ERROR_MESSAGE:
                    self.board_states[board_idx].status_timestamp = timestamp
                    self.board_states[board_idx].is_normal = False
                    self.board_states[board_idx].error_info = result.error_info
                elif result.msg_type == MessageType.CONFIG_RESPONSE:
                    success, command_type = protocol.messages.verify_config_response(
                        msg_id, data
                    )
                    if (
                        success
                        and command_type == protocol.commands.CommandType.CLEAR_ERROR
                    ):
                        self.board_states[board_idx].error_info = None
                        self.board_states[board_idx].is_normal = True
            except ValueError as e:
                # Check if this is an "Invalid message ID" error
                if "Invalid message ID" in str(e):
                    # Downgrade to warning and include message payload
                    logger.warning(f"Unknown message type: {e} - Payload: {data.hex() if data else 'empty'}")
                else:
                    # Keep other errors as errors
                    logger.error(f"Failed to process message: {e}")

    def _clear_board_error(self, board_idx: int) -> bool:
        """Attempt to clear error state for a board

        Args:
            board_idx: Board index to clear error for

        Returns:
            bool: True if error clearance command sent successfully
        """
        # Create and encode clear error command
        clear_cmd = ClearErrorCommand()
        msg_type, clear_data = encode_command(clear_cmd)
        clear_cmd_id = self._get_command_id(msg_type, board_idx)

        # Send command
        if not self.zcan.send_fd_message(self.config.channel, clear_cmd_id, clear_data):
            logger.error(f"Failed to send error clear command to board {board_idx}")
            return False

        return True
    def _prepare_joint_commands(
        self,
        th_rot: Optional[Union[float, JointCommand]] = None,
        th_mcp: Optional[Union[float, JointCommand]] = None,
        th_dip: Optional[Union[float, JointCommand]] = None,
        ff_spr: Optional[Union[float, JointCommand]] = None,
        ff_mcp: Optional[Union[float, JointCommand]] = None,
        ff_dip: Optional[Union[float, JointCommand]] = None,
        mf_mcp: Optional[Union[float, JointCommand]] = None,
        mf_dip: Optional[Union[float, JointCommand]] = None,
        rf_mcp: Optional[Union[float, JointCommand]] = None,
        rf_dip: Optional[Union[float, JointCommand]] = None,
        lf_mcp: Optional[Union[float, JointCommand]] = None,
        lf_dip: Optional[Union[float, JointCommand]] = None,
        control_mode: ControlMode = ControlMode.IMPEDANCE_GRASP,
    ) -> Tuple[List[bool], List[int], List[int], List[int]]:
        """Prepare joint commands by normalizing and scaling all inputs
        
        Args:
            Various joint position arguments (same as move_joints)
            control_mode: Motor control mode
            
        Returns:
            Tuple of (enable_motors, scaled_positions, motor_speeds, motor_currents)
        """
        # Convert all inputs to JointCommand objects for uniform processing
        joint_commands = [
            self._normalize_joint_input(th_dip),
            self._normalize_joint_input(th_mcp),    # Board 0
            self._normalize_joint_input(th_rot),
            self._normalize_joint_input(ff_spr),    # Board 1
            self._normalize_joint_input(ff_dip),
            self._normalize_joint_input(ff_mcp),    # Board 2
            self._normalize_joint_input(mf_dip),
            self._normalize_joint_input(mf_mcp),    # Board 3
            self._normalize_joint_input(rf_dip),
            self._normalize_joint_input(rf_mcp),    # Board 4
            self._normalize_joint_input(lf_dip),
            self._normalize_joint_input(lf_mcp),    # Board 5
        ]
        
        # Create enable_motors list based on which joints are provided
        enable_motors = [cmd is not None for cmd in joint_commands]
        
        # Initialize motor positions, speeds, and currents
        scaled_positions = []
        motor_speeds = []
        motor_currents = []
        
        # Process each joint command
        for i, cmd in enumerate(joint_commands):
            if cmd is not None:
                # Scale position for the specified control mode
                scaled_positions.append(int(self._scale_angle(i, cmd.position, control_mode)))
                
                # Get velocity from joint command or default
                speed = cmd.velocity if cmd.velocity is not None else DEFAULT_MOTOR_SPEED
                motor_speeds.append(speed)
                
                # Get current from joint command or default
                current = cmd.current if cmd.current is not None else DEFAULT_MOTOR_CURRENT
                motor_currents.append(current)
            else:
                scaled_positions.append(0)
                motor_speeds.append(DEFAULT_MOTOR_SPEED)
                motor_currents.append(DEFAULT_MOTOR_CURRENT)
                
        return enable_motors, scaled_positions, motor_speeds, motor_currents
        
    def _send_per_board_commands(
        self,
        enable_motors: List[bool], 
        scaled_positions: List[int], 
        motor_speeds: List[int], 
        motor_currents: List[int],
        control_mode: ControlMode,
        log_level: Optional[LogLevel] = None
    ) -> bool:
        """Send individual commands to each board
        
        Args:
            enable_motors: List of which motors to enable
            scaled_positions: List of scaled position values for each motor
            motor_speeds: List of speed values for each motor
            motor_currents: List of current values for each motor
            control_mode: Motor control mode
            log_level: Logging level for this operation
            
        Returns:
            bool: True if all commands succeeded
        """
        success = True
        for board_idx in range(NUM_BOARDS):
            base_idx = board_idx * 2
            if any(enable_motors[base_idx:base_idx + 2]):
                motor_enable = MOTOR1_ENABLE if enable_motors[base_idx] else 0
                motor_enable |= MOTOR2_ENABLE if enable_motors[base_idx + 1] else 0
                board_success = self.send_motion_command(
                    board_idx=board_idx,
                    motor1_pos=scaled_positions[base_idx],
                    motor2_pos=scaled_positions[base_idx + 1],
                    motor_enable=motor_enable,
                    control_mode=control_mode,
                    motor1_speed=motor_speeds[base_idx],
                    motor2_speed=motor_speeds[base_idx + 1],
                    motor1_current=motor_currents[base_idx],
                    motor2_current=motor_currents[base_idx + 1],
                    log_level=log_level
                )
                if not board_success:
                    logger.error(f"Failed to send command to board {board_idx}")
                    success = False
                    
        # Log success if requested
        if success:
            self._log_at_level("Successfully executed move_joints command", LogLevel.INFO, log_level)
                
        return success
    
    def move_joints(
        self,
        th_rot: Optional[Union[float, JointCommand]] = None,  # thumb rotation
        th_mcp: Optional[Union[float, JointCommand]] = None,  # thumb metacarpophalangeal
        th_dip: Optional[Union[float, JointCommand]] = None,  # thumb coupled distal joints
        ff_spr: Optional[Union[float, JointCommand]] = None,  # four-finger spread
        ff_mcp: Optional[Union[float, JointCommand]] = None,  # first finger metacarpophalangeal
        ff_dip: Optional[Union[float, JointCommand]] = None,  # first finger coupled distal joints
        mf_mcp: Optional[Union[float, JointCommand]] = None,  # middle finger metacarpophalangeal
        mf_dip: Optional[Union[float, JointCommand]] = None,  # middle finger coupled distal joints
        rf_mcp: Optional[Union[float, JointCommand]] = None,  # ring finger metacarpophalangeal
        rf_dip: Optional[Union[float, JointCommand]] = None,  # ring finger coupled distal joints
        lf_mcp: Optional[Union[float, JointCommand]] = None,  # little finger metacarpophalangeal
        lf_dip: Optional[Union[float, JointCommand]] = None,  # little finger coupled distal joints
        control_mode: ControlMode = ControlMode.IMPEDANCE_GRASP,  # Control mode
        use_broadcast: bool = True,  # Whether to use broadcast command (more efficient)
        clear_error: bool = False,  # Whether to clear errors (only for broadcast mode)
        request_feedback: bool = True,  # Whether to request feedback (only for broadcast mode)
        log_level: Optional[LogLevel] = None,  # Log level
    ):
        """Move hand joints to specified angles with optional current and velocity settings.

        For each finger, there are two independent DOFs:
        - MCP (metacarpophalangeal) joint: Controls base joint flexion
        - DIP (coupled): Controls coupled motion of PIP and DIP joints

        Additional DOFs:
        - th_rot: Thumb rotation/opposition
        - ff_spr: Four-finger spread (abduction between fingers)

        Args:
            th_rot: Thumb rotation - float (position only) or JointCommand(position, current, velocity)
            th_mcp: Thumb MCP flexion - float (position only) or JointCommand(position, current, velocity)
            th_dip: Thumb coupled PIP-DIP flexion - float (position only) or JointCommand(position, current, velocity)
            ff_spr: Four-finger spread - float (position only) or JointCommand(position, current, velocity)
            ff_mcp: Index MCP flexion - float (position only) or JointCommand(position, current, velocity)
            ff_dip: Index coupled PIP-DIP flexion - float (position only) or JointCommand(position, current, velocity)
            mf_mcp: Middle MCP flexion - float (position only) or JointCommand(position, current, velocity)
            mf_dip: Middle coupled PIP-DIP flexion - float (position only) or JointCommand(position, current, velocity)
            rf_mcp: Ring MCP flexion - float (position only) or JointCommand(position, current, velocity)
            rf_dip: Ring coupled PIP-DIP flexion - float (position only) or JointCommand(position, current, velocity)
            lf_mcp: Little MCP flexion - float (position only) or JointCommand(position, current, velocity)
            lf_dip: Little coupled PIP-DIP flexion - float (position only) or JointCommand(position, current, velocity)
            control_mode: Motor control mode
            use_broadcast: If True, send a single broadcast command for all joints (more efficient, default: True)
            clear_error: Whether to clear errors (only for broadcast mode)
                        NOTE: KNOWN BUG: The combination of clear_error=True with use_broadcast=True 
                        is not functioning correctly in this version. 
                        If you need to clear errors, use either:
                        1. move_joints(..., clear_error=True, use_broadcast=False)
                        2. clear_errors(use_broadcast=False) followed by move_joints(...)
            request_feedback: Whether to request feedback (only for broadcast mode)
            log_level: Logging level for this operation
            
        Examples:
            # Move a single joint with position only (using default current and velocity)
            hand.move_joints(th_rot=45)
            
            # Move a single joint with full control over position, current, and velocity
            hand.move_joints(th_rot=JointCommand(position=45, current=30, velocity=10000))
            
            # Move multiple joints with different control approaches
            hand.move_joints(
                th_rot=45,  # Position only
                ff_mcp=JointCommand(position=30, current=50),  # Position and current, default velocity
                mf_dip=JointCommand(position=20, velocity=5000)  # Position and velocity, default current
            )
        """
        # Warn about the problematic combination of clear_error=True with use_broadcast=True
        if clear_error and use_broadcast:
            logger.warning("WARNING: move_joints called with clear_error=True and use_broadcast=True, "
                         "which has a known bug. Setting clear_error=False. "
                         "To clear errors, either use clear_error=True with use_broadcast=False, "
                         "or call clear_errors(use_broadcast=False) separately.")
            # Force clear_error to False to avoid the buggy behavior
            clear_error = False
        
        # Prepare all joint commands
        enable_motors, scaled_positions, motor_speeds, motor_currents = self._prepare_joint_commands(
            th_rot=th_rot, th_mcp=th_mcp, th_dip=th_dip,
            ff_spr=ff_spr, ff_mcp=ff_mcp, ff_dip=ff_dip,
            mf_mcp=mf_mcp, mf_dip=mf_dip,
            rf_mcp=rf_mcp, rf_dip=rf_dip,
            lf_mcp=lf_mcp, lf_dip=lf_dip,
            control_mode=control_mode
        )
        
        # Use broadcast mode if requested (more efficient)
        if use_broadcast:
            # Use right hand if this is a RightDexHand instance
            is_right_hand = isinstance(self, RightDexHand)
            
            # Send the broadcast command
            result = self.send_broadcast_command(
                control_mode=control_mode,
                enable_motors=enable_motors,
                positions=scaled_positions,
                speeds=motor_speeds,
                currents=motor_currents,
                clear_error=clear_error,
                request_feedback=request_feedback,
                is_right_hand=is_right_hand,
                log_level=log_level
            )
            
            return result
        else:
            # Use traditional per-board commands
            result = self._send_per_board_commands(
                enable_motors=enable_motors,
                scaled_positions=scaled_positions,
                motor_speeds=motor_speeds,
                motor_currents=motor_currents,
                control_mode=control_mode,
                log_level=log_level
            )
            
            # Also clear errors if requested with non-broadcast mode
            if result and clear_error:
                # Call clear_errors with use_broadcast=False to use individual board commands
                return self.clear_errors(clear_all=True, use_broadcast=False, log_level=log_level)
            
            return result
            
    def _process_joint_feedback(self, board_idx: int, state: BoardState) -> Dict[str, JointFeedback]:
        """Process feedback for joints on a specific board
        
        Args:
            board_idx: Board index
            state: Board state object
            
        Returns:
            Dictionary mapping joint names to JointFeedback objects
        """
        base_idx = board_idx * 2
        timestamp_feedback = time.time_ns()
        result = {}
        
        if state.feedback is None:
            # No feedback available, create empty feedback objects
            for i in range(MOTORS_PER_BOARD):
                joint_idx = base_idx + i
                result[self.joint_names[joint_idx]] = JointFeedback(
                    timestamp=timestamp_feedback,
                    angle=float("nan"),
                    encoder_position=None,
                    current=None,
                    velocity=None,
                    error_code=None,
                    impedance=None
                )
            return result

        # Process joint feedback when available
        motors = [state.feedback.motor1, state.feedback.motor2]
        for i in range(MOTORS_PER_BOARD):
            joint_idx = base_idx + i
            result[self.joint_names[joint_idx]] = JointFeedback(
                timestamp=timestamp_feedback,
                angle=motors[i].angle,
                encoder_position=motors[i].position,
                current=motors[i].current,
                velocity=motors[i].velocity,
                error_code=getattr(motors[i], 'error_code', None),
                impedance=getattr(motors[i], 'impedance', None)
            )
            
        return result
        
    def _process_touch_feedback(self, board_idx: int, state: BoardState) -> Optional[Tuple[str, StampedTouchFeedback]]:
        """Process touch sensor feedback for a specific board
        
        Args:
            board_idx: Board index
            state: Board state object
            
        Returns:
            Tuple of (finger_name, touch_feedback) or None if no touch data
        """
        if state.feedback is None or state.feedback.touch is None:
            return None
            
        if board_idx not in self.finger_map:
            return None
            
        timestamp_feedback = time.time_ns()
        touch_data = state.feedback.touch
        finger_name = self.finger_map[board_idx]
        
        touch_feedback = StampedTouchFeedback(
            timestamp=timestamp_feedback,
            normal_force=touch_data.normal_force,
            normal_force_delta=touch_data.normal_force_delta,
            tangential_force=touch_data.tangential_force,
            tangential_force_delta=touch_data.tangential_force_delta,
            direction=touch_data.direction,
            proximity=touch_data.proximity,
            temperature=touch_data.temperature
        )
        
        return finger_name, touch_feedback
            
    def get_feedback(self) -> HandFeedback:
        """Get feedback from all joints and touch sensors

        Returns:
            HandFeedback object.
        """
        # Record query start time
        query_timestamp = time.time_ns() / 1e9

        # Refresh board states to get feedback
        self._refresh_board_states()

        # Process feedback from all boards
        joint_feedback = {}
        touch = {}
        
        for board_idx, state in self.board_states.items():
            # Process joint feedback
            joint_feedback.update(self._process_joint_feedback(board_idx, state))
            
            # Process touch sensor feedback if available
            touch_result = self._process_touch_feedback(board_idx, state)
            if touch_result:
                finger_name, touch_feedback = touch_result
                touch[finger_name] = touch_feedback

        return HandFeedback(
            query_timestamp=query_timestamp,
            joints=joint_feedback,
            touch=touch,
        )

    def get_errors(self) -> Dict[int, Optional[ErrorInfo]]:
        """Get error information for whole hand

        Returns:
            Dict mapping board index to ErrorInfo if an error is present
        """
        return {i: state.error_info for i, state in self.board_states.items()}

    def clear_errors(self, clear_all=True, use_broadcast=False, log_level: Optional[LogLevel] = None) -> bool:
        """Clear errors for the hand
        
        NOTE: KNOWN BUG: The broadcast mode for clearing errors is not functioning correctly 
              in this version. Always set use_broadcast=False or leave at default.
              This will be fixed in a future release.
        
        Args:
            clear_all: If True, attempt to clear errors for all boards even if not in error state
            use_broadcast: DEPRECATED - use_broadcast=True will not work correctly in this version
                          and will be fixed in a future release. Set to False or leave as default.
            log_level: Optional logging level for the operation
            
        Returns:
            bool: True if commands sent successfully
        """
        # Use the more efficient broadcast command when clearing all errors
        if clear_all and use_broadcast:
            # BUGGY: This implementation does not work correctly
            # Will be fixed in a future release
            logger.warning("WARNING: clear_errors called with use_broadcast=True, which has a known bug. "
                          "Using per-board clear instead. Set use_broadcast=False or leave at default.")
            # Force use_broadcast to False to use the working implementation
            use_broadcast = False
        
        # Use individual commands for clearing (the reliable method)
        success = True
        for board_idx in range(NUM_BOARDS):
            if clear_all or not self.board_states[board_idx].is_normal:
                if not self._clear_board_error(board_idx):
                    success = False
        
        return success
    
    def _normalize_joint_input(self, value: Optional[Union[float, JointCommand]]) -> Optional[JointCommand]:
        """Convert float or JointCommand input to a normalized JointCommand
        
        Args:
            value: Input value, can be None, float (position only), or JointCommand
            
        Returns:
            Normalized JointCommand or None if input is None
        """
        if value is None:
            return None
        elif isinstance(value, (int, float)):
            return JointCommand(position=float(value))
        elif isinstance(value, JointCommand):
            return value
        else:
            raise ValueError(f"Invalid joint input: {value}. Must be None, float, or JointCommand")
    
    def _scale_angle(
        self, motor_idx: int, angle: float, control_mode: ControlMode
    ) -> int:
        """Scale angle based on control mode
        
        Args:
            motor_idx: Motor index (0-11)
            angle: Angle in degrees
            control_mode: Control mode enum
            
        Returns:
            Scaled angle value in hardware units
        """
        if control_mode in (
            ControlMode.HALL_POSITION,
            ControlMode.PROTECT_HALL_POSITION,
        ):
            # For hall position modes, use hall scaling factors
            return int(angle * self._hall_scale[motor_idx])
        else:
            # For other modes (e.g., cascaded PID), use standard scaling
            return int(angle * CASCADE_PID_SCALE)

    def reset_joints(self, use_broadcast: bool = True, log_level: Optional[LogLevel] = None):
        """Reset all joints to their zero positions.

        This is equivalent to setting all joint angles to 0 degrees.
        Uses CASCADED_PID control mode.

        Args:
            use_broadcast: If True, use more efficient broadcast mode (default: True)
            log_level: Logging level for the operation
        """
        return self.move_joints(
            th_rot=0,
            th_mcp=0,
            th_dip=0,
            ff_spr=0,
            ff_mcp=0,
            ff_dip=0,
            mf_mcp=0,
            mf_dip=0,
            rf_mcp=0,
            rf_dip=0,
            lf_mcp=0,
            lf_dip=0,
            control_mode=ControlMode.CASCADED_PID,
            use_broadcast=use_broadcast,
            log_level=log_level
        )

    def close(self):
        """Close CAN communication"""
        if self._owns_zcan:
            self.zcan.close()


def _load_hand_config(hand_type: str) -> dict:
    """Load hand configuration from config file
    
    Args:
        hand_type: Type of hand ("left_hand" or "right_hand")
        
    Returns:
        Hand configuration dictionary
    """
    config_path = os.path.join(
        os.path.dirname(__file__), "../config", "config.yaml"
    )
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    return config["DexHand"][hand_type]


class LeftDexHand(DexHandBase):
    """Control interface for left dexterous hand"""
    NUM_BOARDS = NUM_BOARDS

    def __init__(self, 
                 zcan: Optional[ZCANWrapper] = None, 
                 log_level: Optional[LogLevel] = LogLevel.INFO,
                 device_index: int = 0,
                 auto_init: bool = True):
        """Initialize left hand interface
        
        Args:
            zcan: Optional existing ZCANWrapper instance to share between hands
            log_level: Logging level for operation feedback
            device_index: Device index for ZCAN device if creating new instance
            auto_init: Whether to automatically initialize CAN communication
        """
        super().__init__(
            _load_hand_config("left_hand"),
            BoardID.LEFT_HAND_BASE,
            zcan,
            log_level=log_level,
            device_index=device_index,
            auto_init=auto_init
        )


class RightDexHand(DexHandBase):
    """Control interface for right dexterous hand"""
    NUM_BOARDS = NUM_BOARDS

    def __init__(self, 
                 zcan: Optional[ZCANWrapper] = None, 
                 log_level: Optional[LogLevel] = LogLevel.INFO,
                 device_index: int = 0,
                 auto_init: bool = True):
        """Initialize right hand interface
        
        Args:
            zcan: Optional existing ZCANWrapper instance to share between hands
            log_level: Logging level for operation feedback
            device_index: Device index for ZCAN device if creating new instance
            auto_init: Whether to automatically initialize CAN communication
        """
        super().__init__(
            _load_hand_config("right_hand"),
            BoardID.RIGHT_HAND_BASE,
            zcan,
            log_level=log_level,
            device_index=device_index,
            auto_init=auto_init
        )
