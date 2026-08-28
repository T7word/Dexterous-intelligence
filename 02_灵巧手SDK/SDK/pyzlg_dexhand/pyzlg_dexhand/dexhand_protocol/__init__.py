"""DexHand CANFD Protocol Implementation

This package implements the DexHand CANFD communication protocol, with separate
modules for host-to-board commands and board-to-host messages.
"""

from enum import IntEnum
from typing import Optional

class MessageType(IntEnum):
    """CAN message type identifiers and their base addresses"""
    COMMAND_WRITE = 0x02            # Host -> Hand: Write command
    COMMAND_READ = 0x01             # Host -> Hand: Read request 
    MOTION_COMMAND = 0x100     # Host -> Hand: Motion control
    CONFIG_COMMAND = 0x00      # Host -> Hand: Configuration
    MOTION_FEEDBACK = 0x180    # Hand -> Host: Motion state
    CONFIG_RESPONSE = 0x80     # Hand -> Host: Config response
    ERROR_MESSAGE = 0x600      # Hand -> Host: Errors
    GLOBAL_COMMAND = 0xFF      # Host -> Hand: Global command
    INVALID = 0x1000          # Special: Message decoding failed
    UNKNOWN = 0x1001          # Special: Unknown message type

class BoardID(IntEnum):
    """Base ID values for left and right hand boards"""
    LEFT_HAND_BASE = 0x01
    RIGHT_HAND_BASE = 0x07

class FlashStorageTable:
    """Flash Storage Table for global location"""
    MEMORY_ADDRESS_SYS_ID = 0x01            # System ID
    MEMORY_ADDRESS_FIRMWARE_VERSION = 0x02  # Firmware version
    MEMORY_ADDRESS_STALL_TIME_MOTOR1 = 0x50 # Motor 1 stall time
    MEMORY_ADDRESS_STALL_TIME_MOTOR2 = 0x51 # Motor 2 stall time 
    MEMORY_ADDRESS_SAFE_TEMPERATURE = 0x54  # Safe temperature
    MEMORY_ADDRESS_PRESSURE_LIMIT_VALUE = 0x58 # Pressure limit value
    MEMORY_ADDRESS_MOTOR1_TORQUE = 0x5A     # Motor 1 torque
    MEMORY_ADDRESS_MOTOR2_TORQUE = 0x5B     # Motor 2 torque
    MEMORY_ADDRESS_BOTH_MOTORS_TORQUE = 0x5C  # Both motors torque
    MEMORY_ADDRESS_PRESSURE_LIMIT_ENABLE = 0xA6 # Pressure limit enable 
    MEMORY_ADDRESS_SAVE_TO_FLASH = 0x04     # Save to flash command

def get_message_type(can_id: int) -> Optional[MessageType]:
    """Determine message type from CAN ID"""
    base = can_id & 0xF80  # Keep top 7 bits
    try:
        return MessageType(base)
    except ValueError:
        return None

# Export common types
__all__ = ['MessageType', 'BoardID', 'FlashStorageTable', 'get_message_type', 'commands', 'messages']