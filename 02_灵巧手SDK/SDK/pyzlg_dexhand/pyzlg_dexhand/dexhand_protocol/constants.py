"""Hardware and protocol constants for DexHand"""

# Hardware configuration constants
NUM_MOTORS = 12  # Total motors in hand
NUM_BOARDS = 6   # Number of control boards
MOTORS_PER_BOARD = 2  # Number of motors per board

# Protocol constants
MIN_FIRMWARE_VERSION = 25418  # Minimum recommended firmware version

# Default values
DEFAULT_MOTOR_SPEED = 15000   # Default speed for motors
DEFAULT_MOTOR_CURRENT = 20    # Default current in mA

# Hardware unit conversion constants
HARDWARE_COUNTS_PER_REV = 6   # Encoder counts per revolution
GEAR_RATIO = 25              # Gear ratio (25:1)
RESOLUTION_FACTOR = 2**4     # 16-bit resolution factor
DEG_PER_REV = 360.0          # Degrees per revolution

# Hardware unit conversion for CASCADED_PID mode
CASCADE_PID_SCALE = 100      # Scale factor for CASCADED_PID mode

# Control frames
BROADCAST_FRAME_ID = 0x100   # CAN ID for broadcast frames

# Hardware unit conversion factors
PRESSURE_LIMIT_SCALE = 100   # Scale factor for pressure limit (N to hardware units)

# Motor enable bit flags
MOTOR1_ENABLE = 0x01  # Bit flag to enable motor 1
MOTOR2_ENABLE = 0x02  # Bit flag to enable motor 2
BOTH_MOTORS_ENABLE = 0x03  # Bit flag to enable both motors

# Validation limits
MAX_STALL_TIME = 65535       # Maximum allowed stall time in milliseconds
MAX_PRESSURE_LIMIT = 20      # Maximum allowed pressure limit in N
MIN_MOTOR_CURRENT = 10       # Minimum allowed motor current in mA
MAX_MOTOR_CURRENT = 599      # Maximum allowed motor current in mA

# Message processing constants
CAN_MESSAGE_PADDING_SIZE = 8  # Standard padding size for CAN messages

# Timing constants
DEFAULT_POLL_INTERVAL_MS = 1  # Default polling interval in milliseconds
DEFAULT_TIMEOUT_MS = 100      # Default timeout in milliseconds