from . import ROS_VERSION
from typing import Optional, Any

try:
    import rclpy as ros
    from rclpy.time import Time as ROS2Time
    Time = ROS2Time
except ImportError:
    try:
        import rospy as ros
        from rospy import Time as ROS1Time
        Time = ROS1Time
    except ImportError:
        ros = None
        Time = None

class ROSTime:
    """Wrapper for ROS time operations.

    This class provides a unified interface for time operations that work
    consistently across ROS1 and ROS2.

    Args:
        node: Optional ROS node reference. Required for ROS2 time operations.
    """
    def __init__(self, node: Optional[Any] = None):
        self._node = node
        if ROS_VERSION == 2 and node is None:
            raise ValueError("ROS2 requires a node reference for time operations")

    def now(self) -> float:
        """Get the current ROS time.

        Returns:
            float: Current time in seconds.

        Examples:
            >>> time = node.get_ros_time()  # Get ROSTime instance from node
            >>> current_time = time.now()
            >>> print(f"Current time: {current_time}")
        """
        if ROS_VERSION is None:
            raise ImportError("Neither ROS1 (rospy) nor ROS2 (rclpy) was found")
        if ROS_VERSION == 2:
            return float(self._node.get_clock().now().nanoseconds) / 1e9
        return float(ros.Time.now().to_nsec()) / 1e9



    def to_msg(self) -> Any:
        """Get current time in format suitable for message headers.

        Returns:
            Time message in ROS1 or ROS2 format suitable for header.stamp

        Examples:
            >>> time = node.get_ros_time()
            >>> msg.header.stamp = time.to_msg()
        """
        if ROS_VERSION == 2:
            return self._node.get_clock().now().to_msg()
        return ros.Time.now()

    @staticmethod
    def sleep(duration: float) -> None:
        """Sleep for specified duration in seconds."""
        if ROS_VERSION is None:
            raise ImportError("Neither ROS1 (rospy) nor ROS2 (rclpy) was found")
        if ROS_VERSION == 2:
            ros.time.sleep(duration)
        else:
            ros.sleep(duration)
