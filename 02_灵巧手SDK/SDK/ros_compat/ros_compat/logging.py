from . import ROS_VERSION

try:
    import rclpy as ros
except ImportError:
    try:
        import rospy as ros
    except ImportError:
        ros = None

class ROSLogger:
    """Wrapper for ROS logging functions.

    Provides a unified interface for logging in both ROS1 and ROS2.

    Args:
        node_name (str): Name of the node for logging context
    """
    def __init__(self, node_name: str):
        if ROS_VERSION is None:
            raise ImportError("Neither ROS1 (rospy) nor ROS2 (rclpy) was found")
        self.node_name = node_name
        if ROS_VERSION == 2:
            self.logger = ros.logging.get_logger(node_name)

    def info(self, msg: str) -> None:
        """Log an info message."""
        if ROS_VERSION == 2:
            self.logger.info(msg)
        else:
            ros.loginfo(msg)

    def warn(self, msg: str) -> None:
        """Log a warning message."""
        if ROS_VERSION == 2:
            self.logger.warn(msg)
        else:
            ros.logwarn(msg)

    def error(self, msg: str) -> None:
        """Log an error message."""
        if ROS_VERSION == 2:
            self.logger.error(msg)
        else:
            ros.logerr(msg)

    def debug(self, msg: str) -> None:
        """Log a debug message."""
        if ROS_VERSION == 2:
            self.logger.debug(msg)
        else:
            ros.logdebug(msg)
