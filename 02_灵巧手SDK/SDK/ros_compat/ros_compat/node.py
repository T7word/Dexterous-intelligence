from typing import Any, Callable, Optional, Type, Union
from functools import partial

from . import ROS_VERSION

# Import common ROS interface first
if ROS_VERSION == 2:
    try:
        import rclpy
        ros = rclpy  # For consistent naming
        from rclpy.node import Node as ROS2Node
        from rclpy.callback_groups import ReentrantCallbackGroup
        from rclpy.time import Time as ROS2Time
        from rclpy.timer import Timer as ROS2Timer
        from rclpy.qos import QoSProfile, ReliabilityPolicy
    except ImportError:
        ROS_VERSION = None
        ros = None
elif ROS_VERSION == 1:
    try:
        import rospy
        ros = rospy  # For consistent naming
        from rospy import Time as ROS1Time
    except ImportError:
        ROS_VERSION = None
        ros = None
else:
    ros = None

from .time import ROSTime
from .logging import ROSLogger

class ROSNode:
    """Base class for ROS nodes that work with both ROS1 and ROS2.

    This class provides a unified interface for creating nodes that work
    transparently with both ROS1 (rospy) and ROS2 (rclpy).

    Args:
        node_name (str): Name of the node
    """
    def __init__(self, node_name: str):
        if ROS_VERSION is None:
            raise ImportError("Neither ROS1 (rospy) nor ROS2 (rclpy) was found")
        self.node_name = node_name

        if ROS_VERSION == 2:
            ros.init()
            self.node = ROS2Node(node_name)
            self.callback_group = ReentrantCallbackGroup()
            self._ros_time = ROSTime(self.node)
        else:
            ros.init_node(node_name)
            self.node = ros.get_caller_id()
            self._ros_time = ROSTime()

        self.logger = ROSLogger(node_name)

    def create_publisher(self,
                        msg_type: Type,
                        topic: str,
                        queue_size: int = 10) -> Any:
        """Create a publisher that works with both ROS versions.

        Args:
            msg_type: Message type class
            topic: Topic name to publish to
            queue_size: Publisher queue size

        Returns:
            Publisher object (either ROS1 or ROS2)
        """
        if ROS_VERSION == 2:
            qos = QoSProfile(depth=queue_size,
                           reliability=ReliabilityPolicy.RELIABLE)
            return self.node.create_publisher(msg_type, topic, qos)
        return ros.Publisher(topic, msg_type, queue_size=queue_size)

    def create_subscription(self,
                            msg_type: Type,
                            topic: str,
                            callback: Callable,
                            queue_size: int = 10) -> Any:
        """Create a subscriber that works with both ROS versions.

        Args:
            msg_type: Message type class
            topic: Topic name to subscribe to
            callback: Callback function that takes a message argument
            queue_size: Subscriber queue size

        Returns:
            Subscriber object (either ROS1 or ROS2)
        """
        if ROS_VERSION == 2:
            qos = QoSProfile(depth=queue_size,
                            reliability=ReliabilityPolicy.RELIABLE)
            return self.node.create_subscription(
                msg_type, topic, callback, qos,
                callback_group=self.callback_group
            )
        return ros.Subscriber(topic, msg_type, callback, queue_size=queue_size)

    def create_timer(self,
                    period: float,
                    callback: Callable) -> Any:
        """Create a timer that works with both ROS versions.

        Args:
            period: Timer period in seconds
            callback: Callback function to be called when timer triggers

        Returns:
            Timer object (either ROS1 or ROS2)
        """
        if ROS_VERSION == 2:
            return self.node.create_timer(
                period, callback,
                callback_group=self.callback_group
            )
        return ros.Timer(ros.Duration(period), callback)

    def create_service(self,
                    srv_type: Type,
                    name: str,
                    callback: Callable) -> Any:
        """Create a service server that works with both ROS versions.

        Args:
            srv_type: Service type class
            name: Service name
            callback: Service callback function

        Returns:
            Service server object (either ROS1 or ROS2)
        """
        if ROS_VERSION == 2:
            return self.node.create_service(
                srv_type, name, callback,
                callback_group=self.callback_group
            )
        return ros.Service(name, srv_type, callback)

    def create_client(self,
                    srv_type: Type,
                    name: str,
                    timeout: Optional[float] = None) -> Any:
        """Create a service client that works with both ROS versions.

        Args:
            srv_type: Service type class
            name: Service name
            timeout: Optional timeout for waiting for service

        Returns:
            Service client object (either ROS1 or ROS2)
        """
        if ROS_VERSION == 2:
            client = self.node.create_client(
                srv_type, name,
                callback_group=self.callback_group
            )
            if timeout is not None:
                client.wait_for_service(timeout)
            return client
        return ros.ServiceProxy(name, srv_type, persistent=True)

    def get_ros_time(self) -> ROSTime:
        """Get a ROSTime instance configured for this node.

        Returns:
            ROSTime: Time utility instance configured for this node
        """
        return self._ros_time

    def spin(self) -> None:
        """Spin the node."""
        if ROS_VERSION == 2:
            ros.spin(self.node)
        else:
            ros.spin()

    def spin_once(self) -> None:
        """Execute one spin of the node."""
        if ROS_VERSION == 2:
            ros.spin_once(self.node)
        else:
            ros.spinOnce()

    def shutdown(self) -> None:
        """Shutdown the node."""
        if ROS_VERSION == 2:
            self.node.destroy_node()
            ros.shutdown()
        else:
            ros.signal_shutdown('Node is shutting down')
