from ros_compat import ROSNode
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
from typing import Any
from ros_compat.time import ROSTime

class ExampleNode(ROSNode):
    """Example node demonstrating the usage of ros_compat features."""

    def __init__(self):
        super().__init__('example_node')

        # Get ROS time instance
        self.ros_time = self.get_ros_time()

        # Publishers
        self.string_pub = self.create_publisher(String, 'output')

        # Subscribers
        self.create_subscription(String, 'input', self.message_callback)

        # Timer
        self.create_timer(1.0, self.timer_callback)

        # Service
        self.create_service(SetBool, 'toggle', self.service_callback)

        self.logger.info('Example node initialized')

    def message_callback(self, msg: String) -> None:
        """Handle incoming messages."""
        self.logger.info(f'Received message: {msg.data}')

    def timer_callback(self, event: Any) -> None:
        """Periodic timer callback."""
        msg = String()
        msg.data = f'Current time: {self.ros_time.now()}'
        self.string_pub.publish(msg)

    def service_callback(self, request: SetBool, response: Any) -> Any:
        """Handle service requests."""
        if ROS_VERSION == 2:
            response = SetBool.Response()

        response.success = True
        response.message = f'Service called with data: {request.data}'
        return response

if __name__ == '__main__':
    node = ExampleNode()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
