import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Header
import time

class ZeroAckermannCommandPublisherNode(Node):
    def __init__(self):
        super().__init__('defaultCmd')

        # Create a publisher for the AckermannDriveStamped command
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/high_level/ackermann_cmd_mux/input/default',
            10  # QoS profile depth
        )

        # Create a timer to periodically publish zero Ackermann command
        self.timer = self.create_timer(1.0 / 6, self.publish_zero_ackermann_command)

    def publish_zero_ackermann_command(self):
        # Create a zero AckermannDriveStamped message
        zero_ackermann_msg = AckermannDriveStamped()
        zero_ackermann_msg.header = Header()
        zero_ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        zero_ackermann_msg.drive.speed = 0.0
        zero_ackermann_msg.drive.steering_angle = 0.0

        # Publish the zero AckermannDriveStamped message
        self.publisher.publish(zero_ackermann_msg)
        #self.get_logger().info("Published zero AckermannDriveStamped command")

def main(args=None):
    rclpy.init(args=args)

    zero_ackermann_publisher_node = ZeroAckermannCommandPublisherNode()

    rclpy.spin(zero_ackermann_publisher_node)

    zero_ackermann_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()