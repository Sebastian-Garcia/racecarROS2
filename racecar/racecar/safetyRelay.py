import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped

class TopicRepublisherNode(Node):
    def __init__(self):
        super().__init__('safetyRelay')
        
        # Create a subscriber to the original topic
        self.subscription = self.create_subscription(
            AckermannDriveStamped,
            '/vesc/input/safety',
            self.callback,
            10  # QoS profile depth
        )
        
        # Create a publisher for the republished topic
        self.publisher = self.create_publisher(
            AckermannDriveStamped,
            '/vesc/low_level/input/safety',
            10  # QoS profile depth
        )

    def callback(self, msg):
        # Republish the message to the new topic
        self.publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)

    republisher_node = TopicRepublisherNode()

    rclpy.spin(republisher_node)

    republisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
