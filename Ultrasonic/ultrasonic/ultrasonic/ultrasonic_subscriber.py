import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Range


class UltrasonicSubscriber(Node):

    def __init__(self):
        super().__init__('ultrasonic_subscriber')
        self.subscription = self.create_subscription(
            Range,
            'ultrasonic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.range)


def main(args=None):
    rclpy.init(args=args)

    ultrasonic_subscriber = UltrasonicSubscriber()

    rclpy.spin(ultrasonic_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ultrasonic_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
