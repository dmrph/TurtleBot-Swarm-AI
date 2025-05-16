import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

class UltrasonicSubscriber(Node):
    def __init__(self):
        super().__init__('ultrasonic_subscriber')
        self.get_logger().info('Ultrasonic Subscriber Node Started')

        # Create a publisher for /cmd_vel
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscribe to the ultrasonic sensor topic
        self.subscription = self.create_subscription(
            Range,
            '/ultrasonic_sensor',
            self.listener_callback,
            10)

        # Timer-related variables
        self.backup_timer = None
        self.backup_count = 0
        self.backup_limit = 10  # Number of times to send backup command

    def listener_callback(self, msg):
        """Callback function that runs when a new message is received."""
        distance = msg.range
        self.get_logger().info(f'Received distance: {distance:.2f} meters')

        # If too close, start backing up
        if distance <= 3.0:  # 3 meters (300 cm)
            self.get_logger().info('Obstacle detected! Starting backup...')
            self.start_backup()

    def start_backup(self):
        """Start a timer that publishes backup Twist messages."""
        if self.backup_timer is None:
            self.backup_count = 0
            self.backup_timer = self.create_timer(0.2, self.backup_callback)

    def backup_callback(self):
        """Callback to publish backup movement commands."""
        if self.backup_count < self.backup_limit:
            cmd = Twist()
            cmd.linear.x = -0.2  # Move backward
            cmd.angular.z = 0.0  # No rotation
            self.velocity_publisher.publish(cmd)
            self.get_logger().info(f'Backing up... step {self.backup_count + 1}')
            self.backup_count += 1
        else:
            # Stop backing up after enough steps
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.velocity_publisher.publish(cmd)
            self.get_logger().info('Finished backing up.')

            # Cancel the timer
            self.backup_timer.cancel()
            self.backup_timer = None

def main(args=None):
    rclpy.init(args=args)

    ultrasonic_subscriber = UltrasonicSubscriber()
    
    try:
        rclpy.spin(ultrasonic_subscriber)
    except KeyboardInterrupt:
        ultrasonic_subscriber.get_logger().info('Keyboard Interrupt (CTRL-C)... shutting down.')

    ultrasonic_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
