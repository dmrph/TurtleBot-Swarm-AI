import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from ultrasonic.rpi_interface import RPi_HCS04

class UltrasonicPublisher(Node):

    def __init__(self):
        super().__init__('ultrasonic_publisher')

        self.publisher = self.create_publisher(Range, 'ultrasonic', 10)

        self.hcs04 = RPi_HCS04(
                22,
                17,
        )

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def calculate_distance_cm(self, pulse) -> float:
        return round(float(pulse * 17150), 2)

    def timer_callback(self):
        range_msg = Range()
        range_msg.header.stamp = self.get_clock().now().to_msg()
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.range = self.calculate_distance_cm(self.hcs04.measure_pulse_duration())
        self.publisher.publish(range_msg)
        # Log output to console
        # self.get_logger().info(f'Publishing Distance: {range_msg.range} cm')



def main(args=None):
    rclpy.init(args=args)

    ultrasonic_publisher = UltrasonicPublisher()
    
    rclpy.spin(ultrasonic_publisher)

    ultrasonic_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()