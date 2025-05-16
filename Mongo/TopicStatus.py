import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState, Range
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from sensor_msgs.msg import Range
from pymongo import MongoClient
from datetime import datetime
import threading
import json
import os

class TopicMonitor(Node):
    def __init__(self):
        super().__init__('topic_monitor_logger')

        # Connect to MongoDB with your given configuration
        self.client = MongoClient('mongodb://10.170.9.20:27017/')
        self.db = self.client['turtlebot_db']
        self.status_col = self.db['bot_health']

        # State for topics
        self.topic_states = {
            '/odom': False,
            '/battery_state': False,
            '/ultrasonic': False,
        }
        self.last_msg_times = dict.fromkeys(self.topic_states, self.get_clock().now())

        # Lock for thread safety
        self.state_lock = threading.Lock()

        # Subscriptions for relevant topics
        qos = QoSProfile(depth=10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        self.create_subscription(BatteryState, '/battery_state', self.battery_callback, qos)
        self.create_subscription(Range, '/ultrasonic', self.ultrasonic_callback, qos)

        # Timer for logging state to MongoDB
        self.logging_interval_sec = 10  # Log every 10 seconds
        # Subscriptions
        qos = QoSProfile(depth=10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos)
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, qos)
        self.create_subscription(Range, '/ultrasonic', self.ultrasonic_callback, qos)
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, qos)

        # Timer for logging state to MongoDB
        self.create_timer(self.logging_interval_sec, self.log_status)

        # Timer to check for timeouts
        self.create_timer(1.0, self.update_topic_states)

    def odom_callback(self, msg):
        with self.state_lock:
            self.last_msg_times['odom'] = self.get_clock().now()

    def lidar_callback(self, msg):
        with self.state_lock:
            self.last_msg_times['lidar'] = self.get_clock().now()

    def ultrasonic_callback(self, msg):
        with self.state_lock:
            self.last_msg_times['ultrasonic'] = self.get_clock().now()

    def camera_callback(self, msg):
        with self.state_lock:
            self.last_msg_times['camera'] = self.get_clock().now()

    def update_topic_states(self):
        now = self.get_clock().now()
        timeout_sec = 2.0  # consider offline if no msg for 2 seconds
        with self.state_lock:
            for topic, last_time in self.last_msg_times.items():
                elapsed = (now - last_time).nanoseconds / 1e9
                self.topic_states[topic] = elapsed < timeout_sec

    def log_status(self):
        with self.state_lock:
            status_doc = {
                'timestamp': datetime.utcnow(),
                'topics': {
                    topic: 'online' if state else 'offline'
                    for topic, state in self.topic_states.items()
                }
            }
            self.status_col.insert_one(status_doc)
            self.get_logger().info(f"Logged topic status: {status_doc['topics']}")


def main(args=None):
    rclpy.init(args=args)
    node = TopicMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
