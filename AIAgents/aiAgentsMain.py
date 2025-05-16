import sys
import os
sys.path.append('..')
import asyncio
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import threading
import json
sys.path.extend([
    os.path.expanduser('~/shared_students/CapstoneFinalRepo/MultiCastHMAC'),
    os.path.expanduser('~/shared_students/CapstoneFinalRepo/Yolo'),
    os.path.expanduser('~/shared_students/CapstoneFinalRepo/Startup')
])
from Receiver import Receiver
from Broadcast import Broadcaster
# from detect_onnx import YOLODetector
# Message queue for inter-agent communication
message_queue = asyncio.Queue()

class SonicAgent(Node):
    """Asynchronous Sonic Sensor Agent"""
    def __init__(self, threshold=10):
        super().__init__('ultrasonic_subscriber')
        self.subscription = self.create_subscription(
            Range, # Message type
            'ultrasonic', # Topic name
            self.detect_object, # Callback function
            10) # Subscription queue size
        self.threshold = threshold  # Distance threshold in cm
        
    def detect_object(self, msg):
        distance = msg.range
        if distance < self.threshold:
            print(f"SonicAgent: Object detected at {distance:.2f} cm")

class MoveBotAgent(Node):
    """ROS2 Movement Agent for controlling the robot's movement."""

    def __init__(self):
        super().__init__('move_bot_agent')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("MoveBotAgent initialized.")

    async def move(self, command, duration=2.0):
        """Executes a movement command for a given duration."""
        cmd = Twist()
        if command == 'forward':
            cmd.linear.x = -0.2
        elif command == 'backward':
            cmd.linear.x = 0.2
        elif command == 'left':
            cmd.angular.z = 0.5
        elif command == 'right':
            cmd.angular.z = -0.5
        elif command == 'stop':
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            return

        self.get_logger().info(f'Executing {command} for {duration} seconds')

        for _ in range(int(duration / 0.2)):
            self.publisher.publish(cmd)
            await asyncio.sleep(0.2)

        stop_cmd = Twist()
        self.publisher.publish(stop_cmd)
        self.get_logger().info(f'{command} completed')

        await message_queue.put(f"MoveBotAgent: Executed {command}")


class LidarAgent(Node):
    """ROS2 Lidar Agent listening to /scan topic for obstacle detection."""

    def __init__(self):
        super().__init__('lidar_agent')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        self.get_logger().info("LidarAgent initialized.")

    def scan_callback(self, msg):
        valid_ranges = [r for r in msg.ranges if r != float('inf') and not math.isnan(r)]
        min_distance = min(valid_ranges) if valid_ranges else float('inf')

        if min_distance < 0.5:
            self.get_logger().warn(f"OBJECT DETECTED at {min_distance:.2f} meters!")
            message_queue.put(f"LidarAgent: Object detected at {min_distance:.2f} meters")
        else:
            self.get_logger().info('No objects detected within threshold')

class ObjectLocationSubscriber(Node):  # Define a ROS2 node to subscribe to object location updates.
    """Node to subscribe to object locations and process them."""
    
    def __init__(self):  # Constructor to initialize the subscriber node.
        super().__init__('object_location_subscriber')  # Initialize the node with the name 'object_location_subscriber'.
        
        self.subscription = self.create_subscription(  # Create a subscription to a ROS2 topic.
            String,  # The message type is String.
            '/object_locations',  # The topic name to subscribe to.
            self.location_callback,  # The callback function to process received messages.
            10  # The queue size for the subscription.
        )
        
        self.get_logger().info("ObjectLocationSubscriber initialized.")  # Log a message indicating the node is initialized.

    def location_callback(self, msg):  # Callback function to handle received messages.
        data = json.loads(msg.data)  # Parse the received message data (JSON string) into a Python dictionary.
        bot_id = data["bot_id"]  # Extract the bot ID from the message.
        sensors = data["sensors"]  # Extract the sensor data from the message.
        self.get_logger().info(f"Received object location from bot {bot_id}: {sensors}")  # Log the received data.
        # Process the object location data (e.g., update navigation or swarm behavior).

class BroadcastAgent:
    """Agent responsible for broadcasting information periodically."""
    def __init__(self, secret_key, target_ips, bot_id, interval=15):
        self.broadcaster = Broadcast(secret_key, target_ips, bot_id)
        self.interval = interval
        self.running = True
        self.thread = threading.Thread(target=self.broadcast_info_continuously, daemon=True)

    def start(self):
        self.thread.start()

    def stop(self):
        self.running = False
        self.thread.join()

    def broadcast_info_continuously(self):
        print("BroadcastAgent: Starting broadcast...")
        while self.running:
            try:
                timestamp = time.time()
                location = {"x": 10.5, "y": 20.3, "orientation": 180}
                sensors = {
                    "sonar": [1.2, 2.3, 3.4],
                    "lidar": [0.5, 1.0, 1.5, 2.0],
                    "camera": "base64EncodedStringHere"
                }
                self.broadcaster.broadcast_info(timestamp, location, sensors)
                print("BroadcastAgent: Broadcast message sent")
                time.sleep(self.interval)
            except Exception as e:
                print(f"BroadcastAgent: Error in broadcast thread: {e}")
                time.sleep(2)

# class CameraAgent():
#     def __init__(self):
#         detector = YOLODetector()
#         detector.run()

class MainController:
    """Main Controller listening for notifications from agents."""

    async def listen_for_notifications(self):
        while True:
            message = await message_queue.get()
            print(f"MainController: Received -> {message}")

async def main():
    rclpy.init()
    
    with open("~/shared_students/CapstoneFinalRepo/config.json", "r") as config_file:
        config = json.load(config_file)

    # Initialize agents
    sonic_agent = SonicAgent(30)
    lidar_agent = LidarAgent()
    move_bot_agent = MoveBotAgent()
    broadcast_agent = BroadcastAgent(config['secret_key'].encode(), config['target_ips'], config['bot_id'])
    controller = MainController()
    # camera_agent = CameraAgent() # Added camera agent   
    

    broadcast_agent.start()

    await move_bot_agent.move('forward', 0.4)
    await move_bot_agent.move('backward', 0.4)
    await move_bot_agent.move('left', 0.4)
    await move_bot_agent.move('right', 0.4)
    await move_bot_agent.move('stop')

    move_bot_agent.destroy_node()

    executor = MultiThreadedExecutor()
    executor.add_node(lidar_agent)
    executor.add_node(sonic_agent)

    executor_task = asyncio.to_thread(executor.spin)  # Run executor in a separate thread
    listen_task = asyncio.create_task(controller.listen_for_notifications())

    await asyncio.gather(executor_task, listen_task)

    rclpy.shutdown()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Shutting down...")
