import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range # Used to receive ultrasonic sensor messages.
from geometry_msgs.msg import Twist  # Import the Twist message
import redis # Utilizes the redis database
import numpy as np # Handles numeric operations, like converting arrays to binary for Redis.
import json # Reads configuration
import time # Creates delays between commands

class Redis_Ops:
    def __init__(self):
        with open("config.json", "r") as config_file:
            config = json.load(config_file) # Assign the configuration file to this value
        
        self.bot_id = config['bot_id'] # Assign bot ID from value in config file
        self.redis_client = redis.Redis(host='localhost', port=6379, decode_responses=False) # Connects to local instance
    
    def log_redis_data(self, aprilTagCoords, aprilTagOrient, botCoords, botOrient):
        aprilTagLocation = [aprilTagCoords[0], aprilTagCoords[1], aprilTagCoords[2], aprilTagOrient] # Log coords and orientation into a list for the april tag
        botLocation = [botCoords[0], botCoords[1], botCoords[2], botOrient] # Log the same types of the values for the TurtleBot

        vector_data = np.array(aprilTagLocation + botLocation, dtype=np.float32)

        # Store vector data in Redis
        self.redis_client.set(f"bot:{self.bot_id}", vector_data.tobytes())
        print(f"Added bot: {self.bot_id} with vector: {vector_data}")

    # Erase redisDB data
    def clear_redis_data(self):
        self.redis_client.flushall()

    # Retrieve redisDB data (getter)
    def retrieve_redis_data(self):
        return np.frombuffer(self.redis_client.get(f"bot:{self.bot_id}"), dtype=np.float32)
    

class UltrasonicSubscriber(Node):

    def __init__(self):
        super().__init__('ultrasonic_subscriber')
        self.subscription = self.create_subscription(
            Range,
            'ultrasonic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Create a publisher for controlling the robot's motion
        self.velocity_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',  # The topic name may vary depending on your robot setup
            10)

        # Initialize Redis_Ops instance
        self.redis_ops = Redis_Ops()

    def listener_callback(self, msg):
        """Callback function that runs when a new message is received."""
        distance = msg.range
        self.get_logger().info(f'Received: "{distance}"')

        #Work in Progress
        # Dummy values for logging (Replace with actual sensor data)
        aprilTagCoords = [1.0, 2.0, 3.0]  # Example coordinates
        aprilTagOrient = 0.5  # Example orientation
        botCoords = [4.0, 5.0, 6.0]  # Example bot coordinates
        botOrient = 1.0  # Example bot orientation

        # If an object is detected within a 10-inch radius of the camera
        if distance <= 10:
            self.get_logger().info('Backing up')

            cmd = Twist()
            cmd.linear.x = -0.1  # Move backward with a speed of 0.1 m/s

            # Log data to Redis
            self.redis_ops.log_redis_data(aprilTagCoords, aprilTagOrient, botCoords, botOrient)

            for i in range(10): # Repeat 10 times, will run for a total of 2 seconds
                self.velocity_publisher.publish(cmd)
                time.sleep(0.2) # every .2 seconds
                self.get_logger().info(f"Backing up: Published command {i+1}")

            #Once the loop is complete, the robot will stop
            cmd.linear.x = 0.0
            self.velocity_publisher.publish(cmd)


def main(args=None):
    # Ros 2 initalization
    print("Starting main function")
    rclpy.init(args=args)
    print("rclpy initialized")
    
    try:
        ultrasonic_subscriber = UltrasonicSubscriber() # Create the ultrasonic subscripter node
        print("Node created successfully")
        rclpy.spin(ultrasonic_subscriber) # Listen to and react to sensor data
    except Exception as e:
        print(f"Error occurred: {e}") # Error handling
    
    ultrasonic_subscriber.destroy_node() #Shut down ROS library
    rclpy.shutdown()


if __name__ == '__main__':
    main()
