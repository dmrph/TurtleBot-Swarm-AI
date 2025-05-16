#!/usr/bin/env python3
import sys
import os
sys.path.append('..')
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
import cv2
import numpy as np
from apriltag import Detector, DetectorOptions
import asyncio
import time
import signal
import logging
import json
from scipy.spatial.transform import Rotation as R

# Add path to UnicastHMAC directory (updated from MultiCastHMAC)
sys.path.append(os.path.expanduser('~/shared_students/CapstoneFinalRepo/MultiCastHMAC'))
from UnicastComm import UnicastComm

# ---------------------------
# Configure logging
# ---------------------------
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

# ---------------------------
# Global variables
# ---------------------------
is_shutting_down = False
CONFIG_PATH = os.path.expanduser('~/shared_students/CapstoneFinalRepo/config.json')

# ---------------------------
# Configuration class
# ---------------------------
class AprilTagConfig:
    CAMERA_INDEX = 0
    CAMERA_WIDTH = 320
    CAMERA_HEIGHT = 240

    # Base station tags ONLY
    BASE_STATION_TAGS = {
        "north": 33,
        "east": 32,
        "south": 34,
        "west": 31
    }

    TAG_SIZE = 0.0521  # meters
    CAMERA_MATRIX = np.array([
    [672.9,   0.0, 960.0],  # fx,  0,  cx
    [  0.0, 378.8, 540.0],  #  0, fy,  cy
    [  0.0,   0.0,   1.0]   #  0,  0,   1
    ])
    SPIN_SPEED = 0.4  # rad/s


# ---------------------------
# Utilities
# ---------------------------
def euler_from_quaternion(x, y, z, w):
    """Convert quaternion to Euler angles."""
    r = R.from_quat([x, y, z, w])
    return r.as_euler('xyz', degrees=False)


def load_target_ips():
    """Load target IPs from config file or use defaults"""
    try:
        with open(CONFIG_PATH, 'r') as config_file:
            config = json.load(config_file)
        
        # Check if network_config section exists
        if 'network_config' in config and 'target_ips' in config['network_config']:
            return config['network_config']['target_ips']
    except Exception as e:
        logging.error(f"Error loading target IPs from config: {e}")
    
    # Default IP range for TurtleBots - you should customize these based on your network
    return [
        "192.168.1.10",  # Example IP for Bot 1
        "192.168.1.11",  # Example IP for Bot 2
        "192.168.1.12"   # Example IP for Bot 3
    ]


# ---------------------------
# Main Detector Node
# ---------------------------
class TurtleBotPositionDetector(Node):
    def __init__(self):
        super().__init__('turtlebot_position_detector')
        self.get_logger().info("TurtleBot Position Detector node initialized")

        # Initialize publisher for robot movement
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Setup AprilTag detector
        options = DetectorOptions(
            families="tag25h9",
            nthreads=2,
            quad_decimate=2.0,
            refine_edges=True
        )
        self.tag_detector = Detector(options)
        self.config = AprilTagConfig()

        # QoS for Odometry subscription
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Subscribe to Odometry for rotation tracking
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )

        # Variables to track orientation
        self.starting_yaw = None
        self.current_yaw = 0.0
        self.spin_speed = self.config.SPIN_SPEED
        self.done_spinning = False
        self.position_calculated = False
        self.bot_id = None
        self.secret_key = None
        self.unicast_comm = None
        self.payload = None
        
        # Status tracking variables
        self.status = "initializing"  # initializing, detecting, broadcasting, waiting_done, completed
        self.this_bot_done = False  # Flag to track if this bot has completed its task
        self.bots_done_status = {}  # Track which bots have sent "done" messages
        self.done_broadcast_interval = 5.0  # How often to broadcast "done" status (seconds)
        self.last_done_broadcast = 0  # Last time we broadcast "done" status
        
        # Define which bots we need to track
        self.all_bot_ids = ["1", "2", "3"]
        self.other_bot_ids = []  # Will be populated once we know our bot_id

        # Load bot_id and secret_key from config
        try:
            with open(CONFIG_PATH, 'r') as f:
                config = json.load(f)
                important_info = config.get("important_info", {})
                self.bot_id = important_info.get("bot_id", 1)
                self.secret_key = important_info.get("secret_key", "")
                
                # Convert bot_id to string for consistent comparison
                self.bot_id = str(self.bot_id)
                
                # Populate other_bot_ids
                self.other_bot_ids = [bid for bid in self.all_bot_ids if bid != self.bot_id]
                
                # Initialize done status tracking for all bots
                for bot_id in self.all_bot_ids:
                    self.bots_done_status[bot_id] = False
                
                # Mark our own bot as not done yet
                self.bots_done_status[self.bot_id] = False
                
                # Get target IPs
                target_ips = load_target_ips()
                
                if self.secret_key:
                    # Initialize UnicastComm instead of MulticastComm
                    self.unicast_comm = UnicastComm(
                        secret_key=self.secret_key.encode(),
                        bot_id=self.bot_id,
                        target_ips=target_ips,
                        use_redis=False  # Don't use Redis for this application
                    )
                    # Start the background receiver thread
                    self.unicast_comm.start()
                    self.get_logger().info(f"UnicastComm initialized for bot {self.bot_id} with targets: {target_ips}")
                else:
                    self.get_logger().error("No secret key found in config")
        except Exception as e:
            self.get_logger().error(f"Error loading config: {e}")

        # Initialize camera
        self._init_camera()

    def odom_callback(self, msg):
        """Handle incoming odometry data."""
        orientation_q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        self.current_yaw = yaw
        if self.starting_yaw is None:
            self.starting_yaw = yaw
            self.get_logger().info(f"Starting yaw recorded: {np.degrees(self.starting_yaw):.2f} degrees")

    def _init_camera(self):
        """Initialize camera capture."""
        try:
            self.cap = cv2.VideoCapture(self.config.CAMERA_INDEX)
            if not self.cap.isOpened():
                for idx in range(4):
                    if idx == self.config.CAMERA_INDEX:
                        continue
                    self.cap = cv2.VideoCapture(idx)
                    if self.cap.isOpened():
                        self.config.CAMERA_INDEX = idx
                        break
            if not self.cap.isOpened():
                self.get_logger().error("Failed to open camera")
                raise RuntimeError("Camera initialization failed")
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.CAMERA_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.CAMERA_HEIGHT)
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            time.sleep(1.0)
            self.get_logger().info(f"Camera initialized (index: {self.config.CAMERA_INDEX})")
        except Exception as e:
            self.get_logger().error(f"Camera initialization error: {e}")
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
            raise

    def broadcast_done_message(self):
        """Broadcast a message indicating this bot is done with its task"""
        if not self.unicast_comm:
            self.get_logger().error("Cannot broadcast done message - UnicastComm not initialized")
            return False
            
        try:
            done_payload = {
                "botId": self.bot_id,
                "message_type": "done_status",
                "status": "done",
                "timestamp": time.time()
            }
            
            self.unicast_comm.broadcast_info(done_payload)
            self.last_done_broadcast = time.time()
            self.get_logger().info(f"🏁 Broadcast DONE message for bot {self.bot_id}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to broadcast done message: {e}")
            return False

    def get_base_direction_for_tag(self, tag_id):
        """Map tag ID to cardinal direction."""
        for direction, base_tag_id in self.config.BASE_STATION_TAGS.items():
            if tag_id == base_tag_id:
                return direction
        return None

    def stop_robot(self):
        """Stop the robot's motion."""
        stop_cmd = Twist()
        for _ in range(5):
            self.publisher.publish(stop_cmd)
        self.get_logger().info("Robot stopped")

    async def detect_base_station(self):
        """Detect base station and compute robot's position relative to it."""
        self.status = "detecting"
        self.get_logger().info("Starting base station detection")
        detection_start_time = time.time()
        detection_timeout = 30  # seconds

        while not is_shutting_down:
            if time.time() - detection_start_time > detection_timeout:
                self.get_logger().error("Detection timed out")
                self.stop_robot()
                return False

            ret, frame = self.cap.read()
            if not ret or frame is None:
                await asyncio.sleep(0.1)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            detections = self.tag_detector.detect(gray)

            if detections:
                for detection in detections:
                    # ✅ Process only base station tags (31–34)
                    if detection.tag_id not in self.config.BASE_STATION_TAGS.values():
                        self.get_logger().info(f"Ignored tag ID {detection.tag_id}, not a base station tag")
                        continue

                    direction = self.get_base_direction_for_tag(detection.tag_id) or "unknown"
                    distance, angle = self.calculate_distance_and_angle(detection)

                    # Log position
                    self.get_logger().info(
                        f"Base station detected (Tag ID: {detection.tag_id}, Direction: {direction})"
                    )
                    self.get_logger().info(
                        f"Distance to base station: {distance:.2f} m, Angle: {np.degrees(angle):.2f}°"
                    )

                    # ✅ Safe config write with properly calculated global coordinates
                    try:
                        # Load existing config, or start new if missing or corrupted
                        try:
                            with open(CONFIG_PATH, 'r') as f:
                                config = json.load(f)
                        except (FileNotFoundError, json.JSONDecodeError):
                            config = {}

                        pos_payload = {
                            "botId": self.bot_id,
                            "base_station_tag_id": detection.tag_id,
                            "direction": direction,
                            "position": {
                                "x": 0.0,
                                "y": 0.0, 
                            },
                            "distance": distance,
                            "timestamp": time.time()
                        }

                        # Set current_position
                        config["current_position"] = {
                            "base_station_tag_id": detection.tag_id,
                            "direction": direction,
                            "position": {
                                "x": 0.0,
                                "y": 0.0, 
                            },
                            "distance": distance,
                            "timestamp": time.time()
                        }

                        self.payload = pos_payload

                        # Broadcast position immediately using UnicastComm
                        if self.unicast_comm and self.payload:
                            try:
                                self.unicast_comm.broadcast_info(self.payload)
                                self.get_logger().info(f"Initial position broadcast successful for bot {self.bot_id}")
                            except Exception as e:
                                self.get_logger().error(f"Failed to broadcast initial position: {e}")

                        with open(CONFIG_PATH, 'w') as f:
                            json.dump(config, f, indent=4)

                        self.get_logger().info(f"✅ Config file updated: {CONFIG_PATH}")

                    except Exception as e:
                        self.get_logger().error(f"❌ Failed to update config: {e}")

                    self.stop_robot()
                    self.done_spinning = True
                    self.position_calculated = True
                    return True

            await asyncio.sleep(0.05)

        return False

    async def spin_back_to_start(self):
        """After detection, return to starting yaw orientation."""
        if self.starting_yaw is None:
            self.get_logger().warning("Starting yaw not yet received, waiting...")
            # Wait for yaw to be received
            wait_start = time.time()
            while self.starting_yaw is None and not is_shutting_down:
                await asyncio.sleep(0.05)
                # Break after 5 seconds if still not received
                if time.time() - wait_start > 5:
                    self.get_logger().error("Timeout waiting for starting yaw. Cannot spin back.")
                    return

        spin_cmd = Twist()
        self.get_logger().info(f"Spinning back to starting orientation: {np.degrees(self.starting_yaw):.2f} degrees")
        self.get_logger().info(f"Current orientation: {np.degrees(self.current_yaw):.2f} degrees")

        while not is_shutting_down:
            yaw_error = self.normalize_angle(self.starting_yaw - self.current_yaw)
            self.get_logger().info(f"Yaw error: {np.degrees(yaw_error):.2f} degrees")
            
            if abs(yaw_error) < 0.01:
                self.get_logger().info("Target orientation reached")
                break
                
            spin_cmd.angular.z = self.spin_speed if yaw_error > 0 else -self.spin_speed
            self.publisher.publish(spin_cmd)
            await asyncio.sleep(0.05)

        self.stop_robot()
        self.get_logger().info("Returned to starting orientation.")

    @staticmethod
    def normalize_angle(angle):
        """Normalize angle between -pi and pi."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def calculate_distance_and_angle(self, detection):
        """Calculate distance and angle to detected tag."""
        fx = float(self.config.CAMERA_MATRIX[0, 0])
        cx = float(self.config.CAMERA_MATRIX[0, 2])
        center_x = float(detection.center[0])

        corners = detection.corners
        side_lengths = [
            float(np.linalg.norm(corners[i] - corners[(i + 1) % 4])) for i in range(4)
        ]
        avg_side_px = float(sum(side_lengths) / 4)
        distance = float((self.config.TAG_SIZE * fx) / avg_side_px)
        distance = distance * 0.3163044934120271
        angle = float(np.arctan2(center_x - cx, fx))

        return distance, angle

    async def publish_cmd(self, cmd, duration=0.2):
        """Helper to publish a command for a duration."""
        publish_count = int(duration / 0.1)
        for _ in range(publish_count):
            self.publisher.publish(cmd)
            await asyncio.sleep(0.1)

    async def control_loop(self):
        """Continuous spin control loop."""
        self.get_logger().info("Starting spin control loop")
        while not is_shutting_down and not self.done_spinning:
            spin_cmd = Twist()
            spin_cmd.angular.z = self.spin_speed
            await self.publish_cmd(spin_cmd, duration=0.1)
            await asyncio.sleep(0.1)

    def are_all_bots_done(self):
        """Check if all bots (including this one) have sent done messages"""
        for bot_id, status in self.bots_done_status.items():
            if bot_id in self.other_bot_ids and not status:
                return False
        return True

    async def wait_for_position_and_done_messages(self, timeout=300):
        """Wait for position data and done messages from other bots."""
        self.status = "waiting_done"
        self.get_logger().info("Waiting for position data and done messages from other TurtleBots...")
        
        # First, clear any existing data for other bots in the config
        # to ensure we're getting fresh data
        try:
            with open(CONFIG_PATH, 'r') as f:
                config = json.load(f)
            
            # Clear existing data
            modified = False
            for bot_id in self.other_bot_ids:
                other_bot_key = f"other_bot{bot_id}"
                if other_bot_key in config:
                    config[other_bot_key] = []
                    modified = True
            
            if modified:
                with open(CONFIG_PATH, 'w') as f:
                    json.dump(config, f, indent=4)
                self.get_logger().info("Cleared existing bot data from config")
            
            self.get_logger().info(f"Waiting for data from bots: {self.other_bot_ids}")
            
        except Exception as e:
            self.get_logger().error(f"Error clearing config: {e}")
            return False
            
        # Track position detection per bot
        position_detected = {bot_id: False for bot_id in self.other_bot_ids}
        all_positions_detected = False
        
        # Track broadcast times
        broadcast_interval = 15  # seconds
        last_broadcast = time.time()
        start_time = time.time()
        
        # Mark this bot as done once all positions are detected
        self.this_bot_done = False
        self.bots_done_status[self.bot_id] = False
        
        while not is_shutting_down:
            # Check if we've timed out
            if time.time() - start_time > timeout:
                self.get_logger().warning(f"⚠️ Timeout after {timeout} seconds waiting for other TurtleBots")
                break
                
            # Check if it's time to broadcast our position again
            current_time = time.time()
            if self.unicast_comm and self.payload and (current_time - last_broadcast >= broadcast_interval):
                try:
                    # Update timestamp in payload before broadcasting
                    self.payload["timestamp"] = current_time
                    self.unicast_comm.broadcast_info(self.payload)
                    self.get_logger().info(f"Position rebroadcast for bot {self.bot_id}")
                    last_broadcast = current_time
                except Exception as e:
                    self.get_logger().error(f"Failed to broadcast: {e}")
            
            # If we're done detecting all bot positions but haven't marked ourselves as done yet
            if all_positions_detected and not self.this_bot_done:
                self.this_bot_done = True
                self.bots_done_status[self.bot_id] = True
                self.broadcast_done_message()
                self.get_logger().info("🏁 Marked this bot as DONE and broadcast status")
            
            # If we're done, keep broadcasting done message periodically
            if self.this_bot_done and (current_time - self.last_done_broadcast >= self.done_broadcast_interval):
                self.broadcast_done_message()
            
            # Check if we've received position data from the other bots
            if not all_positions_detected:
                try:
                    with open(CONFIG_PATH, 'r') as f:
                        config = json.load(f)
                    
                    # Check each bot
                    for bot_id in self.other_bot_ids:
                        if position_detected[bot_id]:
                            continue
                            
                        # Check if we have data for this bot
                        other_bot_key = f"other_bot{bot_id}"
                        if other_bot_key in config and config[other_bot_key]:
                            self.get_logger().info(f"✅ Detected position data from bot {bot_id}")
                            position_detected[bot_id] = True
                            
                    # Check if we have detected all positions
                    if all(position_detected.values()):
                        self.get_logger().info("✅ Received position data from all required TurtleBots!")
                        all_positions_detected = True
                except Exception as e:
                    self.get_logger().error(f"Error checking config: {e}")
            
            # Check if all bots are done
            if self.are_all_bots_done():
                self.get_logger().info("🎉 All bots have reported DONE status!")
                self.status = "completed"
                return True
                
            # Sleep briefly to avoid CPU spinning
            await asyncio.sleep(0.5)
            
        return False

    # Extend the UnicastComm with a callback for handling bot position and done messages
    def setup_message_callbacks(self):
        """Set up callbacks for processing position data and done messages"""
        if not self.unicast_comm:
            self.get_logger().error("UnicastComm not initialized")
            return
            
        # We'll extend the process_received_data method of UnicastComm
        original_process_method = self.unicast_comm.process_received_data
        
        def extended_process_data(json_data, addr):
            # First call the original method
            original_process_method(json_data, addr)
            
            # Then add our custom processing
            try:
                received_payload = json_data.get("payload")
                received_hmac = json_data.get("hmac")
                
                if received_payload and received_hmac and self.unicast_comm.verify_hmac(received_payload, received_hmac):
                    # Get the bot ID
                    bot_id = None
                    if "botId" in received_payload:
                        bot_id = str(received_payload.get("botId"))  # Ensure it's a string
                    elif "bot_id" in received_payload:
                        bot_id = str(received_payload.get("bot_id"))
                    
                    if bot_id is None:
                        self.get_logger().warning(f"Missing bot_id in payload: {received_payload}")
                        return
                        
                    # Skip our own broadcasts
                    if bot_id == self.bot_id:
                        self.get_logger().debug(f"Ignoring our own broadcast (bot_id: {bot_id})")
                        return
                    
                    # Check message type
                    message_type = received_payload.get("message_type", "")
                    
                    # Handle "done" messages
                    if message_type == "done_status" and received_payload.get("status") == "done":
                        self.get_logger().info(f"🏁 Received DONE message from bot {bot_id}")
                        self.bots_done_status[bot_id] = True
                        return
                    
                    # Handle position data messages
                    if "position" in received_payload:
                        self.get_logger().info(f"Processing position data from bot {bot_id}")
                        
                        # Get timestamp, default to current time if not present
                        timestamp = received_payload.get("timestamp", time.time())
                        
                        # Update the config
                        try:
                            with open(CONFIG_PATH, 'r') as f:
                                config = json.load(f)
                                
                            # Update the appropriate other_bot entry
                            other_bot_key = f"other_bot{bot_id}"
                            config[other_bot_key] = [{
                                "bot_id": bot_id,
                                "direction": received_payload.get("direction", "unknown"),
                                "position": received_payload.get("position", {"x": 0.0, "y": 0.0}),
                                "distance": received_payload.get("distance", 0.0),
                                "timestamp": timestamp
                            }]
                            
                            with open(CONFIG_PATH, 'w') as f:
                                json.dump(config, f, indent=4)
                                
                            self.get_logger().info(f"✅ Updated position for bot {bot_id}")
                        except Exception as e:
                            self.get_logger().error(f"Failed to update config: {e}")
            except Exception as e:
                self.get_logger().error(f"Error in extended process data: {e}")
                
        # Replace the process_received_data method with our extended version
        self.unicast_comm.process_received_data = extended_process_data
        self.get_logger().info("Message callbacks set up")

    def shutdown(self):
        """Shutdown and clean up."""
        self.get_logger().info("Shutting down...")
        self.stop_robot()
        
        # Stop the UnicastComm receiver thread
        if self.unicast_comm:
            self.unicast_comm.stop()
            
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()


def handle_shutdown(signum, frame):
    global is_shutting_down
    print("\nShutdown signal received!")
    is_shutting_down = True


async def spin_node(node):
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not is_shutting_down:
            executor.spin_once(timeout_sec=0.1)
            await asyncio.sleep(0.01)
    finally:
        executor.remove_node(node)


async def main():
    rclpy.init()
    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    try:
        detector = TurtleBotPositionDetector()
        
        # Set up the message callbacks before we start
        detector.setup_message_callbacks()
        
        # Create tasks for detection and control
        detection_task = asyncio.create_task(detector.detect_base_station())
        control_task = asyncio.create_task(detector.control_loop())
        spin_task = asyncio.create_task(spin_node(detector))

        # Wait for the detection to complete
        await detection_task

        if detector.position_calculated:
            # Return to starting orientation
            await detector.spin_back_to_start()
            detector.get_logger().info("TurtleBot position calculation complete.")
            
            # Now wait for other bots' positions and done messages
            success = await detector.wait_for_position_and_done_messages(timeout=300)
            
            if success:
                detector.get_logger().info("✅ All TurtleBots have completed their tasks and reported done")
            else:
                detector.get_logger().warning("⚠️ Timed out waiting for all TurtleBots to complete")
        else:
            detector.get_logger().error("No base station tag detected.")

    except Exception as e:
        logging.error(f"Error in main loop: {e}")
    finally:
        if 'detector' in locals():
            detector.shutdown()
            detector.destroy_node()
        rclpy.shutdown()
        logging.info("Shutdown complete")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Keyboard interrupt detected")
    except Exception as e:
        print(f"Error: {e}")