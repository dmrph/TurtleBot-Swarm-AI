import os
import sys
import socket
import json
import asyncio
import math
import random
import rclpy
import signal
import time
import threading

from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan, Range

# Extend sys.path for local module access
sys.path.append(os.path.expanduser('~/shared_students/CapstoneFinalRepo/MultiCastHMAC'))
sys.path.append(os.path.expanduser('~/shared_students/CapstoneFinalRepo/Yolo'))
sys.path.append(os.path.expanduser('~/shared_students/CapstoneFinalRepo/Startup'))

# Import CoordinateNavigator from the second file
# We're assuming this file is saved as coordinate_navigator.py in the same directory
sys.path.append(os.path.expanduser('~/shared_students/CapstoneFinalRepo/SwarmAI'))
from coordinate_navigator import CoordinateNavigator

from UnicastComm import UnicastComm
from detect_onnx import YOLODetector
from startup_pos import main as startup_main

# Global flags
is_shutting_down = False
object_detected = False
charm_detected = False
charm_approach_in_progress = False  # Track charm approach
charm_coordinates_received = False  # Track if we've received coordinates from another bot
target_coordinate = None  # Store target coordinate received from other bots
coordinate_navigator = None  # Global reference to coordinate navigator

CONFIG_PATH = os.path.expanduser('~/shared_students/CapstoneFinalRepo/config.json')

# Start camera initialization early in a background thread
def initialize_camera_early():
    """Initialize camera in background thread and return detector"""
    print("Starting early camera initialization in background thread...")
    detector = None
    try:
        detector = YOLODetector(
            model_path='../Yolo/best_gray.onnx', 
            show_display=False
        )
        # Just initialize - don't do anything else yet
        success = detector.initialize_camera()
        print(f"Early camera initialization {'succeeded' if success else 'failed'}")
    except Exception as e:
        print(f"Error in early camera initialization: {e}")
    return detector

# Start initialization in background
background_detector = initialize_camera_early()

class WanderingBot(Node):
    def __init__(self):
        super().__init__('wandering_bot')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Inside your WanderingBot class (__init__)
        self.ultrasonic_sub = self.create_subscription(
            Range,  # Using Range type for ultrasonic sensor
            '/ultrasonic',
            self.ultrasonic_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Load bot ID from config
        self.bot_id = self.load_bot_id()
        self.get_logger().info(f"Bot ID: {self.bot_id}")

        self.current_cmd = Twist()
        self.is_turning_to_obstacle = False
        self.last_move_type = None
        self.avoidance_in_progress = False
        self.current_avoidance_task = None

        self.relative_position = [0.0, 0.0, 0.0]
        self.current_position = [0.0, 0.0, 0.0]  # Track absolute position
        self.start_position = self.load_start_position()

        self.timer = self.create_timer(1.0, self.update_config_position)
        
        # Reduced speeds for safety
        self.safe_linear_speed = 0.10  # Reduced from 0.2
        self.safe_angular_speed = 0.25  # Reduced from 0.3
        self.backup_speed = -0.10  # Safe backing up speed
        
        # Camera detector (will use the background initialized one if available)
        self.detector = None
        
        # Detection stats
        self.last_detection_time = time.time()
        self.detection_count = 0
        self.detection_fps = 0.0
        
        # Communication setup
        self.unicast_comm = None
        self.setup_communication()
        
        # Navigation variables
        self.navigation_in_progress = False
        self.navigation_task = None
        self.charm_found_position = None
        
        # Message queue for received charm locations
        self.charm_location_queue = asyncio.Queue()
        
        # Initialize coordinate navigator for precise navigation
        self.coordinate_navigator = None
        
        self.get_logger().info("WanderingBot initialized and wandering")

    def load_bot_id(self):
        """Load this bot's ID from config"""
        try:
            with open(CONFIG_PATH, "r") as f:
                config = json.load(f)
            bot_id = config.get("bot_id", f"bot_{socket.gethostname()}")
            return bot_id
        except Exception as e:
            self.get_logger().error(f"Error loading bot ID: {e}")
            return f"bot_{socket.gethostname()}"

    def setup_communication(self):
        """Set up UnicastComm for inter-bot communication"""
        try:
            with open(CONFIG_PATH, "r") as f:
                config = json.load(f)
                
            # Get secret key from config - in the important_info section
            secret_key = config.get("important_info", {}).get("secret_key", "default_key")
            if isinstance(secret_key, str):
                secret_key = secret_key.encode('utf-8')
                
            # Get target IPs from the network_config section
            target_ips = config.get("network_config", {}).get("target_ips", [])
            
            self.get_logger().info(f"Found target IPs: {target_ips}")
            
            # Create the UnicastComm object
            self.unicast_comm = UnicastComm(
                secret_key=secret_key,
                bot_id=self.bot_id,
                target_ips=target_ips,
                use_redis=False,  # No Redis for simplicity
                use_ros=True  # Use ROS2 features
            )
            
            # Store the original process method
            original_process = self.unicast_comm.process_received_data
            
            # Override the process method to handle charm locations
            def extended_process(json_data, addr):
                # First do the original processing
                original_process(json_data, addr)
                
                # Then handle our custom message types
                try:
                    payload = json_data.get("payload", {})
                    
                    # Skip our own messages
                    sender_bot_id = payload.get("botId")
                    if sender_bot_id == self.bot_id:
                        return
                    
                    # Check if this is a charm location message
                    if payload.get("message_type") == "charm_location":
                        position = payload.get("position", {})
                        x = position.get("x")
                        y = position.get("y")
                        
                        if x is not None and y is not None:
                            self.get_logger().info(f"📍 Received charm location: {x:.2f}, {y:.2f} from {sender_bot_id}")
                            
                            # Add to the location queue for processing
                            global charm_coordinates_received, target_coordinate
                            charm_coordinates_received = True
                            target_coordinate = [x, y, sender_bot_id]  # Store source bot id along with coordinates
                            
                            # Also add to async queue for the dedicated task
                            asyncio.run_coroutine_threadsafe(
                                self.charm_location_queue.put([x, y, sender_bot_id]), 
                                asyncio.get_event_loop()
                            )
                except Exception as e:
                    self.get_logger().error(f"Error processing custom message: {e}")
            
            # Replace the process method
            self.unicast_comm.process_received_data = extended_process
            
            # Start the receiver thread
            self.unicast_comm.start()
            
            self.get_logger().info(f"Communication initialized for bot {self.bot_id}")
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize communication: {e}")

    def broadcast_charm_location(self, x, y):
        """Broadcast the location of a detected charm"""
        if not self.unicast_comm:
            self.get_logger().error("Cannot broadcast - communication not initialized")
            return False
            
        try:
            # Create payload
            payload = {
                "botId": self.bot_id,
                "message_type": "charm_location",
                "position": {
                    "x": x,
                    "y": y
                },
                "timestamp": time.time()
            }
            
            # Broadcast using UnicastComm
            self.unicast_comm.broadcast_info(payload)
            self.get_logger().info(f"📡 Broadcast charm location: {x:.2f}, {y:.2f}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to broadcast charm location: {e}")
            return False

    def setup_detector(self):
        """Set up detector, using background-initialized one if available"""
        global background_detector
        
        if self.detector is not None:
            return True
            
        try:
            if background_detector is not None:
                self.get_logger().info("Using pre-initialized camera detector")
                self.detector = background_detector
                background_detector = None  # Only use it once
                return True
            else:
                self.get_logger().info("Initializing new camera detector")
                self.detector = YOLODetector(
                    model_path='../Yolo/best_gray.onnx', 
                    show_display=False
                )
                return self.detector.initialize_camera()
        except Exception as e:
            self.get_logger().error(f"Failed to set up detector: {e}")
            return False

    def stop_robot(self):
        """Hard stop the robot immediately"""
        self.get_logger().info("Emergency stop triggered - halting robot")
        stop_cmd = Twist()
        # Publish multiple times to ensure the command is received
        for _ in range(5):
            self.publisher.publish(stop_cmd)
            time.sleep(0.05)  # Small delay between stop commands
        self.current_cmd = stop_cmd

    def cancel_avoidance(self):
        """Cancel current avoidance task if it exists"""
        if self.current_avoidance_task and not self.current_avoidance_task.done():
            self.current_avoidance_task.cancel()

        self.avoidance_in_progress = False
        self.is_turning_to_obstacle = False

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.relative_position = [
            position.x - self.start_position[0],
            position.y - self.start_position[1],
            0.0
        ]
        
        # Update absolute position
        self.current_position = [
            self.start_position[0] + self.relative_position[0],
            self.start_position[1] + self.relative_position[1],
            0.0
        ]
        
        self.update_config_position()

    def ultrasonic_callback(self, msg):
        global object_detected

        # Process the incoming ultrasonic data - Range msg has a single range value, not an array
        if msg.range <= 0.01:
            return  # Invalid reading
            
        min_distance = msg.range  # Single value from the Range message

        if min_distance < 0.2:
            if not object_detected:
                object_detected = True
                self.get_logger().warn(f"ULTRASONIC DETECTED obstacle at {min_distance:.2f}m!")
                
                # If not already avoiding, start backup
                if not self.avoidance_in_progress:
                    self.current_avoidance_task = asyncio.create_task(self.back_up_safely(min_distance))
        else:
            if object_detected:
                object_detected = False  # No longer detected

    def load_start_position(self):
        if not os.path.exists(CONFIG_PATH):
            self.get_logger().error(f"Config file not found: {CONFIG_PATH}")
            return [0.0, 0.0, 0.0]
        try:
            with open(CONFIG_PATH, "r") as f:
                config = json.load(f)
            pos = config.get("current_position", {}).get("position", {})
            return [pos.get("x", 0.0), pos.get("y", 0.0), 0.0]
        except Exception as e:
            self.get_logger().error(f"Error loading config: {e}")
            return [0.0, 0.0, 0.0]

    def update_config_position(self):
        try:
            if os.path.exists(CONFIG_PATH):
                with open(CONFIG_PATH, "r") as f:
                    config = json.load(f)
            else:
                config = {}

            # Only update the "current_position" section
            config.setdefault("current_position", {"position": {"x": 0.0, "y": 0.0}, "last_updated": ""})

            config["current_position"]["position"]["x"] = self.start_position[0] + self.relative_position[0]
            config["current_position"]["position"]["y"] = self.start_position[1] + self.relative_position[1]
            config["current_position"]["last_updated"] = time.strftime("%Y-%m-%d %H:%M:%S")

            with open(CONFIG_PATH, "w") as f:
                json.dump(config, f, indent=4)

        except Exception as e:
            self.get_logger().error(f"Failed to update config: {e}")

    def update_detection_fps(self):
        """Calculate and update detection FPS"""
        current_time = time.time()
        elapsed = current_time - self.last_detection_time
        
        if elapsed >= 5.0:  # Update every 5 seconds
            self.detection_fps = self.detection_count / elapsed
            self.detection_count = 0
            self.last_detection_time = current_time
            self.get_logger().info(f"Charm detection rate: {self.detection_fps:.2f} per second")

    async def move(self, command, duration=1.0, force=False):
        """Execute a movement command for a specified duration with priority handling"""
        global object_detected, is_shutting_down
        
        # Check if we should prevent this movement
        if is_shutting_down and not force:
            self.get_logger().debug(f"Ignoring {command} command - shutdown in progress")
            return
            
        if object_detected and not self.is_turning_to_obstacle and not force:
            self.get_logger().debug(f"Ignoring {command} command - obstacle detected")
            return
        
        # Create movement command
        cmd = Twist()
        if command == 'forward':
            cmd.linear.x = self.safe_linear_speed
        elif command == 'backward':
            cmd.linear.x = self.backup_speed
        elif command == 'left':
            cmd.angular.z = self.safe_angular_speed
        elif command == 'right':
            cmd.angular.z = -self.safe_angular_speed
        elif command == 'stop':
            pass
        else:
            self.get_logger().warn(f'Unknown command: {command}')
            return

        self.get_logger().info(f'Moving {command} for {duration:.1f} seconds')
        self.last_move_type = command
        self.current_cmd = cmd

        movement_task = None
        try:
            # Create a task to control movement
            movement_task = asyncio.create_task(self._execute_movement(cmd, duration))
            
            # Wait for either task completion or interruption
            await movement_task
        except asyncio.CancelledError:
            self.get_logger().info(f"Movement {command} was cancelled")
            if movement_task and not movement_task.done():
                movement_task.cancel()
            
            # Always stop when cancelled
            stop_cmd = Twist()
            self.publisher.publish(stop_cmd)
            self.current_cmd = stop_cmd
        
        # Always stop after movement
        stop_cmd = Twist()
        self.publisher.publish(stop_cmd)
        self.current_cmd = stop_cmd

    async def _execute_movement(self, cmd, duration):
        """Execute the actual movement with small sleep intervals for responsiveness"""
        global is_shutting_down, object_detected
        
        start_time = time.time()
        end_time = start_time + duration
        
        # Short movement intervals to allow for quick interruption
        while time.time() < end_time:
            # Check for shutdown or obstacle
            if is_shutting_down:
                self.get_logger().info("Movement interrupted due to shutdown")
                break
                
            # Allow interruption if a new obstacle is detected during movement
            if object_detected and not self.is_turning_to_obstacle and cmd.linear.x > 0:
                self.get_logger().warn(f"Movement interrupted by obstacle detection")
                break
                
            self.publisher.publish(cmd)
            await asyncio.sleep(0.05)  # More responsive control with shorter sleeps

    async def wander(self):
        """Wander around, avoiding obstacles"""
        global is_shutting_down, charm_detected, charm_approach_in_progress, charm_coordinates_received
        
        await asyncio.sleep(1)
        while not is_shutting_down:
            # Check for obstacles, charm detection, charm approach, or navigation in progress
            if (object_detected or self.is_turning_to_obstacle or 
                charm_detected or charm_approach_in_progress or 
                charm_coordinates_received or self.navigation_in_progress):
                self.get_logger().debug("Pausing wandering due to other operations")
                await asyncio.sleep(0.5)
                continue

            weights = [0.7, 0.12, 0.12, 0.06]
            move_type = random.choices(
                ['forward', 'left', 'right', 'forward_arc'],
                weights=weights
            )[0]

            if (move_type in ['left', 'right']) and move_type == self.last_move_type:
                move_type = 'forward'

            if move_type == 'forward':
                await self.move('forward', random.uniform(2.0, 4.0))
            elif move_type == 'left':
                await self.move('left', random.uniform(0.4, 0.8))
            elif move_type == 'right':
                await self.move('right', random.uniform(0.4, 0.8))
            elif move_type == 'forward_arc':
                cmd = Twist()
                cmd.linear.x = self.safe_linear_speed
                cmd.angular.z = random.uniform(0.15, -0.15)
                self.current_cmd = cmd
                duration = random.uniform(1.5, 3.0)
                
                start_time = time.time()
                end_time = start_time + duration
                
                while (time.time() < end_time and not is_shutting_down 
                       and not object_detected and not charm_detected 
                       and not charm_approach_in_progress and not charm_coordinates_received):
                    self.publisher.publish(cmd)
                    await asyncio.sleep(0.1)
                
                stop_cmd = Twist()
                self.publisher.publish(stop_cmd)
                self.current_cmd = stop_cmd
            
            await asyncio.sleep(0.3)
    
    async def approach_charm(self):
        """
        Improved charm detection and approach that works with object avoidance and broadcasts when found
        """
        global object_detected, is_shutting_down, charm_detected, charm_approach_in_progress
        
        self.get_logger().info("Starting charm detection behavior")
        
        # Set up detector (try to use the pre-initialized one)
        if not self.setup_detector():
            self.get_logger().error("Failed to set up camera - will retry periodically")
        
        # Main detection loop - runs continuously in background
        try:
            while not is_shutting_down:
                # Skip detection if obstacle avoidance is active
                if object_detected or self.avoidance_in_progress:
                    self.get_logger().debug("Pausing charm detection due to obstacle")
                    await asyncio.sleep(0.5)
                    continue
                
                # Skip detection if we're navigating to a charm location
                if charm_coordinates_received or self.navigation_in_progress:
                    self.get_logger().debug("Pausing charm detection - navigating to known charm")
                    await asyncio.sleep(0.5)
                    continue
                
                # Try to ensure detector is ready
                if self.detector is None or not self.detector.camera_initialized:
                    self.get_logger().warn("Camera not ready - attempting to initialize")
                    self.setup_detector()
                    await asyncio.sleep(1.0)
                    continue
                
                # Run detection
                try:
                    # Track stats
                    self.detection_count += 1
                    self.update_detection_fps()
                    
                    # Run a single detection cycle with timeout protection
                    start_time = time.time()
                    charm_found = await self.detector.detect_charm_center_and_move(
                        continuous=False,  # Process just one frame
                        max_frames=1,
                        show_fps=True
                    )
                    
                    detection_time = time.time() - start_time
                    if detection_time > 0.5:
                        self.get_logger().warn(f"Detection took {detection_time:.2f} seconds - may be blocking")
                    
                    # Update flags based on detection result
                    charm_detected = charm_found
                    
                    if charm_found:
                        self.get_logger().info("Charm detected - approach in progress")
                        charm_approach_in_progress = True
                        
                        # After successful detection, do several consecutive detections
                        # to maintain approach as long as the charm is visible
                        consecutive_detections = 0
                        consecutive_misses = 0
                        
                        # Continue approaching as long as we can see the charm
                        while not is_shutting_down and consecutive_misses < 5:
                            # Check for obstacles - they take priority
                            if object_detected or self.avoidance_in_progress:
                                self.get_logger().warn("Obstacle detected during charm approach - pausing")
                                await asyncio.sleep(0.5)
                                continue
                            
                            # Run another detection cycle
                            still_visible = await self.detector.detect_charm_center_and_move(
                                continuous=False,
                                max_frames=1,
                                show_fps=True
                            )
                            
                            # Update tracking based on detection
                            if still_visible:
                                consecutive_detections += 1
                                consecutive_misses = 0
                                self.get_logger().info(f"Charm still visible (detection #{consecutive_detections})")
                                
                                # After a few successful detections, consider charm found and broadcast position
                                if consecutive_detections >= 3:
                                    # Save charm position
                                    self.charm_found_position = self.current_position.copy()
                                    self.get_logger().info(f"🎯 FOUND CHARM at position: {self.charm_found_position[0]:.2f}, {self.charm_found_position[1]:.2f}")
                                    
                                    # Broadcast to other bots
                                    success = self.broadcast_charm_location(
                                        self.charm_found_position[0], 
                                        self.charm_found_position[1]
                                    )
                                    self.get_logger().info(f"📡 Charm position broadcast success: {success}")
                                    
                                    # Wait for other bots to converge to this position
                                    self.get_logger().info("Waiting at charm position...")
                                    await asyncio.sleep(60.0)  # Wait for a minute
                                    break  # Exit the detection loop
                            else:
                                consecutive_misses += 1
                                self.get_logger().info(f"Lost charm (miss #{consecutive_misses})")
                            
                            # Brief yield to allow other tasks to run
                            await asyncio.sleep(0.1)
                        
                        # We've lost the charm or completed approach
                        self.get_logger().info("Charm approach complete or charm lost from view")
                        charm_approach_in_progress = False
                        charm_detected = False
                    else:
                        # No charm detected this cycle
                        charm_approach_in_progress = False
                        
                except Exception as e:
                    self.get_logger().error(f"Error in charm detection cycle: {e}")
                    # Don't disable flags on error - just continue
                    await asyncio.sleep(0.5)
                
                # Always yield control before next detection
                # This is crucial to prevent blocking and allow object avoidance to run
                await asyncio.sleep(0.2)
                
        except asyncio.CancelledError:
            self.get_logger().info("Charm detection task cancelled")
            charm_approach_in_progress = False
            charm_detected = False
            raise
        except Exception as e:
            self.get_logger().error(f"Error in approach_charm: {e}")
            charm_approach_in_progress = False
            charm_detected = False
        finally:
            # Clean up detector on shutdown
            if self.detector:
                self.detector.release_camera()

    async def navigate_to_target(self, target_position):
        """
        Legacy navigation to a target position received from another bot
        This method is kept for backward compatibility but will be bypassed
        in favor of the CoordinateNavigator for precise navigation
        """
        global charm_coordinates_received
        
        self.navigation_in_progress = True
        
        try:
            self.get_logger().info(f"Starting navigation to target: {target_position[0]:.2f}, {target_position[1]:.2f}")
            
            # Calculate distance to target
            dx = target_position[0] - self.current_position[0]
            dy = target_position[1] - self.current_position[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            while distance > 0.5 and not is_shutting_down:  # 0.5m threshold to consider "arrived"
                # Check for obstacles - they take priority
                if object_detected or self.avoidance_in_progress:
                    self.get_logger().info("Obstacle during navigation - pausing")
                    await asyncio.sleep(0.5)
                    continue
                
                # Recalculate distance and direction
                dx = target_position[0] - self.current_position[0]
                dy = target_position[1] - self.current_position[1]
                distance = math.sqrt(dx*dx + dy*dy)
                target_angle = math.degrees(math.atan2(dy, dx))
                
                # Log navigation info
                self.get_logger().info(f"Navigation: distance={distance:.2f}m, angle={target_angle:.2f}°")
                
                # Simple navigation - move forward with periodic corrections
                # This could be improved with more sophisticated path planning
                if distance > 2.0:
                    # Longer forward movements when far away
                    await self.move('forward', duration=2.0, force=True)
                    # Brief turn to help estimate orientation
                    await self.move('left', duration=0.3, force=True)
                else:
                    # Shorter, more careful movements when closer
                    await self.move('forward', duration=0.8, force=True)
                    await self.move('left', duration=0.2, force=True)
                
                await asyncio.sleep(0.2)  # Brief pause between movements
            
            self.get_logger().info(f"Arrived at target location!")
            
            # Wait a bit at the target location
            await asyncio.sleep(10.0)
            
        except asyncio.CancelledError:
            self.get_logger().info("Navigation cancelled")
            raise
        except Exception as e:
            self.get_logger().error(f"Error in navigation: {e}")
        finally:
            self.navigation_in_progress = False
            charm_coordinates_received = False

    async def back_up_safely(self, distance):
        """Back up safely when obstacle is detected very close"""
        self.get_logger().warn(f"BACKING UP: obstacle at {distance:.2f}m")
        
        # Always back up - this is a safety-critical maneuver
        backup_cmd = Twist()
        backup_cmd.linear.x = self.backup_speed
        
        # Back up for longer if obstacle is very close
        backup_duration = 1.0 if distance < 0.2 else 1.0
        
        start_time = time.time()
        end_time = start_time + backup_duration
        
        while time.time() < end_time and not is_shutting_down:
            self.publisher.publish(backup_cmd)
            await asyncio.sleep(0.1)
        
        # Stop after backing up
        stop_cmd = Twist()
        self.publisher.publish(stop_cmd)
        self.current_cmd = stop_cmd
        
        # Give a moment to settle
        await asyncio.sleep(0.2)

    async def avoid_object(self, angle, distance):
        """Full obstacle avoidance routine with backing up capability"""
        global object_detected
        
        # Don't start a new avoidance if outside our detection range
        if not (angle < 130 or angle > 230):
            object_detected = False
            return
            
        self.is_turning_to_obstacle = True
        self.avoidance_in_progress = True
        
        self.get_logger().warn(f"OBJECT AVOIDANCE: angle={angle:.2f}°, distance={distance:.2f}m")
        
        try:
            # Emergency stop first
            self.stop_robot()
            
            # Very close obstacle - back up first (safety critical)
            if distance < 0.25:
                await self.back_up_safely(distance)
                
                # Check if we're shutting down
                if is_shutting_down:
                    self.cleanup_avoidance()
                    return
            
            # Determine turn direction - if obstacle is on left side, turn right
            turn_direction = 'right' if angle < 180 else 'left'
            
            # Make a substantial turn away from obstacle
            self.get_logger().info(f"Turning {turn_direction} to avoid obstacle")
            await self.move(turn_direction, duration=1.8, force=True)
            
            if is_shutting_down:
                self.cleanup_avoidance()
                return
                
            # Move forward slightly 
            self.get_logger().info("Moving forward to clear obstacle")
            await self.move('forward', duration=1.2, force=True)
            
            if is_shutting_down:
                self.cleanup_avoidance()
                return
                
            # Make another turn to ensure clearance
            self.get_logger().info(f"Additional {turn_direction} turn for clearance")
            await self.move(turn_direction, duration=0.6, force=True)
            
            if is_shutting_down:
                self.cleanup_avoidance()
                return
                
            # Short move forward to clear the area
            await self.move('forward', duration=0.8, force=True)
            
            self.get_logger().info("Obstacle avoidance complete")
            
        except asyncio.CancelledError:
            self.get_logger().warn("Avoidance routine was cancelled")
            raise
        except Exception as e:
            self.get_logger().error(f"Error in avoid_object: {e}")
        finally:
            self.cleanup_avoidance()
    
    def cleanup_avoidance(self):
        """Clean up after avoidance routine completes or is interrupted"""
        self.is_turning_to_obstacle = False
        self.avoidance_in_progress = False
        global object_detected
        object_detected = False
        
    async def check_received_coordinates(self):
        """Check for received charm coordinates and navigate to them if available"""
        global charm_coordinates_received, target_coordinate, coordinate_navigator
        
        while not is_shutting_down:
            try:
                # Check if we're already at a charm or navigating
                if self.charm_found_position is not None or self.navigation_in_progress:
                    await asyncio.sleep(1.0)
                    continue
                
                # Check if we have received coordinates from broadcast
                if charm_coordinates_received and target_coordinate:
                    self.get_logger().info(f"Processing received coordinates: {target_coordinate}")
                    
                    # Use CoordinateNavigator for precise navigation if available
                    if coordinate_navigator is not None:
                        x, y = target_coordinate[0], target_coordinate[1]
                        source_bot_id = target_coordinate[2] if len(target_coordinate) > 2 else None
                        
                        self.get_logger().info(f"Using CoordinateNavigator to navigate to ({x}, {y}) from bot {source_bot_id}")
                        self.navigation_in_progress = True
                        
                        # Navigate using the coordinate navigator (which handles the coordinates transformation)
                        success = await coordinate_navigator.navigate_to_coordinate(x, y, source_bot_id)
                        
                        if success:
                            self.get_logger().info("Successfully navigated to charm coordinates using CoordinateNavigator")
                        else:
                            self.get_logger().warn("Failed to navigate to charm coordinates")
                        
                        # Reset flags
                        self.navigation_in_progress = False
                        charm_coordinates_received = False
                        target_coordinate = None
                    else:
                        # Fallback to basic navigation if CoordinateNavigator isn't available
                        self.get_logger().warn("CoordinateNavigator not available - using basic navigation")
                        self.navigation_task = asyncio.create_task(
                            self.navigate_to_target(target_coordinate[:2])  # Just use the x,y coordinates
                        )
                    
                # Try to get coordinates from queue with timeout
                try:
                    coordinates = await asyncio.wait_for(
                        self.charm_location_queue.get(), 
                        timeout=1.0
                    )
                    
                    # Only navigate if we're not already doing so
                    if not self.navigation_in_progress and not self.charm_found_position:
                        self.get_logger().info(f"Got coordinates from queue: {coordinates}")
                        
                        # Store the coordinates and source bot ID
                        target_coordinate = coordinates
                        charm_coordinates_received = True
                        
                        # Check if we have the coordinate navigator available
                        if coordinate_navigator is not None:
                            x, y = coordinates[0], coordinates[1]
                            source_bot_id = coordinates[2] if len(coordinates) > 2 else None
                            
                            self.get_logger().info(f"Using CoordinateNavigator to navigate to ({x}, {y}) from bot {source_bot_id}")
                            self.navigation_in_progress = True
                            
                            # Navigate using the coordinate navigator
                            success = await coordinate_navigator.navigate_to_coordinate(x, y, source_bot_id)
                            
                            if success:
                                self.get_logger().info("Successfully navigated to charm coordinates using CoordinateNavigator")
                            else:
                                self.get_logger().warn("Failed to navigate to charm coordinates")
                            
                            # Reset flags
                            self.navigation_in_progress = False
                            charm_coordinates_received = False
                            target_coordinate = None
                        else:
                            # Fallback to basic navigation
                            self.get_logger().warn("CoordinateNavigator not available - using basic navigation")
                            self.navigation_task = asyncio.create_task(
                                self.navigate_to_target([coordinates[0], coordinates[1]])
                            )
                        
                except asyncio.TimeoutError:
                    # This is normal - no messages in queue
                    pass
                    
            except asyncio.CancelledError:
                self.get_logger().info("Coordinate checker cancelled")
                raise
            except Exception as e:
                self.get_logger().error(f"Error checking coordinates: {e}")
            
            await asyncio.sleep(0.5)


class LidarAgent(Node):
    def __init__(self, wander_bot):
        super().__init__('lidar_agent')
        self.wander_bot = wander_bot
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        self.get_logger().info("LidarAgent initialized.")
        self.last_detection_time = time.time()
        
        # Safety thresholds
        self.critical_distance = 0.20  # Distance that requires immediate backup
        self.normal_distance = 0.35    # Distance for normal obstacle avoidance
        
        # Rate limit for new avoidance actions
        self.min_detection_interval = 0.5  # seconds

    def scan_callback(self, msg):
        global object_detected, is_shutting_down
        
        if is_shutting_down:
            return
        
        current_time = time.time()
        # Rate limit how often we process new obstacles
        if current_time - self.last_detection_time < self.min_detection_interval:
            return

        try:
            valid_ranges = [(i, r) for i, r in enumerate(msg.ranges)
                            if r != float('inf') and not math.isnan(r) and r > 0.05]
            if not valid_ranges:
                return

            min_index, min_distance = min(valid_ranges, key=lambda x: x[1])
            angle_increment = msg.angle_increment
            angle_min = msg.angle_min
            obstacle_angle = math.degrees((angle_min + min_index * angle_increment))

            # Only consider obstacles in front half of robot
            if not (0 <= obstacle_angle <= 90 or 270 <= obstacle_angle <= 360):
                return
                
            # Show obstacle info periodically
            if current_time - self.last_detection_time > 2.0:
                self.get_logger().info(f"Nearest object: distance={min_distance:.2f}m, angle={obstacle_angle:.2f}°")
                
            # Critical case: Very close obstacle detected
            if min_distance < self.critical_distance:
                object_detected = True
                self.last_detection_time = current_time
                
                self.get_logger().warn(f"CRITICAL: Object at {min_distance:.2f}m, angle: {obstacle_angle:.2f}°")
                
                # Cancel any existing avoidance routine
                if self.wander_bot.avoidance_in_progress:
                    self.get_logger().warn("Interrupting current avoidance for critical obstacle")
                    self.wander_bot.cancel_avoidance()
                    self.wander_bot.stop_robot()
                    
                # Create new avoidance task with immediate backup
                self.wander_bot.current_avoidance_task = asyncio.create_task(
                    self.wander_bot.avoid_object(obstacle_angle, min_distance)
                )
                
            # Normal case: Obstacle detected at safe distance
            elif min_distance < self.normal_distance:
                object_detected = True
                self.last_detection_time = current_time
                
                # Don't interrupt existing avoidance unless it's been a while
                if not self.wander_bot.avoidance_in_progress:
                    self.get_logger().warn(f"Object detected at {min_distance:.2f}m, angle: {obstacle_angle:.2f}°")
                    self.wander_bot.current_avoidance_task = asyncio.create_task(
                        self.wander_bot.avoid_object(obstacle_angle, min_distance)
                    )
                
        except Exception as e:
            self.get_logger().error(f"Error in scan_callback: {e}")


def handle_shutdown(wanderbot, signum, frame):
    global is_shutting_down, charm_approach_in_progress, charm_coordinates_received
    print("\nShutdown signal received! Stopping the robot...")
    is_shutting_down = True
    charm_approach_in_progress = False
    charm_coordinates_received = False
    
    if wanderbot:
        wanderbot.stop_robot()
        # Clean up communication
        if hasattr(wanderbot, 'unicast_comm'):
            wanderbot.unicast_comm.stop()
            
    print("Robot stopped. Exiting...")
    sys.exit(0)

async def spin_node(node):
    global is_shutting_down
    if is_shutting_down:
        return
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        while rclpy.ok() and not is_shutting_down:
            executor.spin_once(timeout_sec=0.1)
            await asyncio.sleep(0.01)
    finally:
        executor.remove_node(node)

async def main():
    global is_shutting_down, charm_approach_in_progress, charm_coordinates_received
    
    # Initialize flags
    charm_coordinates_received = False
    
    # Initialize ROS
    rclpy.init()
    
    try:
        with open(CONFIG_PATH, "r") as config_file:
            config = json.load(config_file)
    except FileNotFoundError:
        print("Config file not found, proceeding with default behavior")
        config = {}

    wander_bot = WanderingBot()
    lidar_agent = LidarAgent(wander_bot)
    
    # Set up signal handlers for clean shutdown
    signal.signal(signal.SIGINT, lambda signum, frame: handle_shutdown(wander_bot, signum, frame))
    signal.signal(signal.SIGTERM, lambda signum, frame: handle_shutdown(wander_bot, signum, frame))

    # Start node spinning
    wander_spin_task = asyncio.create_task(spin_node(wander_bot))
    lidar_task = asyncio.create_task(spin_node(lidar_agent))
    
    # Start tasks with priorities:
    # 1. Check for received coordinates (highest priority) - do this before wandering
    coordinate_check_task = asyncio.create_task(wander_bot.check_received_coordinates())
    
    # 2. Charm detection (second priority)
    charm_task = asyncio.create_task(wander_bot.approach_charm())
    
    # 3. Wandering (lowest priority) - will only happen when other tasks aren't active
    wander_task = asyncio.create_task(wander_bot.wander())

    try:
        await asyncio.gather(
            wander_spin_task, 
            lidar_task, 
            wander_task, 
            charm_task,
            coordinate_check_task
        )
    except asyncio.CancelledError:
        pass
    except KeyboardInterrupt:
        print("Keyboard interrupt detected")
        is_shutting_down = True
        charm_approach_in_progress = False
        charm_coordinates_received = False
        wander_bot.stop_robot()
    finally:
        is_shutting_down = True
        charm_approach_in_progress = False
        charm_coordinates_received = False
        
        # Clean up
        wander_bot.stop_robot()
        if hasattr(wander_bot, 'unicast_comm'):
            wander_bot.unicast_comm.stop()
            
        wander_bot.destroy_node()
        lidar_agent.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Shutting down (top level)...")