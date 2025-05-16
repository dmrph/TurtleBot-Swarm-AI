#!/usr/bin/env python3
import sys
import os
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
from sensor_msgs.msg import LaserScan

# Extend sys.path for local module access
sys.path.append(os.path.expanduser('~/shared_students/CapstoneFinalRepo/MultiCastHMAC'))
sys.path.append(os.path.expanduser('~/shared_students/CapstoneFinalRepo/Yolo'))
sys.path.append(os.path.expanduser('~/shared_students/CapstoneFinalRepo/Startup'))

from Receiver import Receiver as receive
from Broadcast import Broadcaster
from detect_onnx import YOLODetector
from startup_pos import main as startup_main


# Global flags
is_shutting_down = False
object_detected = False
charm_detected = False
charm_approach_in_progress = False  # New flag to track charm approach

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
        self.get_logger().info("WanderingBot initialized and wandering")
        self.current_cmd = Twist()
        self.is_turning_to_obstacle = False
        self.last_move_type = None
        self.avoidance_in_progress = False
        self.current_avoidance_task = None

        self.relative_position = [0.0, 0.0, 0.0]
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
            self.get_logger().warn("Cancelling current avoidance task")
        self.avoidance_in_progress = False
        self.is_turning_to_obstacle = False

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.relative_position = [
            position.x - self.start_position[0],
            position.y - self.start_position[1],
            0.0
        ]
        self.update_config_position()

    def ultrasonic_callback(self, msg):
        global object_detected

        # Process the incoming ultrasonic data - Range msg has a single range value, not an array
        if msg.range <= 0.01:  # Ignore invalid readings
            return
            
        min_distance = msg.range  # Single value from the Range message

        if min_distance < 0.2:  # Threshold in meters
            if not object_detected:
                object_detected = True
                self.get_logger().warn(f"ULTRASONIC DETECTED obstacle at {min_distance:.2f}m!")
                
                # If not already avoiding, trigger avoidance (no longer backing up)
                if not self.avoidance_in_progress:
                    # Use a neutral angle (0) since ultrasonic doesn't give direction
                    # The lidar will provide better angle information
                    self.current_avoidance_task = asyncio.create_task(self.avoid_object(0, min_distance))
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

            # ✨ Safely update only modified fields
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
        """Modified to pause wandering when charm approach is in progress"""
        await asyncio.sleep(1)
        while not is_shutting_down:
            # Check for obstacles, charm detection, or charm approach in progress
            if object_detected or self.is_turning_to_obstacle or charm_detected or charm_approach_in_progress:
                self.get_logger().debug("Pausing wandering due to obstacle/charm operations")
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
                
                while time.time() < end_time and not is_shutting_down and not object_detected and not charm_detected and not charm_approach_in_progress:
                    self.publisher.publish(cmd)
                    await asyncio.sleep(0.1)
                
                stop_cmd = Twist()
                self.publisher.publish(stop_cmd)
                self.current_cmd = stop_cmd
            
            await asyncio.sleep(0.3)
    
    async def approach_charm(self):
        """
        Improved charm detection and approach that works with object avoidance
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

    async def avoid_object(self, angle, distance):
        """Turn 45 degrees left or right depending on which side of 0 the object is located"""
        global object_detected
        
        self.is_turning_to_obstacle = True
        self.avoidance_in_progress = True
        
        self.get_logger().warn(f"OBJECT AVOIDANCE: angle={angle:.2f}°, distance={distance:.2f}m")
        
        try:
            # Determine turn direction based on angle
            # Normalize angle to 0-360 range if needed
            angle = angle % 360
            
            # First check if the object is in the ignore zone (behind the robot)
            if 110 <= angle <= 250:
                self.get_logger().info(f"Ignoring object at angle {angle:.2f}° - it's behind the robot")
                # Clean up and return early
                self.cleanup_avoidance()
                return
                
            # Simple rule: if object is at angle < 180, turn right; otherwise turn left
            # This means object on left side -> turn right, object on right side -> turn left
            if angle < 180:
                # Object is on left half - turn right to avoid
                turn_direction = 'right'
            else:
                # Object is on right half - turn left to avoid
                turn_direction = 'left'
            
            # Log the decision
            self.get_logger().info(f"Turning {turn_direction} to avoid obstacle at angle {angle:.2f}°")
            
            # Execute a 45-degree turn in the chosen direction
            # 45 degrees = π/4 radians
            # Time = angle / angular_speed
            turn_duration = math.pi/4 / self.safe_angular_speed
            
            # Start the turn
            turn_cmd = Twist()
            turn_cmd.angular.z = self.safe_angular_speed if turn_direction == 'left' else -self.safe_angular_speed
            self.current_cmd = turn_cmd
            
            # Monitor and continue turning while checking for more obstacles
            start_time = time.time()
            end_time = start_time + turn_duration
            
            while time.time() < end_time and not is_shutting_down:
                # Continue the turn
                self.publisher.publish(turn_cmd)
                
                # Short sleep to allow for sensor updates
                await asyncio.sleep(0.05)
            
            # After turn is complete, move forward
            self.get_logger().info("Continuing forward after obstacle avoidance turn")
            
            # Create forward command
            forward_cmd = Twist()
            forward_cmd.linear.x = self.safe_linear_speed
            self.current_cmd = forward_cmd
            
            # Move forward for a short time while continuously checking for obstacles
            forward_duration = 1.0  # 1 second of forward movement
            start_time = time.time()
            end_time = start_time + forward_duration
            
            # Continue to move forward while checking for obstacles
            while time.time() < end_time and not is_shutting_down:
                # If another obstacle is detected during forward movement,
                # exit early to handle the new obstacle
                if object_detected and not self.is_turning_to_obstacle:
                    self.get_logger().warn("New obstacle detected during avoidance - ending current avoidance")
                    break
                
                # Continue forward movement
                self.publisher.publish(forward_cmd)
                await asyncio.sleep(0.05)
            
            # Always stop at the end
            stop_cmd = Twist()
            self.publisher.publish(stop_cmd)
            self.current_cmd = stop_cmd
            
            self.get_logger().info("Obstacle avoidance cycle complete")
            
        except asyncio.CancelledError:
            self.get_logger().warn("Avoidance routine was cancelled")
            stop_cmd = Twist()
            self.publisher.publish(stop_cmd)
            self.current_cmd = stop_cmd
            raise
        except Exception as e:
            self.get_logger().error(f"Error in avoid_object: {e}")
        finally:
            # Only cleanup if we didn't detect a new obstacle
            if not object_detected:
                self.cleanup_avoidance()
            else:
                # Reset turning flag but keep avoidance_in_progress true
                # This allows a new avoidance cycle to start immediately
                self.is_turning_to_obstacle = False

    def load_bot_id(self):
        """Load this bot's ID from config"""
        try:
            with open(CONFIG_PATH, "r") as f:
                config = json.load(f)
            bot_id = str(config.get("important_info", {}).get("bot_id", "unknown"))
            self.get_logger().info(f"Loaded bot ID: {bot_id}")
            return bot_id
        except Exception as e:
            self.get_logger().error(f"Error loading bot ID: {e}")
            return "unknown"

    def setup_communication(self):
        """Set up UnicastComm for inter-bot communication"""
        try:
            with open(CONFIG_PATH, "r") as f:
                config = json.load(f)
            
            # Get configuration values
            bot_id = str(config.get("important_info", {}).get("bot_id", "unknown"))
            secret_key = config.get("important_info", {}).get("secret_key", "default_key")
            target_ips = config.get("network_config", {}).get("target_ips", [])
            
            # Initialize UnicastComm
            self.unicast_comm = UnicastComm(
                secret_key=secret_key.encode(),
                bot_id=bot_id,
                target_ips=target_ips,
                use_redis=False,
                use_ros=True
            )
            
            # Start the receiver thread
            self.unicast_comm.start()
            
            # Add message processing callback for charm convergence
            self.setup_message_callbacks()
            
            self.get_logger().info(f"Communication initialized for bot {bot_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize communication: {e}")

    def setup_navigator(self):
        """Set up the coordinate navigator for charm convergence"""
        try:
            self.navigator = CoordinateNavigator()
            self.get_logger().info("CoordinateNavigator initialized")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize navigator: {e}")

    def setup_message_callbacks(self):
        """Set up callbacks for processing charm location and convergence messages"""
        if not self.unicast_comm:
            self.get_logger().error("UnicastComm not initialized")
            return
            
        # Store the original process method for chaining
        original_process_method = self.unicast_comm.process_received_data
        
        def extended_process_data(json_data, addr):
            # First call the original method
            original_process_method(json_data, addr)
            
            # Then add our custom processing for charm messages
            try:
                global charm_detected, charm_convergence_active, all_bots_converged
                
                received_payload = json_data.get("payload")
                received_hmac = json_data.get("hmac")
                
                if received_payload and received_hmac and self.unicast_comm.verify_hmac(received_payload, received_hmac):
                    # Get the bot ID
                    bot_id = None
                    if "botId" in received_payload:
                        bot_id = str(received_payload.get("botId"))
                    elif "bot_id" in received_payload:
                        bot_id = str(received_payload.get("bot_id"))
                    
                    if bot_id is None:
                        self.get_logger().warning(f"Missing bot_id in payload: {received_payload}")
                        return
                        
                    # Skip our own broadcasts
                    if bot_id == self.bot_id:
                        return
                    
                    # Check message type
                    message_type = received_payload.get("message_type", "")
                    
                    # Handle charm location message
                    if message_type == "charm_location":
                        self.get_logger().info(f"📍 Received CHARM LOCATION from bot {bot_id}")
                        
                        # Extract charm location
                        charm_x = received_payload.get("position", {}).get("x")
                        charm_y = received_payload.get("position", {}).get("y")
                        
                        if charm_x is not None and charm_y is not None:
                            self.get_logger().info(f"Charm location: ({charm_x}, {charm_y}) from bot {bot_id}")
                            
                            # Activate convergence if not already active
                            if not charm_convergence_active:
                                charm_convergence_active = True
                                charm_detected = True  # Set detection flag to pause wandering
                                
                                # Start convergence to the charm location
                                asyncio.create_task(self.converge_to_charm(charm_x, charm_y, bot_id))
                    
                    # Handle convergence acknowledgment message
                    elif message_type == "charm_converged":
                        self.get_logger().info(f"🏁 Received CONVERGENCE acknowledgment from bot {bot_id}")
                        
                        # Update convergence status
                        self.convergence_status[bot_id] = True
                        
                        # Check if all bots have converged
                        if self.check_all_bots_converged():
                            self.get_logger().info("🎯 ALL BOTS HAVE CONVERGED at charm location!")
                            all_bots_converged = True
                            
            except Exception as e:
                self.get_logger().error(f"Error in message processing: {e}")
                
        # Replace the process method with our extended version
        self.unicast_comm.process_received_data = extended_process_data
        self.get_logger().info("Charm message callbacks set up")

    def broadcast_charm_location(self, x, y):
        """Broadcast charm location to other bots"""
        if not self.unicast_comm:
            self.get_logger().error("Cannot broadcast charm location - UnicastComm not initialized")
            return False
            
        try:
            charm_payload = {
                "botId": self.bot_id,
                "message_type": "charm_location",
                "position": {"x": x, "y": y},
                "timestamp": time.time()
            }
            
            self.unicast_comm.broadcast_info(charm_payload)
            self.get_logger().info(f"📢 Broadcast CHARM LOCATION ({x}, {y}) from bot {self.bot_id}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to broadcast charm location: {e}")
            return False

    def broadcast_convergence_status(self):
        """Broadcast convergence status to other bots"""
        if not self.unicast_comm:
            self.get_logger().error("Cannot broadcast convergence - UnicastComm not initialized")
            return False
            
        try:
            convergence_payload = {
                "botId": self.bot_id,
                "message_type": "charm_converged",
                "status": "converged",
                "timestamp": time.time()
            }
            
            self.unicast_comm.broadcast_info(convergence_payload)
            self.last_convergence_broadcast = time.time()
            self.get_logger().info(f"🏁 Broadcast CONVERGENCE status for bot {self.bot_id}")
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to broadcast convergence: {e}")
            return False

    def check_all_bots_converged(self):
        """Check if all bots have converged to the charm location"""
        try:
            with open(CONFIG_PATH, "r") as f:
                config = json.load(f)
            
            # Count how many other bots we expect
            expected_bots = []
            
            # Check for other_bot1
            if "other_bot1" in config and len(config["other_bot1"]) > 0:
                other_bot = config["other_bot1"][0].get("bot_id")
                if other_bot:
                    expected_bots.append(str(other_bot))
            
            # Check for other_bot2
            if "other_bot2" in config and len(config["other_bot2"]) > 0:
                other_bot = config["other_bot2"][0].get("bot_id")
                if other_bot:
                    expected_bots.append(str(other_bot))
            
            # Make sure we've heard from all expected bots
            for bot_id in expected_bots:
                if bot_id not in self.convergence_status or not self.convergence_status[bot_id]:
                    return False
            
            # All bots have converged
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error checking all bots converged: {e}")
            return False

    async def converge_to_charm(self, x, y, source_bot_id):
        """Navigate to a charm location broadcast by another bot"""
        self.get_logger().info(f"🔄 Starting convergence to charm at ({x}, {y}) from bot {source_bot_id}")
        
        global charm_convergence_active
        
        try:
            # Use the CoordinateNavigator to navigate to the charm location
            if self.navigator:
                # First mark ourselves as not converged
                self.convergence_status[self.bot_id] = False
                
                # Navigate to the charm location
                success = await self.navigator.navigate_to_coordinate(x, y, source_bot_id)
                
                if success:
                    self.get_logger().info(f"✅ Successfully navigated to charm location ({x}, {y})")
                    
                    # Mark ourselves as converged
                    self.convergence_status[self.bot_id] = True
                    
                    # Broadcast convergence status
                    self.broadcast_convergence_status()
                    
                    # Continue broadcasting convergence status periodically until all bots converge
                    while charm_convergence_active and not all_bots_converged:
                        # Check if it's time to broadcast again
                        current_time = time.time()
                        if current_time - self.last_convergence_broadcast >= self.convergence_broadcast_interval:
                            self.broadcast_convergence_status()
                        
                        # Check if all bots have converged
                        if self.check_all_bots_converged():
                            self.get_logger().info("🎯 ALL BOTS HAVE CONVERGED at charm location!")
                            break
                            
                        await asyncio.sleep(1.0)
                    
                    self.get_logger().info("Convergence complete")
                else:
                    self.get_logger().error(f"❌ Failed to navigate to charm location")
            else:
                self.get_logger().error("Cannot converge - navigator not initialized")
                
        except Exception as e:
            self.get_logger().error(f"Error during convergence: {e}")
        finally:
            # Reset flags if this bot initiated the convergence
            if source_bot_id == self.bot_id:
                charm_convergence_active = False

    async def wait_for_all_bots(self):
        """Wait for all bots to converge at the charm location"""
        global charm_convergence_active, all_bots_converged
        
        self.get_logger().info("Waiting for all bots to converge at charm location...")
        
        # Broadcast status periodically
        while charm_convergence_active and not all_bots_converged and not is_shutting_down:
            # Check if it's time to broadcast again
            current_time = time.time()
            if current_time - self.last_convergence_broadcast >= self.convergence_broadcast_interval:
                self.broadcast_convergence_status()
            
            # Check if all bots have converged
            if self.check_all_bots_converged():
                self.get_logger().info("🎯 ALL BOTS HAVE CONVERGED at charm location!")
                all_bots_converged = True
                break
                
            await asyncio.sleep(1.0)
        
        self.get_logger().info("Convergence wait complete")
        return all_bots_converged
    
    def cleanup_avoidance(self):
        """Clean up after avoidance routine completes or is interrupted"""
        self.is_turning_to_obstacle = False
        self.avoidance_in_progress = False
        global object_detected
        object_detected = False
        
    # Removed wall detection function


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
        self.critical_distance = 0.20  # Distance that requires immediate response
        self.normal_distance = 0.35    # Distance for normal obstacle avoidance
        
        # Rate limit for new avoidance actions - set to check every 4 seconds
        self.min_detection_interval = 4.0  # Only check for new objects every 4 seconds
        
        # Create a timer for continuous environment monitoring
        self.continuous_monitor_timer = self.create_timer(4.0, self.continuous_monitor)
        
        # Store the last scan message for continuous monitoring
        self.last_scan_msg = None

    def scan_callback(self, msg):
        global object_detected, is_shutting_down
        
        if is_shutting_down:
            return
            
        # Store the last scan message for continuous monitoring
        self.last_scan_msg = msg
        
        current_time = time.time()
        # Rate limit how often we process new obstacles
        if current_time - self.last_detection_time < self.min_detection_interval:
            return

        try:
            # Process scan data
            self.process_scan_data(msg, current_time)
                
        except Exception as e:
            self.get_logger().error(f"Error in scan_callback: {e}")
            
    def process_scan_data(self, msg, current_time):
        """Process scan data and initiate avoidance if needed"""
        global object_detected
        
        valid_ranges = [(i, r) for i, r in enumerate(msg.ranges)
                        if r != float('inf') and not math.isnan(r) and r > 0.05]
        if not valid_ranges:
            return

        min_index, min_distance = min(valid_ranges, key=lambda x: x[1])
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        obstacle_angle = math.degrees((angle_min + min_index * angle_increment))

        # Only consider obstacles in front portion of robot
        # Avoid objects between 0-110 degrees and 250-360 degrees
        # Explicitly ignore objects between 110-250 degrees (behind the robot)
        if 110 <= obstacle_angle <= 250:
            self.get_logger().debug(f"Ignoring object at angle {obstacle_angle:.2f}° - it's behind the robot")
            return
            
        # Show obstacle info periodically
        if current_time - self.last_detection_time > 2.0:
            self.get_logger().info(f"Nearest object: distance={min_distance:.2f}m, angle={obstacle_angle:.2f}°")
            
        # Critical case: Very close obstacle detected - trigger immediate avoidance
        if min_distance < self.critical_distance:
            object_detected = True
            self.last_detection_time = current_time
            
            self.get_logger().warn(f"CRITICAL: Object at {min_distance:.2f}m, angle: {obstacle_angle:.2f}°")
            
            # Cancel any existing avoidance routine
            if self.wander_bot.avoidance_in_progress:
                self.get_logger().warn("Interrupting current avoidance for critical obstacle")
                self.wander_bot.cancel_avoidance()
                
            # Create new avoidance task - now calls avoid_object directly without backing up
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
                
    def continuous_monitor(self):
        """Continuously monitor the environment for obstacles, especially walls"""
        global object_detected, is_shutting_down
        
        if is_shutting_down or self.last_scan_msg is None:
            return
            
        # Check if we're already avoiding something
        if self.wander_bot.avoidance_in_progress:
            return
            
        # Reprocess the most recent scan data to constantly check for obstacles
        current_time = time.time()
        
        # Only process if we haven't recently triggered an avoidance
        if current_time - self.last_detection_time > self.min_detection_interval:
            self.process_scan_data(self.last_scan_msg, current_time)


def handle_shutdown(wanderbot, signum, frame):
    global is_shutting_down, charm_approach_in_progress
    print("\nShutdown signal received! Stopping the robot...")
    is_shutting_down = True
    charm_approach_in_progress = False  # Make sure to clear this flag
    if wanderbot:
        wanderbot.stop_robot()
    print("Robot stopped. Exiting...")
    sys.exit(0)

async def spin_node(node):
    global is_shutting_down  # Make sure to access the global variable
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
    global is_shutting_down, charm_approach_in_progress  # Ensure global variables are available
    
    # await startup_main()  # Uncomment if needed

    rclpy.init()
    try:
        with open(CONFIG_PATH, "r") as config_file:
            config = json.load(config_file)
    except FileNotFoundError:
        print("Config file not found, proceeding with default behavior")
        config = {}

    wander_bot = WanderingBot()
    lidar_agent = LidarAgent(wander_bot)
    signal.signal(signal.SIGINT, lambda signum, frame: handle_shutdown(wander_bot, signum, frame))
    signal.signal(signal.SIGTERM, lambda signum, frame: handle_shutdown(wander_bot, signum, frame))

    wander_spin_task = asyncio.create_task(spin_node(wander_bot))
    lidar_task = asyncio.create_task(spin_node(lidar_agent))
    
    # Start charm task first - it has priority over wandering
    charm_task = asyncio.create_task(wander_bot.approach_charm())
    
    # Wandering will only
    #!/usr/bin/env python3
import sys
import os
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
from sensor_msgs.msg import LaserScan

# Extend sys.path for local module access
sys.path.append(os.path.expanduser('~/shared_students/CapstoneFinalRepo/MultiCastHMAC'))
sys.path.append(os.path.expanduser('~/shared_students/CapstoneFinalRepo/Yolo'))
sys.path.append(os.path.expanduser('~/shared_students/CapstoneFinalRepo/Startup'))

from Receiver import Receiver as receive
from Broadcast import Broadcaster
from detect_onnx import YOLODetector
from startup_pos import main as startup_main


# Global flags
is_shutting_down = False
object_detected = False
charm_detected = False
charm_approach_in_progress = False  # New flag to track charm approach

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
        self.get_logger().info("WanderingBot initialized and wandering")
        self.current_cmd = Twist()
        self.is_turning_to_obstacle = False
        self.last_move_type = None
        self.avoidance_in_progress = False
        self.current_avoidance_task = None

        self.relative_position = [0.0, 0.0, 0.0]
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
            self.get_logger().warn("Cancelling current avoidance task")
        self.avoidance_in_progress = False
        self.is_turning_to_obstacle = False

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.relative_position = [
            position.x - self.start_position[0],
            position.y - self.start_position[1],
            0.0
        ]
        self.update_config_position()

    def ultrasonic_callback(self, msg):
        global object_detected

        # Process the incoming ultrasonic data - Range msg has a single range value, not an array
        if msg.range <= 0.01:  # Ignore invalid readings
            return
            
        min_distance = msg.range  # Single value from the Range message

        if min_distance < 0.2:  # Threshold in meters
            if not object_detected:
                object_detected = True
                self.get_logger().warn(f"ULTRASONIC DETECTED obstacle at {min_distance:.2f}m!")
                
                # If not already avoiding, trigger avoidance (no longer backing up)
                if not self.avoidance_in_progress:
                    # Use a neutral angle (0) since ultrasonic doesn't give direction
                    # The lidar will provide better angle information
                    self.current_avoidance_task = asyncio.create_task(self.avoid_object(0, min_distance))
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

            # ✨ Safely update only modified fields
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
        """Modified to pause wandering when charm approach is in progress"""
        await asyncio.sleep(1)
        while not is_shutting_down:
            # Check for obstacles, charm detection, or charm approach in progress
            if object_detected or self.is_turning_to_obstacle or charm_detected or charm_approach_in_progress:
                self.get_logger().debug("Pausing wandering due to obstacle/charm operations")
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
                
                while time.time() < end_time and not is_shutting_down and not object_detected and not charm_detected and not charm_approach_in_progress:
                    self.publisher.publish(cmd)
                    await asyncio.sleep(0.1)
                
                stop_cmd = Twist()
                self.publisher.publish(stop_cmd)
                self.current_cmd = stop_cmd
            
            await asyncio.sleep(0.3)
    
    async def approach_charm(self):
        """
        Improved charm detection and approach that works with object avoidance
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

    async def avoid_object(self, angle, distance):
        """Turn 45 degrees left or right depending on which side of 0 the object is located"""
        global object_detected
        
        self.is_turning_to_obstacle = True
        self.avoidance_in_progress = True
        
        self.get_logger().warn(f"OBJECT AVOIDANCE: angle={angle:.2f}°, distance={distance:.2f}m")
        
        try:
            # Determine turn direction based on angle
            # Normalize angle to 0-360 range if needed
            angle = angle % 360
            
            # First check if the object is in the ignore zone (behind the robot)
            if 110 <= angle <= 250:
                self.get_logger().info(f"Ignoring object at angle {angle:.2f}° - it's behind the robot")
                # Clean up and return early
                self.cleanup_avoidance()
                return
                
            # Simple rule: if object is at angle < 180, turn right; otherwise turn left
            # This means object on left side -> turn right, object on right side -> turn left
            if angle < 180:
                # Object is on left half - turn right to avoid
                turn_direction = 'right'
            else:
                # Object is on right half - turn left to avoid
                turn_direction = 'left'
            
            # Log the decision
            self.get_logger().info(f"Turning {turn_direction} to avoid obstacle at angle {angle:.2f}°")
            
            # Execute a 45-degree turn in the chosen direction
            # 45 degrees = π/4 radians
            # Time = angle / angular_speed
            turn_duration = math.pi/4 / self.safe_angular_speed
            
            # Start the turn
            turn_cmd = Twist()
            turn_cmd.angular.z = self.safe_angular_speed if turn_direction == 'left' else -self.safe_angular_speed
            self.current_cmd = turn_cmd
            
            # Monitor and continue turning while checking for more obstacles
            start_time = time.time()
            end_time = start_time + turn_duration
            
            while time.time() < end_time and not is_shutting_down:
                # Continue the turn
                self.publisher.publish(turn_cmd)
                
                # Short sleep to allow for sensor updates
                await asyncio.sleep(0.05)
            
            # After turn is complete, move forward
            self.get_logger().info("Continuing forward after obstacle avoidance turn")
            
            # Create forward command
            forward_cmd = Twist()
            forward_cmd.linear.x = self.safe_linear_speed
            self.current_cmd = forward_cmd
            
            # Move forward for a short time while continuously checking for obstacles
            forward_duration = 1.0  # 1 second of forward movement
            start_time = time.time()
            end_time = start_time + forward_duration
            
            # Continue to move forward while checking for obstacles
            while time.time() < end_time and not is_shutting_down:
                # If another obstacle is detected during forward movement,
                # exit early to handle the new obstacle
                if object_detected and not self.is_turning_to_obstacle:
                    self.get_logger().warn("New obstacle detected during avoidance - ending current avoidance")
                    break
                
                # Continue forward movement
                self.publisher.publish(forward_cmd)
                await asyncio.sleep(0.05)
            
            # Always stop at the end
            stop_cmd = Twist()
            self.publisher.publish(stop_cmd)
            self.current_cmd = stop_cmd
            
            self.get_logger().info("Obstacle avoidance cycle complete")
            
        except asyncio.CancelledError:
            self.get_logger().warn("Avoidance routine was cancelled")
            stop_cmd = Twist()
            self.publisher.publish(stop_cmd)
            self.current_cmd = stop_cmd
            raise
        except Exception as e:
            self.get_logger().error(f"Error in avoid_object: {e}")
        finally:
            # Only cleanup if we didn't detect a new obstacle
            if not object_detected:
                self.cleanup_avoidance()
            else:
                # Reset turning flag but keep avoidance_in_progress true
                # This allows a new avoidance cycle to start immediately
                self.is_turning_to_obstacle = False
    
    def cleanup_avoidance(self):
        """Clean up after avoidance routine completes or is interrupted"""
        self.is_turning_to_obstacle = False
        self.avoidance_in_progress = False
        global object_detected
        object_detected = False
        
    # Removed wall detection function


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
        self.critical_distance = 0.20  # Distance that requires immediate response
        self.normal_distance = 0.35    # Distance for normal obstacle avoidance
        
        # Rate limit for new avoidance actions - set to check every 4 seconds
        self.min_detection_interval = 4.0  # Only check for new objects every 4 seconds
        
        # Create a timer for continuous environment monitoring
        self.continuous_monitor_timer = self.create_timer(4.0, self.continuous_monitor)
        
        # Store the last scan message for continuous monitoring
        self.last_scan_msg = None

    def scan_callback(self, msg):
        global object_detected, is_shutting_down
        
        if is_shutting_down:
            return
            
        # Store the last scan message for continuous monitoring
        self.last_scan_msg = msg
        
        current_time = time.time()
        # Rate limit how often we process new obstacles
        if current_time - self.last_detection_time < self.min_detection_interval:
            return

        try:
            # Process scan data
            self.process_scan_data(msg, current_time)
                
        except Exception as e:
            self.get_logger().error(f"Error in scan_callback: {e}")
            
    def process_scan_data(self, msg, current_time):
        """Process scan data and initiate avoidance if needed"""
        global object_detected
        
        valid_ranges = [(i, r) for i, r in enumerate(msg.ranges)
                        if r != float('inf') and not math.isnan(r) and r > 0.05]
        if not valid_ranges:
            return

        min_index, min_distance = min(valid_ranges, key=lambda x: x[1])
        angle_increment = msg.angle_increment
        angle_min = msg.angle_min
        obstacle_angle = math.degrees((angle_min + min_index * angle_increment))

        # Only consider obstacles in front portion of robot
        # Avoid objects between 0-110 degrees and 250-360 degrees
        # Explicitly ignore objects between 110-250 degrees (behind the robot)
        if 110 <= obstacle_angle <= 250:
            self.get_logger().debug(f"Ignoring object at angle {obstacle_angle:.2f}° - it's behind the robot")
            return
            
        # Show obstacle info periodically
        if current_time - self.last_detection_time > 2.0:
            self.get_logger().info(f"Nearest object: distance={min_distance:.2f}m, angle={obstacle_angle:.2f}°")
            
        # Critical case: Very close obstacle detected - trigger immediate avoidance
        if min_distance < self.critical_distance:
            object_detected = True
            self.last_detection_time = current_time
            
            self.get_logger().warn(f"CRITICAL: Object at {min_distance:.2f}m, angle: {obstacle_angle:.2f}°")
            
            # Cancel any existing avoidance routine
            if self.wander_bot.avoidance_in_progress:
                self.get_logger().warn("Interrupting current avoidance for critical obstacle")
                self.wander_bot.cancel_avoidance()
                
            # Create new avoidance task - now calls avoid_object directly without backing up
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
                
    def continuous_monitor(self):
        """Continuously monitor the environment for obstacles, especially walls"""
        global object_detected, is_shutting_down
        
        if is_shutting_down or self.last_scan_msg is None:
            return
            
        # Check if we're already avoiding something
        if self.wander_bot.avoidance_in_progress:
            return
            
        # Reprocess the most recent scan data to constantly check for obstacles
        current_time = time.time()
        
        # Only process if we haven't recently triggered an avoidance
        if current_time - self.last_detection_time > self.min_detection_interval:
            self.process_scan_data(self.last_scan_msg, current_time)


def handle_shutdown(wanderbot, signum, frame):
    global is_shutting_down, charm_approach_in_progress
    print("\nShutdown signal received! Stopping the robot...")
    is_shutting_down = True
    charm_approach_in_progress = False  # Make sure to clear this flag
    if wanderbot:
        wanderbot.stop_robot()
    print("Robot stopped. Exiting...")
    sys.exit(0)

async def spin_node(node):
    global is_shutting_down  # Make sure to access the global variable
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
    global is_shutting_down, charm_approach_in_progress  # Ensure global variables are available
    
    # await startup_main()  # Uncomment if needed

    rclpy.init()
    try:
        with open(CONFIG_PATH, "r") as config_file:
            config = json.load(config_file)
    except FileNotFoundError:
        print("Config file not found, proceeding with default behavior")
        config = {}

    wander_bot = WanderingBot()
    lidar_agent = LidarAgent(wander_bot)
    signal.signal(signal.SIGINT, lambda signum, frame: handle_shutdown(wander_bot, signum, frame))
    signal.signal(signal.SIGTERM, lambda signum, frame: handle_shutdown(wander_bot, signum, frame))

    wander_spin_task = asyncio.create_task(spin_node(wander_bot))
    lidar_task = asyncio.create_task(spin_node(lidar_agent))
    
    # Start charm task first - it has priority over wandering
    charm_task = asyncio.create_task(wander_bot.approach_charm())
    
    # Wandering will only happen when charm detection is not active
    wander_task = asyncio.create_task(wander_bot.wander())

    try:
        await asyncio.gather(wander_spin_task, lidar_task, wander_task, charm_task)
    except asyncio.CancelledError:
        pass
    except KeyboardInterrupt:
        print("Keyboard interrupt detected")
        is_shutting_down = True
        charm_approach_in_progress = False
        wander_bot.stop_robot()
    finally:
        is_shutting_down = True  # Make sure flag is set
        charm_approach_in_progress = False
        wander_bot.stop_robot()
        wander_bot.destroy_node()
        lidar_agent.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Shutting down (top level)...")