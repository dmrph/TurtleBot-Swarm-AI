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