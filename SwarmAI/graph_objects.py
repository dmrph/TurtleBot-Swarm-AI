#!/usr/bin/env python3
import sys
import os
import json
import asyncio
import math
import random
import rclpy
import signal
import time
import threading
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Range

# Path setup
sys.path.extend([
    os.path.expanduser('~/shared_students/CapstoneFinalRepo/MultiCastHMAC'),
    os.path.expanduser('~/shared_students/CapstoneFinalRepo/Yolo'),
    os.path.expanduser('~/shared_students/CapstoneFinalRepo/Startup')
])

# Handle optional imports
try:
    from Receiver import Receiver
    from Broadcast import Broadcaster
except ImportError as e:
    print(f"Warning: Could not import Receiver/Broadcaster - some features disabled. Error: {e}")
    Receiver = None
    Broadcaster = None

try:
    from detect_onnx import YOLODetector
except ImportError as e:
    print(f"Warning: Could not import YOLODetector - charm detection disabled. Error: {e}")
    YOLODetector = None

try:
    from startup_pos import main as startup_main
except ImportError as e:
    print(f"Warning: Could not import startup_pos - position tracking may be limited. Error: {e}")
    startup_main = None

# Setup for point data - minimal addition for graphing
POINT_DATA_DIR = os.path.expanduser('~/shared_students/CapstoneFinalRepo/point_data')
print(f"Output directory: {POINT_DATA_DIR}")
os.makedirs(POINT_DATA_DIR, exist_ok=True)

# Global flags
is_shutting_down = False
object_detected = False
charm_detected = False
charm_approach_in_progress = False
CONFIG_PATH = os.path.expanduser('~/shared_students/CapstoneFinalRepo/config.json')
TESTING_MODE = True  # Set to False when running with multiple bots

# Global variables for plotting
fig = None
ax = None
robot_path_line = None
obstacle_scatter = None
detection_circle = None
critical_circle = None
all_obstacles = []
plot_lock = threading.Lock()  # Lock for thread safety when updating plot

def initialize_plot():
    """Initialize matplotlib plot for real-time visualization"""
    global fig, ax, robot_path_line, obstacle_scatter
    
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Initialize plot components
    robot_path_line, = ax.plot([], [], 'b-', linewidth=2, label='Robot Path')
    obstacle_scatter = ax.scatter([], [], c='r', s=10, alpha=0.5, label='Detected Objects')
    
    # Add base station marker
    ax.scatter([0], [0], c='g', s=100, marker='*', label='Base Station')
    ax.annotate('Base Station', (0, 0), textcoords="offset points", 
               xytext=(0,10), ha='center')
    
    # Set labels and title
    ax.set_xlabel('X Position (m)')
    ax.set_ylabel('Y Position (m)')
    ax.set_title('Robot Path Relative to Base Station')
    ax.grid(True)
    ax.legend()
    
    # Save initial version
    plt.tight_layout()
    output_path = os.path.join(POINT_DATA_DIR, "latest_plot.png")
    fig.savefig(output_path)
    
    return fig, ax

def update_plot(position_data, obstacles_cart):
    """Update the plot with new position and obstacle data"""
    global robot_path_line, obstacle_scatter, ax, fig, plot_lock
    
    with plot_lock:
        if len(position_data) > 0:
            # Extract x and y coordinates for robot path, using relative positions
            x_pos = [p['relative_x'] for p in position_data]
            y_pos = [p['relative_y'] for p in position_data]
            
            # Make sure we have coordinate data
            if len(x_pos) == 0 or len(y_pos) == 0:
                print("Warning: No position data points to plot")
                return
                
            # Debug info
            print(f"Plotting robot path with {len(x_pos)} points")
            print(f"X range: {min(x_pos):.2f} to {max(x_pos):.2f}")
            print(f"Y range: {min(y_pos):.2f} to {max(y_pos):.2f}")
            
            # Clear previous plot elements
            ax.clear()
            
            # Add base station marker
            ax.scatter([0], [0], c='g', s=100, marker='*', label='Base Station')
            ax.annotate('Base Station', (0, 0), textcoords="offset points", 
                       xytext=(0,10), ha='center')
            
            # Plot robot path
            ax.plot(x_pos, y_pos, 'b-', linewidth=2, label='Robot Path')
            
            # Add current position marker
            if len(position_data) > 0:
                latest_pos = position_data[-1]
                ax.scatter([latest_pos['relative_x']], [latest_pos['relative_y']], 
                          c='blue', s=100, marker='o', 
                          label='Current Position')
            
            # Plot all detected objects
            if len(obstacles_cart) > 0:
                # Separate cones and charms
                cones = [o for o in obstacles_cart if o.get('type') == 'cone']
                charms = [o for o in obstacles_cart if o.get('type') == 'charm']
                
                if cones:
                    x_cones = [o['x'] for o in cones]
                    y_cones = [o['y'] for o in cones]
                    ax.scatter(x_cones, y_cones, c='r', s=50, marker='^', label='Cones')
                    print(f"Plotting {len(x_cones)} cone points")
                
                if charms:
                    x_charms = [o['x'] for o in charms]
                    y_charms = [o['y'] for o in charms]
                    ax.scatter(x_charms, y_charms, c='b', s=50, marker='o', label='Charms')
                    print(f"Plotting {len(x_charms)} charm points")
            
            # Set axes limits with margin
            if len(x_pos) > 0 and len(y_pos) > 0:
                margin = 0.5
                min_x = min(x_pos) - margin
                max_x = max(x_pos) + margin
                min_y = min(y_pos) - margin
                max_y = max(y_pos) + margin
                
                # Ensure the base station (0,0) is always visible
                min_x = min(min_x, -margin)
                max_x = max(max_x, margin)
                min_y = min(min_y, -margin)
                max_y = max(max_y, margin)
                
                ax.set_xlim(min_x, max_x)
                ax.set_ylim(min_y, max_y)
            
            # Set labels and title
            ax.set_xlabel('X Position (m)')
            ax.set_ylabel('Y Position (m)')
            ax.set_title('Robot Path Relative to Base Station')
            ax.grid(True)
            ax.legend()
            
            # Save the plot
            try:
                # Save to latest_plot.png
                latest_path = os.path.join(POINT_DATA_DIR, "latest_plot.png")
                fig.tight_layout()
                fig.savefig(latest_path)
                print(f"Updated plot saved to {latest_path}")
            except Exception as e:
                print(f"Error saving plot: {e}")

def convert_polar_to_cartesian(robot_position, obstacles):
    """Convert polar obstacle coordinates to cartesian coordinates relative to robot position"""
    obstacles_cart = []
    
    robot_x, robot_y = robot_position['relative_x'], robot_position['relative_y']
    
    for obs in obstacles:
        # Convert polar (distance, angle) to cartesian
        angle_rad = math.radians(obs['angle'])
        obstacle_x = robot_x + obs['distance'] * math.cos(angle_rad)
        obstacle_y = robot_y + obs['distance'] * math.sin(angle_rad)
        
        obstacles_cart.append({
            'x': obstacle_x,
            'y': obstacle_y,
            'distance': obs['distance'],
            'timestamp': obs['timestamp']
        })
    
    return obstacles_cart

def initialize_camera_early():
    """Initialize camera in background thread"""
    print("Starting early camera initialization...")
    try:
        # Use the correct path to the model
        model_path = os.path.expanduser('~/shared_students/CapstoneFinalRepo/Yolo/best_gray.onnx')
        print(f"Loading YOLO model from: {model_path}")
        if not os.path.exists(model_path):
            print(f"Error: Model file not found at {model_path}")
            return None
            
        detector = YOLODetector(model_path=model_path, show_display=False)
        if detector.initialize_camera():
            print("Camera initialized successfully")
            return detector
        else:
            print("Failed to initialize camera")
    except Exception as e:
        print(f"Camera init error: {e}")
    return None

# Initialize camera early
background_detector = initialize_camera_early()

class WanderingBot(Node):
    def __init__(self):
        super().__init__('wandering_bot')
        # Setup publishers/subscribers
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.ultrasonic_sub = self.create_subscription(
            Range, '/ultrasonic', self.ultrasonic_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Movement parameters
        self.safe_linear_speed = 0.10
        self.safe_angular_speed = 0.5
        self.backup_speed = -0.10
        self.current_cmd = Twist()
        self.rotation_time_90deg = 1.57
        
        # State tracking
        self.relative_position = [0.0, 0.0, 0.0]
        self.start_position = self.load_start_position()
        self.detector = None
        self.is_turning_to_obstacle = False
        self.avoidance_in_progress = False
        self.current_avoidance_task = None
        self.last_move_type = None
        self.timer = self.create_timer(1.0, self.update_config_position)
        
        # Graph data tracking
        self.position_history = []
        self.graph_timer = self.create_timer(2.0, self.save_position_data)
        
        # Plot update timer
        self.plot_timer = self.create_timer(5.0, self.update_live_plot)
        
        # Detection timer - run detection every 0.1 seconds
        self.detection_timer = self.create_timer(0.1, self.run_detection)
        
        # Initialize detector if available
        self.setup_detector()

    def odom_callback(self, msg):
        """Handle odometry updates"""
        position = msg.pose.pose.position
        print(f"ODOM update: x={position.x:.2f}, y={position.y:.2f}")
        
        self.relative_position = [
            position.x - self.start_position[0],
            position.y - self.start_position[1],
            0.0
        ]
        self.update_config_position()
        
        # Save position data for graphing
        self.position_history.append({
            'timestamp': time.time(),
            'x': position.x,
            'y': position.y,
            'relative_x': self.relative_position[0],
            'relative_y': self.relative_position[1]
        })

    def update_live_plot(self):
        """Update the live plot with current data"""
        global all_obstacles
        
        # Only update if we have position history
        if not self.position_history:
            print("No position history to plot yet")
            return
            
        # Convert recent obstacles to cartesian coordinates
        latest_position = self.position_history[-1]
        obstacles_cart = []
        
        # Print debug info
        print(f"Updating plot with robot at position: x={latest_position['x']:.2f}, y={latest_position['y']:.2f}")
        print(f"Robot's relative position: x={latest_position['relative_x']:.2f}, y={latest_position['relative_y']:.2f}")
        
        with plot_lock:
            recent_obstacles = all_obstacles[-100:] if len(all_obstacles) > 100 else all_obstacles
            obstacles_cart = convert_polar_to_cartesian(latest_position, recent_obstacles)
            print(f"Plotting {len(obstacles_cart)} obstacle points")
        
        # Update the plot
        update_plot(self.position_history, obstacles_cart)

    def save_position_data(self):
        """Save position data to file for graphing"""
        if not self.position_history:
            return
            
        timestamp = int(time.time())
        filename = os.path.join(POINT_DATA_DIR, f"position_data_{timestamp}.json")
        
        try:
            with open(filename, 'w') as f:
                json.dump({
                    'bot_id': 'wandering_bot',
                    'positions': self.position_history,
                    'start_position': self.start_position
                }, f, indent=2)
                
            print(f"Saved {len(self.position_history)} position data points to {filename}")
            self.get_logger().info(f"Saved position data to {filename}")
            # Clear history after saving to avoid memory buildup
            self.position_history = []
        except Exception as e:
            self.get_logger().error(f"Failed to save position data: {e}")

    def load_start_position(self):
        """Load starting position from config file"""
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
        """Update config file with current position and direction"""
        try:
            if os.path.exists(CONFIG_PATH):
                with open(CONFIG_PATH, "r") as f:
                    config = json.load(f)
            else:
                config = {}

            # Only update the direction tag if it exists
            if "current_position" in config and "base_station_tag_id" in config["current_position"]:
                # Get current direction from config
                current_direction = config["current_position"].get("direction", "unknown")
                
                # Update only the direction tag
                config["current_position"]["direction"] = current_direction
                config["current_position"]["last_updated"] = time.strftime("%Y-%m-%d %H:%M:%S")

                with open(CONFIG_PATH, "w") as f:
                    json.dump(config, f, indent=4)
                    
        except Exception as e:
            self.get_logger().error(f"Failed to update config: {e}")

    def setup_detector(self):
        """Setup YOLO detector"""
        global background_detector
        if self.detector is None and background_detector is not None:
            self.detector = background_detector
            background_detector = None
            self.get_logger().info("Using pre-initialized camera")
            return True
        elif self.detector is None:
            try:
                model_path = os.path.expanduser('~/shared_students/CapstoneFinalRepo/Yolo/best_gray.onnx')
                if not os.path.exists(model_path):
                    self.get_logger().error(f"Model file not found at {model_path}")
                    return False
                    
                self.detector = YOLODetector(model_path=model_path, show_display=False)
                if self.detector.initialize_camera():
                    self.get_logger().info("Camera initialized successfully")
                    return True
                else:
                    self.get_logger().error("Failed to initialize camera")
            except Exception as e:
                self.get_logger().error(f"Camera setup error: {e}")
        return False

    def ultrasonic_callback(self, msg):
        global object_detected
        if msg.range <= 0.01:  # Invalid reading
            return
            
        if msg.range < 0.2:  # Emergency stop threshold
            if not object_detected:
                object_detected = True
                self.get_logger().warn(f"ULTRASONIC OBSTACLE at {msg.range:.2f}m!")
                if not self.avoidance_in_progress:
                    self.current_avoidance_task = asyncio.create_task(self.back_up_safely(msg.range))
        else:
            object_detected = False

    async def back_up_safely(self, distance):
        self.get_logger().warn(f"BACKING UP: {distance:.2f}m")
        backup_cmd = Twist()
        backup_cmd.linear.x = self.backup_speed
        start_time = time.time()
        
        while time.time() - start_time < 1.0 and not is_shutting_down:
            self.publisher.publish(backup_cmd)
            await asyncio.sleep(0.1)
        
        stop_cmd = Twist()
        self.publisher.publish(stop_cmd)
        self.current_cmd = stop_cmd
        await asyncio.sleep(0.2)
        self.cleanup_avoidance()

    def stop_robot(self):
        stop_cmd = Twist()
        for _ in range(5):
            self.publisher.publish(stop_cmd)
            time.sleep(0.05)
        self.current_cmd = stop_cmd

    def cleanup_avoidance(self):
        self.is_turning_to_obstacle = False
        self.avoidance_in_progress = False
        global object_detected
        object_detected = False

    async def move(self, command, duration=1.0, force=False):
        """Execute movement command"""
        if is_shutting_down and not force:
            return
            
        if object_detected and not self.is_turning_to_obstacle and not force:
            return
        
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
        await self._execute_movement(cmd, duration)

    async def _execute_movement(self, cmd, duration):
        start_time = time.time()
        end_time = start_time + duration
        
        while time.time() < end_time and not is_shutting_down:
            if object_detected and not self.is_turning_to_obstacle and cmd.linear.x > 0:
                break
            self.publisher.publish(cmd)
            await asyncio.sleep(0.05)

        stop_cmd = Twist()
        self.publisher.publish(stop_cmd)
        self.current_cmd = stop_cmd

    async def wander(self):
        await asyncio.sleep(1)
        while not is_shutting_down:
            if object_detected or charm_detected or charm_approach_in_progress:
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
            elif move_type in ['left', 'right']:
                await self.move(move_type, random.uniform(0.4, 0.8))
            elif move_type == 'forward_arc':
                cmd = Twist()
                cmd.linear.x = self.safe_linear_speed
                cmd.angular.z = random.uniform(0.15, -0.15)
                await self._execute_movement(cmd, random.uniform(1.5, 3.0))
            
            await asyncio.sleep(0.3)

    async def approach_charm(self):
        global charm_detected, charm_approach_in_progress
        if not self.setup_detector():
            self.get_logger().error("Camera setup failed")
            return

        while not is_shutting_down:
            if object_detected or self.avoidance_in_progress:
                await asyncio.sleep(0.5)
                continue

            try:
                charm_found, _ = await self.detector.detect_charm_center_and_move(
                    continuous=False, max_frames=1)
                
                if charm_found:
                    charm_detected = True
                    charm_approach_in_progress = True
                    self.get_logger().info("CHARM DETECTED - PERFORMING 90° TURN")
                    
                    # Stop current movement
                    self.stop_robot()
                    
                    # Execute 90° left turn
                    turn_cmd = Twist()
                    turn_cmd.angular.z = self.safe_angular_speed
                    self.publisher.publish(turn_cmd)
                    await asyncio.sleep(self.rotation_time_90deg)
                    
                    # Stop turning
                    self.stop_robot()
                    
                    # Continue forward
                    forward_cmd = Twist()
                    forward_cmd.linear.x = self.safe_linear_speed
                    self.publisher.publish(forward_cmd)
                    await asyncio.sleep(1.0)  # Move forward for 1 second
                    
                    # Reset flags
                    charm_approach_in_progress = False
                    charm_detected = False
                    
            except Exception as e:
                self.get_logger().error(f"Charm detection error: {e}")
            await asyncio.sleep(0.2)

    async def avoid_object(self, angle, distance):
        """Full obstacle avoidance routine"""
        global object_detected
        if not (angle < 130 or angle > 230):
            object_detected = False
            return
            
        self.is_turning_to_obstacle = True
        self.avoidance_in_progress = True
        
        self.get_logger().warn(f"AVOIDING: angle={angle:.2f}°, distance={distance:.2f}m")
        
        try:
            self.stop_robot()
            
            if distance < 0.25:
                await self.back_up_safely(distance)
                if is_shutting_down:
                    return
            
            turn_direction = 'right' if angle < 180 else 'left'
            await self.move(turn_direction, duration=1.8, force=True)
            
            if is_shutting_down:
                return
                
            await self.move('forward', duration=1.2, force=True)
            
            if is_shutting_down:
                return
                
            await self.move(turn_direction, duration=0.6, force=True)
            
            if is_shutting_down:
                return
                
            await self.move('forward', duration=0.8, force=True)
            
        except asyncio.CancelledError:
            self.get_logger().warn("Avoidance cancelled")
        except Exception as e:
            self.get_logger().error(f"Avoidance error: {e}")
        finally:
            self.cleanup_avoidance()

    def cancel_avoidance(self):
        """Cancel current avoidance task"""
        if self.current_avoidance_task and not self.current_avoidance_task.done():
            self.current_avoidance_task.cancel()
        self.cleanup_avoidance()

    async def run_detection(self):
        """Run continuous object detection"""
        if self.detector is None:
            return
            
        try:
            # Run detection
            detections = await self.detector.detect_objects()
            
            if detections:
                # Convert detections to obstacle format
                for det in detections:
                    if det['class'] in ['cone', 'charm']:  # Only process cones and charms
                        obstacle_data = {
                            'timestamp': time.time(),
                            'type': det['class'],
                            'x': det['x'],
                            'y': det['y'],
                            'confidence': det['confidence']
                        }
                        
                        # Add to global obstacle list for plotting
                        with plot_lock:
                            all_obstacles.append(obstacle_data)
                            
        except Exception as e:
            self.get_logger().error(f"Detection error: {e}")

class LidarAgent(Node):
    def __init__(self, wander_bot):
        super().__init__('lidar_agent')
        self.wander_bot = wander_bot
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        )
        self.last_detection_time = time.time()
        self.critical_distance = 0.20
        self.normal_distance = 0.35
        self.min_detection_interval = 0.5
        
        # For graphing
        self.obstacle_history = []
        self.graph_timer = self.create_timer(5.0, self.save_obstacle_data)

    def scan_callback(self, msg):
        global object_detected, all_obstacles
        if is_shutting_down:
            return

        current_time = time.time()
        if current_time - self.last_detection_time < self.min_detection_interval:
            return

        try:
            valid_ranges = [(i, r) for i, r in enumerate(msg.ranges)
                           if r != float('inf') and not math.isnan(r) and r > 0.05]
            if not valid_ranges:
                return

            min_index, min_distance = min(valid_ranges, key=lambda x: x[1])
            angle = math.degrees(msg.angle_min + min_index * msg.angle_increment)
            
            # Save data for graphing
            obstacle_data = {
                'timestamp': current_time,
                'distance': min_distance,
                'angle': angle
            }
            self.obstacle_history.append(obstacle_data)
            
            # Add to global obstacle list for plotting
            with plot_lock:
                all_obstacles.append(obstacle_data)
                # Keep the list at a reasonable size
                if len(all_obstacles) > 1000:
                    all_obstacles = all_obstacles[-1000:]

            if min_distance < self.critical_distance:
                object_detected = True
                self.last_detection_time = current_time
                self.wander_bot.get_logger().warn(f"CRITICAL OBSTACLE: {min_distance:.2f}m")
                
                if self.wander_bot.avoidance_in_progress:
                    self.wander_bot.cancel_avoidance()
                
                self.wander_bot.current_avoidance_task = asyncio.create_task(
                    self.wander_bot.avoid_object(angle, min_distance))
                    
            elif min_distance < self.normal_distance:
                object_detected = True
                self.last_detection_time = current_time
                
                if not self.wander_bot.avoidance_in_progress:
                    self.wander_bot.current_avoidance_task = asyncio.create_task(
                        self.wander_bot.avoid_object(angle, min_distance))
        except Exception as e:
            self.get_logger().error(f"LIDAR error: {e}")
            
    def save_obstacle_data(self):
        """Save obstacle data to file for graphing"""
        if not self.obstacle_history:
            return
            
        timestamp = int(time.time())
        filename = os.path.join(POINT_DATA_DIR, f"obstacle_data_{timestamp}.json")
        
        try:
            with open(filename, 'w') as f:
                json.dump({
                    'bot_id': 'lidar_agent',
                    'obstacles': self.obstacle_history
                }, f, indent=2)
                
            self.get_logger().info(f"Saved obstacle data to {filename}")
            # Clear history after saving to avoid memory buildup
            self.obstacle_history = []
        except Exception as e:
            self.get_logger().error(f"Failed to save obstacle data: {e}")

def handle_shutdown(wanderbot, signum, frame):
    global is_shutting_down
    is_shutting_down = True
    if wanderbot:
        wanderbot.stop_robot()
    sys.exit(0)

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
    global is_shutting_down, fig, ax
    
    try:
        if not rclpy.ok():
            rclpy.init()
    except:
        pass

    # Initialize plot
    print("Initializing plot...")
    initialize_plot()
    
    # Run startup sequence if available
    if startup_main is not None:
        print("Running startup sequence...")
        try:
            # Create a temporary node for startup
            startup_node = Node('startup_node')
            startup_executor = rclpy.executors.SingleThreadedExecutor()
            startup_executor.add_node(startup_node)
            
            # Run startup with testing mode flag
            startup_config = {
                'testing_mode': TESTING_MODE,
                'skip_communication': TESTING_MODE  # Skip waiting for other bots in testing mode
            }
            
            # Run startup in a separate thread
            startup_thread = threading.Thread(
                target=lambda: startup_main(startup_node, startup_config)
            )
            startup_thread.start()
            
            # Wait for startup to complete or timeout
            timeout = 60  # 60 second timeout
            start_time = time.time()
            while startup_thread.is_alive() and time.time() - start_time < timeout:
                startup_executor.spin_once(timeout_sec=0.1)
                await asyncio.sleep(0.1)
            
            # Cleanup startup node
            startup_node.destroy_node()
            startup_executor.shutdown()
            
            if startup_thread.is_alive():
                print("Warning: Startup sequence timed out")
                startup_thread.join(timeout=1.0)
            else:
                print("Startup sequence completed successfully")
                
            # Add a delay after startup to ensure config is updated
            await asyncio.sleep(2.0)
            
        except Exception as e:
            print(f"Error during startup: {e}")
    else:
        print("Warning: startup_pos module not available, skipping startup sequence")

    try:
        with open(CONFIG_PATH, "r") as config_file:
            config = json.load(config_file)
    except FileNotFoundError:
        config = {}

    wander_bot = WanderingBot()
    lidar_agent = LidarAgent(wander_bot)
    
    signal.signal(signal.SIGINT, lambda s, f: handle_shutdown(wander_bot, s, f))
    signal.signal(signal.SIGTERM, lambda s, f: handle_shutdown(wander_bot, s, f))

    # First spin to find base station
    print("Spinning to find base station...")
    spin_cmd = Twist()
    spin_cmd.angular.z = 0.5  # Moderate spin speed
    for _ in range(20):  # Spin for about 2 seconds
        wander_bot.publisher.publish(spin_cmd)
        await asyncio.sleep(0.1)
    
    # Stop spinning
    stop_cmd = Twist()
    wander_bot.publisher.publish(stop_cmd)
    await asyncio.sleep(0.5)

    tasks = [
        asyncio.create_task(spin_node(wander_bot)),
        asyncio.create_task(spin_node(lidar_agent)),
        asyncio.create_task(wander_bot.wander())
    ]

    # Only add charm task if detector is available
    if YOLODetector is not None and wander_bot.detector is not None:
        tasks.append(asyncio.create_task(wander_bot.approach_charm()))

    try:
        await asyncio.gather(*tasks)
    except (asyncio.CancelledError, KeyboardInterrupt):
        pass
    finally:
        is_shutting_down = True
        wander_bot.stop_robot()
        wander_bot.destroy_node()
        lidar_agent.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nShutting down gracefully...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        print("Cleanup complete")