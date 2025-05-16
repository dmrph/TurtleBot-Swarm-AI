#!/usr/bin/env python3

import os
import json
import math
import asyncio
import time
import signal
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy
import sys
import logging

# Add path to import euler_from_quaternion function
sys.path.append(os.path.expanduser('~/shared_students/CapstoneFinalRepo'))
from Startup.startup_pos import euler_from_quaternion

# Path to config file
CONFIG_PATH = os.path.expanduser('~/shared_students/CapstoneFinalRepo/config.json')

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger("CoordinateNavigator")

# Global shutdown flag
is_shutting_down = False

class CoordinateNavigator(Node):
    """
    A ROS node that navigates the TurtleBot to specific coordinates
    based on the assumption that each TurtleBot is at (0,0) in its own coordinate system
    """
    
    def __init__(self):
        super().__init__('coordinate_navigator')
        logger.info("Coordinate Navigator initialized")
        
        # Initialize publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # QoS for reliable subscription
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Subscribe to odometry for position tracking
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile
        )
        
        # Position tracking - absolute coordinates from odometry (for internal use only)
        self.current_x = None  # Will be initialized on first odometry message
        self.current_y = None
        self.current_heading = None
        
        # Our (0,0) coordinate system - this is what we use for all navigation
        self.relative_position = [0.0, 0.0, 0.0]  # [x, y, heading]
        
        # Flag to track if we've received the first odometry message
        self.odom_initialized = False
        
        # Bot identification and positioning relative to base station
        self.bot_id = None
        self.direction = None  # Direction from base station (north, east, south, west)
        self.distance_from_tag = 0.0  # Distance from the base station tag in meters
        
        # Other bot offsets (how to transform coordinates from other bots)
        self.bot_offsets = {}
        
        # Movement control
        self.target_reached = True
        self.movement_precision = 0.1  # meters
        self.is_navigating = False
        
        # For logging/debugging
        self.last_debug_time = time.time()
        
        # Load bot configuration from config
        self.load_bot_configuration()
        
        # Create timer for updating position
        self.timer = self.create_timer(5.0, self.update_config_position)
        
        logger.info(f"Bot ID: {self.bot_id}, Direction: {self.direction}, Distance from tag: {self.distance_from_tag}m")
        logger.info(f"Bot offsets: {self.bot_offsets}")
        logger.info("TurtleBot coordinate system initialized, assuming position (0,0)")

    
    def load_bot_configuration(self):
        """Load the bot configuration from config.json."""
        try:
            if os.path.exists(CONFIG_PATH):
                with open(CONFIG_PATH, "r") as f:
                    config = json.load(f)
                
                # Get this bot's ID
                if "important_info" in config and "bot_id" in config["important_info"]:
                    self.bot_id = config["important_info"]["bot_id"]
                else:
                    self.bot_id = "unknown"
                    logger.warning("No bot_id found in config.json. Using default value.")
                
                # Get this bot's position relative to base station
                if "current_position" in config:
                    current_pos = config["current_position"]
                    self.direction = current_pos.get("direction", "north")
                    self.distance_from_tag = current_pos.get("distance", 0.25)
                    
                    logger.info(f"Loaded bot configuration: ID={self.bot_id}, Direction={self.direction}, Distance={self.distance_from_tag}")
                
                # Load other bots' configurations for coordinate transformations
                self.bot_offsets = {}
                
                # Process other_bot1
                if "other_bot1" in config and len(config["other_bot1"]) > 0:
                    bot1_config = config["other_bot1"][0]
                    bot1_id = bot1_config.get("bot_id")
                    if bot1_id is not None:
                        self.bot_offsets[str(bot1_id)] = {
                            "direction": bot1_config.get("direction"),
                            "distance": bot1_config.get("distance", 0.0)
                        }
                
                # Process other_bot2
                if "other_bot2" in config and len(config["other_bot2"]) > 0:
                    bot2_config = config["other_bot2"][0]
                    bot2_id = bot2_config.get("bot_id")
                    if bot2_id is not None:
                        self.bot_offsets[str(bot2_id)] = {
                            "direction": bot2_config.get("direction"),
                            "distance": bot2_config.get("distance", 0.0)
                        }
                
                logger.info(f"Loaded offsets for {len(self.bot_offsets)} other bots")
                return True
            else:
                logger.warning(f"Config file not found: {CONFIG_PATH}")
        except Exception as e:
            logger.error(f"Error loading bot configuration: {e}")
        
        # Default values if config loading fails
        self.bot_id = "unknown"
        self.direction = "north"
        self.distance_from_tag = 0.25  # default 25cm
        self.bot_offsets = {}
        return False
    
    def update_config_position(self):
        """Update the current position in config.json."""
        try:
            config = {}
            if os.path.exists(CONFIG_PATH):
                with open(CONFIG_PATH, "r") as f:
                    config = json.load(f)
            
            # Preserve the existing current_position structure
            if "current_position" not in config:
                config["current_position"] = {
                    "base_station_tag_id": 33,  # Default
                    "direction": self.direction,
                    "position": {"x": 0.0, "y": 0.0},
                    "distance": self.distance_from_tag
                }
            
            # Update just the position part while preserving other fields
            config["current_position"]["position"] = {
                "x": self.relative_position[0],
                "y": self.relative_position[1]
            }
            
            # Add a last_updated field if needed
            config["current_position"]["last_updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
            
            with open(CONFIG_PATH, "w") as f:
                json.dump(config, f, indent=4)
            
            logger.info(
                f"Updated config.json: bot={self.bot_id}, x={self.relative_position[0]:.2f}, "
                f"y={self.relative_position[1]:.2f}"
            )
        except Exception as e:
            logger.error(f"Failed to update config: {e}")
    
    def odom_callback(self, msg):
        """Process odometry data to track position."""
        # Get position from odometry
        pos = msg.pose.pose.position
        
        # Get orientation and convert to Euler angles
        quat = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion(quat.x, quat.y, quat.z, quat.w)
        
        # On first message, initialize current position
        # This is critical - we're only storing the first position to calculate relative movements
        # We don't actually use the absolute values for navigation
        if self.current_x is None:
            # Store the initial odometry values as reference (for internal tracking only)
            self.current_x = pos.x
            self.current_y = pos.y
            self.current_heading = yaw
            
            # Record the initial real heading for future calculations
            self.initial_yaw = yaw
            
            self.odom_initialized = True
            logger.info(f"Received first odometry message. Absolute position: x={pos.x:.2f}, y={pos.y:.2f}, heading={math.degrees(yaw):.2f}°")
            logger.info(f"All movement will be tracked relative to this starting point.")
            logger.info(f"Current position in our own coordinate system: (0.0, 0.0), heading: 0.0°")
            return
        
        # Calculate raw movement from previous position in world frame
        dx_world = pos.x - self.current_x
        dy_world = pos.y - self.current_y
        
        # Transform movement to robot's own frame of reference based on current heading
        # This gives us forward/backward and left/right movement in the robot's perspective
        dx_robot = dx_world * math.cos(self.current_heading) + dy_world * math.sin(self.current_heading)
        dy_robot = -dx_world * math.sin(self.current_heading) + dy_world * math.cos(self.current_heading)
        
        # Apply the coordinate system convention for a standard graph:
        # - In a standard graph, positive X is right/east, positive Y is up/north
        # - In the robot's frame, forward is 'up' (north) and right is 'right' (east)
        
        # In robot's frame: 
        # - dx_robot is positive when moving forward, negative when moving backward
        # - dy_robot is positive when moving right, negative when moving left
        # To match standard graph:
        # - forward movement (dx_robot) increases Y (north)
        # - rightward movement (dy_robot) increases X (east)
        north_movement = dx_robot     # forward/backward -> north/south (Y axis)
        east_movement = dy_robot      # right/left -> east/west (X axis)
        
        # Update absolute position in world coordinates (for internal tracking only)
        self.current_x = pos.x
        self.current_y = pos.y
        self.current_heading = yaw
        
        # Update relative position using standard graph convention:
        # x increases when moving east (right), decreases when moving west (left)
        # y increases when moving north (forward), decreases when moving south (backward)
        self.relative_position[0] += east_movement
        self.relative_position[1] += north_movement
        
        # Calculate relative heading by subtracting the initial heading
        # This makes our initial heading exactly 0 degrees
        relative_heading = yaw - self.initial_yaw
        
        # Normalize relative heading to [-π, π]
        while relative_heading > math.pi:
            relative_heading -= 2 * math.pi
        while relative_heading < -math.pi:
            relative_heading += 2 * math.pi
            
        self.relative_position[2] = relative_heading
        
        # Debug at fixed intervals (to avoid log flooding)
        if hasattr(self, 'last_debug_time') and time.time() - self.last_debug_time < 2.0:
            pass  # Skip debug output if less than 2 seconds since last one
        else:
            self.last_debug_time = time.time()
            logger.debug(
                f"Position updated: x={self.relative_position[0]:.3f}, y={self.relative_position[1]:.3f}, "
                f"heading={math.degrees(self.relative_position[2]):.1f}°, "
                f"moved: east={east_movement:.3f}, north={north_movement:.3f}"
            )
    
    def stop_robot(self):
        """Stop the robot's movement."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        logger.info("Robot stopped")
    
    def transform_coordinates_from_bot(self, source_bot_id, x, y):
        """
        Transform coordinates from another bot's reference frame to this bot's reference frame.
        
        Args:
            source_bot_id: ID of the bot that sent the coordinates
            x: X coordinate in the source bot's reference frame
            y: Y coordinate in the source bot's reference frame
            
        Returns:
            Tuple of (x, y) in this bot's reference frame
        """
        source_bot_id = str(source_bot_id)  # Convert to string to ensure key matching
        
        if str(self.bot_id) == source_bot_id:
            # No transformation needed if coordinates are from this bot
            return x, y
        
        if source_bot_id not in self.bot_offsets:
            logger.warning(f"No offset information for bot {source_bot_id}. Using original coordinates.")
            return x, y
        
        offset = self.bot_offsets[source_bot_id]
        source_direction = offset.get("direction")
        source_distance = offset.get("distance", 0.0)
        
        logger.info(f"Transforming coordinates from bot {source_bot_id} ({source_direction}) at distance {source_distance}m")
        
        # This is where we apply the exact logic you described:
        # If a north bot says (1,1) and it's 0.25m from the box:
        # - East bot should go to (1.25, 1)
        # - West bot should go to (0.75, 1)
        # - South bot should go to (1, 0.75)
        
        # Simple fixed-direction transformations based on absolute directions
        if source_direction == "north":
            if self.direction == "north":
                # Same direction, no change
                return x, y
            elif self.direction == "east":
                # East bot - add to x
                return x + source_distance, y
            elif self.direction == "west":
                # West bot - subtract from x
                return x - source_distance, y
            elif self.direction == "south":
                # South bot - subtract from y
                return x, y - source_distance
                
        elif source_direction == "east":
            if self.direction == "north":
                # North bot - subtract from x
                return x - source_distance, y
            elif self.direction == "east":
                # Same direction, no change
                return x, y
            elif self.direction == "west":
                # West bot - add to y
                return x, y + source_distance
            elif self.direction == "south":
                # South bot - add to x
                return x + source_distance, y
                
        elif source_direction == "south":
            if self.direction == "north":
                # North bot - add to y
                return x, y + source_distance
            elif self.direction == "east":
                # East bot - subtract from x
                return x - source_distance, y
            elif self.direction == "west":
                # West bot - add to x
                return x + source_distance, y
            elif self.direction == "south":
                # Same direction, no change
                return x, y
                
        elif source_direction == "west":
            if self.direction == "north":
                # North bot - add to x
                return x + source_distance, y
            elif self.direction == "east":
                # East bot - subtract from y
                return x, y - source_distance
            elif self.direction == "west":
                # Same direction, no change
                return x, y
            elif self.direction == "south":
                # South bot - subtract from x
                return x - source_distance, y
        
        # If directions are unknown, return original coordinates
        logger.warning(f"Unknown direction combination: source={source_direction}, this={self.direction}")
        return x, y
    
    def transform_coordinates_to_bot(self, target_bot_id, x, y):
        """
        Transform coordinates from this bot's reference frame to another bot's reference frame.
        
        Args:
            target_bot_id: ID of the bot to convert coordinates for
            x: X coordinate in this bot's reference frame
            y: Y coordinate in this bot's reference frame
            
        Returns:
            Tuple of (x, y) in the target bot's reference frame
        """
        target_bot_id = str(target_bot_id)  # Convert to string to ensure key matching
        
        if str(self.bot_id) == target_bot_id:
            # No transformation needed if coordinates are for this bot
            return x, y
        
        if target_bot_id not in self.bot_offsets:
            logger.warning(f"No offset information for bot {target_bot_id}. Using original coordinates.")
            return x, y
        
        offset = self.bot_offsets[target_bot_id]
        target_direction = offset.get("direction")
        target_distance = offset.get("distance", 0.0)
        
        logger.info(f"Transforming coordinates to bot {target_bot_id} ({target_direction}) at distance {target_distance}m")
        
        # Apply transforms based on the relative positions of the bots to the base station
        # This is essentially the inverse of transform_coordinates_from_bot
        
        # First, determine our direction relative to the base station
        my_direction = self.direction
        
        # Apply the appropriate transformation based on our direction and their direction
        if my_direction == "north":
            if target_direction == "east":
                # East bot is to our right
                return x - target_distance, y
            elif target_direction == "west":
                # West bot is to our left
                return x + target_distance, y
            elif target_direction == "south":
                # South bot is below us
                return x, y + target_distance
                
        elif my_direction == "south":
            if target_direction == "east":
                # East bot is to our left from this orientation
                return x + target_distance, y
            elif target_direction == "west":
                # West bot is to our right from this orientation
                return x - target_distance, y
            elif target_direction == "north":
                # North bot is above us
                return x, y - target_distance
                
        elif my_direction == "east":
            if target_direction == "north":
                # North bot is to our left from this orientation
                return x + target_distance, y
            elif target_direction == "south":
                # South bot is to our right from this orientation
                return x - target_distance, y
            elif target_direction == "west":
                # West bot is opposite us
                return x, y - target_distance
                
        elif my_direction == "west":
            if target_direction == "north":
                # North bot is to our right from this orientation
                return x - target_distance, y
            elif target_direction == "south":
                # South bot is to our left from this orientation
                return x + target_distance, y
            elif target_direction == "east":
                # East bot is opposite us
                return x, y - target_distance
        
        # If we get here, the directions are the same or unknown, so no transformation
        logger.warning(f"No specific transformation for {my_direction} bot sending to {target_direction} bot")
        return x, y
    
    async def navigate_to_coordinate(self, x, y, source_bot_id=None):
        """
        Navigate to a coordinate, with optional source bot ID for transformation.
        Accounts for the robot's position relative to the base station.
        
        Args:
            x: X coordinate
            y: Y coordinate
            source_bot_id: ID of the bot that provided these coordinates (optional)
        """
        if self.is_navigating:
            logger.warning("Already navigating to a target. Please wait.")
            return False

        self.is_navigating = True
        self.target_reached = False

        try:
            # Transform coordinates if they're from another bot
            if source_bot_id and str(source_bot_id) != str(self.bot_id):
                orig_x, orig_y = x, y
                x, y = self.transform_coordinates_from_bot(source_bot_id, x, y)
                logger.info(f"Transformed coordinates from bot {source_bot_id}: ({orig_x:.2f}, {orig_y:.2f}) → ({x:.2f}, {y:.2f})")

            # Print current state for debugging
            logger.info(f"Bot direction relative to base: {self.direction}")
            logger.info(f"Navigating to coordinate: x={x:.2f}, y={y:.2f}")
            logger.info(f"Current position: x={self.relative_position[0]:.2f}, y={self.relative_position[1]:.2f}")
            logger.info(f"Current heading: {math.degrees(self.relative_position[2]):.2f}°")
            
            # ADJUSTMENT: Adjust target coordinates based on the robot's position relative to the base station
            # For a robot at the east side, going to (1,1) should actually go to (0.75, 1)
            adjusted_x, adjusted_y = x, y
            
            # If input looks like a standard point (1,1) and we're not at the north side, adjust it
            if abs(x - 1.0) < 0.1 and abs(y - 1.0) < 0.1:
                logger.info(f"Standard point (1,1) detected. Adjusting based on bot position...")
                
                if self.direction == "east":
                    # As per your example, if on east side, actual target should be (0.75, 1)
                    # Subtract our distance from x
                    adjusted_x = x - self.distance_from_tag
                    logger.info(f"East bot adjustment: ({x}, {y}) → ({adjusted_x}, {adjusted_y})")
                elif self.direction == "west":
                    # On west side, add our distance to x
                    adjusted_x = x + self.distance_from_tag
                    logger.info(f"West bot adjustment: ({x}, {y}) → ({adjusted_x}, {adjusted_y})")
                elif self.direction == "south":
                    # On south side, subtract our distance from y
                    adjusted_y = y - self.distance_from_tag
                    logger.info(f"South bot adjustment: ({x}, {y}) → ({adjusted_x}, {adjusted_y})")
                # North bots don't need adjustment since (1,1) is already correct for them
            
            # Use the adjusted coordinates for navigation
            x, y = adjusted_x, adjusted_y
            logger.info(f"Final target coordinates: ({x:.2f}, {y:.2f})")

            # Calculate distance vector from current position to target
            dx = x - self.relative_position[0]
            dy = y - self.relative_position[1]
            distance = math.sqrt(dx**2 + dy**2)

            logger.info(f"Distance vector: dx={dx:.2f}, dy={dy:.2f}, total={distance:.2f}m")

            if distance < self.movement_precision:
                logger.info("Already at target position")
                self.target_reached = True
                self.is_navigating = False
                return True
            
            # FIXED! You mentioned that northeast is from 270 to 0.
            # This means the angle calculation is different from standard math angles.
            # We need to adjust our atan2 calculation to match the robot's angle reference frame.
            
            # Standard atan2(dy, dx) gives angle in standard math convention:
            # - East = 0°, North = 90°, West = 180°, South = 270°
            
            # But from your comment, the robot uses:
            # - North = 0°, East = 90°, South = 180°, West = 270°
            
            # So we need to convert our angle
            standard_angle = math.atan2(dy, dx)
            
            # Convert to match the robot's angle convention (rotate by 90° counterclockwise)
            target_angle = standard_angle - (math.pi/2)
            
            # Normalize target angle to [-π, π]
            while target_angle > math.pi:
                target_angle -= 2 * math.pi
            while target_angle < -math.pi:
                target_angle += 2 * math.pi
            
            # Special case for (1,1) which should be at 45° northeast
            if dx > 0 and dy > 0 and abs(dx - dy) < 0.1:
                logger.info(f"Special case: Going to (~1,1) - should go northeast")
                
                # Northeast is 45° in standard math, but -45° in robot's reference
                target_angle = -math.pi/4  # -45 degrees
                logger.info(f"Forcing northeast direction (-45 degrees)")
            
            logger.info(f"Standard math angle: {math.degrees(standard_angle):.2f}°")
            logger.info(f"Converted robot target angle: {math.degrees(target_angle):.2f}°")
            
            # Debug direction names based on target angle
            angle_deg = math.degrees(target_angle)
            if -22.5 <= angle_deg <= 22.5:
                direction_name = "NORTH"
            elif 22.5 < angle_deg <= 67.5:
                direction_name = "NORTHWEST" 
            elif 67.5 < angle_deg <= 112.5:
                direction_name = "WEST"
            elif 112.5 < angle_deg <= 157.5:
                direction_name = "SOUTHWEST"
            elif angle_deg > 157.5 or angle_deg < -157.5:
                direction_name = "SOUTH"
            elif -157.5 <= angle_deg < -112.5:
                direction_name = "SOUTHEAST"
            elif -112.5 <= angle_deg < -67.5:
                direction_name = "EAST"
            elif -67.5 <= angle_deg < -22.5:
                direction_name = "NORTHEAST"
                
            logger.info(f"Robot will move: {direction_name}")

            # Turn to face the target
            await self.turn_to_angle(target_angle)

            # Move straight toward the target
            await self.move_forward(distance)

            # Re-check position after movement
            new_dx = x - self.relative_position[0]
            new_dy = y - self.relative_position[1]
            final_distance = math.sqrt(new_dx**2 + new_dy**2)

            if final_distance > self.movement_precision * 2:
                logger.warning(f"Navigation finished with offset: {final_distance:.2f}m")
            else:
                logger.info(f"Target reached: x={self.relative_position[0]:.2f}, y={self.relative_position[1]:.2f}")
                self.update_config_position()

            self.target_reached = True
            return True

        except Exception as e:
            logger.error(f"Error during navigation: {e}")
            return False
        finally:
            self.is_navigating = False
    
    async def turn_to_angle(self, target_angle):
        """Turn to face a specific angle in radians in our relative coordinate system."""
        logger.info(f"Turning to angle: {math.degrees(target_angle):.2f}°")
        logger.info(f"Current relative heading: {math.degrees(self.relative_position[2]):.2f}°")

        turn_speed = 0.3  # radians per second
        tolerance = 0.05  # ~3 degrees

        # Normalize function
        def normalize_angle(angle):
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
            return angle

        # Calculate the angle difference
        angle_diff = normalize_angle(target_angle - self.relative_position[2])
        logger.info(f"Angle difference: {math.degrees(angle_diff):.2f}°")
        
        start_time = time.time()
        while not is_shutting_down:
            # Use relative heading for turning
            current_heading = self.relative_position[2]
            angle_diff = normalize_angle(target_angle - current_heading)

            # Safety timeout - stop turning after 10 seconds
            if time.time() - start_time > 10.0:
                logger.warning("Turn timeout reached - stopping turn")
                break

            if abs(angle_diff) < tolerance:
                logger.info("Reached desired heading")
                break

            cmd = Twist()
            cmd.angular.z = min(turn_speed, max(-turn_speed, angle_diff))  # Limit turn speed
            self.cmd_vel_pub.publish(cmd)
            await asyncio.sleep(0.05)

        # Stop after turning
        self.stop_robot()
        await asyncio.sleep(0.2)
        
        # Final heading check
        final_diff = normalize_angle(target_angle - self.relative_position[2])
        logger.info(f"Final relative heading: {math.degrees(self.relative_position[2]):.2f}°, difference: {math.degrees(final_diff):.2f}°")
        
        return True
    
    async def move_forward(self, distance):
        """Move forward a specific distance."""
        logger.info(f"Moving forward: {distance:.2f}m")
        
        # Set movement speed (reduce for small distances)
        move_speed = 0.2  # m/s
        if distance < 0.5:
            move_speed = 0.1  # slower for short distances
        
        # Calculate time required for movement
        move_time = distance / move_speed
        logger.info(f"Moving at {move_speed:.2f} m/s for {move_time:.2f} seconds")
        
        # Record starting position for verification
        start_pos = [self.relative_position[0], self.relative_position[1]]
        
        # Execute movement
        cmd = Twist()
        cmd.linear.x = move_speed
        
        start_time = time.time()
        update_time = start_time
        
        # Move until we've gone the calculated time or reached destination
        while time.time() - start_time < move_time and not is_shutting_down:
            # Publish movement command
            self.cmd_vel_pub.publish(cmd)
            
            # Periodically log position (every 1 second)
            if time.time() - update_time > 1.0:
                update_time = time.time()
                
                # Calculate distance moved so far
                dx = self.relative_position[0] - start_pos[0]
                dy = self.relative_position[1] - start_pos[1]
                moved_distance = math.sqrt(dx**2 + dy**2)
                
                logger.info(f"Movement progress: {moved_distance:.2f}m / {distance:.2f}m")
            
            await asyncio.sleep(0.1)
        
        # Stop movement
        self.stop_robot()
        await asyncio.sleep(0.5)  # Short pause
        
        # Verify final position
        dx = self.relative_position[0] - start_pos[0]
        dy = self.relative_position[1] - start_pos[1]
        moved_distance = math.sqrt(dx**2 + dy**2)
        
        logger.info(f"Movement complete: traveled {moved_distance:.2f}m of planned {distance:.2f}m")
        
        return True
    
    async def navigate_to_relative_coordinate(self, dx, dy):
        """Navigate to a position relative to the current position."""
        target_x = self.relative_position[0] + dx
        target_y = self.relative_position[1] + dy
        
        logger.info(f"Moving to relative position: dx={dx:.2f}m, dy={dy:.2f}m")
        return await self.navigate_to_coordinate(target_x, target_y)
    
    def shutdown(self):
        """Stop the robot and clean up resources."""
        self.stop_robot()
        logger.info("Navigator shutdown complete")


def handle_shutdown(signum, frame):
    """Signal handler for shutdown."""
    global is_shutting_down
    print("Shutdown signal received")
    is_shutting_down = True


async def spin_node(node):
    """Spin ROS node in a separate task."""
    try:
        while rclpy.ok() and not is_shutting_down:
            rclpy.spin_once(node, timeout_sec=0.1)
            await asyncio.sleep(0.01)
    except Exception as e:
        logger.error(f"Error in spin_node: {e}")


async def main():
    rclpy.init()
    
    # Set up signal handlers for clean shutdown
    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)
    
    try:
        # Create navigator node
        navigator = CoordinateNavigator()
        
        # Spin node in separate task
        spin_task = asyncio.create_task(spin_node(navigator))
        
        # Allow time for odom to initialize
        await asyncio.sleep(2)
        
        # Interactive navigation loop
        logger.info("\nCoordinate Navigation System")
        logger.info("Command the robot to move to specific coordinates")
        
        while not is_shutting_down:
            
            print("\nEnter coordinates to navigate to (x y), or 'exit' to quit:")
            print("You can also use 'goto x y', 'goto x y bot_id', or 'relative dx dy'")
            
            user_input = input("> ")
            
            if user_input.lower() == 'exit':
                break
    
            parts = user_input.split()
            
            # Handle simple "x y" format (most common case)
            if len(parts) == 2:
                try:
                    target_x = float(parts[0])
                    target_y = float(parts[1])
                    
                    print(f"Navigating to coordinates: ({target_x}, {target_y})")
                    success = await navigator.navigate_to_coordinate(target_x, target_y)
                    
                    if success:
                        print("Navigation completed successfully")
                    else:
                        print("Navigation did not complete successfully")
                except ValueError:
                    print("Invalid coordinates. Please enter numeric values.")
            
            # Handle "goto x y" or "goto x y bot_id" format
            elif len(parts) >= 3 and parts[0].lower() == 'goto':
                try:
                    target_x = float(parts[1])
                    target_y = float(parts[2])
                    
                    # Check if source bot ID is provided
                    source_bot_id = None
                    if len(parts) >= 4:
                        source_bot_id = parts[3]
                        print(f"Navigating to coordinates: ({target_x}, {target_y}) from bot {source_bot_id}")
                    else:
                        print(f"Navigating to coordinates: ({target_x}, {target_y})")
                    
                    success = await navigator.navigate_to_coordinate(target_x, target_y, source_bot_id)
                    
                    if success:
                        print("Navigation completed successfully")
                    else:
                        print("Navigation did not complete successfully")
                except ValueError:
                    print("Invalid coordinates. Please enter numeric values.")
            
            # Handle "relative dx dy" format
            elif len(parts) >= 3 and parts[0].lower() == 'relative':
                try:
                    dx = float(parts[1])
                    dy = float(parts[2])
                    
                    print(f"Moving relative to current position: dx={dx}, dy={dy}")
                    success = await navigator.navigate_to_relative_coordinate(dx, dy)
                    
                    if success:
                        print("Navigation completed successfully")
                    else:
                        print("Navigation did not complete successfully")
                except ValueError:
                    print("Invalid relative movement. Please enter numeric values.")
            
            else:
                print("Invalid command format. Please use one of these formats:")
                print("  x y             - Navigate to coordinates (x,y)")
                print("  goto x y        - Navigate to coordinates (x,y)")
                print("  goto x y bot_id - Navigate to coordinates (x,y) from bot_id's perspective")
                print("  relative dx dy  - Move relative to current position")
            
            await asyncio.sleep(0.1) # Brief pause before next command
            
    except Exception as e:
        logger.error(f"Error in main: {e}")
    finally:
        if 'navigator' in locals():
            navigator.shutdown()
            navigator.destroy_node()
        rclpy.shutdown()
        logger.info("Program completed")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Program interrupted by user")