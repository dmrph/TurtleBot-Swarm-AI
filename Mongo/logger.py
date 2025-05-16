#!/usr/bin/env python3
"""
MongoDB Logger for TurtleBot Mission

This module implements a ROS2 node for TurtleBot robots in an autonomous
mission. It logs sensor data, position, health status, and system information
to a central MongoDB database.

Mission Requirements:
- Collect all charms in the environment
- Navigate using LiDAR
- Track charm and obstacle locations
- Use AprilTags for identification
- Log all activity to MongoDB
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from pymongo import MongoClient
from bson.objectid import ObjectId
from datetime import datetime, timezone, timedelta
import json
import math
import socket
import psutil
import subprocess
from threading import Lock
import argparse
import os

# ROS message types
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, BatteryState, Range 
from std_msgs.msg import String, Bool, Float32


class BotLogger(Node):
    """
    ROS2 node for logging TurtleBot data in a charm collection mission.
    
    This class handles sensor data collection, health monitoring, and database storage.
    It tracks the position of robots and monitors system health.
    
    Attributes:
        config (dict): Configuration settings loaded from JSON
        bot_id (str): Unique identifier for this robot
        mongo_connected (bool): Current MongoDB connection status
        client (MongoClient): MongoDB client connection
        db (Database): MongoDB database instance
        sensor_collection (Collection): MongoDB collection for sensor data
        health_collection (Collection): MongoDB collection for health data
        mission_collection (Collection): MongoDB collection for mission data
        data_lock (Lock): Thread lock for safe data access
        location (dict): Current robot position (x, y, orientation)
        lidar_data (dict): Processed LiDAR readings
        ultrasonic_data (dict) : Processed Sonar data
        odom_data (dict): Display odometry data of bot
        
        health_data (dict): Robot system health information
        mission_data (dict): Mission data
        sensor_log_timer (Timer): Timer for periodic sensor logging
        health_check_timer (Timer): Timer for periodic health checks
        mission_update_timer (Timer): Timer for periodic mission updates
        maintenance_timer (Timer): Timer for database cleanup

    """
    
    def __init__(self, config_path='config.json'):
        """
        Initialize the BotLogger node with configuration and set up all components.
        
        Args:
            config_path (str): Path to the JSON configuration file
            
        Raises:
            FileNotFoundError: If configuration file cannot be read (handled with defaults)
            ConnectionError: If MongoDB connection fails (handled with status flag)
        """
        # Load configuration
        self.config = self.load_config(config_path)
        
        
        mongo_uri = "mongodb://127.0.0.1:27017/"
        db_name = "mission_db"


        # Extract configuration values
        self.bot_id = self.config["bot"]["id"]
        mongo_uri = self.config["mongodb"]["uri"]
        db_name = self.config["mongodb"]["database"]
        
        # Initialize the ROS2 node
        node_name = f"{self.config['ros2']['node_name_prefix']}_{self.bot_id}"
        super().__init__(node_name)
        
        # MongoDB setup and connection check
        self.mongo_connected = False
        try:
            self.client = MongoClient(mongo_uri, serverSelectionTimeoutMS=2000)
            # Test the connection
            self.client.server_info()
            self.db = self.client[db_name]
            
            # Set up collections
            self.sensor_collection = self.db[self.config["mongodb"]["sensor_collection"]]
            self.health_collection = self.db[self.config["mongodb"]["health_collection"]]
            self.mission_collection = self.db[self.config["mongodb"]["mission_collection"]]
            
            self.mongo_connected = True
            self.get_logger().info("MongoDB connection successful")
        except Exception as e:
            self.get_logger().error(f"MongoDB connection failed: {e}")
        
        # Data storage with thread safety
        self.data_lock = Lock()
        
        # Sensor and position data
        self.location = {"x": 0.0, "y": 0.0, "orientation": 0.0}
        self.lidar_data = {}
        
        # Health monitoring data
        self.health_data = {
            "battery": {
                "percentage": None,
                "voltage": None,
                "status": "UNKNOWN"
            },
            "network": {
                "interface": None,
                "ip_address": None,
                "signal_strength": None
            },
            "errors": [],
            "ros_topics": {},
            "sensors": {}
        }
        
        # Mission data
        self.mission_data = {
            "charms_collected": [],
            "charms_detected": [],
            "assigned_area": None
        }
        
        # Set up QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to sensor topics
        # Add the rest of the sensors here
        self.create_subscription(
            Odometry,
            self.config["ros2"]["topics"]["odometry"],
            self.odom_callback,
            10
        )
        
        if self.config["sensors"]["lidar"]["enabled"]:
            self.create_subscription(
                LaserScan,
                self.config["ros2"]["topics"]["lidar"],
                self.lidar_callback,
                sensor_qos
            )
            
            # Initialize lidar status
            self.health_data["sensors"]["lidar"] = {
                "status": "OFFLINE",
                "valid_readings": 0,
                "total_readings": 0
            }
        
        # Add the ultrasonic sensor subscription
        if self.config["sensors"]["ultrasonic"]["enabled"]:
            self.create_subscription(
                Range,
                self.config["ros2"]["topics"].get("ultrasonic", "/ultrasonic"),
                self.ultrasonic_callback,
                10
            )
            
            # Initialize ultrasonic status in health data
            self.health_data["sensors"]["ultrasonic"] = {
                "status": "OFFLINE",
                "distance": None,
                "timestamp": None,
                "readings_count": 0 
            }

            self.ultrasonic_data = {
                "distance": None,
                "timestamp": None,
                "status": "OFFLINE"
            }

        # Battery monitoring if enabled
        if self.config["sensors"]["battery"]["enabled"]:
            self.create_subscription(
                BatteryState,
                self.config["ros2"]["topics"]["battery"],
                self.battery_callback,
                10
            )
        
        # Create timers for periodic operations
        self.sensor_log_timer = self.create_timer(
            self.config["logging"]["sensor_interval_seconds"],
            self.log_sensor_data
        )
        
        self.health_check_timer = self.create_timer(
            self.config["logging"]["health_interval_seconds"],
            self.perform_health_check
        )
        
        self.mission_update_timer = self.create_timer(
            self.config["logging"]["mission_interval_seconds"],
            self.update_mission_status
        )
        
        # Maintenance timer to clean old logs
        self.maintenance_timer = self.create_timer(
            3600.0,  # Once per hour
            self.cleanup_old_logs
        )
        
        # Initialize topics list for health monitoring
        for topic in [self.config["ros2"]["topics"]["odometry"],
                      self.config["ros2"]["topics"]["lidar"],
                      self.config["ros2"]["topics"]["battery"]]:
            self.health_data["ros_topics"][topic] = {
                "status": "OFFLINE",
                "last_message": None
            }
        
        self.get_logger().info(f"Bot MongoDB Logger initialized for Bot {self.bot_id}")
    
    def load_config(self, config_path):
        """
        Load configuration from a JSON file with fallback to default values.
        
        Args:
            config_path (str): Path to the configuration file
            
        Returns:
            dict: Configuration dictionary with all required settings
            
        Raises:
            FileNotFoundError: If the configuration file cannot be found (handled internally)
            json.JSONDecodeError: If the configuration file has invalid JSON (handled internally)
        """
        try:
            with open(config_path, 'r') as file:
                config = json.load(file)
                return config
        except Exception as e:
            self.get_logger().error(f"Error loading configuration: {e}")
            self.get_logger().info("Using default configuration")
            
            # Default configuration
            return {
                "bot": {"id": "1"},
                "ros2": {
                    "node_name_prefix": "bot_logger",
                    "topics": {
                        "odometry": "/odom",
                        "lidar": "/scan",
                        "ultrasonic": "/ultrasonic",
                        "battery": "/battery_state"
                    }
                },
                "mongodb": {
                    "uri": "mongodb://localhost:27017/",
                    "database": "mission_db",
                    "sensor_collection": "sensor_logs",
                    "health_collection": "health_logs",
                    "mission_collection": "mission_data"
                },
                "logging": {
                    "sensor_interval_seconds": 1.0,
                    "health_interval_seconds": 5.0,
                    "mission_interval_seconds": 2.0,
                    "retention_hours": 24
                },
                "sensors": {
                    "camera": {"enabled": False, "type": "YOLO"},
                    "lidar": {"enabled": True, "sample_rate": 10},
                    "ultrasonic": {"enabled": True},
                    "battery": {"enabled": True}
                }
            }
    
    def quaternion_to_yaw(self, x, y, z, w):
        """
        Convert quaternion orientation to yaw angle in degrees.
        
        Args:
            x (float): X component of quaternion
            y (float): Y component of quaternion
            z (float): Z component of quaternion
            w (float): W component of quaternion
            
        Returns:
            float: Yaw angle in degrees (0-360)
        """
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return math.degrees(yaw) % 360
    
    def odom_callback(self, msg):
        """
        Process odometry data to update robot position.
        
        Args:
            msg (Odometry): ROS Odometry message with position and orientation
        """
        with self.data_lock:
            # Update topic status in health data
            self.update_topic_status('/odom')
            
            # Extract position
            self.location["x"] = msg.pose.pose.position.x
            self.location["y"] = msg.pose.pose.position.y
            
            # Extract orientation (convert quaternion to degrees)
            quaternion = (
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            )
            self.location["orientation"] = self.quaternion_to_yaw(
                quaternion[0], quaternion[1], quaternion[2], quaternion[3]
            )
    
    def lidar_callback(self, msg):
        """
        Process LiDAR scan data for obstacle detection and navigation.
        
        Args:
            msg (LaserScan): ROS LaserScan message with distance readings
            
        Raises:
            Exception: If processing fails (caught internally)
        """
        with self.data_lock:
            # Update topic status in health data
            self.update_topic_status('/scan')
            
            try:
                # Get configured sample rate
                sample_rate = self.config["sensors"]["lidar"]["sample_rate"]
                
                # Create a list of valid readings (non-NaN values) with their angles
                readings = []
                valid_count = 0
                for i in range(0, len(msg.ranges), sample_rate):
                    if i < len(msg.ranges):
                        # Check if value is valid (not NaN and within range)
                        range_val = msg.ranges[i]
                        if not math.isnan(range_val) and range_val > 0:
                            valid_count += 1
                            angle = msg.angle_min + (i * msg.angle_increment)
                            # Convert to degrees for easier readability
                            angle_degrees = math.degrees(angle) % 360
                            
                            # Get intensity if available
                            intensity = None
                            if i < len(msg.intensities):
                                intensity = float(msg.intensities[i]) if not math.isnan(msg.intensities[i]) else None
                            
                            readings.append({
                                "angle": angle_degrees,
                                "distance": float(range_val),
                                "intensity": intensity
                            })
                
                # Store scan metadata and readings
                self.lidar_data = {
                    "frame_id": msg.header.frame_id,
                    "timestamp": msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9),
                    "angle_min_degrees": math.degrees(float(msg.angle_min)),
                    "angle_max_degrees": math.degrees(float(msg.angle_max)),
                    "sample_rate": sample_rate,
                    "readings": readings
                }
                
                # Update lidar health status
                self.health_data["sensors"]["lidar"] = {
                    "status": "ONLINE" if valid_count > 0 else "ERROR",
                    "valid_readings": valid_count,
                    "total_readings": len(msg.ranges)
                }
                
                if valid_count == 0:
                    self.log_error("LiDAR reporting no valid readings")
                
            except Exception as e:
                self.get_logger().error(f"Error processing lidar scan: {e}")
                self.log_error(f"LiDAR processing error: {str(e)}")
                self.health_data["sensors"]["lidar"]["status"] = "ERROR"
    
    def ultrasonic_callback(self, msg):
        #
        with self.data_lock:
            # Update topic status in health data
            topic_name = self.config["ros2"]["topics"].get("ultrasonic", "/ultrasonic")
            self.update_topic_status(topic_name)
            
            # Update ultrasonic sensor status to ONLINE
            self.health_data["sensors"]["ultrasonic"] = {
                "status": "ONLINE",
                "distance": msg.range,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "readings_count": self.health_data["sensors"]["ultrasonic"].get("readings_count", 0) +1
            }

                # Update sensor data storage
            self.ultrasonic_data = {
                "distance": msg.range,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "status": "ONLINE"
            }
    def battery_callback(self, msg):
        """
        Process battery state data to monitor power levels.
        
        Args:
            msg (BatteryState): ROS BatteryState message with percentage and voltage
        """
        with self.data_lock:
            # Update topic status in health data
            self.update_topic_status('/battery_state')
            
            # Process battery data
            self.health_data["battery"]["percentage"] = msg.percentage * 100 if msg.percentage <= 1.0 else msg.percentage
            self.health_data["battery"]["voltage"] = msg.voltage
            
            # Set battery status based on percentage
            if msg.percentage < 0.15:  # Less than 15%
                self.health_data["battery"]["status"] = "CRITICAL"
            elif msg.percentage < 0.25:  # Less than 25%
                self.health_data["battery"]["status"] = "LOW"
            else:
                self.health_data["battery"]["status"] = "NORMAL"
    
    # YOLO detection functionality is commented out for future implementation
    # """
    # def yolo_callback(self, msg):
    #     # Process YOLO object detection data
    #     with self.data_lock:
    #         # Update topic status in health data
    #         self.update_topic_status('/detection/yolo')
            
    #         try:
    #             yolo_output = msg.data
    #             # Parse the YOLO output
    #             match = re.search(r'0: (\d+x\d+) (.*?), \d+\.\d+ms', yolo_output)
    #             if not match:
    #                 return
                
    #             resolution = match.group(1)
    #             detections_text = match.group(2)
                
    #             # Parse individual detections
    #             detections = []
    #             for item in detections_text.split(', '):
    #                 # Split count and class
    #                 parts = item.strip().split(' ', 1)
    #                 if len(parts) == 2:
    #                     count = int(parts[0])
    #                     class_name = parts[1]
    #                     detections.append({
    #                         "class": class_name,
    #                         "count": count
    #                     })
                
    #             # Extract timing information
    #             timing_match = re.search(r'Speed: (\d+\.\d+)ms preprocess, (\d+\.\d+)ms inference, (\d+\.\d+)ms postprocess', yolo_output)
    #             timing = {}
    #             if timing_match:
    #                 timing = {
    #                     "preprocess_ms": float(timing_match.group(1)),
    #                     "inference_ms": float(timing_match.group(2)),
    #                     "postprocess_ms": float(timing_match.group(3))
    #                 }
                
    #             # Update the stored YOLO data
    #             self.yolo_detections = {
    #                 "timestamp": datetime.now(timezone.utc).isoformat(),
    #                 "resolution": resolution,
    #                 "detections": detections,
    #                 "timing": timing
    #             }
                
    #             # Process detections to identify charms
    #             self.process_charm_detections(detections)
                
    #             self.last_yolo_update = datetime.now(timezone.utc)
    #         except Exception as e:
    #             self.get_logger().error(f"Error parsing YOLO output: {e}")
    #             self.log_error(f"YOLO detection error: {str(e)}")
    
    # def process_charm_detections(self, detections):
    #     # Process YOLO detections to identify and track charms
    #     for detection in detections:
    #         if "charm" in detection["class"].lower() or "tag" in detection["class"].lower():
    #             # Record the charm detection in mission data
    #             charm_info = {
    #                 "type": detection["class"],
    #                 "count": detection["count"],
    #                 "location": {
    #                     "x": self.location["x"],
    #                     "y": self.location["y"]
    #                 },
    #                 "timestamp": datetime.now(timezone.utc).isoformat(),
    #                 "collected": False
    #             }
    #             # Add to detected charms if not already collected
    #             if not any(c.get("type") == detection["class"] for c in self.mission_data["charms_collected"]):
    #                 self.mission_data["charms_detected"].append(charm_info)
    #                 self.get_logger().info(f"Detected charm: {detection['class']}")
    # """
    
    def update_topic_status(self, topic_name):
        """
        Update the status of a ROS topic in the health monitoring data.
        
        Args:
            topic_name (str): Name of the ROS topic
        """
        self.health_data["ros_topics"][topic_name] = {
            "status": "ONLINE",
            "last_message": datetime.now(timezone.utc).isoformat()
        }
    
    def perform_health_check(self):
        """
        Perform various health checks on the system and update status.
        
        Raises:
            Exception: If health check fails (caught internally)
        """
        try:
            # Check for topics that haven't received messages recently
            current_time = datetime.now(timezone.utc)
            with self.data_lock:
                for topic, status in self.health_data["ros_topics"].items():
                    if status["last_message"]:
                        last_msg_time = datetime.fromisoformat(status["last_message"].replace('Z', '+00:00'))
                        # If no message in 10 seconds, mark as stale
                        if (current_time - last_msg_time).total_seconds() > 10:
                            self.health_data["ros_topics"][topic]["status"] = "STALE"
                            self.log_error(f"Topic {topic} has stale data")
            
            # Check database connection
            try:
                if not self.mongo_connected:
                    self.client.server_info()
                    self.mongo_connected = True
                    self.get_logger().info("MongoDB connection restored")
            except Exception:
                if self.mongo_connected:
                    self.mongo_connected = False
                    self.log_error("MongoDB connection lost")
            
            # Update network information
            self.update_network_info()
            
            # Log health data
            self.log_health_data()
            
        except Exception as e:
            self.get_logger().error(f"Error during health check: {e}")
            self.log_error(f"Health check failed: {str(e)}")
    
    def update_network_info(self):
        """
        Get information about the current network connection.
        
        Raises:
            Exception: If network information retrieval fails (caught internally)
        """
        try:
            # Try to get the default network interface
            with self.data_lock:
                # Find the active network interface
                for interface, addrs in psutil.net_if_addrs().items():
                    if interface != 'lo':  # Skip loopback
                        for addr in addrs:
                            if addr.family == socket.AF_INET:  # IPv4
                                self.health_data["network"]["interface"] = interface
                                self.health_data["network"]["ip_address"] = addr.address
                                
                                # Try to get WiFi signal strength if it's a wireless interface
                                try:
                                    # Use iwconfig to get wireless info
                                    if interface.startswith(('wl', 'wlan')):
                                        result = subprocess.run(['iwconfig', interface], 
                                                                capture_output=True, text=True)
                                        if "Signal level" in result.stdout:
                                            signal_parts = result.stdout.split("Signal level=")[1].split()[0]
                                            self.health_data["network"]["signal_strength"] = signal_parts
                                except:
                                    pass
        except Exception as e:
            self.get_logger().error(f"Error getting network info: {e}")
    
    def log_error(self, error_message):
        """
        Log an error with timestamp to the health data and ROS logger.
        
        Args:
            error_message (str): Error message to log
        """
        with self.data_lock:
            self.health_data["errors"].append({
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "message": error_message
            })
            # Keep only the last 10 errors
            if len(self.health_data["errors"]) > 10:
                self.health_data["errors"] = self.health_data["errors"][-10:]
        self.get_logger().error(error_message)
    
    def log_sensor_data(self):
        """
        Log sensor data to MongoDB for tracking and analysis.
        
        Raises:
            Exception: If logging fails (caught internally)
        """
        with self.data_lock:
            # Create log entry
            log_data = {
                "botId": self.bot_id,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "location": {
                    "x": self.location["x"],
                    "y": self.location["y"],
                    "orientation": self.location["orientation"]
                },
                "sensors": {}
            }
            
            # Add lidar data if available
            if self.config["sensors"]["lidar"]["enabled"] and self.lidar_data:
                log_data["sensors"]["lidar"] = self.lidar_data
            
            #Add ultrasonic data
            
            if self.config["sensors"]["ultrasonic"]["enabled"] and self.ultrasonic_data:
                log_data["sensors"]["ultrasonic"] = {
                    "distance": self.ultrasonic_data.get("distance"),
                    "status": self.ultrasonic_data.get("status", "UNKNOWN"),
                    "timestamp": self.ultrasonic_data.get("timestamp")
                }
            
            # Code for logging YOLO detections would go here if implemented
        
        try:
            # Insert into MongoDB if connected
            if self.mongo_connected:
                result = self.sensor_collection.insert_one(log_data)
                self.get_logger().info(f"Logged sensor data with ID: {result.inserted_id}")
            else:
                self.get_logger().warning("Could not log sensor data: MongoDB disconnected")
        except Exception as e:
            self.get_logger().error(f"Error logging sensor data: {e}")
    
    def log_health_data(self):
        """
        Log health status to MongoDB for system monitoring.
        
        Raises:
            Exception: If logging fails (caught internally)
        """
        with self.data_lock:
            # Create health log entry
            health_log = {
                "botId": self.bot_id,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "battery": self.health_data["battery"],
                "network": self.health_data["network"],
                "topics": {k: v["status"] for k, v in self.health_data["ros_topics"].items()},
                "sensors": {k: v["status"] for k, v in self.health_data["sensors"].items() if "status" in v},
                "errors": self.health_data["errors"],
                "database": "CONNECTED" if self.mongo_connected else "DISCONNECTED"
            }
        
        try:
            # Insert into MongoDB if connected
            if self.mongo_connected:
                result = self.health_collection.insert_one(health_log)
                self.get_logger().debug(f"Logged health data with ID: {result.inserted_id}")
            else:
                self.get_logger().warning("Could not log health data: MongoDB disconnected")
        except Exception as e:
            self.get_logger().error(f"Error logging health data: {e}")
    
    def update_mission_status(self):
        """
        Update and log mission status information.
        
        Raises:
            Exception: If status update fails (caught internally)
        """
        with self.data_lock:
            # Create mission status log entry
            mission_log = {
                "botId": self.bot_id,
                "timestamp": datetime.now(timezone.utc).isoformat(),
                "location": {
                    "x": self.location["x"],
                    "y": self.location["y"]
                },
                "charms_collected": self.mission_data["charms_collected"],
                "charms_detected": self.mission_data["charms_detected"],
                "assigned_area": self.mission_data["assigned_area"]
            }
        
        try:
            # Insert into MongoDB if connected
            if self.mongo_connected:
                result = self.mission_collection.insert_one(mission_log)
                self.get_logger().debug(f"Updated mission status with ID: {result.inserted_id}")
            else:
                self.get_logger().warning("Could not update mission status: MongoDB disconnected")
        except Exception as e:
            self.get_logger().error(f"Error updating mission status: {e}")
    
    def cleanup_old_logs(self):
        """
        Delete logs older than the retention period.
        
        Raises:
            Exception: If cleanup fails (caught internally)
        """
        if not self.mongo_connected:
            return
            
        # Calculate the cutoff time
        retention_hours = self.config["logging"]["retention_hours"]
        cutoff_time = datetime.now(timezone.utc) - timedelta(hours=retention_hours)
        cutoff_str = cutoff_time.isoformat()
        
        # Delete old logs from all collections
        try:
            # Sensor logs
            result1 = self.sensor_collection.delete_many(
                {"botId": self.bot_id, "timestamp": {"$lt": cutoff_str}}
            )
            
            # Health logs
            result2 = self.health_collection.delete_many(
                {"botId": self.bot_id, "timestamp": {"$lt": cutoff_str}}
            )
            
            # Mission logs
            result3 = self.mission_collection.delete_many(
                {"botId": self.bot_id, "timestamp": {"$lt": cutoff_str}}
            )
            
            total_deleted = result1.deleted_count + result2.deleted_count + result3.deleted_count
            if total_deleted > 0:
                self.get_logger().info(f"Cleaned up {total_deleted} logs older than {retention_hours} hours")
        except Exception as e:
            self.get_logger().error(f"Error cleaning up old logs: {e}")


def main():
    """
    Main function to initialize and run the BotLogger node.
    
    Parses command line arguments, initializes ROS2, creates the logger node,
    and handles graceful shutdown.
    """
    # Set up argument parser
    parser = argparse.ArgumentParser(description='Robot MongoDB Logger')
    parser.add_argument('--config', type=str, default='config.json', 
                        help='Path to configuration file (default: config.json)')
    parser.add_argument('--bot-id', type=str, 
                        help='Bot ID (overrides config file)')
    args = parser.parse_args()
    
    # Initialize rclpy
    rclpy.init()
    
    # Create the node with configuration
    logger = BotLogger(args.config)
    
    # Override bot_id if provided as command line argument
    if args.bot_id:
        logger.bot_id = args.bot_id
    
    print(f"Bot Logger started for TurtleBot {logger.bot_id}")
    print(f"Logging sensor data every {logger.config['logging']['sensor_interval_seconds']} seconds")
    print(f"Performing health checks every {logger.config['logging']['health_interval_seconds']} seconds")
    print(f"Updating mission status every {logger.config['logging']['mission_interval_seconds']} seconds")
    print("Press Ctrl+C to stop")
    
    try:
        # Keep the node running
        rclpy.spin(logger)
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        # Clean shutdown
        logger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
