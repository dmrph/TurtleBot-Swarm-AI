# AI Agents

This Python script implements a system of ROS2 agents for controlling a robot and managing sensor data. It includes agents for movement, lidar scanning, and data broadcasting, along with a main controller for handling inter-agent communication.

## Overview

The system consists of the following components:

-   **MoveBotAgent:** Controls the robot's movement based on commands.
-   **LidarAgent:** Processes lidar scan data to detect obstacles.
-   **BroadcastAgent:** Periodically broadcasts robot information.
-   **MainController:** Listens for and handles messages from other agents.

## Dependencies

-   `rclpy` (ROS2 Python client library)
-   `sensor_msgs` (ROS2 sensor message definitions)
-   `geometry_msgs` (ROS2 geometry message definitions)
-   `std_msgs` (ROS2 standard message definitions)
-   `asyncio` (Asynchronous I/O library)
-   `math` (Mathematical functions)
-   `time` (Time-related functions)
-   `threading` (Threading support)
-   `json` (JSON encoding and decoding)
-   `Receiver` (Custom module for receiving broadcasted info)
-   `Broadcast` (Custom module for broadcasting info)

## File Structure

-   `move_bot_agent.py` (This script)
-   `Receiver.py` (Custom module for receiving broadcasted info)
-   `Broadcast.py` (Custom module for broadcasting info)
-   `config.json` (Configuration file containing secret key, target IPs, and bot ID)

## Code Description

### MoveBotAgent

-   Initializes a ROS2 node and a publisher for the `/cmd_vel` topic.
-   Provides an `async move` method to control robot movement (forward, backward, left, right, stop) for a specified duration.
-   Publishes `Twist` messages to control the robot's linear and angular velocities.
-   Uses `asyncio.sleep` for time-based movement control.
-   Puts messages into the message queue to communicate executed commands.

### LidarAgent

-   Initializes a ROS2 node and a subscriber for the `/scan` topic.
-   Uses a QoS profile with `BEST_EFFORT` reliability for lidar data.
-   Implements a `scan_callback` method to process `LaserScan` messages.
-   Detects obstacles based on minimum distance from lidar data.
-   Logs obstacle detections and puts messages into the message queue.

### BroadcastAgent

-   Initializes a broadcasting agent with a secret key, target IPs, and bot ID.
-   Uses a separate thread to periodically broadcast robot information.
-   Broadcasts timestamp, location, and sensor data (sonar, lidar, camera).
-   Handles potential errors in the broadcast thread.

### MainController

-   Implements an `async listen_for_notifications` method to listen for messages from other agents.
-   Retrieves messages from the `message_queue` and prints them.

### Main Execution

-   Initializes ROS2.
-   Loads configuration from `config.json`.
-   Initializes `MoveBotAgent`, `BroadcastAgent`, and `MainController`.
-   Starts the `BroadcastAgent` thread.
-   Executes a sequence of movement commands using `MoveBotAgent`.
-   Shuts down ROS2 for the move bot agent.
-   Creates asynchronous tasks for `LidarAgent` and `MainController`.
-   Runs the asynchronous tasks using `asyncio.gather`.
-   Handles `KeyboardInterrupt` for graceful shutdown.

## Configuration (config.json)

The `config.json` file should contain the following structure:

```json
{
    "bot_id": 3,
    "target_ips": ["10.170.10.166", "10.170.9.14"],
    "secret_key": "488_fail",
    "apriltag_ids": [1, 2, 3, 4, 5],
    "bot_1_start_pos": [],
    "bot_2_start_pos": []
}
```

## Tests

This project includes a full test suite written using Python’s built-in `unittest` framework. These tests help make sure all agents work correctly and safely, especially when dealing with sensor data or unexpected input.

### 🔧 How to Run the Tests

Make sure you are in a **ROS2-enabled terminal** and have all required Python modules installed.

Then run:
```bash
python3 aiAgentsTests.py
```