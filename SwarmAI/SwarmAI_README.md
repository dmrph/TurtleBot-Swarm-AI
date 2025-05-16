# SwarmAI

This document describes two Python scripts: `detect.py` and `swarmTest.py`. The first script uses YOLOv8 for object detection from a camera feed, and the second script integrates ROS2 with YOLOv8 and lidar to enable a robot to wander and approach detected cones.

## detect.py

This script implements a class `YOLODetector` that uses YOLOv8 to perform real-time object detection from a camera feed.

### Class: YOLODetector

-   **`__init__(model_path, camera_index, show_fps)`:** Initializes the YOLOv8 model, camera index, and FPS display settings.
    -   `model_path`: Path to the YOLOv8 model weights.
    -   `camera_index`: Index of the camera or path to a video file.
    -   `show_fps`: Boolean to enable or disable FPS display.
-   **`setup_camera()`:** Initializes the camera capture.
-   **`run()`:** Runs the object detection loop.
    -   Captures frames from the camera.
    -   Calculates and displays FPS.
    -   Runs YOLOv8 detection on each frame.
    -   Displays the annotated frame with detection results.
    -   Handles keyboard input to exit.
-   **`cleanup()`:** Releases the camera capture and closes all windows.

### Dependencies

-   `cv2` (OpenCV)
-   `time`
-   `ultralytics` (YOLOv8)

## swarmTest.py

This script implements a ROS2 node that enables a robot to wander randomly and approach detected cones using lidar and YOLOv8.

### Global Variables

-   `is_shutting_down`: A boolean flag to indicate if the system is shutting down.
-   `cone_detected`: A boolean flag to indicate if a cone has been detected.

### Class: WanderingBot

-   **`__init__()`:** Initializes the ROS2 node and publisher for the `/cmd_vel` topic.
-   **`stop_robot()`:** Publishes stop commands to halt the robot.
-   **`move(command, duration)`:** Publishes movement commands (forward, backward, left, right, stop) for a specified duration.
-   **`wander()`:** Implements the robot's wandering behavior using random movements.
-   **`approach_cone(angle, distance)`:** Implements the robot's approach to a detected cone based on lidar data.
    -   Turns the robot to face the cone.
    -   Moves the robot towards the cone.
    -   Starts YOLOv8 detection after reaching the cone.

### Class: LidarAgent

-   **`__init__(wander_bot)`:** Initializes the ROS2 node and subscriber for the `/scan` topic.
-   **`scan_callback(msg)`:** Callback function that processes lidar scan data.
    -   Detects cones based on minimum distance.
    -   Calls `WanderingBot.approach_cone()` when a cone is detected.

### Functions

-   **`handle_shutdown(wanderbot, signum, frame)`:** Handles shutdown signals (SIGINT, SIGTERM) and stops the robot.
-   **`spin_node(node)`:** Spins a ROS2 node using an executor.
-   **`main()`:** Initializes ROS2, creates `WanderingBot` and `LidarAgent` nodes, and runs the wandering and lidar detection tasks.

### Dependencies

-   `socket`
-   `json`
-   `asyncio`
-   `math`
-   `random`
-   `rclpy`
-   `signal`
-   `sys`
-   `geometry_msgs.msg.Twist`
-   `rclpy.qos.QoSProfile`
-   `rclpy.qos.ReliabilityPolicy`
-   `sensor_msgs.msg.LaserScan`
-   `Receiver` (Custom module)
-   `detect.YOLODetector` (From `detect.py`)

### Configuration (config.json)

The `config.json` file should be located at `config.json`
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
### Usage

1.  Ensure ROS2 is installed and configured.
2.  Ensure YOLOv8 and its dependencies are installed.
3.  Place `detect.py` and `swarmTest.py` in the same directory.
4.  Run the `swarmTest.py` script: `ros2 run <package_name> swarmTest.py`

### Notes

-   The `Receiver` module is assumed to be a custom module.
-   The YOLOv8 model path in `detect.py` is hardcoded and may need adjustment.
-   The robot's movement commands and durations in `swarmTest.py` may need to be tuned based on the robot's capabilities and environment.
-   Error handling is minimal and should be expanded for production use.
-   The package name in the ros2 run command will need to be replaced with the actual name of the package containing the swarmTest.py file.
