
# AprilTags

## camera_testing.py
This Python script, `camera_testing.py`, implements a comprehensive camera agent for a robotic swarm system. It utilizes AprilTag detection and YOLO object detection to perceive the environment, locate swarm members (Turtlebots), and identify relevant objects like cones and charms.

## Dependencies

-   `cv2` (OpenCV): For camera input and image processing.
    -   Installation: `pip install opencv-python`
-   `numpy`: For numerical operations and matrix handling.
    -   Installation: `pip install numpy`
-   `asyncio`: For asynchronous programming, enabling efficient concurrent processing.
    -   (Standard library, no installation needed)
-   `logging`: For structured logging.
    -   (Standard library, no installation needed)
-   `time`: For time-related operations and performance measurement.
    -   (Standard library, no installation needed)
-   `typing`: For type hinting.
    -   (Standard library, no installation needed)
-   `apriltag`: For AprilTag detection.
    -   Installation: `pip install apriltag`
-   `ultralytics` (YOLOv8): For object detection.
    -   Installation: `pip install ultralytics`

## Class: SwarmCameraAgent

The `SwarmCameraAgent` class is the core of this script. It manages the camera feed, detects AprilTags to locate Turtlebots, identifies objects using YOLO, and provides an API for other agents to access this information.

### Configuration (SwarmConfig)

The `SwarmConfig` class defines various parameters for the camera agent, including:

-   `CAMERA_INDEX`: Index of the camera to use.
-   `MODEL_PATH`: Path to the YOLO object detection model.
-   `TURTLEBOT_TAG_MAPPING`: Mapping of AprilTag IDs to Turtlebot names.
-   `SWARM_TAG_IDS`: List of AprilTag IDs to detect.
-   `CONE_CLASS_IDS`, `CHARM_CLASS_IDS`: Class IDs for cones and charms in the YOLO model.
-   `TAG_SIZE`: Physical size of the AprilTags in meters.
-   `CAMERA_MATRIX`, `DIST_COEFFS`: Camera intrinsic parameters.
-   `DEPLOYMENT_MODE`: Toggle for deployment or testing mode.
-   `VISUALIZATION_LEVEL`: Level of visualization (0: none, 1: minimal, 2: full).
-   `VISUALIZATION_RATE`: Update visualization every N frames.
-   `TARGET_FPS`: Target frame rate.
-   `FRAME_SKIP`: Skip frames to reduce processing load.
-   `YOLO_CONF_THRESHOLD`: Confidence threshold for YOLO detections.
-   `CAMERA_WIDTH`, `CAMERA_HEIGHT`: Camera resolution.
-   `TURTLEBOT_COLORS`: Colors for visualizing different Turtlebots.
-   `ENABLE_DISTANCE_CALCULATION`: Enable distance calculation to AprilTags.
-   `LOGGING_INTERVAL`: Log detections every N frames.

### Methods

-   `_init_camera()`: Initializes the camera.
-   `_init_detection_systems()`: Initializes the YOLO object detection model.
-   `get_turtlebot_for_tag(tag_id)`: Determines which Turtlebot a tag belongs to.
-   `get_detection_by_id(detections, tag_id)`: Filters detections by tag ID.
-   `calculate_distance(detection, camera_params)`: Calculates distance to an AprilTag.
-   `detect_swarm_members(gray_frame)`: Detects AprilTags in a grayscale frame.
-   `detect_environment_objects(frame)`: Detects objects using YOLO.
-   `log_detections(tag_detections, object_detections, turtlebot_detections)`: Logs detection information.
-   `update_visualization(frame, tag_detections, object_detections, turtlebot_detections, camera_params)`: Updates the visualization frame.
-   `handle_frame_acquisition_failure()`: Handles cases where frame acquisition fails.
-   `run_detection_loop()`: The main detection loop.
-   `_shutdown()`: Cleans up resources.
-   `update_turtlebot_positions()`: Updates stored Turtlebot positions.
-   `get_turtlebot_positions()`: Returns Turtlebot positions.
-   `get_detected_objects()`: Returns detected objects.

### Usage

1.  **Configuration:** Modify the `SwarmConfig` class to match your setup.
2.  **Running:** Execute the script directly: `python camera_testing.py`.
3.  **API:** Other agents can use the `get_turtlebot_positions()` and `get_detected_objects()` methods to access the camera agent's data.

## visualizeAprilTag.py
This Python script, `visualizeAprilTag.py`, provides functions to detect and visualize AprilTags in images. It utilizes the `apriltag` library for detection and OpenCV (`cv2`) for drawing visual overlays.

## Dependencies

-   `numpy`: For numerical operations, especially matrix manipulation.
-   `opencv-python`: For image processing and drawing functions.
-   `apriltag`: For detecting AprilTags in images.

## Methods

-   `_draw_pose_box`: Draws a 3D pose box around the detected AprilTag.
-   `_draw_pose_axes`: Draws 3D coordinate axes at the center of the AprilTag.
-   `_annotate_detection`: Annotates the AprilTag with its ID text label.
-   `visualize`: Main function to detect and visualize AprilTags.

Installation:

```bash
pip install numpy opencv-python apriltag
```

## image_capture.py

This Python script, `image_capture.py`, is a simple utility for capturing a single frame from a webcam and saving it as an image file. It ensures that each captured image has a unique filename by checking for existing files and incrementing a counter.

## Dependencies

- `cv2` (OpenCV): For accessing the camera and saving the image.
  - Installation: `pip install opencv-python`
- `os`: For checking the existence of files and managing filenames.
  - (Standard library, no installation needed)

## Methods

- `get_next_filename`: This helper function generates the next available filename in the format `capture.jpg`, `capture1.jpg`, `capture2.jpg`, etc., ensuring that no existing files are overwritten.

### Parameters

- `base_name` (str): Base name for the file (default is `"capture"`).
- `extension` (str): File extension (default is `".jpg"`).

### Returns

- `filename` (str): A unique filename that does not already exist in the working directory.

## Workflow

1. **Camera Initialization:**
   - Opens the default camera using OpenCV (`cv2.VideoCapture(0)`).
   - Sets the resolution to 1920x1080 pixels.

2. **Frame Capture:**
   - Captures a single frame from the video stream.

3. **File Naming:**
   - Calls `get_next_filename()` to get a unique filename for saving the image.

4. **Saving the Frame:**
   - Writes the captured frame to a `.jpg` file using the generated filename.
   - Prints a confirmation message with the filename.

5. **Cleanup:**
   - Releases the camera resource.
   - Destroys any OpenCV windows.

## Usage

1. Run the script:  
   ```bash
   python image_capture.py
   ```

2. The captured image will be saved as `capture.jpg`, or `capture1.jpg`, `capture2.jpg`, etc., depending on existing files.

3. Useful for quick debugging, data collection, or testing camera setup.
