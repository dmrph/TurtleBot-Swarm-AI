#!/usr/bin/env python3
import cv2
import numpy as np
import asyncio
import logging
import time
from typing import Dict, Tuple, Optional, List
from apriltag import Detector, Detection, DetectorOptions
import visualizeAprilTag
from ultralytics import YOLO
import os

# Master toggle for deployment/testing mode
# Set to True when running on TurtleBot hardware
# Set to False when testing on development machine
TURTLEBOT_DEPLOYMENT = True

class SwarmConfig:
    CAMERA_INDEX = 0
    MODEL_PATH = "runs/detect/train2/weights/best.pt"
    
    # Define AprilTag to Turtlebot mappings
    TURTLEBOT_TAG_MAPPING = {
        "turtlebot1": {0, 1, 2},
        "turtlebot2": {3, 4, 5},
        "turtlebot3": {6, 7, 8}
    }
    
    # Tags to specifically look for (all tags from all turtlebots)
    SWARM_TAG_IDS = list(range(9))  # 0-8 inclusive
    
    CONE_CLASS_IDS = [0, 2, 3, 4, 5]
    CHARM_CLASS_IDS = [1]    
    TAG_SIZE = 0.0521  # Size in meters
    CAMERA_MATRIX = np.array([
        [1.35491825e+03, 0.00000000e+00, 9.35817930e+02],
        [0.00000000e+00, 1.35089637e+03, 4.92841991e+02],
        [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
    ])
    DIST_COEFFS = np.zeros(5)
    
    # Deployment settings - auto-configured based on TURTLEBOT_DEPLOYMENT
    DEPLOYMENT_MODE = TURTLEBOT_DEPLOYMENT
    VISUALIZATION_LEVEL = 0 if TURTLEBOT_DEPLOYMENT else 2  # 0=none, 1=minimal, 2=full
    VISUALIZATION_RATE = 2   # Update visualization every N frames
    
    # Performance settings
    TARGET_FPS = 10 
    FRAME_SKIP = 0
    YOLO_CONF_THRESHOLD = 0.6
    
    # Camera settings for deployment
    CAMERA_WIDTH = 640   
    CAMERA_HEIGHT = 480
    
    # Colors for different turtlebots (B,G,R format)
    TURTLEBOT_COLORS = {
        "turtlebot1": (255, 0, 0),    # Blue
        "turtlebot2": (0, 255, 0),    # Green
        "turtlebot3": (0, 0, 255)     # Red
    }
    
    # Enable distance calculation and reporting
    ENABLE_DISTANCE_CALCULATION = True
    
    # Logging settings
    LOGGING_INTERVAL = 30  # Log detections every N frames

class SwarmCameraAgent:
    def __init__(self, config: SwarmConfig = SwarmConfig()):
        self.config = config
        self.frame_counter = 0
        self.swarm_locations: Dict[str, Dict[int, Tuple[float, float, float]]] = {
            "turtlebot1": {},
            "turtlebot2": {},
            "turtlebot3": {}
        }
        
        # Last known positions
        self.last_known_positions = {}
        
        # Performance metrics
        self.fps = 0
        self.process_times = []
        self.last_fps_update = time.time()
        self.fps_update_interval = 1.0
        
        # Camera retry tracking
        self.camera_retry_attempts = 0
        self.max_camera_retries = 5
        
        # Initialize hardware interfaces
        self._init_camera()
        self._init_detection_systems()
        
        # AprilTag detector configuration
        options = DetectorOptions(
            families="tag25h9",
            nthreads=2, 
            quad_decimate=1.5
        )
        self.tag_detector = Detector(options)
        logging.info("AprilTag detector initialized with tag25h9 family")
        
        # Cache of turtlebot distances
        self.turtlebot_distances = {}
        
        # For deployment mode
        self.display_frame = None
        self.is_visualization_frame = False
        
        # Tracking for object detections
        self.last_object_detections = []

    def _init_camera(self):
        """Initialize the camera with appropriate settings for deployment/testing mode"""
        try:
            # Close any existing camera
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
                
            # Try to open with specified index
            self.cap = cv2.VideoCapture(self.config.CAMERA_INDEX)
            
            # If that fails and we're in deployment, try additional indices
            if not self.cap.isOpened() and self.config.DEPLOYMENT_MODE:
                for idx in range(4):  # Try indices 0-3
                    if idx == self.config.CAMERA_INDEX:
                        continue  # Skip the one we already tried
                    logging.info(f"Trying alternate camera index: {idx}")
                    self.cap.release()
                    self.cap = cv2.VideoCapture(idx)
                    if self.cap.isOpened():
                        self.config.CAMERA_INDEX = idx
                        break
            
            if not self.cap.isOpened():
                raise RuntimeError("Camera failed to initialize on any available index")
            
            # Set resolution based on deployment mode
            if self.config.DEPLOYMENT_MODE:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.CAMERA_WIDTH)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.CAMERA_HEIGHT)
            else:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            
            # Reduce buffer size to minimize latency
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            logging.info(f"Camera initialized at index {self.config.CAMERA_INDEX} with resolution {actual_width}x{actual_height}")
            
            # Read a test frame to verify camera is working
            ret, test_frame = self.cap.read()
            if not ret or test_frame is None:
                raise RuntimeError("Camera opened but could not read frames")
                
        except Exception as e:
            logging.error(f"Camera initialization failed: {e}")
            if hasattr(self, 'cap') and self.cap is not None:
                self.cap.release()
                self.cap = None
            raise

    def _init_detection_systems(self):
        """Initialize YOLO object detection model"""
        try:
            # Check if model path exists
            if os.path.exists(self.config.MODEL_PATH):
                self.object_detector = YOLO(self.config.MODEL_PATH)
                
                # Apply optimizations for Turtlebot
                if self.config.DEPLOYMENT_MODE:
                    self.object_detector.fuse()
                    
                logging.info(f"Loaded YOLO model from {self.config.MODEL_PATH}")
            else:
                logging.warning(f"Model path {self.config.MODEL_PATH} not found")
                self.object_detector = None
        except Exception as e:
            logging.critical(f"YOLO model failed to load: {e}")
            self.object_detector = None
    
    def get_turtlebot_for_tag(self, tag_id: int) -> Optional[str]:
        """Determine which turtlebot a tag belongs to."""
        for turtlebot, tag_ids in self.config.TURTLEBOT_TAG_MAPPING.items():
            if tag_id in tag_ids:
                return turtlebot
        return None

    def get_detection_by_id(self, detections: List[Detection], tag_id: int) -> Optional[Detection]:
        """Filter detections by tag ID and return the one with highest goodness score."""
        filtered_detections = [d for d in detections if d.tag_id == tag_id]
        if not filtered_detections:
            return None
        return max(filtered_detections, key=lambda detection: detection.goodness)

    def calculate_distance(self, detection: Detection, camera_params) -> float:
        """Calculate distance to AprilTag based on pose information."""
        try:
            # Extract necessary camera parameters
            fx, fy, cx, cy = camera_params
            
            # Use pose information if available
            if hasattr(detection, 'pose_t') and detection.pose_t is not None:
                # Direct distance from pose translation vector
                return float(np.linalg.norm(detection.pose_t))
            
            # Fallback - estimate distance based on apparent tag size
            # Get the four corners of the tag
            corners = detection.corners
            
            # Calculate side lengths
            side_lengths = []
            for i in range(4):
                next_i = (i + 1) % 4
                side = np.sqrt((corners[i][0] - corners[next_i][0])**2 + 
                               (corners[i][1] - corners[next_i][1])**2)
                side_lengths.append(side)
            
            # Average side length in pixels
            avg_side_px = sum(side_lengths) / 4
            
            # Estimate distance using focal length and actual tag size
            distance = (self.config.TAG_SIZE * fx) / avg_side_px
            
            return distance
            
        except Exception as e:
            logging.error(f"Error calculating distance: {e}")
            return float('inf')

    async def detect_swarm_members(self, gray_frame: np.ndarray) -> List[Detection]:
        """Detect AprilTags in the grayscale frame"""
        # Downscale for turtlebot
        if self.config.DEPLOYMENT_MODE and gray_frame.shape[0] > self.config.CAMERA_HEIGHT:
            gray_frame = cv2.resize(gray_frame, 
                                    (self.config.CAMERA_WIDTH, self.config.CAMERA_HEIGHT),
                                    interpolation=cv2.INTER_AREA)
        
        all_detections = self.tag_detector.detect(gray_frame)
        
        # Log detected tags in deployment mode periodically
        if all_detections and self.config.DEPLOYMENT_MODE and self.frame_counter % self.config.LOGGING_INTERVAL == 0:
            tag_ids = [detection.tag_id for detection in all_detections]
            logging.info(f"Detected AprilTags with IDs: {tag_ids}")
        
        return all_detections

    async def detect_environment_objects(self, frame: np.ndarray) -> list:
        """Detect objects (cones, charms) in the frame using YOLO"""
        # If the YOLO model wasn't loaded successfully, return an empty list
        if not self.object_detector:
            return []
        
        # Resize the frame for deployment mode to reduce computational load
        if self.config.DEPLOYMENT_MODE and frame.shape[0] > self.config.CAMERA_HEIGHT:
            frame = cv2.resize(frame, 
                               (self.config.CAMERA_WIDTH, self.config.CAMERA_HEIGHT),
                               interpolation=cv2.INTER_AREA)
        
        # Run YOLO object detection on the current frame
        results = self.object_detector(frame, verbose=False, conf=self.config.YOLO_CONF_THRESHOLD)
        
        detections = []
        
        # Parse detection results from YOLO
        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])  # Detected class ID
                conf = float(box.conf[0])  # Confidence score
                x1, y1, x2, y2 = map(int, box.xyxy[0])  # Bounding box coordinates
                detections.append((cls_id, conf, x1, y1, x2, y2))
                
                # Log high-confidence detections if in deployment mode and at logging interval
                if conf > 0.75 and self.config.DEPLOYMENT_MODE and self.frame_counter % self.config.LOGGING_INTERVAL == 0:
                    obj_type = "Cone" if cls_id in self.config.CONE_CLASS_IDS else "Charm"
                    logging.info(f"High confidence {obj_type} detected: {conf:.2f}")
        
        # Store detections for access via API method
        self.last_object_detections = detections
        
        return detections

    def log_detections(self, tag_detections, object_detections, turtlebot_detections):
        """Log detection information when in deployment mode"""
        if self.frame_counter % self.config.LOGGING_INTERVAL != 0:
            return
            
        # Log turtlebot detections
        for turtlebot_name, detections in turtlebot_detections.items():
            if turtlebot_name == "unknown" or not detections:
                continue
                
            tag_ids = [d.tag_id for d in detections]
            if turtlebot_name in self.turtlebot_distances:
                avg_distance = sum(d[1] for d in self.turtlebot_distances[turtlebot_name]) / len(self.turtlebot_distances[turtlebot_name])
                logging.info(f"Detected {turtlebot_name} (tags: {tag_ids}) at distance: {avg_distance:.2f}m")
            else:
                logging.info(f"Detected {turtlebot_name} (tags: {tag_ids})")
                
        # Log object detections
        cone_count = 0
        charm_count = 0
        
        for cls_id, conf, _, _, _, _ in object_detections:
            if cls_id in self.config.CONE_CLASS_IDS:
                cone_count += 1
            elif cls_id in self.config.CHARM_CLASS_IDS:
                charm_count += 1
        
        if cone_count > 0:
            logging.info(f"Detected {cone_count} cones")
        if charm_count > 0:
            logging.info(f"Detected {charm_count} charms")

    def update_visualization(self, frame, tag_detections, object_detections, turtlebot_detections, camera_params):
        """Update visualization frame - separated for performance optimization"""
        if self.config.VISUALIZATION_LEVEL == 0:
            return frame

        # Use visualizeAprilTag to draw pose estimations
        if tag_detections:
            frame = visualizeAprilTag.visualize(
                overlay=frame,
                detection_results=tag_detections,
                detector=self.tag_detector,
                camera_params=camera_params,
                tag_size=self.config.TAG_SIZE,
                vizualization=3 if self.config.VISUALIZATION_LEVEL > 1 else 1,
                annotation=True  # Always show tag IDs
            )
        
        # Add turtlebot identification visualizations
        for turtlebot_name, detections in turtlebot_detections.items():
            if turtlebot_name == "unknown" or not detections:
                continue
                
            # Get the color for this turtlebot
            color = self.config.TURTLEBOT_COLORS.get(turtlebot_name, (125, 125, 125))
            
            # Calculate average position of all tags for this turtlebot
            centers = [detection.center for detection in detections]
            if centers:
                avg_center_x = int(sum(c[0] for c in centers) / len(centers))
                avg_center_y = int(sum(c[1] for c in centers) / len(centers))
                center = (avg_center_x, avg_center_y)
                
                # Draw a distintive marker for the turtlebot
                cv2.circle(frame, center, 25, color, 3)
                cv2.putText(frame, f"{turtlebot_name}", 
                            (center[0]-70, center[1]-30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
                
                # Always show distance information if available
                if turtlebot_name in self.turtlebot_distances:
                    avg_distance = sum(d[1] for d in self.turtlebot_distances[turtlebot_name]) / len(self.turtlebot_distances[turtlebot_name])
                    cv2.putText(frame, f"Dist: {avg_distance:.2f}m", 
                              (center[0]-70, center[1]+30),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # Process YOLO visualizations - always show labels with confidence
        for cls_id, conf, x1, y1, x2, y2 in object_detections:
            color = (0, 165, 255) if cls_id in self.config.CONE_CLASS_IDS else (255, 0, 255)
            label = "Cone" if cls_id in self.config.CONE_CLASS_IDS else "Charm"
            
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            # Always show confidence regardless of visualization level
            cv2.putText(frame, f"{label}: {conf:.2f}", (x1, y1-10),
                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        # Add minimal info panel showing detected turtlebots
        if self.config.VISUALIZATION_LEVEL > 0:
            y_offset = 30
            for turtlebot_name, detections in turtlebot_detections.items():
                if turtlebot_name == "unknown" or not detections:
                    continue
                color = self.config.TURTLEBOT_COLORS.get(turtlebot_name, (125, 125, 125))
                
                # Show which tags were detected for this turtlebot
                tag_text = f"{turtlebot_name}: {len(detections)} tags"
                cv2.putText(frame, tag_text, (10, y_offset), 
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
                y_offset += 25
        
        # Display FPS on frame
        cv2.putText(frame, f"FPS: {self.fps:.1f}", 
                   (frame.shape[1] - 120, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        return frame

    async def handle_frame_acquisition_failure(self):
        """Handle cases where frame acquisition fails"""
        self.camera_retry_attempts += 1
        
        # If we've had too many consecutive failures, try to reinitialize the camera
        if self.camera_retry_attempts >= self.max_camera_retries:
            logging.warning(f"Too many frame acquisition failures ({self.camera_retry_attempts}). Reinitializing camera...")
            try:
                self._init_camera()
                self.camera_retry_attempts = 0
            except Exception as e:
                logging.error(f"Failed to reinitialize camera: {e}")
                
        await asyncio.sleep(0.5)  # Wait a bit before trying again

    async def run_detection_loop(self):
        """Main detection loop"""
        logging.info("Swarm Camera Agent - Operational")
        try:
            last_time = time.time()
            
            while True:
                start_time = time.time()
                
                # Check if camera is available
                if self.cap is None or not self.cap.isOpened():
                    logging.error("Camera not initialized or not open")
                    await self.handle_frame_acquisition_failure()
                    continue
                
                # Try to acquire a frame
                ret, frame = self.cap.read()
                if not ret or frame is None:
                    logging.error("Frame acquisition failed")
                    await self.handle_frame_acquisition_failure()
                    continue
                    
                # Reset retry counter on successful frame acquisition
                self.camera_retry_attempts = 0

                # Frame skipping for CPU optimization
                self.frame_counter += 1
                if self.config.FRAME_SKIP > 0 and self.frame_counter % self.config.FRAME_SKIP != 0:
                    continue

                # Check if this is a visualization frame
                self.is_visualization_frame = (self.frame_counter % self.config.VISUALIZATION_RATE == 0)
                
                # Make a copy for visualization only if needed
                display_frame = frame.copy() if self.is_visualization_frame and self.config.VISUALIZATION_LEVEL > 0 else None
                
                # Prepare grayscale frame for AprilTag detection
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # Run detection tasks (AprilTag is more important than YOLO for robot coordination)
                tag_detections = await self.detect_swarm_members(gray)
                object_detections = await self.detect_environment_objects(frame)

                # Group detected tags by turtlebot
                turtlebot_detections = {
                    "turtlebot1": [],
                    "turtlebot2": [],
                    "turtlebot3": [],
                    "unknown": []
                }

                # Extract camera parameters
                camera_params = (
                    self.config.CAMERA_MATRIX[0,0],  # fx
                    self.config.CAMERA_MATRIX[1,1],  # fy
                    self.config.CAMERA_MATRIX[0,2],  # cx
                    self.config.CAMERA_MATRIX[1,2]   # cy
                )

                # Reset distances for this frame
                self.turtlebot_distances = {}
                
                # Process AprilTag detections
                if tag_detections:
                    for detection in tag_detections:
                        turtlebot = self.get_turtlebot_for_tag(detection.tag_id)
                        if turtlebot:
                            turtlebot_detections[turtlebot].append(detection)
                            
                            # Calculate and store distance if enabled
                            if self.config.ENABLE_DISTANCE_CALCULATION:
                                distance = self.calculate_distance(detection, camera_params)
                                if turtlebot not in self.turtlebot_distances:
                                    self.turtlebot_distances[turtlebot] = []
                                self.turtlebot_distances[turtlebot].append((detection.tag_id, distance))
                        else:
                            turtlebot_detections["unknown"].append(detection)
                    
                    # Store location data for this frame
                    for turtlebot_name, detections in turtlebot_detections.items():
                        if turtlebot_name == "unknown" or not detections:
                            continue
                            
                        # Store location data for this turtlebot
                        for detection in detections:
                            distance = next((d[1] for d in self.turtlebot_distances.get(turtlebot_name, []) 
                                            if d[0] == detection.tag_id), 0.0)
                            
                            self.swarm_locations[turtlebot_name][detection.tag_id] = (
                                detection.center[0], 
                                detection.center[1],
                                distance
                            )
                
                # Only update visualization on visualization frames
                if self.is_visualization_frame and self.config.VISUALIZATION_LEVEL > 0:
                    display_frame = self.update_visualization(
                        display_frame, 
                        tag_detections, 
                        object_detections, 
                        turtlebot_detections, 
                        camera_params
                    )
                    self.display_frame = display_frame
                
                # For deployment mode, log detections periodically
                if self.config.DEPLOYMENT_MODE:
                    self.log_detections(tag_detections, object_detections, turtlebot_detections)
                
                # FPS calculation
                current_time = time.time()
                elapsed = current_time - last_time
                if elapsed > 0:
                    current_fps = 1.0 / elapsed
                    self.process_times.append(current_fps)
                    if len(self.process_times) > 10:  # Reduced buffer size
                        self.process_times.pop(0)
                    
                    # Update FPS periodically
                    if current_time - self.last_fps_update > self.fps_update_interval:
                        self.fps = sum(self.process_times) / len(self.process_times)
                        self.last_fps_update = current_time
                        
                        # Log FPS for monitoring in deployment mode
                        if self.config.DEPLOYMENT_MODE:
                            logging.info(f"Current FPS: {self.fps:.1f}")
                        
                last_time = current_time
                
                # Display output (only on visualization frames to save CPU)
                if display_frame is not None and self.is_visualization_frame:
                    cv2.imshow("Swarm Perception", display_frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                # Update overall turtlebot positions for the API
                self.update_turtlebot_positions()
                
                # Adaptive sleep for target FPS - crucial for consistent timing
                process_time = time.time() - start_time
                target_time_per_frame = 1.0 / self.config.TARGET_FPS
                sleep_time = max(0, target_time_per_frame - process_time)
                
                # Dynamic log to help with tuning
                if self.frame_counter % 30 == 0 and process_time > target_time_per_frame:
                    logging.debug(f"Process time {process_time:.3f}s > target {target_time_per_frame:.3f}s")
                
                await asyncio.sleep(sleep_time)

        except KeyboardInterrupt:
            logging.info("Operator termination requested")
        except Exception as e:
            logging.error(f"Error in detection loop: {e}", exc_info=True)
        finally:
            self._shutdown()
            
            # Print final statistics
            avg_fps = sum(self.process_times) / len(self.process_times) if self.process_times else 0
            logging.info(f"Average FPS: {avg_fps:.2f}")

    def _shutdown(self):
        """Clean shutdown of resources"""
        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        cv2.destroyAllWindows()
        logging.info("Swarm Camera Agent - Shutdown complete")

    def update_turtlebot_positions(self):
        """Updates the stored positions of turtlebots based on recent detections"""
        positions = {}
        
        for turtlebot_name, detections in self.swarm_locations.items():
            if not detections:
                continue
                
            # Calculate average position from all tags for this turtlebot
            positions_list = list(detections.values())
            if positions_list:
                avg_x = sum(p[0] for p in positions_list) / len(positions_list)
                avg_y = sum(p[1] for p in positions_list) / len(positions_list)
                avg_dist = sum(p[2] for p in positions_list) / len(positions_list)
                positions[turtlebot_name] = (avg_x, avg_y, avg_dist)
        
        # Store calculated positions
        self.last_known_positions = positions

    def get_turtlebot_positions(self) -> Dict[str, Tuple[float, float, float]]:
        """Returns the average position and distance of each detected turtlebot.
        Returns a dictionary with turtlebot names as keys and (x, y, distance) as values.
        This is the main API for other agents to use."""
        return self.last_known_positions

    def get_detected_objects(self) -> List[Tuple[str, float, Tuple[int, int, int, int]]]:
        """Returns a list of detected objects (cones/charms) with their positions.
        Returns a list of (object_type, confidence, (x1,y1,x2,y2)) tuples."""
        formatted_objects = []
        
        for cls_id, conf, x1, y1, x2, y2 in self.last_object_detections:
            object_type = "cone" if cls_id in self.config.CONE_CLASS_IDS else "charm"
            formatted_objects.append((object_type, conf, (x1, y1, x2, y2)))
            
        return formatted_objects

if __name__ == "__main__":
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(levelname)s - %(message)s'
    )
    
    # Start the agent
    agent = SwarmCameraAgent()
    asyncio.run(agent.run_detection_loop())