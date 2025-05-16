import cv2
import time
import numpy as np
from onnx import ONNXProcessor
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import json
import os
import argparse

class YOLODetector(Node):
    def __init__(self, model_path="Yolo/best_gray.onnx", camera_id=0, conf_threshold=0.5, iou_threshold=0.5, show_display=True):
        super().__init__('yolo_detector')
        self.model_path = model_path
        self.camera_id = camera_id
        self.show_display = show_display
        self.cap = None
        self.onnx_processor = ONNXProcessor(model_path, conf_thres=conf_threshold, iou_thres=iou_threshold)
        
        # Charm tracking
        self.detected_charm_locations = []  # List of (x, y, confidence, timestamp)
        self.charm_detected = False
        self.last_detection_time = 0
        self.detection_cooldown = 2.0  # seconds between detections
        
        # Position tracking
        self.last_position = None
        self.last_orientation = None
        self.position_threshold = 0.1  # meters - minimum movement required for new detection
        self.orientation_threshold = 0.1  # radians - minimum rotation required
        
        # Subscribe to odometry for position tracking
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Load previously detected charms
        self.load_detected_charms()
        
        # Class mapping for object detection
        self.class_mapping = {
            0: "cone",
            1: "football",  # This is what the model was trained to detect
        }
    
    def load_detected_charms(self):
        """Load previously detected charms from file"""
        try:
            if os.path.exists('detected_charms.json'):
                with open('detected_charms.json', 'r') as f:
                    self.detected_charm_locations = json.load(f)
                self.get_logger().info(f"Loaded {len(self.detected_charm_locations)} previously detected charms")
        except Exception as e:
            self.get_logger().error(f"Error loading detected charms: {e}")
    
    def save_detected_charms(self):
        """Save detected charms to file"""
        try:
            with open('detected_charms.json', 'w') as f:
                json.dump(self.detected_charm_locations, f)
            self.get_logger().info(f"Saved {len(self.detected_charm_locations)} detected charms")
        except Exception as e:
            self.get_logger().error(f"Error saving detected charms: {e}")
    
    def odom_callback(self, msg):
        """Update robot position and orientation from odometry data"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # Convert quaternion to yaw
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        self.last_position = (position.x, position.y)
        self.last_orientation = yaw
        
    def has_moved_significantly(self, current_pos, current_orientation):
        """Check if robot has moved or rotated significantly"""
        if self.last_position is None or self.last_orientation is None:
            return True
            
        # Check position change
        dx = current_pos[0] - self.last_position[0]
        dy = current_pos[1] - self.last_position[1]
        position_distance = (dx**2 + dy**2)**0.5
        
        # Check orientation change
        orientation_diff = abs(current_orientation - self.last_orientation)
        orientation_diff = min(orientation_diff, 2*np.pi - orientation_diff)  # Handle wrap-around
        
        return position_distance > self.position_threshold or orientation_diff > self.orientation_threshold
    
    def is_duplicate_detection(self, charm_location, confidence):
        """Check if this is a duplicate charm detection"""
        current_time = time.time()
        
        # Check cooldown period
        if current_time - self.last_detection_time < self.detection_cooldown:
            return True
            
        # Convert pixel coordinates to relative world coordinates
        if self.last_position is not None and self.last_orientation is not None:
            # Calculate relative position based on camera view
            # This is a simplified model - you may need to adjust based on your camera setup
            relative_x = (charm_location[0] - 320) / 320  # Normalize to [-1, 1]
            relative_y = (charm_location[1] - 240) / 240  # Normalize to [-1, 1]
            
            # Convert to world coordinates
            world_x = self.last_position[0] + relative_x * np.cos(self.last_orientation)
            world_y = self.last_position[1] + relative_y * np.sin(self.last_orientation)
            
            # Check against previously detected charms
            for prev_charm in self.detected_charm_locations:
                prev_x, prev_y = prev_charm[0], prev_charm[1]
                distance = ((world_x - prev_x)**2 + (world_y - prev_y)**2)**0.5
                
                if distance < self.position_threshold:
                    return True
        
        return False
    
    def initialize_camera(self):
        """Initialize the camera if not already initialized"""
        if self.cap is None or not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                raise Exception(f"Failed to open camera with ID {self.camera_id}")
        
        if self.show_display:
            cv2.namedWindow("Detected Objects", cv2.WINDOW_NORMAL)
    
    def release_camera(self):
        """Release the camera resources"""
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.cap = None
        
        if self.show_display:
            cv2.destroyAllWindows()
    
    def process_frame(self, frame=None):
        """Process a single frame and return detection results"""
        if frame is None:
            # If no frame is provided, capture one from the camera
            if self.cap is None:
                self.initialize_camera()
            
            # Flush camera buffer to reduce latency
            for _ in range(5):  # Increase this number if needed
                self.cap.grab()  # Grab but don't decode

            ret, frame = self.cap.read()
            if not ret:
                return None, None, None, None

        # Convert to grayscale for model processing
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Run inference
        boxes, scores, class_ids = self.onnx_processor(gray_frame)

        # Draw detections on the frame
        detected_img = self.onnx_processor.draw_detections(gray_frame)

        return boxes, scores, class_ids, detected_img
    
    def get_highest_confidence_object(self, scores, class_ids):
        """Get the object with highest confidence score"""
        if not scores or len(scores) == 0:
            return None
        
        # Find the index of the highest score
        max_score_idx = scores.index(max(scores))
        
        # Get the class ID for that index
        class_id = class_ids[max_score_idx]
        
        # Map to the actual object name
        object_name = self.class_mapping.get(class_id, f"unknown-{class_id}")
        
        return object_name
    
    def run(self, continuous=False, max_frames=1, show_fps=False):
        """
        Run the detector
        
        If continuous is True, this will keep running until stopped
        Otherwise, it will process up to max_frames frames and return the highest confidence object
        """
        self.initialize_camera()
        prev_time = 0
        frame_count = 0
        detected_objects = []
        
        try:
            while self.cap.isOpened():
                # Process a frame
                boxes, scores, class_ids, detected_img = self.process_frame()
                
                if boxes is None:  # Failed to get frame
                    break
                
                # FPS calculation
                if show_fps:
                    current_time = time.time()
                    fps = 1 / (current_time - prev_time) if prev_time else 0
                    prev_time = current_time
                    
                    if self.show_display:
                        cv2.putText(detected_img, f"FPS: {fps:.2f}", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    else:
                        print(f"FPS: {fps:.2f}")
                
                # Display the image if needed
                if self.show_display:
                    cv2.imshow("Detected Objects", detected_img)
                
                # Record detected object if any
                if scores and len(scores) > 0:
                    object_name = self.get_highest_confidence_object(scores, class_ids)
                    if object_name:
                        detected_objects.append((object_name, max(scores)))
                    
                    # Check if football is detected
                    if object_name.lower() == "football":
                        # Get the location of the detected football
                        if len(boxes) > 0:
                            box = boxes[0]  # Get the first detected box
                            charm_location = (box[0], box[1])  # x, y coordinates of the charm
                            confidence = max(scores)
                            
                            # Check for duplicate detection
                            if not self.is_duplicate_detection(charm_location, confidence):
                                print("New charm detected!")
                                self.last_detection_time = time.time()
                                
                                # Convert to world coordinates and store
                                if self.last_position is not None and self.last_orientation is not None:
                                    relative_x = (charm_location[0] - 320) / 320
                                    relative_y = (charm_location[1] - 240) / 240
                                    world_x = self.last_position[0] + relative_x * np.cos(self.last_orientation)
                                    world_y = self.last_position[1] + relative_y * np.sin(self.last_orientation)
                                    
                                    self.detected_charm_locations.append((world_x, world_y, confidence, time.time()))
                                    self.save_detected_charms()
                                    
                                    # Add visual feedback on the frame
                                    if self.show_display:
                                        cv2.putText(detected_img, "NEW FOOTBALL DETECTED!", (10, 70),
                                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                            else:
                                print("Duplicate charm detection - ignoring")
                
                frame_count += 1
                
                # Check if we should continue
                if not continuous and frame_count >= max_frames:
                    break
                
                # Allow for keyboard interrupt
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            # Return the highest confidence object if any were detected
            if detected_objects:
                detected_objects.sort(key=lambda x: x[1], reverse=True)
                return detected_objects[0][0]
            return None
            
        finally:
            if not continuous:
                self.release_camera()
                self.save_detected_charms()

# If run directly as a script
if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Run YOLO detector with optional display')
    parser.add_argument('--display', action='store_true', help='Enable visual display')
    args = parser.parse_args()
    
    rclpy.init()
    detector = YOLODetector(show_display=args.display)
    try:
        detector.run(continuous=True, show_fps=True)
    finally:
        detector.release_camera()
        rclpy.shutdown()
