import cv2
import time
import sys
import asyncio
import numpy as np
import rclpy
import os
import threading
from pymongo import MongoClient

sys.path.append(os.path.expanduser('~/shared_students/CapstoneFinalRepo/AIAgents'))
sys.path.append(os.path.expanduser('~/shared_students/CapstoneFinalRepo/Yolo/onnx'))
import ONNXProcessor
from aiAgentsMain import MoveBotAgent

class YOLODetector:
    # rclpy.init()
    def __init__(self, model_path="./best_gray.onnx", camera_id=0, conf_threshold=0.5, iou_threshold=0.5, show_display=True):
        self.model_path = model_path
        self.camera_id = camera_id
        self.show_display = show_display
        self.cap = None
        self.onnx_processor = None
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        
        # Camera and processing state
        self.camera_lock = threading.Lock()
        self.camera_initialized = False
        self.camera_error = False
        self.frame_count = 0
        self.fps = 0.0
        self.last_frame_time = time.time()
        self.fps_update_time = time.time()

        self.class_mapping = {
            0: "cone",
            1: "football"
        }

        # Yolo charm detection, centering, and movement
        self.move_agent = MoveBotAgent()

        # Initialize MongoDB connection - this can be slow, so do it at startup
        try:
            self.mongo_client = MongoClient("mongodb://10.170.9.20:27017/", 
                                            serverSelectionTimeoutMS=2000)  # 2 sec timeout
            self.mongo_db = self.mongo_client["robot_data"]
            self.mongo_collection = self.mongo_db["detections"]
            # Test connection
            self.mongo_client.server_info()
            self.mongo_available = True
        except Exception as e:
            print(f"MongoDB connection failed: {e}")
            self.mongo_available = False

    def initialize_camera(self):
        """Initialize camera with error handling and retry logic"""
        with self.camera_lock:
            if self.camera_initialized:
                return True
                
            # Clear any previous errors
            self.camera_error = False
            
            try:
                print("Initializing camera...")
                # First release if already open (in case of reinit)
                if self.cap is not None:
                    try:
                        self.cap.release()
                    except:
                        pass
                    self.cap = None
                
                # Try to open camera
                self.cap = cv2.VideoCapture(self.camera_id)
                if not self.cap.isOpened():
                    raise Exception(f"Failed to open camera with ID {self.camera_id}")
                
                # Test read a frame
                for _ in range(3):  # Try a few frames to stabilize
                    ret, test_frame = self.cap.read()
                    if not ret:
                        time.sleep(0.1)  # Wait a bit
                    else:
                        break
                
                if not ret:
                    raise Exception(f"Camera opened but couldn't read frame")
                
                # Initialize ONNX processor (helps detect early issues)
                if self.onnx_processor is None:
                    print("Initializing ONNX processor...")
                    self.onnx_processor = ONNXProcessor.ONNXProcessor(
                        self.model_path, 
                        conf_thres=self.conf_threshold, 
                        iou_thres=self.iou_threshold
                    )
                
                # Set up display if needed
                if self.show_display:
                    try:
                        cv2.namedWindow("Detected Objects", cv2.WINDOW_NORMAL)
                    except:
                        print("[WARNING] GUI not supported — continuing without display")
                        self.show_display = False
                
                print("Camera initialized successfully")
                self.camera_initialized = True
                return True
                
            except Exception as e:
                print(f"Camera initialization error: {e}")
                self.camera_error = True
                self.camera_initialized = False
                
                # Try to release if something went wrong
                if self.cap is not None:
                    try:
                        self.cap.release()
                    except:
                        pass
                    self.cap = None
                
                return False

    def release_camera(self):
        """Safely release camera resources"""
        with self.camera_lock:
            if self.cap and self.cap.isOpened():
                try:
                    self.cap.release()
                except Exception as e:
                    print(f"Error releasing camera: {e}")
                finally:
                    self.cap = None
                    
            if self.show_display:
                try:
                    cv2.destroyAllWindows()
                except:
                    print("[WARNING] Could not destroy windows")
                    
            self.camera_initialized = False

    def process_frame(self, frame=None):
        """Process a single frame with error handling"""
        if not self.camera_initialized and not self.initialize_camera():
            return None, None, None, None
            
        with self.camera_lock:
            if frame is None:
                try:
                    # Skip a few frames to get fresh data
                    for _ in range(2):
                        self.cap.grab()
                        
                    ret, frame = self.cap.read()
                    if not ret:
                        print("Failed to read frame")
                        return None, None, None, None
                except Exception as e:
                    print(f"Error reading frame: {e}")
                    self.camera_error = True
                    return None, None, None, None

            try:
                # Process frame
                gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                boxes, scores, class_ids = self.onnx_processor(gray_frame)
                detected_img = self.onnx_processor.draw_detections(gray_frame)
                
                # Update FPS stats
                current_time = time.time()
                self.frame_count += 1
                
                # Update FPS once per second
                if current_time - self.fps_update_time >= 1.0:
                    self.fps = self.frame_count / (current_time - self.fps_update_time)
                    self.frame_count = 0
                    self.fps_update_time = current_time
                    print(f"Camera FPS: {self.fps:.2f}")
                
                return boxes, scores, class_ids, detected_img
                
            except Exception as e:
                print(f"Error processing frame: {e}")
                return None, None, None, None

    def get_highest_confidence_object(self, scores, class_ids):
        if scores is None or len(scores) == 0:
            return None
        max_score_idx = int(np.argmax(scores))
        class_id = class_ids[max_score_idx]
        return self.class_mapping.get(class_id, f"unknown-{class_id}")

    def test_camera(self):
        """Test camera and return True if working"""
        success = self.initialize_camera()
        if success:
            # Try to read a frame
            try:
                ret, frame = self.cap.read()
                if ret and frame is not None:
                    print(f"Camera test: success (frame shape: {frame.shape})")
                    return True
                else:
                    print("Camera test: failed to read frame")
                    return False
            except Exception as e:
                print(f"Camera test error: {e}")
                return False
        return False

    def run(self, continuous=False, max_frames=1, show_fps=False):
        """Run detection in synchronous mode (for debugging)"""
        if not self.initialize_camera():
            return None
            
        frame_count = 0
        detected_objects = []

        try:
            while self.cap.isOpened():
                boxes, scores, class_ids, detected_img = self.process_frame()
                if boxes is None:
                    break

                if show_fps and detected_img is not None:
                    if self.show_display:
                        cv2.putText(detected_img, f"FPS: {self.fps:.2f}", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    else:
                        print(f"FPS: {self.fps:.2f}")

                if self.show_display and detected_img is not None:
                    cv2.imshow("Detected Objects", detected_img)

                if scores is not None and len(scores) > 0:
                    object_name = self.get_highest_confidence_object(scores, class_ids)
                    if object_name:
                        detected_objects.append((object_name, float(np.max(scores))))

                frame_count += 1
                if not continuous and frame_count >= max_frames:
                    break
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            if detected_objects:
                detected_objects.sort(key=lambda x: x[1], reverse=True)
                return detected_objects[0][0]
            return None

        finally:
            if not continuous:
                self.release_camera()

    async def detect_charm_center_and_move(self, continuous=False, max_frames=1, show_fps=False):
        """
        Detect charm center and move toward it in a non-blocking way
        """
        # Make sure camera is initialized
        if not self.camera_initialized and not self.initialize_camera():
            print("Failed to initialize camera")
            return False
            
        # Check if we should be using global flags for robot coordination
        using_global_flags = ('is_shutting_down' in globals() and 
                             'object_detected' in globals())
            
        frame_count = 0
        charm_detected = False

        try:
            # Just process the requested number of frames
            while frame_count < max_frames:
                # Check global shutdown flag if available
                if using_global_flags and globals()['is_shutting_down']:
                    print("Shutdown detected - stopping charm detection")
                    return False
                
                # Process a single frame (with timeout protection)
                try:
                    # Yield control to allow other tasks to run 
                    await asyncio.sleep(0.01)
                    
                    # Process the frame in a thread to avoid blocking
                    loop = asyncio.get_event_loop()
                    boxes, scores, class_ids, detected_img = await loop.run_in_executor(
                        None, self.process_frame
                    )
                    
                    if boxes is None:
                        print("No frame available")
                        # Allow other tasks to run before continuing
                        await asyncio.sleep(0.1)
                        continue
                except Exception as e:
                    print(f"Frame processing error: {e}")
                    await asyncio.sleep(0.1)
                    continue

                # Update FPS display
                if show_fps and detected_img is not None:
                    if self.show_display:
                        cv2.putText(detected_img, f"FPS: {self.fps:.2f}", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    else:
                        print(f"FPS: {self.fps:.2f}")

                # Process detections
                if scores is not None and len(scores) > 0:
                    for box, score, class_id in zip(boxes, scores, class_ids):
                        object_name = self.class_mapping.get(class_id)
                        
                        # Looking for football (charm)
                        if object_name == "football":
                            print(f"Football detected with confidence {score:.2f}")
                            charm_detected = True
                            
                            # Extract coordinates
                            x1, y1, x2, y2 = map(int, box)
                            x_center = (x1 + x2) // 2
                            frame_width = detected_img.shape[1]
                            margin = 40
                            
                            # Check if there's an obstacle - pause if so
                            if using_global_flags and globals()['object_detected']:
                                print("Object detected by sensors, pausing charm approach")
                                await asyncio.sleep(0.2)
                                break
                                
                            # Movement commands based on football position
                            try:
                                if abs(x_center - frame_width // 2) <= margin:
                                    print("Football centered — moving forward")
                                    await self.move_agent.move("backward", 0.3)
                                elif x_center < frame_width // 2 - margin:
                                    print("Football left — rotating left")
                                    await self.move_agent.move("left", 0.15)
                                else:
                                    print("Football right — rotating right")
                                    await self.move_agent.move("right", 0.15)
                            except Exception as e:
                                print(f"Movement error: {e}")

                            # Allow other tasks to run between movements
                            await asyncio.sleep(0.05)

                            # Record detection in database if available
                            if self.mongo_available:
                                try:
                                    # Get robot coordinates (using default if unavailable)
                                    try:
                                        robot_coords = self.move_agent.get_current_coords()
                                    except:
                                        robot_coords = (0, 0)
                                        
                                    detection_doc = {
                                        "timestamp": time.time(),
                                        "object": object_name,
                                        "coords": robot_coords,
                                        "confidence": float(score)
                                    }
                                    
                                    # Run DB operation in thread pool to avoid blocking
                                    await asyncio.get_event_loop().run_in_executor(
                                        None, 
                                        lambda: self.mongo_collection.insert_one(detection_doc)
                                    )
                                except Exception as e:
                                    print(f"DB error: {e}")
                                    
                            break  # Process only the first football detection

                # Display the frame if requested
                if self.show_display and detected_img is not None:
                    cv2.imshow("Detected Objects", detected_img)
                    # Allow non-blocking key check
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                # Increment frame counter
                frame_count += 1
                
                # Critical: yield control after each frame to let other tasks run
                await asyncio.sleep(0.01)

            # Return whether we detected a charm
            return charm_detected
            
        except asyncio.CancelledError:
            print("Charm detection cancelled")
            raise
        except Exception as e:
            print(f"Error in charm detection: {e}")
            return False
        finally:
            # Don't release the camera between calls - just keep it open
            if not continuous and not charm_detected:
                pass  # Keep camera initialized for next call
                
    def __del__(self):
        """Clean up resources when object is destroyed"""
        self.release_camera()

if __name__ == "__main__":
    # Test the detector standalone
    detector = YOLODetector(show_display=True)
    
    async def test_async():
        """Test async detection"""
        print("Testing async detection...")
        try:
            for i in range(10):
                print(f"Detection attempt {i+1}")
                result = await detector.detect_charm_center_and_move(
                    continuous=False, 
                    max_frames=5,
                    show_fps=True
                )
                print(f"Detection result: {result}")
                await asyncio.sleep(0.5)
        except Exception as e:
            print(f"Test error: {e}")
        finally:
            detector.release_camera()
    
    try:
        if len(sys.argv) > 1 and sys.argv[1] == "--async":
            # Run async test
            asyncio.run(test_async())
        else:
            # Run synchronous test
            detector.run(continuous=True, show_fps=True)
    finally:
        detector.release_camera()