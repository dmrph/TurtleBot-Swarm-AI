import cv2
import time
import sys
import asyncio
import numpy as np
import rclpy
from pymongo import MongoClient
from log_pose_data import update_config_position

sys.path.append('../AIAgents')
from aiAgentsMain import MoveBotAgent
from onnx import ONNXProcessor

class YOLODetector:
    def __init__(self, model_path="./best_gray.onnx", camera_id=0, conf_threshold=0.5, iou_threshold=0.5, show_display=True):
        self.model_path = model_path
        self.camera_id = camera_id
        self.show_display = show_display
        self.cap = None
        self.onnx_processor = ONNXProcessor(model_path, conf_thres=conf_threshold, iou_thres=iou_threshold)

        self.class_mapping = {
            0: "cone",
            1: "football"
        }

        rclpy.init()
        self.move_agent = MoveBotAgent()

        self.mongo_client = MongoClient("mongodb://10.170.9.20:27017/")
        self.mongo_db = self.mongo_client["robot_data"]
        self.mongo_collection = self.mongo_db["detections"]

    def initialize_camera(self):
        if self.cap is None or not self.cap.isOpened():
            self.cap = cv2.VideoCapture(self.camera_id)
            if not self.cap.isOpened():
                raise Exception(f"Failed to open camera with ID {self.camera_id}")
        if self.show_display:
            try:
                cv2.namedWindow("Detected Objects", cv2.WINDOW_NORMAL)
            except:
                print("[WARNING] GUI not supported — continuing without display")
                self.show_display = False

    def release_camera(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.cap = None
        if self.show_display:
            try:
                cv2.destroyAllWindows()
            except:
                print("[WARNING] Could not destroy windows")

    def process_frame(self, frame=None):
        if frame is None:
            if self.cap is None:
                self.initialize_camera()
            for _ in range(5):
                self.cap.grab()
            ret, frame = self.cap.read()
            if not ret:
                return None, None, None, None

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        boxes, scores, class_ids = self.onnx_processor(gray_frame)
        detected_img = self.onnx_processor.draw_detections(gray_frame)
        return boxes, scores, class_ids, detected_img

    def get_highest_confidence_object(self, scores, class_ids):
        if scores is None or len(scores) == 0:
            return None
        max_score_idx = int(np.argmax(scores))
        class_id = class_ids[max_score_idx]
        return self.class_mapping.get(class_id, f"unknown-{class_id}")

    async def detect_charm_center_and_move(self, continuous=False, max_frames=1, show_fps=False):
        self.initialize_camera()
        prev_time = 0
        frame_count = 0
        detected_objects = []

        try:
            while self.cap.isOpened():
                boxes, scores, class_ids, detected_img = self.process_frame()
                if boxes is None:
                    break

                # Debugging: Check the boxes, scores, and class_ids
                print(f"Boxes: {boxes}")
                print(f"Scores: {scores}")
                print(f"Class IDs: {class_ids}")

                if show_fps:
                    current_time = time.time()
                    fps = 1 / (current_time - prev_time) if prev_time else 0
                    prev_time = current_time
                    if self.show_display:
                        cv2.putText(detected_img, f"FPS: {fps:.2f}", (10, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    else:
                        print(f"FPS: {fps:.2f}")

                if scores is not None and len(scores) > 0:
                    for box, score, class_id in zip(boxes, scores, class_ids):
                        print(f"Detected class ID: {class_id}, Score: {score}")
                        object_name = self.class_mapping.get(class_id)
                        if object_name == "football":
                            print("Football detected!")

                            x1, y1, x2, y2 = map(int, box)
                            x_center = (x1 + x2) // 2
                            frame_width = detected_img.shape[1]
                            margin = 40
                            if abs(x_center - frame_width // 2) <= margin:
                                print("Football centered — moving forward")
                                await self.move_agent.move("backward", 0.5)
                            elif x_center < frame_width // 2 - margin:
                                print("Football left — rotating left")
                                await self.move_agent.move("left", 0.2)
                                await asyncio.sleep(0.2)
                            else:
                                print("Football right — rotating right")
                                await self.move_agent.move("right", 0.2)
                                await asyncio.sleep(0.2)

                            # Debug: Log robot's actual coordinates
                            robot_coords = self.move_agent.get_current_coords()
                            print(f"Robot Coordinates (Raw): {robot_coords}")

                            # Check if coords are in the expected format and valid
                            if isinstance(robot_coords, list) and len(robot_coords) >= 2:
                                print(f"Robot Coordinates (Processed): x={robot_coords[0]}, y={robot_coords[1]}")
                            else:
                                print("Robot Coordinates are not in the expected format.")

                            # Update the robot position in MongoDB
                            update_config_position(robot_coords)

                            detection_doc = {
                                "timestamp": time.time(),
                                "object": object_name,
                                "coords": robot_coords
                            }
                            self.mongo_collection.insert_one(detection_doc)
                            break

                if self.show_display:
                    cv2.imshow("Detected Objects", detected_img)

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

if __name__ == "__main__":
    detector = YOLODetector(show_display=False)
    try:
        detector.run(continuous=True, show_fps=True)
    finally:
        detector.release_camera()
