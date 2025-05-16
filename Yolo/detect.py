#!/usr/bin/env python3
import cv2
import time
import os
from ultralytics import YOLO

class YOLODetector:

    def __init__(self, model_path="~/shared_students/CapstoneFinalRepo/Yolo/turtlebot-eyelevel.pt", camera_index="/dev/video0", show_fps=True):
        expanded_model_path = os.path.expanduser(model_path)
        self.model = YOLO(expanded_model_path)
        self.camera_index = camera_index
        self.show_fps = show_fps
        self.cap = None

    def setup_camera(self):
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            raise RuntimeError(f"Failed to open camera: {self.camera_index}")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)

    def run(self):
        self.setup_camera()
        prev_frame_time = 0

        for i in range(3):
            ret, frame = self.cap.read()
            if not ret:
                break

            # Calculate FPS
            new_frame_time = time.time()
            fps = 1 / (new_frame_time - prev_frame_time) if prev_frame_time > 0 else 0
            prev_frame_time = new_frame_time

            # Run YOLOv8 detection
            results = self.model(frame, conf=0.50, iou=0.45, max_det=10)

            # Optional: print detection results summary to console
            if results:
                print(f"Detections: {results[0].boxes}")

            # FPS for logging
            if self.show_fps:
                print(f"FPS: {fps:.2f}")

        self.cleanup()

    def cleanup(self):
        if self.cap:
            self.cap.release()

if __name__ == "__main__":
    detector = YOLODetector()
    detector.run()
