#!/usr/bin/env python3
import cv2
import time
from ultralytics import YOLO

class YOLODetector:

    def __init__(self, model_path= r"Yolo/best_gray.onnx", camera_index=0, show_fps=True):
        self.model = YOLO(model_path)
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

        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            # Calculate FPS
            new_frame_time = time.time()
            fps = 1 / (new_frame_time - prev_frame_time) if prev_frame_time > 0 else 0
            prev_frame_time = new_frame_time

            # Run YOLOv8 detection
            results = self.model(frame, conf=0.50, iou=0.45, max_det=10)
            annotated_frame = results[0].plot()

            #Add FPS overlay
            if self.show_fps:
                fps_text = f"FPS: {int(fps)}"
                cv2.putText(annotated_frame, fps_text, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            # Show the annotated frame
            cv2.imshow("YOLOv8 Detection", annotated_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.cleanup()

    def cleanup(self):
        if self.cap:
            self.cap.release()
        # cv2.destroyAll

if __name__ == "__main__":
    detector = YOLODetector()
    detector.run()
