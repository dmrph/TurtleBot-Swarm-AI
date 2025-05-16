import sys
import asyncio
sys.path.append('../Yolo')
from detect_onnx import YOLODetector

detector = YOLODetector(model_path='../Yolo/best_gray.onnx', show_display=True)
asyncio.run(detector.detect_charm_center_and_move(continuous=True, max_frames=1, show_fps=True))
