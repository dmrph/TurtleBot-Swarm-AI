import subprocess
import os

# Set PYTHONPATH to include AIAgents folder
env = os.environ.copy()
env["PYTHONPATH"] = (
    "/home/Thomas/shared_students/CapstoneFinalRepo/AIAgents:"
    + env.get("PYTHONPATH", "")
)

# Launch both scripts concurrently
p1 = subprocess.Popen(
    ["python3", "/home/Thomas/shared_students/CapstoneFinalRepo/AprilTags/visualizeAprilTag.py"],
    env=env
)
p2 = subprocess.Popen(
    ["python3", "/home/Thomas/shared_students/CapstoneFinalRepo/Yolo/detect_onnx.py"],
    env=env
)

# Wait for both to finish
p1.wait()
p2.wait()
