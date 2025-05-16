# YOLOv8 Nano Object Detection Model Documentation

## Overview

This guide documents the process of training a YOLOv8 model to detect and classify colored 
cones. Specifically, our dataset includes green, orange, purple, and yellow cones. The trained 
model is then deployed on a TurtleBot for real-time detection. 

## Setup and Dependencies

### Required Libraries
Add a new notebook in Google Colab and create a new code cell and run it
```python
!pip install ultralytics
!pip install roboflow
```

### Environment Setup
The model is configured to run on GPU for optimal training performance, but can also run on CPU with reduced speed.

### Dataset Access
The dataset is accessed through Roboflow:

Create a new code cell and run it
```python
from roboflow import Roboflow

# Initialize Roboflow with API key
rf = Roboflow(api_key="YOUR-API-KEY") # Replace with your API key https://universe.roboflow.com/cmpsc-488/cmpsc488/dataset/6

# Access the CMPSC-488 project
project = rf.workspace("cmpsc-488").project("cmpsc488")

# Download version 6 of the dataset in YOLOv8 format
dataset = project.version("6").download("yolov8")
```

## Dataset Configuration

The dataset is stored at `/content/cmpsc488-6/` with the following structure:
- `data.yaml`: Configuration file containing class names and dataset paths
- `train/`: Training images and labels
- `valid/`: Validation images and labels
- `test/`: Test images and labels (if available)

The `data.yaml` file defines:
- Class names
- Number of classes
- Paths to train, validation, and test sets

## Training Configuration

Create a new code cell and run it
```python
from ultralytics import YOLO

# Load the pre-trained YOLOv8 nano model
model = YOLO("yolov8n.pt")

# Train the model
results = model.train(
    data="/content/cmpsc488-6/data.yaml",
    epochs=100,                  # Total training epochs
    imgsz=640,                   # Input image size
    batch=16,                    # Batch size
    patience=15,                 # Early stopping patience
    optimizer="AdamW",           # Optimizer algorithm
    lr0=0.01,                    # Initial learning rate
    lrf=0.001,                   # Final learning rate
    weight_decay=0.0005,         # Weight decay for regularization
    warmup_epochs=3,             # Learning rate warmup epochs
    cos_lr=True,                 # Use cosine learning rate scheduler
    augment=True,                # Enable data augmentation
    mixup=0.1,                   # Apply mixup augmentation
    mosaic=1.0,                  # Apply mosaic augmentation
    close_mosaic=10,             # Disable mosaic in final epochs
    save=True,                   # Save model checkpoints
)
```
## Evaluation

After training, the model is evaluated on the validation dataset:
Create a new code cell and run it
```python
# Evaluate the model
results = model.val()  # Evaluate on validation set
```

## Model Export

The trained model is exported in multiple formats for deployment:

Create a new code cell and run it
```python
# Evaluate the model
results = model.val()  # Evaluate on validation set

# Export model for deployment
# Standard PyTorch model
model.export()
# Download the trained model files
from google.colab import files
files.download(f"{model.trainer.save_dir}/weights/best.pt")  # Best model

print("Training complete! Models downloaded.")
```

## Using YOLO model on TurtleBot
ssh into TurtleBot using the ssh -X"YOURNAME""YOURIP"
Create a new folder in directory of choice.
Create a ```detect.py```script and paste this code:
```python
#!/usr/bin/env python3
import cv2
import time
import os
import sys

# First check if the model file exists
model_path = "REPLACE WITH MODEL PATH"
if not os.path.exists(model_path):
    print(f"ERROR: Model file not found at {model_path}")
    sys.exit(1)

# Print OpenCV version for debugging
print(f"OpenCV version: {cv2.__version__}")

# Try to import YOLO with error handling
try:
    from ultralytics import YOLO
    model = YOLO(model_path)
    print("Model loaded successfully")
except Exception as e:
    print(f"Failed to load YOLO model: {e}")
    sys.exit(1)


print("Attempting to open camera...")
cap = cv2.VideoCapture("/dev/video0")

# Check if camera opened successfully
if not cap.isOpened():
    print("Failed to open camera at /dev/video0")
    # Try alternative indices
    for i in range(1, 5):
        print(f"Trying camera index {i}...")
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            print(f"Successfully opened camera at index {i}")
            break
    
    if not cap.isOpened():
        print("Could not open any camera")
        sys.exit(1)

# Set camera resolution
print("Setting camera resolution...")
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)

# Try to read a single frame first
print("Attempting to read a frame...")
ret, frame = cap.read()
if not ret:
    print("Failed to read frame from camera")
    cap.release()
    sys.exit(1)
else:
    print(f"Successfully read frame with shape {frame.shape}")
    
# Try to run inference on a single frame
try:
    print("Attempting inference on a single frame...")
    results = model(frame, conf=0.25, iou=0.45, max_det=10)
    print("Inference successful")
except Exception as e:
    print(f"Inference failed: {e}")
    cap.release()
    sys.exit(1)

# Display FPS counter
prev_frame_time = 0
new_frame_time = 0

print("Starting main loop...")
print("Press 'q' to exit...")

frame_count = 0
# Initialize FPS variables better
prev_frame_time = 0
fps = 0  # Start with a defined value

# In your main loop:
while True:
    # Read a frame from the camera
    ret, frame = cap.read()
    if not ret:
        print("Failed to read frame")
        break
    
    # Calculate FPS - improved method
    new_frame_time = time.time()
    if prev_frame_time > 0:  # Only calculate after the first frame
        fps = 1 / (new_frame_time - prev_frame_time)
    prev_frame_time = new_frame_time
    
    fps_text = f"FPS: {int(fps)}"
    
    # Rest of your code...
    
    # Run detection on the frame
    results = model(frame, conf=0.25, iou=0.45, max_det=10)
    
    # Draw the results on the frame
    annotated_frame = results[0].plot()
    
    # Add FPS counter to frame
    cv2.putText(annotated_frame, fps_text, (10, 30), 
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    # Display the annotated frame
    cv2.imshow("YOLOv8 Detection", annotated_frame)
    
    # Break the loop if 'q' is pressed - use a longer wait time
    key = cv2.waitKey(100) & 0xFF  # Wait for 100ms
    if key == ord('q'):
        print("Q pressed, exiting")
        break

# Release resources
print("Releasing camera and closing windows...")
cap.release()
cv2.destroyAllWindows()
print("Script completed")
```

Create a virtual envronment:
```
python3 -m venv .venv
# Activate virtual environment
source .venv/bin/activate
```

Install dependencies
```
pip3 install ultralytics opencv-python
```

Make the ```detect_cones.py``` script executable:
```
chmod +x detect_cones.py
```

Run detection script:
```python3 detect_cones.py```
- Test the model with actual cones placed in front of the TurtleBot's camera.
- Verify that the model correctly identifies and classifies each cone color.







