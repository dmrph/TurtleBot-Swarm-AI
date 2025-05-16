import cv2
import os

def get_next_filename(base_name="capture", extension=".jpg"):
    i = 0
    filename = f"{base_name}{i if i > 0 else ''}{extension}"
    while os.path.exists(filename):
        i += 1
        filename = f"{base_name}{i}{extension}"
    return filename

# Initialize camera
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Capture frame
ret, frame = cap.read()

# Get next available filename
filename = get_next_filename()

# Save the image
cv2.imwrite(filename, frame)
print(f"Saved image as {filename}")

# Clean up
cap.release()
cv2.destroyAllWindows()
