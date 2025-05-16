import collections
import numpy
import cv2
from apriltag import Detector, DetectorOptions
import rclpy
# code get from https://github.com/Tinker-Twins/AprilTag/blob/main/scripts/apriltag_video.py

# Function to draw a 3D pose box around the detected AprilTag
def _draw_pose_box(overlay, camera_params, tag_size, pose, z_sign=1):
    # Define the 3D coordinates of the box corners relative to the tag
    opoints = (
        numpy.array(
            [
                -1,
                -1,
                0,
                1,
                -1,
                0,
                1,
                1,
                0,
                -1,
                1,
                0,
                -1,
                -1,
                -2 * z_sign,
                1,
                -1,
                -2 * z_sign,
                1,
                1,
                -2 * z_sign,
                -1,
                1,
                -2 * z_sign,
            ]
        ).reshape(-1, 1, 3)
        * 0.5
        * tag_size
    )

    # Define the edges that connect the box corners
    edges = numpy.array(
        [0, 1, 1, 2, 2, 3, 3, 0, 0, 4, 1, 5, 2, 6, 3, 7, 4, 5, 5, 6, 6, 7, 7, 4]
    ).reshape(-1, 2)

    fx, fy, cx, cy = camera_params

    # Camera matrix
    K = numpy.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

    # Convert rotation matrix to rotation vector
    rvec, _ = cv2.Rodrigues(pose[:3, :3])
    # Extract translation vector
    tvec = pose[:3, 3]

    # Distortion coefficients (assumed zero)
    dcoeffs = numpy.zeros(5)

    # Project the 3D box points into 2D image points
    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)

    # Convert to integer coordinates
    ipoints = numpy.round(ipoints).astype(int)
    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]

    # Draw each edge of the box on the overlay
    for i, j in edges:
        cv2.line(overlay, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)


######################################################################

# Function to draw 3D coordinate axes at the center of the AprilTag
def _draw_pose_axes(overlay, camera_params, tag_size, pose, center):
    # Extract camera parameters
    fx, fy, cx, cy = camera_params
    # Camera matrix
    K = numpy.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)

    # Convert rotation matrix to rotation vector
    rvec, _ = cv2.Rodrigues(pose[:3, :3])
    # Extract translation vector
    tvec = pose[:3, 3]

    # Distortion coefficients (assumed zero)
    dcoeffs = numpy.zeros(5)

    # Define 3D endpoints for the coordinate axes (X, Y, Z)
    opoints = (
        numpy.float32([[1, 0, 0], [0, -1, 0], [0, 0, -1]]).reshape(-1, 3) * tag_size
    )

    # Project the axis points to 2D
    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)
    ipoints = numpy.round(ipoints).astype(int)

    # Convert center point to integer tuple
    center = numpy.round(center).astype(int)
    center = tuple(center.ravel())

    # Draw the axes on the image (X-red, Y-green, Z-blue)
    cv2.line(overlay, center, tuple(ipoints[0].ravel()), (0, 0, 255), 2)
    cv2.line(overlay, center, tuple(ipoints[1].ravel()), (0, 255, 0), 2)
    cv2.line(overlay, center, tuple(ipoints[2].ravel()), (255, 0, 0), 2)


# Function to annotate AprilTag with its ID text label
def _annotate_detection(overlay, detection, center):
    text = str(detection.tag_id)
    font = cv2.FONT_HERSHEY_SIMPLEX

    # Estimate tag size in pixels for scaling the text
    tag_size_px = numpy.sqrt(
        (detection.corners[1][0] - detection.corners[0][0]) ** 2
        + (detection.corners[1][1] - detection.corners[0][1]) ** 2
    )
    font_size = tag_size_px / 22

    # Calculate text size and position
    text_size = cv2.getTextSize(text, font, font_size, 2)[0]
    tag_center = [detection.center[0], detection.center[1]]
    text_x = int(tag_center[0] - text_size[0] / 2)
    text_y = int(tag_center[1] + text_size[1] / 2)

    # Draw text on the overlay
    cv2.putText(overlay, text, (text_x, text_y), font, font_size, (0, 255, 255), 2)


# Main function to visualize AprilTag detections
def visualize(
    overlay,
    detection_results,
    detector: Detector,
    camera_params=(1430.0, 1430.0, 480.0, 0.0),
    tag_size=0.0521,
    vizualization=3,
    verbose=0,
    annotation=0,
):
    """
    Detect AprilTags from image.

    Args:   image [image]: Input image to run detection algorithm on
            detector [detector]: AprilTag Detector object
            camera_params [_camera_params]: Intrinsic parameters for camera (fx, fy, cx, cy)
            tag_size [float]: Physical size of tag in user defined units (m or mm recommended)
            vizualization [int]: 0 - Highlight
                                 1 - Highlight + Boxes
                                 2 - Highlight + Axes
                                 3 - Highlight + Boxes + Axes
            verbose [int]: 0 - Silent
                           1 - Number of detections
                           2 - Detection data
                           3 - Detection and pose data
            annotation [bool]: Render annotated text on detection window
    """

    # Count number of detected tags
    num_detections = len(detection_results)

    # Print number of detections if verbose
    if verbose == 1 or verbose == 2 or verbose == 3:
        print("Detected {} tags\n".format(num_detections))
    
    # Set print formatting for pose matrices
    numpy.set_printoptions(suppress=True, formatter={"float_kind": "{:0.4f}".format})

    # Loop through each detection
    for i, detection in enumerate(detection_results):
        # Print detection metadata
        if verbose == 2 or verbose == 3:
            print("Detection {} of {}:".format(i + 1, num_detections))
            print()
            print(detection.tostring(indent=2))

        # Estimate pose of tag
        pose, e0, e1 = detector.detection_pose(detection, camera_params, tag_size)

        # Choose visualization mode
        if vizualization == 1:
            _draw_pose_box(overlay, camera_params, tag_size, pose)
        elif vizualization == 2:
            _draw_pose_axes(overlay, camera_params, tag_size, pose, detection.center)
        elif vizualization == 3:
            _draw_pose_box(overlay, camera_params, tag_size, pose)
            _draw_pose_axes(overlay, camera_params, tag_size, pose, detection.center)

        # Draw tag ID if annotation enabled
        if annotation is True:
            _annotate_detection(overlay, detection, tag_size)

        # Print detailed pose information
        if verbose == 3:
            print(
                detection.tostring(
                    collections.OrderedDict(
                        [("Pose", pose), ("InitError", e0), ("FinalError", e1)]
                    ),
                    indent=2,
                )
            )
            print()

    return overlay
if __name__ == "__main__":
    rclpy.init()

    cap = cv2.VideoCapture(0)
    options = DetectorOptions(families="tag25h9")
    detector = Detector(options)

    print("Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = detector.detect(gray)

        output = visualize(
            overlay=frame,
            detection_results=detections,
            detector=detector,
            vizualization=3,
            annotation=1,
            verbose=1,
        )

        cv2.imshow("AprilTag Detection", output)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    rclpy.shutdown()