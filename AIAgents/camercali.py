import cv2
import numpy as np
import os

# Path to your checkerboard images
CONFIG_PATH = os.path.expanduser('~/shared_students/CapstoneFinalRepo/AIAgents/checkerboard_pics')


def calibrate_camera(config_path, prefix, image_format, square_size):
    config_path = os.path.abspath(os.path.expanduser(config_path))
    print(f"Using directory: {config_path}")

    if not os.path.exists(config_path):
        print(f"Error: Directory {config_path} does not exist!")
        return None, None, None, None, None

    # For a 10x7 checkerboard, internal corners = 9x6
    pattern_size = (9,6)  # (columns, rows of internal corners)

    # Prepare object points in 3D space
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    objpoints = []
    imgpoints = []

    all_files = os.listdir(config_path)
    images = [
        os.path.join(config_path, f)
        for f in all_files
        if f.lower().startswith(prefix.lower()) and f.lower().endswith(image_format.lower())
    ]

    print(f" Found {len(images)} image(s) matching pattern '{prefix}*{image_format}'")

    if not images:
        print("No images found. Exiting.")
        return None, None, None, None, None

    save_count = 1

    for fname in images:
        print(f"Processing: {fname}")
        img = cv2.imread(fname)
        if img is None:
            print(f"⚠️ Warning: Cannot read image {fname}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        print(f"Corners found: {len(corners) if ret else 0}")

        if ret:
            # Sub-pixel corner refinement
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

            objpoints.append(objp)
            imgpoints.append(corners_refined)

            # Draw corners
            img_drawn = cv2.drawChessboardCorners(img, pattern_size, corners_refined, ret)

            # output_path = os.path.join(config_path, f"output_chessboard_{save_count:02d}.jpg")
            # cv2.imwrite(output_path, img_drawn)
            # print(f"Saved visualization to: {output_path}")
            # save_count += 1

    if objpoints and imgpoints:
        print("Calibrating camera...")
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )

        if ret:
            print("\nCamera calibration successful!\n")
            return ret, mtx, dist, rvecs, tvecs
        else:
            print("Calibration failed during cv2.calibrateCamera()")
            return None, None, None, None, None
    else:
        print("Not enough valid corners for calibration.")
        return None, None, None, None, None


# --- Usage ---
prefix = 'chessboard'       # Your image filename prefix
image_format = '.JPG'       # Or '.jpg' depending on how they’re saved
square_size = 0.025         # Real-world size of each square in meters

ret, mtx, dist, rvecs, tvecs = calibrate_camera(CONFIG_PATH, prefix, image_format, square_size)

if ret:
    print("Calibration Results:")
    print("\nCamera Matrix:\n", mtx)
    print("\nDistortion Coefficients:\n", dist)
else:
    print("Calibration failed.")
