# Instructions for camera calibration:
# 1) Print out a 10x7 checkerboard pattern on paper 
    # Checkerboard print link: https://github.com/kyle-bersani/opencv-examples/blob/master/CalibrationByChessboard/chessboard-to-print.pdf 
# 2) Attach the pattern to a solid board (cardboard/clipboard) to prevent warping while taking photos
# 3) Get the measurement in meters of the individual squares and edit variable "SQUARE_SIZE" (line: 18) accordingly
# 4) Run the program and take at least 15-20 photos of the pattern at different angles and distances using left click
    # The mouse cursor must be within the camera window for the input to be recognized
    # Ensure the entire board is within view and that the detection pattern is overlayed in the display to get a good image
# 5) End calibration by pressing q key, this will generate the file "camera_calibration.npz"
    # This file is necessary for accurate distance measurement using poseEstimator.py

import cv2 # Image processing/Camera capture
import numpy as np # Numerical arrays
import glob #Pattern-matching filenames on disk
import os # Creating directories

# Settings
CHECKERBOARD = (9, 6)   # Number of INNER corners — a standard 10x7 square board has 9x6 inner corners
SQUARE_SIZE = 0.0254     # Physical size of one checkerboard square in metres (measure printed board)
IMAGES_DIR = "calib_images"
OUTPUT_FILE = "camera_calibration.npz"

# Capture checkerboard images from the webcam
def capture_calibration_images():
    os.makedirs(IMAGES_DIR, exist_ok=True) # Creates calibration image directory if does not exist
    cap = cv2.VideoCapture(0) # Opens default system camera

    # Error message if camera failed to open
    if not cap.isOpened(): 
        print("Error: could not open camera")
        return

    print("Point the camera at the checkerboard from different angles/distances.")
    print("Press SPACE to capture an image, 'q' to finish and calibrate.")

    # Capture count
    count = 0

    capture_requested = False  # Flag set by the mouse callback

    # Capture photos with left click
    def on_mouse(event, x, y, flags, param):
        nonlocal capture_requested
        if event == cv2.EVENT_LBUTTONDOWN:
            capture_requested = True

    cv2.namedWindow("Calibration Capture")
    cv2.setMouseCallback("Calibration Capture", on_mouse)

    # Main capture loop
    while True:
        ret, frame = cap.read()
        if not ret: # Break if camera does not return
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Converts image to grayscale
        # Searches for marker's inner-corner pattern & tracks coordinates
        found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None) 

        # Frame copy for displaying & providing visual feedback (marker orientation overlay)
        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, CHECKERBOARD, corners, found)

        # Running counter of captured images
        cv2.putText(display, f"Captured: {count}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Calibration Capture", display) # Displays preview window

        if capture_requested and found:  # Capture frame
            path = os.path.join(IMAGES_DIR, f"calib_{count:02d}.png")
            cv2.imwrite(path, frame)
            print(f"Saved {path}")
            count += 1
        capture_requested = False  # reset every loop, whether or not it fired

        # User input
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'): # End program
            break

    cap.release() # Releases camera to be used by other programs
    cv2.destroyAllWindows() # Close display window
    print(f"\nCaptured {count} images. Aim for at least 15-20 before calibrating.")

# Run calibration against the captured images
def run_calibration():
    # Generates coordinate grid of checkerboard's coordinates
    objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE # Translates to meter mesurements

    objpoints = []  # 3D points in real-world space
    imgpoints = []  # 2D points in image plane

    # Checks for images within calibration folder "calib_images"
    images = glob.glob(os.path.join(IMAGES_DIR, "*.png"))
    if len(images) < 5:
        print("Not enough calibration images found. Run capture first.")
        return # Prevents calibration from continueing if data is too small

    # Image editing for calibration
    img_shape = None
    for fname in images:
        img = cv2.imread(fname) # Grabs
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) # Set to grayscale
        img_shape = gray.shape[::-1] # Records pixel dimensions as (width, height)

        # Corner detection rerun on saved image
        found, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria) # Refines detection to subpixel
            objpoints.append(objp)
            imgpoints.append(corners_refined)

    print(f"Using {len(objpoints)} valid images for calibration...") # Some images may be too blury or low quality

    # Solves for camera's matrix and lens distorsion
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, img_shape, None, None)

    print("\nCalibration complete.")
    print(f"Reprojection error: {ret:.4f} (lower is better, aim for < 0.5)")
    print("Camera matrix:\n", camera_matrix)
    print("Distortion coefficients:\n", dist_coeffs)

    # Serialization of both arrays (camera_matrix & dist_coeffs) into an .npz file for use in poseEstimator.py
    np.savez(OUTPUT_FILE, camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)
    print(f"\nSaved calibration to {OUTPUT_FILE}")

if __name__ == "__main__":
    capture_calibration_images()
    run_calibration()