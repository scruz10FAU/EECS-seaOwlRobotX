import cv2 # Image processing/ArUco detection/Camera capture
import numpy as np #Loading calibration data/Defining 3d corner template/Estimating distance

MARKER_SIZE = 0.0635  # Marker physical size in metres — must match your printed marker (does not include white border)
CALIBRATION_FILE = "camera_calibration.npz"

# Load camera calibration (see calibrate_camera.py)
calib_data = np.load(CALIBRATION_FILE)
camera_matrix = calib_data["camera_matrix"]
dist_coeffs = calib_data["dist_coeffs"]

# Library for marker recognition
# Available ArUco dictionaries
# DICT_4X4_50, DICT_6X6_250, DICT_7X7_1000, DICT_ARUCO_ORIGINAL:
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50) # Ensure library detected matches library used
aruco_params = cv2.aruco.DetectorParameters()# Default parameters for detection

detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

# 3D coordinates of a marker's corners in its own local coordinate frame centered on the marker with Z=0 (since the marker is flat).
# Corner order: top-left, top-right, bottom-right, bottom-left.
MARKER_OBJECT_POINTS = np.array([
    [-MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
    [ MARKER_SIZE / 2,  MARKER_SIZE / 2, 0],
    [ MARKER_SIZE / 2, -MARKER_SIZE / 2, 0],
    [-MARKER_SIZE / 2, -MARKER_SIZE / 2, 0]
], dtype=np.float32)

# Detection function
def detect_aruco(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) # Transforms image to grayscale
    corners, ids, rejected = detector.detectMarkers(gray)
    # corners: 4-point array of located marker's corners
    # ids: IDs of located markers

    # Detection visual feedback
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids) # Draws box using found corners
        print(f"Detected {len(ids)} marker(s): IDs = {ids.flatten()}")
    return corners, ids, image

#Solve pose per marker
def estimate_pose(corners):
    rvecs = [] # Rotation vector
    tvecs = [] # Translation vector
    for c in corners:
        success, rvec, tvec = cv2.solvePnP( # Computes marker's relative pose to the camera 
            MARKER_OBJECT_POINTS, c[0], camera_matrix, dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        rvecs.append(rvec)
        tvecs.append(tvec)
    return rvecs, tvecs

# Visualization of marker's orientation in real world
def draw_axes(image, corners, ids, rvecs, tvecs):
    if ids is None:
        return image # Stops if no marker is detected
    flat_ids = ids.flatten()  # Normalizes shapes to a plain 1D array
    for i in range(len(flat_ids)):
        cv2.drawFrameAxes(image, camera_matrix, dist_coeffs,rvecs[i], tvecs[i], MARKER_SIZE * 0.5) # Draws orientation axes

        x, y, z = tvecs[i].flatten()
        distance = np.sqrt(x**2 + y**2 + z**2)

        c = corners[i][0]
        text_pos = (int(c[0][0]), int(c[0][1]) - 10)
        cv2.putText(image, f"ID:{flat_ids[i]} D:{distance:.2f}m",
                    text_pos, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    return image


# Computer webcam
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: could not open camera")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: failed to grab frame")
        break

    corners, ids, annotated = detect_aruco(frame)

    if ids is not None:
        rvecs, tvecs = estimate_pose(corners)
        annotated = draw_axes(annotated, corners, ids, rvecs, tvecs)

    cv2.imshow('ArUco Detection', annotated)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release() # Releases camera to be used by other programs
cv2.destroyAllWindows() # Close display window