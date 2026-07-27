import cv2 # Image processing/ArUco detection/Camera capture

# Library for marker recognition
# Available ArUco dictionaries:
# DICT_4X4_50, DICT_6X6_250, DICT_7X7_1000, DICT_ARUCO_ORIGINAL
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50) # Ensure library detected matches library used
aruco_params = cv2.aruco.DetectorParameters() # Default parameters for detection

detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

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

# Opens computer webcam
cap = cv2.VideoCapture(0)

# Check to see if webcam is functioning 
if not cap.isOpened():
    print("Error: could not open camera")
    exit() # Exit program if webcam failed to open correctly

# Main loop
while True:
    ret, frame = cap.read() # Continuously graps camera frame
    if not ret:
        print("Error: failed to grab frame")
        break # Exits program if fails to capture frame

    corners, ids, annotated = detect_aruco(frame) # Runs detection on current frame
    cv2.imshow('ArUco Detection', annotated) # Displays detection annotations if marker is found

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break # End program if q key is pressed

cap.release() # Releases camera to be used by other programs
cv2.destroyAllWindows() # Close display window