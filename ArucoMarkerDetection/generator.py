import cv2 # ArUco library/generator
import numpy as np # Allocate blank pixel array for drawing marker

# Library for marker creation
# Available ArUco dictionaries
# DICT_4X4_50, DICT_6X6_250, DICT_7X7_1000, DICT_ARUCO_ORIGINAL:
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# Generate and save marker with set ID
marker_id = 5 # Available IDs: 0 - 49 (Amount available defined by "DICT_4X4_50")
marker_size = 200  # Image resolution (pixels)

# Create marker image
marker_img = np.zeros((marker_size, marker_size), dtype=np.uint8) # Creates image buffer
cv2.aruco.generateImageMarker(aruco_dict, marker_id, marker_size, marker_img) # Render marker onto buffer

# Add white border for easier detection
bordered = cv2.copyMakeBorder(marker_img, 20, 20, 20, 20, cv2.BORDER_CONSTANT, value=255)

# Writes marker image onto current directory
cv2.imwrite(f'aruco_marker_{marker_id}.png', bordered)

print(f"Saved aruco_marker_{marker_id}.png")
print("Make sure to update physical marker size in pose estimator in order to function correctly")