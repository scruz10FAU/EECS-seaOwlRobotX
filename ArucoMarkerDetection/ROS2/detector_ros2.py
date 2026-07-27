import rclpy # ROS2 Python client library
from rclpy.node import Node
from sensor_msgs.msg import Image # Standard ROS2 image message type
from cv_bridge import CvBridge # Converts between ROS2 Image msgs and OpenCV images
import cv2 # Image processing/ArUco detection

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

    # Detection feedback (console only — no visual display)
    if ids is not None:
        print(f"Detected {len(ids)} marker(s): IDs = {ids.flatten()}")
    return corners, ids

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # Subscribes to the drone's image topic instead of opening a camera device directly
        # Update the topic name to match what the drone's camera driver actually publishes
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # <-- change to match your drone's actual image topic
            self.image_callback,
            10  # QoS queue depth
        )

        self.bridge = CvBridge() # Handles ROS Image <-> OpenCV numpy array conversion

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8') # Converts ROS2 Image msg to OpenCV BGR frame
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        corners, ids = detect_aruco(frame) # Runs detection on the received frame, no display

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()

    try:
        rclpy.spin(node) # Keeps node alive, calling image_callback whenever a new frame arrives
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
