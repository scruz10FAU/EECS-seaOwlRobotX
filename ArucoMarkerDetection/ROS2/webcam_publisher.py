import rclpy # ROS2 Python client library
from rclpy.node import Node
from sensor_msgs.msg import Image # Standard ROS2 image message type
from cv_bridge import CvBridge # Converts between OpenCV images and ROS2 Image msgs
import cv2 # Camera capture

class WebcamPublisherNode(Node):
    def __init__(self):
        super().__init__('webcam_publisher_node')

        # Must match the topic name the detector node subscribes to
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)

        self.bridge = CvBridge() # Handles OpenCV numpy array -> ROS Image conversion

        # Opens the laptop's built-in/default webcam for local testing
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam")
            raise RuntimeError("Could not open webcam")

        timer_period = 1.0 / 30.0  # Publish at ~30 FPS
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to grab frame")
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8') # Converts OpenCV frame to ROS2 Image msg
        self.publisher.publish(msg)

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WebcamPublisherNode()

    try:
        rclpy.spin(node) # Keeps node alive, firing timer_callback ~30x/sec
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
