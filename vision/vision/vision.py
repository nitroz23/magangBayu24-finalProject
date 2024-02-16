import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer_ = self.create_timer(0.1, self.publish_image)
        self.bridge = CvBridge()

    def publish_image(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.get_logger().error("Failed to open camera")
            return

        ret, frame = cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame from camera")
            return

        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.publisher_.publish(ros_image)
        cap.release()

        cv2.imshow("Camera", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
