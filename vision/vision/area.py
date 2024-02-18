# display_subscriber_node_1.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Import String message type
from cv_bridge import CvBridge
import cv2

class DisplaySubscriberNode1(Node):
    def __init__(self):
        super().__init__('area')
        self.image_subscription = self.create_subscription(
            Image,
            'camera',
            self.image_callback,
            10)
        self.image_subscription
        self.bridge = CvBridge()

        # Create a publisher for a new topic
        self.publisher_ = self.create_publisher(String, 'area/msgs', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("Camera Feed Subscriber 1", cv_image)
        cv2.waitKey(1)

        # Publish a message to the custom topic
        self.publish_custom_message()

    def publish_custom_message(self):
        custom_msg = String()
        custom_msg.data = "Hello from Subscriber Node 1"
        self.publisher_.publish(custom_msg)

def main(args=None):
    rclpy.init(args=args)
    area = DisplaySubscriberNode1()
    rclpy.spin(area)
    area.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
