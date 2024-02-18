# display_subscriber_node_2.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Import String message type
from cv_bridge import CvBridge
import cv2

class DisplaySubscriberNode2(Node):
    def __init__(self):
        super().__init__('circle')
        self.image_subscription = self.create_subscription(
            Image,
            'camera',
            self.image_callback,
            10)
        self.image_subscription
        self.bridge = CvBridge()

        # Create a publisher for a new topic
        self.publisher_ = self.create_publisher(String, 'circle/msgs', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv2.imshow("Camera Feed Subscriber 2", cv_image)
        cv2.waitKey(1)

        # Publish a message to the custom topic
        self.publish_custom_message()

    def publish_custom_message(self):
        custom_msg = String()
        custom_msg.data = "Hello from Subscriber Node 2"
        self.publisher_.publish(custom_msg)

def main(args=None):
    rclpy.init(args=args)
    circle = DisplaySubscriberNode2()
    rclpy.spin(circle)
    circle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
