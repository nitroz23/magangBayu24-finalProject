# display_subscriber_node_1.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32  # Import String message type
from cv_bridge import CvBridge
import cv2
import numpy as np

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
        self.middle_publisher_ = self.create_publisher(Int32, 'area/middle', 10)
        self.count_publisher_ = self.create_publisher(Int32, 'area/count', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Define the color range
        lowerBound = np.array([150, 150, 150], dtype=np.uint8)
        upperBound = np.array([255, 255, 255], dtype=np.uint8)

        # Create a mask using the color range
        mask = cv2.inRange(cv_image, lowerBound, upperBound)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        middle_points = [] 

        # Iterate through each contour
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)

            # If the area of the contour is greater than a threshold, draw a bounding box
            if 1000 < area < 10000:
                color = (255, 0, 0)  # Red color for the bounding box
                thickness = 2
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), color, thickness)
                middle_x = x + w // 2
                middle_y = y + h // 2
                middle_point = (middle_x, middle_y)
                middle_points.append(middle_point)

        cv2.imshow("Camera Feed Subscriber 1", cv_image)
        key = cv2.waitKey(1)

        # Publish a message to the custom topic
        if key & 0xFF == ord('q'):
            # Publish middle points as integer values
            self.publish_middle_points(middle_points)
            # Publish the number of circles
            self.publish_area_count(len(middle_points))
            # Publish a message to the custom topic
            self.publish_custom_message()
            # Print the number of circles and their middle points
            print(f"{len(middle_points)} area(s) found when 'a' was clicked:")
            for idx, point in enumerate(middle_points):
                print(f"Area {idx+1}: Middle Point = {point}")

    def publish_custom_message(self):
        custom_msg = String()
        custom_msg.data = "Hello from Subscriber Node 1"
        self.publisher_.publish(custom_msg)

    def publish_middle_points(self, middle_points):
        for point in middle_points:
            middle_msg = Int32()
            # Assuming you want to publish the x-coordinate of the middle point
            middle_msg.data = point[0]  
            self.middle_publisher_.publish(middle_msg)

    def publish_area_count(self, count):
        count_msg = Int32()
        count_msg.data = count
        self.count_publisher_.publish(count_msg)

def main(args=None):
    rclpy.init(args=args)
    area = DisplaySubscriberNode1()
    rclpy.spin(area)
    area.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
