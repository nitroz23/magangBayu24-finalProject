import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
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

        # Create publishers for new topics
        self.max_x_publisher_ = self.create_publisher(Int32, 'area/max_x', 10)
        self.min_x_publisher_ = self.create_publisher(Int32, 'area/min_x', 10)
        self.max_y_publisher_ = self.create_publisher(Int32, 'area/max_y', 10)
        self.min_y_publisher_ = self.create_publisher(Int32, 'area/min_y', 10)
        self.middle_x_publisher_ = self.create_publisher(Int32, 'area/middle_x', 10)
        self.middle_y_publisher_ = self.create_publisher(Int32, 'area/middle_y', 10)


    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Define the color range
        lowerBound = np.array([150, 150, 100], dtype=np.uint8)
        upperBound = np.array([255, 255, 255], dtype=np.uint8)

        # Create a mask using the color range
        mask = cv2.inRange(cv_image, lowerBound, upperBound)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        max_x_points = []  # List to store maximum x-coordinates
        min_x_points = []  # List to store minimum x-coordinates
        max_y_points = []  # List to store maximum y-coordinates
        min_y_points = []  # List to store minimum y-coordinates
        middle_x_points = []
        middle_y_points = []

        # Iterate through each contour
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)

            # If the area of the contour is within a certain range, draw a bounding box
            if 1000 < area < 10000:
                color = (255, 0, 0)  # Red color for the bounding box
                thickness = 2
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), color, thickness)

                max_x_points.append(x + w)
                min_x_points.append(x)
                max_y_points.append(y + h)
                min_y_points.append(y)

                middle_x = x + w // 2
                middle_y = y + h // 2
                middle_x_points.append(middle_x)
                middle_y_points.append(middle_y)


        cv2.imshow("area", cv_image)
        key = cv2.waitKey(1)

        # Publish a message to the custom topic
        if key & 0xFF == ord('q'):
            # Publish middle points as integer values
            self.publish_points(max_x_points, min_x_points, max_y_points, min_y_points, middle_x_points, middle_y_points)
            # Print the number of areas and their coordinates
            print(f"{len(max_x_points)} area(s) found when 'q' was clicked:")
            for idx, (max_x, min_x, max_y, min_y) in enumerate(zip(max_x_points, min_x_points, max_y_points, min_y_points)):
                print(f"Area {idx+1}: Max X = {max_x}, Min X = {min_x}, Max Y = {max_y}, Min Y = {min_y}")

    def publish_points(self, max_x_points, min_x_points, max_y_points, min_y_points, middle_x_points, middle_y_points):
        for max_x, min_x, max_y, min_y, middle_x, middle_y in zip(max_x_points, min_x_points, max_y_points, min_y_points, middle_x_points, middle_y_points):
            max_x_msg = Int32()
            max_x_msg.data = max_x
            self.max_x_publisher_.publish(max_x_msg)

            min_x_msg = Int32()
            min_x_msg.data = min_x
            self.min_x_publisher_.publish(min_x_msg)

            max_y_msg = Int32()
            max_y_msg.data = max_y
            self.max_y_publisher_.publish(max_y_msg)

            min_y_msg = Int32()
            min_y_msg.data = min_y
            self.min_y_publisher_.publish(min_y_msg)

            middle_x_msg = Int32()
            middle_x_msg.data = middle_x
            self.middle_x_publisher_.publish(middle_x_msg)

            middle_y_msg = Int32()
            middle_y_msg.data = middle_y
            self.middle_y_publisher_.publish(middle_y_msg)

def main(args=None):
    rclpy.init(args=args)
    area = DisplaySubscriberNode1()
    rclpy.spin(area)
    area.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
