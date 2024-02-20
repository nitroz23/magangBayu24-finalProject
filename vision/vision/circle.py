import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge
import cv2
import numpy as np

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

        self.publisher_ = self.create_publisher(String, 'circle/msgs', 10)
        self.middle_x_publisher_ = self.create_publisher(Int32, 'circle/middle_x', 10)
        self.middle_y_publisher_ = self.create_publisher(Int32, 'circle/middle_y', 10)
        self.count_publisher_ = self.create_publisher(Int32, 'circle/count', 10)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Define the color range
        lowerBound = np.array([100, 100, 200], dtype=np.uint8)
        upperBound = np.array([200, 200, 250], dtype=np.uint8)

        # Create a mask using the color range
        mask = cv2.inRange(cv_image, lowerBound, upperBound)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        middle_x_points = []  # List to store x-coordinates of middle points
        middle_y_points = []  # List to store y-coordinates of middle points

        # Iterate through each contour
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            area = cv2.contourArea(contour)

            # If the area of the contour is greater than a threshold, draw a bounding box
            if area > 750:
                color = (255, 0, 0)  # Red color for the bounding box
                thickness = 2
                cv2.rectangle(cv_image, (x, y), (x + w, y + h), color, thickness)

                # Calculate middle point of bounding box
                middle_x = x + w // 2
                middle_y = y + h // 2
                middle_x_points.append(middle_x)
                middle_y_points.append(middle_y)

        # Display the processed images
        cv2.imshow("circle", cv_image)

        # Wait for key press
        key = cv2.waitKey(1)

        # Check if the pressed key is "a"
        if key & 0xFF == ord('a'):
            # Publish middle points as integer values
            self.publish_middle_points(middle_x_points, middle_y_points)
            # Publish the number of circles
            self.publish_circle_count(len(middle_x_points))
            # Publish a message to the custom topic
            self.publish_custom_message()
            # Print the number of circles and their middle points
            print(f"{len(middle_x_points)} circle(s) found when 'a' was clicked:")
            for idx, (x, y) in enumerate(zip(middle_x_points, middle_y_points)):
                print(f"Circle {idx+1}: Middle Point = ({x}, {y})")

    def publish_custom_message(self):
        custom_msg = String()
        custom_msg.data = "Hello from Subscriber Node 2"
        self.publisher_.publish(custom_msg)

    def publish_middle_points(self, middle_x_points, middle_y_points):
        for x, y in zip(middle_x_points, middle_y_points):
            middle_x_msg = Int32()
            middle_x_msg.data = x
            self.middle_x_publisher_.publish(middle_x_msg)

            middle_y_msg = Int32()
            middle_y_msg.data = y
            self.middle_y_publisher_.publish(middle_y_msg)

    def publish_circle_count(self, count):
        count_msg = Int32()
        count_msg.data = count
        self.count_publisher_.publish(count_msg)

def main(args=None):
    rclpy.init(args=args)
    circle = DisplaySubscriberNode2()
    rclpy.spin(circle)
    circle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
