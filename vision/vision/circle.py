import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, Int32MultiArray
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
        self.bridge = CvBridge()

        self.middle_x_publisher_ = self.create_publisher(Int32, 'circle/middle_x', 10)
        self.middle_y_publisher_ = self.create_publisher(Int32, 'circle/middle_y', 10)

        self.middle_x_list = []  # List to store x-coordinates of middle points
        self.middle_y_list = []  # List to store y-coordinates of middle points
        self.selected_integers_list = []  # List to store selected integers

        # Create subscriptions to middle_x, middle_y, and selected_integers topics
        self.middle_x_subscription = self.create_subscription(
            Int32,
            'area/middle_x',
            self.middle_x_callback,
            10)

        self.middle_y_subscription = self.create_subscription(
            Int32,
            'area/middle_y',
            self.middle_y_callback,
            10)

        self.selected_integers_subscription = self.create_subscription(
            Int32MultiArray,
            'selected_integers',
            self.selected_integers_callback,
            10)

    def middle_x_callback(self, msg):
        self.middle_x_list.append(msg.data)

    def middle_y_callback(self, msg):
        self.middle_y_list.append(msg.data)

    def selected_integers_callback(self, msg):
        # Clear the list to remove any previously received integers
        self.selected_integers_list.clear()

        # Add the received integers to the list
        for integer in msg.data:
            self.selected_integers_list.append(integer)

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Define the color range
        lowerBound = np.array([100, 80, 140], dtype=np.uint8)
        upperBound = np.array([170, 130, 200], dtype=np.uint8)

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

        # Draw blue circles based on selected_integers_list
        for integer in self.selected_integers_list:
            index = integer - 1
            # Check if the integer is within the range of available middle points
            if 0 <= integer < len(self.middle_x_list):
                # Get the corresponding middle point
                middle_x = self.middle_x_list[index]
                middle_y = self.middle_y_list[index]

                # Define the color blue
                blue_color = (255, 0, 0)

                # Draw circle
                cv2.circle(cv_image, (middle_x, middle_y), 25, blue_color, -1)

        # Display the processed images
        cv2.imshow("circle", cv_image)

        # Wait for key press
        key = cv2.waitKey(1)

        # Check if the pressed key is "a"
        if key & 0xFF == ord('a'):
            # Publish middle points as integer values
            self.publish_middle_points(middle_x_points, middle_y_points)
            # Print the number of circles and their middle points
            print(f"{len(middle_x_points)} circle(s) found when 'a' was clicked:")
            for idx, (x, y) in enumerate(zip(middle_x_points, middle_y_points)):
                print(f"Circle {idx+1}: Middle Point = ({x}, {y})")

    def publish_middle_points(self, middle_x_points, middle_y_points):
        for x, y in zip(middle_x_points, middle_y_points):
            middle_x_msg = Int32()
            middle_x_msg.data = x
            self.middle_x_publisher_.publish(middle_x_msg)

            middle_y_msg = Int32()
            middle_y_msg.data = y
            self.middle_y_publisher_.publish(middle_y_msg)

def main(args=None):
    rclpy.init(args=args)
    circle = DisplaySubscriberNode2()
    rclpy.spin(circle)
    circle.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
