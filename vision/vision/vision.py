import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import numpy as np

class CountAggregator(Node):
    def __init__(self):
        super().__init__('count_aggregator')
        
        self.circle_x = np.array([], dtype=np.int32)
        self.circle_y = np.array([], dtype=np.int32)
        self.area_maxX = np.array([], dtype=np.int32)
        self.area_minX = np.array([], dtype=np.int32)
        self.area_maxY = np.array([], dtype=np.int32)
        self.area_minY = np.array([], dtype=np.int32)

        self.middle_x_subscription = self.create_subscription(
            Int32,
            'circle/middle_x',
            self.middle_x_callback,
            10)
        self.middle_y_subscription = self.create_subscription(
            Int32,
            'circle/middle_y',
            self.middle_y_callback,
            10)
        self.max_x_subscription = self.create_subscription(
            Int32,
            'area/max_x',
            self.max_x_callback,
            10)
        self.min_x_subscription = self.create_subscription(
            Int32,
            'area/min_x',
            self.min_x_callback,
            10)
        self.max_y_subscription = self.create_subscription(
            Int32,
            'area/max_y',
            self.max_y_callback,
            10)
        self.min_y_subscription = self.create_subscription(
            Int32,
            'area/min_y',
            self.min_y_callback,
            10)

    def middle_x_callback(self, msg):
        self.circle_x = np.append(self.circle_x, msg.data)
        self.display_positions()

    def middle_y_callback(self, msg):
        self.circle_y = np.append(self.circle_y, msg.data)
        self.display_positions()

    def max_x_callback(self, msg):
        self.area_maxX = np.append(self.area_maxX, msg.data)
        self.display_positions()

    def min_x_callback(self, msg):
        self.area_minX = np.append(self.area_minX, msg.data)
        self.display_positions()

    def max_y_callback(self, msg):
        self.area_maxY = np.append(self.area_maxY, msg.data)
        self.display_positions()

    def min_y_callback(self, msg):
        self.area_minY = np.append(self.area_minY, msg.data)
        self.display_positions()

    def display_positions(self):
        try:
            print("Circle Positions:")
            for i in range(len(self.circle_x)):
                print(f"Circle {i+1}: {self.circle_x[i]}, {self.circle_y[i]}")
        except IndexError:
            print("No circles detected")

        try:
            print("Area Positions:")
            for i in range(len(self.area_maxX)):
                print(f"Area {i+1}: {self.area_maxX[i]}, {self.area_minX[i]}, {self.area_maxY[i]}, {self.area_minY[i]}")
        except IndexError:
            print("No areas detected")

def main(args=None):
    rclpy.init(args=args)
    count_aggregator = CountAggregator()
    rclpy.spin(count_aggregator)
    count_aggregator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
