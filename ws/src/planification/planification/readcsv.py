import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import csv

import os
from ament_index_python.packages import get_package_share_directory

package_share_directory = get_package_share_directory('planification')


class CSVReaderNode(Node):
    def __init__(self):
        super().__init__('csv_reader_node')
        self.csv_file = self.declare_parameter('csv_file', 'map2.csv').value  # Lee el parámetro 'csv_file'
        self.publisher = self.create_publisher(Int32MultiArray, 'matriz_2d', 10)
        self.dimension_publisher = self.create_publisher(Int32MultiArray, 'dimensiones', 10)
        timer_period = 1
        self.timer = self.create_timer(timer_period, self.read_csv_and_publish)

    def read_csv_and_publish(self):
        csv_file = os.path.join(package_share_directory, 'data', self.csv_file)
        with open(csv_file, 'r') as file:
            csv_reader = csv.reader(file)
            matrix_data = list(csv_reader)

        num_rows = len(matrix_data)
        num_cols = len(matrix_data[0])

        dimension_message = Int32MultiArray(data=[num_cols, num_rows])
        self.dimension_publisher.publish(dimension_message)

        matrix = Int32MultiArray()
        for row in matrix_data:
            matrix.data.extend(map(int, row))

        self.publisher.publish(matrix)

def main(args=None):
    rclpy.init(args=args)
    csv_reader_node = CSVReaderNode()
    rclpy.spin(csv_reader_node)
    csv_reader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()