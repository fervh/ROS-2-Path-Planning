import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import OccupancyGrid

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_publisher')
        self.subscription_matrix = self.create_subscription(
            Int32MultiArray,
            'matriz_2d',
            self.matrix_callback,
            10
        )
        self.subscription_dimensions = self.create_subscription(
            Int32MultiArray,
            'dimensiones',
            self.dimensions_callback,
            10
        )
        self.publisher = self.create_publisher(OccupancyGrid, 'occupancy_grid', 10)
        self.occupancy_grid = None

    def matrix_callback(self, msg):
        if self.occupancy_grid is not None:
            occupancy_data = [0 if val == 0 else 100 for val in msg.data]
            self.occupancy_grid.data = occupancy_data
            self.publisher.publish(self.occupancy_grid)

    def dimensions_callback(self, msg):
        self.occupancy_grid = OccupancyGrid()
        self.occupancy_grid.header.frame_id = 'map'
        self.occupancy_grid.info.width = msg.data[0]
        self.occupancy_grid.info.height = msg.data[1]
        self.occupancy_grid.info.resolution = 1.0
        self.occupancy_grid.info.origin.position.x = 0.0
        self.occupancy_grid.info.origin.position.y = 0.0
        self.occupancy_grid.data = [0] * (msg.data[0] * msg.data[1])
        

def main(args=None):
    rclpy.init(args=args)
    occupancy_grid_publisher = OccupancyGridPublisher()
    rclpy.spin(occupancy_grid_publisher)
    occupancy_grid_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
