import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class ClickHandlerNode(Node):
    def __init__(self):
        super().__init__('click_handler_node')
        self.subscription = self.create_subscription(PointStamped, 'clicked_point', self.clicked_point_callback, 10)
        self.start_point = None
        self.end_point = None
        self.x = 0

    def clicked_point_callback(self, msg):
        msg.point.x = float(int(msg.point.x))+0.5
        msg.point.y = float(int(msg.point.y))+0.5
        msg.point.z = float(int(msg.point.z))
        if self.x == 0:
            self.start_point = msg
            self.get_logger().info(f"Start Point: ({msg.point.x}, {msg.point.y}, {msg.point.z})")
            self.x = 1
        else:
            self.end_point = msg
            self.get_logger().info(f"End Point: ({msg.point.x}, {msg.point.y}, {msg.point.z})")
            self.publish_start_end_points()
            self.x = 0

    def publish_start_end_points(self):
        if self.start_point is not None and self.end_point is not None:
            self.get_logger().info("Publishing start and end points")
            self.create_publisher(PointStamped, 'start_point', 10).publish(self.start_point)
            self.create_publisher(PointStamped, 'end_point', 10).publish(self.end_point)

def main(args=None):
    rclpy.init(args=args)
    node = ClickHandlerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
