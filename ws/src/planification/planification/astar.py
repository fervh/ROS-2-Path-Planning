import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, GridCells
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from queue import PriorityQueue  # Cambio a PriorityQueue en lugar de Queue
import math
import time

class AStarSolverNode(Node):  # Cambio de nombre de la clase

    def __init__(self):
        super().__init__('a_star_solver_node')  # Cambio del nombre del nodo
        self.start_point = None
        self.end_point = None
        self.grid = None
        self.route = []

        self.declare_parameter('sleep_time', 0.1)

        self.start_sub = self.create_subscription(PointStamped, 'start_point', self.start_callback, 10)
        self.end_sub = self.create_subscription(PointStamped, 'end_point', self.end_callback, 10)
        self.grid_sub = self.create_subscription(OccupancyGrid, 'occupancy_grid', self.grid_callback, 10)
        self.route_pub = self.create_publisher(GridCells, 'optimal_route', 10)
        self.visited_pub = self.create_publisher(GridCells, 'visited_grid', 10)

    def start_callback(self, msg):
        self.start_point = msg

    def end_callback(self, msg):
        self.end_point = msg
        if self.start_point is not None and self.grid is not None:
            self.solve()

    def grid_callback(self, msg):
        self.grid = msg
        if self.start_point is not None and self.end_point is not None:
            self.solve()

    def solve(self):
        self.route = []

        grid_width = self.grid.info.width
        grid_height = self.grid.info.height
        grid_data = self.grid.data

        def is_valid(x, y):
            return 0 <= x < grid_width and 0 <= y < grid_height and grid_data[y * grid_width + x] == 0

        visited = set()
        priority_queue = PriorityQueue()
        g_cost = {}
        f_cost = {}

        start_x = int((self.start_point.point.x - self.grid.info.origin.position.x) / self.grid.info.resolution)
        start_y = int((self.start_point.point.y - self.grid.info.origin.position.y) / self.grid.info.resolution)
        end_x = int((self.end_point.point.x - self.grid.info.origin.position.x) / self.grid.info.resolution)
        end_y = int((self.end_point.point.y - self.grid.info.origin.position.y) / self.grid.info.resolution)

        priority_queue.put((0, (start_x, start_y)))
        g_cost[(start_x, start_y)] = 0
        f_cost[(start_x, start_y)] = self.heuristic(start_x, start_y, end_x, end_y)

        while not priority_queue.empty():
            _, (x, y) = priority_queue.get()

            if x == end_x and y == end_y:
                break

            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = x + dx, y + dy
                if is_valid(nx, ny):
                    tentative_g = g_cost[(x, y)] + 1 
                    if (nx, ny) not in g_cost or tentative_g < g_cost[(nx, ny)]:
                        g_cost[(nx, ny)] = tentative_g
                        f_cost[(nx, ny)] = tentative_g + self.heuristic(nx, ny, end_x, end_y)
                        priority_queue.put((f_cost[(nx, ny)], (nx, ny)))

                        visited.add((nx, ny))

                        visited_cells = [Point(x=nx * self.grid.info.resolution + self.grid.info.origin.position.x + 0.5,
                                                y=ny * self.grid.info.resolution + self.grid.info.origin.position.y + 0.5,
                                                z=0.0) for nx, ny in visited]

                        visited_grid_msg = GridCells()
                        visited_grid_msg.header = Header()
                        visited_grid_msg.header.frame_id = 'map'
                        visited_grid_msg.cell_width = self.grid.info.resolution
                        visited_grid_msg.cell_height = self.grid.info.resolution
                        visited_grid_msg.cells = visited_cells
                        self.visited_pub.publish(visited_grid_msg)

                        sleep_time = self.get_parameter('sleep_time').value
                        time.sleep(sleep_time)

        if (end_x, end_y) in g_cost:
            current_x, current_y = end_x, end_y
            while (current_x, current_y) != (start_x, start_y):
                self.route.append(Point(x=current_x * self.grid.info.resolution + self.grid.info.origin.position.x + 0.5,
                                        y=current_y * self.grid.info.resolution + self.grid.info.origin.position.y + 0.5,
                                        z=0.0))
                current_x, current_y = self.get_parent(g_cost, current_x, current_y)

            self.route.reverse()

            grid_cells_msg = GridCells()
            grid_cells_msg.header = Header()
            grid_cells_msg.header.frame_id = 'map'
            grid_cells_msg.cell_width = self.grid.info.resolution
            grid_cells_msg.cell_height = self.grid.info.resolution
            grid_cells_msg.cells = self.route
            self.route_pub.publish(grid_cells_msg)

    def heuristic(self, x, y, end_x, end_y):
        return math.sqrt((x - end_x) ** 2 + (y - end_y) ** 2)

    def get_parent(self, g_cost, x, y):
        for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
            nx, ny = x + dx, y + dy
            if (nx, ny) in g_cost and g_cost[(nx, ny)] == g_cost[(x, y)] - 1:
                return nx, ny

def main(args=None):
    rclpy.init(args=args)
    node = AStarSolverNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
