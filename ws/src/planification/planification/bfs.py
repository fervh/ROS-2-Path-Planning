import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, GridCells
from geometry_msgs.msg import Point
from std_msgs.msg import Header
# Utilizaremos la biblioteca queue, que permitira gestionar la cola de nodos a explorar en el mapa, asegurando que se explore primero el nodo más antiguo, lo que permite encontrar la ruta óptima en un entorno de navegación robótica.
from queue import Queue
# La biblioteca time la utilizamos para poner un tiempo de espera entre cada iteracion y poder observar el funcionamiento del algoritmo.
import time

class BFSSolverNode(Node):

    def __init__(self):
        super().__init__('bfs_solver_node')
        self.start_point = None
        self.end_point = None
        self.grid = None
        self.route = []

        self.declare_parameter('sleep_time', 0.1)

        # Suscripción a los tópicos de inicio y fin
        self.start_sub = self.create_subscription(PointStamped, 'start_point', self.start_callback, 10)
        self.end_sub = self.create_subscription(PointStamped, 'end_point', self.end_callback, 10)

        # Suscripción al tópico de mapa de ocupación
        self.grid_sub = self.create_subscription(OccupancyGrid, 'occupancy_grid', self.grid_callback, 10)

        # Publicación de la ruta óptima en el tópico GridCells
        self.route_pub = self.create_publisher(GridCells, 'optimal_route', 10)

        # Publicación de las casillas visitadas en el tópico GridCells
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
        # Limpia la lista de rutas anteriores
        self.route = []

        # Implementa el algoritmo BFS para encontrar la ruta óptima
        grid_width = self.grid.info.width
        grid_height = self.grid.info.height
        grid_data = self.grid.data

        def is_valid(x, y):
            return 0 <= x < grid_width and 0 <= y < grid_height and grid_data[y * grid_width + x] == 0

        visited = set()
        queue = Queue()
        parent = {}

        start_x = int((self.start_point.point.x - self.grid.info.origin.position.x) / self.grid.info.resolution)
        start_y = int((self.start_point.point.y - self.grid.info.origin.position.y) / self.grid.info.resolution)
        end_x = int((self.end_point.point.x - self.grid.info.origin.position.x) / self.grid.info.resolution)
        end_y = int((self.end_point.point.y - self.grid.info.origin.position.y) / self.grid.info.resolution)

        queue.put((start_x, start_y))
        visited.add((start_x, start_y))

        while not queue.empty():
            x, y = queue.get()

            if x == end_x and y == end_y:
                break

            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = x + dx, y + dy
                if is_valid(nx, ny) and (nx, ny) not in visited:
                    queue.put((nx, ny))
                    visited.add((nx, ny))
                    parent[(nx, ny)] = (x, y)

                    # Publica las casillas visitadas en el tópico visited_grid
                    visited_cells = [Point(x=x * self.grid.info.resolution + self.grid.info.origin.position.x + 0.5,
                                            y=y * self.grid.info.resolution + self.grid.info.origin.position.y + 0.5,
                                            z=0.0) for x, y in visited]

                    visited_grid_msg = GridCells()
                    visited_grid_msg.header = Header()
                    visited_grid_msg.header.frame_id = 'map'
                    visited_grid_msg.cell_width = self.grid.info.resolution
                    visited_grid_msg.cell_height = self.grid.info.resolution
                    visited_grid_msg.cells = visited_cells
                    self.visited_pub.publish(visited_grid_msg)

                    sleep_time = self.get_parameter('sleep_time').value
                    time.sleep(sleep_time)

        # Reconstruye la ruta óptima desde el punto final al punto inicial
        if (end_x, end_y) in parent:
            current_x, current_y = end_x, end_y
            while (current_x, current_y) != (start_x, start_y):
                self.route.append(Point(x=current_x * self.grid.info.resolution + self.grid.info.origin.position.x + 0.5,
                                        y=current_y * self.grid.info.resolution + self.grid.info.origin.position.y + 0.5,
                                        z=0.0))
                current_x, current_y = parent[(current_x, current_y)]
            self.route.reverse()

            # Publica la ruta óptima en el tópico GridCells
            grid_cells_msg = GridCells()
            grid_cells_msg.header = Header()
            grid_cells_msg.header.frame_id = 'map'
            grid_cells_msg.cell_width = self.grid.info.resolution
            grid_cells_msg.cell_height = self.grid.info.resolution
            grid_cells_msg.cells = self.route
            self.route_pub.publish(grid_cells_msg)

        

def main(args=None):
    rclpy.init(args=args)
    node = BFSSolverNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
