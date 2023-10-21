"""
Autor: Fernando Vela Hidalgo (https://github.com/fervh)
Asignatura: Introducción a la Planificación de Robots
Universidad: Universidad Carlos III de Madrid (UC3M)
Fecha: Octubre 2023

Descripción:
El nodo llamado 'BFSSolverNode' está diseñado para resolver problemas de búsqueda de rutas utilizando el algoritmo BFS. 
Este nodo se encarga de planificar una ruta óptima desde un punto de inicio a un punto de destino en un mapa representado como una cuadrícula de celdas. 
El nodo recibe información sobre el punto de inicio, el punto de destino y el mapa del entorno, y luego calcula y publica la ruta óptima a seguir.

BFS comienza desde el nodo raíz y explora todos los nodos vecinos a una profundidad dada antes de moverse a los nodos de la siguiente profundidad. 
Se implementa comúnmente utilizando una estructura de datos cola (queue) para llevar un registro de los nodos que deben explorarse a continuación.

Explicación de 'Queue':
La clase 'Queue' proporciona una estructura de datos de cola, que es fundamental para implementar BFS. 
Permite agregar elementos al final de la cola y quitar elementos del principio.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, GridCells
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from queue import Queue
import time

# Define una clase llamada BFSSolverNode para el nodo principal.
class BFSSolverNode(Node):

    def __init__(self):

        # Inializa valores iniciales.
        super().__init__('bfs_solver_node')
        self.start_point = None
        self.end_point = None
        self.grid = None
        self.route = []

        # Establece 'sleep_time' para reproducir o simular de manera más lenta/rápida.
        self.declare_parameter('sleep_time', 0.1)

        # Crea suscripciones a los tópicos de inicio, destino y mapa, y publicaciones para la ruta óptima y celdas visitadas.
        self.start_sub = self.create_subscription(PointStamped, 'start_point', self.start_callback, 10)
        self.end_sub = self.create_subscription(PointStamped, 'end_point', self.end_callback, 10)
        self.grid_sub = self.create_subscription(OccupancyGrid, 'occupancy_grid', self.grid_callback, 10)
        self.route_pub = self.create_publisher(GridCells, 'optimal_route', 10)
        self.visited_pub = self.create_publisher(GridCells, 'visited_grid', 10)
    
    # Definición de funciones de devolución de llamada para los tópicos de inicio, destino y mapa.
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

    # Función para resolver el problema BFS.
    def solve(self):
        # Limpia la lista de rutas anteriores
        self.route = []

        # Obtención de dimensiones y datos del mapa.
        grid_width = self.grid.info.width
        grid_height = self.grid.info.height
        grid_data = self.grid.data

        # Comprobar si es válida la celda (dentro de los límites y no es un obstáculo (!=1)).
        def is_valid(x, y):
            return 0 <= x < grid_width and 0 <= y < grid_height and grid_data[y * grid_width + x] == 0

        visited = set()
        queue = Queue()
        parent = {} # Diccionario con los padres de las celdas.

        # Cálculo de coordenadas en el mapa para inicio y destino.
        start_x = int((self.start_point.point.x - self.grid.info.origin.position.x) / self.grid.info.resolution)
        start_y = int((self.start_point.point.y - self.grid.info.origin.position.y) / self.grid.info.resolution)
        end_x = int((self.end_point.point.x - self.grid.info.origin.position.x) / self.grid.info.resolution)
        end_y = int((self.end_point.point.y - self.grid.info.origin.position.y) / self.grid.info.resolution)

        queue.put((start_x, start_y))
        visited.add((start_x, start_y))

        while not queue.empty():
            x, y = queue.get()

            # Si llegamos al destino, terminamos.
            if x == end_x and y == end_y:
                break

            # Exploramos los vecinos del nodo actual en 4 direcciones: arriba, abajo, izquierda, derecha.
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = x + dx, y + dy

                # Verifica si el vecino es válido (dentro del mapa y no obstáculo).
                if is_valid(nx, ny) and (nx, ny) not in visited:
                    queue.put((nx, ny))
                    visited.add((nx, ny))
                    parent[(nx, ny)] = (x, y) # Agrega la celda visitada a la "queue" y la anterior a la celda "parent (padre)".

                    # Publica las casillas visitadas en el tópico visited_grid
                    visited_cells = [Point(x=x * self.grid.info.resolution + self.grid.info.origin.position.x + 0.5,
                                            y=y * self.grid.info.resolution + self.grid.info.origin.position.y + 0.5,
                                            z=0.0) for x, y in visited]

                    # Publica las celdas visitadas en el tópico 'visited_grid'.
                    visited_grid_msg = GridCells()
                    visited_grid_msg.header = Header()
                    visited_grid_msg.header.frame_id = 'map'
                    visited_grid_msg.cell_width = self.grid.info.resolution
                    visited_grid_msg.cell_height = self.grid.info.resolution
                    visited_grid_msg.cells = visited_cells
                    self.visited_pub.publish(visited_grid_msg)

                    sleep_time = self.get_parameter('sleep_time').value
                    time.sleep(sleep_time)

        # Reconstruye la ruta óptima si se encontró una solución.
        if (end_x, end_y) in parent:
            current_x, current_y = end_x, end_y
            while (current_x, current_y) != (start_x, start_y):
                self.route.append(Point(x=current_x * self.grid.info.resolution + self.grid.info.origin.position.x + 0.5,
                                        y=current_y * self.grid.info.resolution + self.grid.info.origin.position.y + 0.5,
                                        z=0.0))
                current_x, current_y = parent[(current_x, current_y)]
            self.route.reverse()

            # Publica la ruta óptima en el tópico 'optimal_route'.
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
