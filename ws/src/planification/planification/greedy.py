"""
Autor: Fernando Vela Hidalgo (https://github.com/fervh)
Asignatura: Introducción a la Planificación de Robots
Universidad: Universidad Carlos III de Madrid (UC3M)
Fecha: Octubre 2023

Descripción:
El nodo llamado 'GREEDYSolverNode' está diseñado para resolver problemas de búsqueda de rutas utilizando el algoritmo Greedy. 
Este nodo se encarga de planificar una ruta óptima desde un punto de inicio a un punto de destino en un mapa representado como una cuadrícula de celdas. 
El nodo recibe información sobre el punto de inicio, el punto de destino y el mapa del entorno, y luego calcula y publica la ruta óptima a seguir.

El algoritmo se basa en elegir el nodo que se encuentre con una distancia más corta a la meta utilizando costos/prioridad o 'PriorityQueue'.

Explicación de 'PriorityQueue':
El nodo utiliza 'PriorityQueue' para administrar los nodos que se están explorando durante el proceso de búsqueda de rutas. 
Se utiliza para elegir el próximo nodo a explorar en función de su costo estimado y prioridad. 
Esta elección se basa en una estimación heurística de cuánto costaría llegar desde el nodo actual al destino. 


"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, GridCells
from geometry_msgs.msg import Point
from std_msgs.msg import Header
import math
from queue import PriorityQueue
import time

# Define una clase llamada GREEDYSolverNode para el nodo principal.
class GREEDYSolverNode(Node):

    def __init__(self):

        # Inializa valores iniciales.
        super().__init__('greedy_solver_node')
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
        
    def grid_callback(self, msg):
        self.grid = msg
        if self.start_point is not None and self.end_point is not None:
            start_time = time.time()  # Registra el tiempo de inicio.
            self.solve()
            end_time = time.time()  # Registra el tiempo de finalización.
            elapsed_time = end_time - start_time
            self.get_logger().info(f'Time={elapsed_time:.4f} ; Start Point= ({self.start_point.point.x:.2f}, {self.start_point.point.y:.2f}) ; End Point = ({self.end_point.point.x:.2f}, {self.end_point.point.y:.2f})')


    # Función para resolver el problema Greedy.
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
        priority_queue = PriorityQueue()
        parent = {} # Diccionario con los padres de las celdas.

        # Cálculo de coordenadas en el mapa para inicio y destino.
        start_x = int((self.start_point.point.x - self.grid.info.origin.position.x) / self.grid.info.resolution)
        start_y = int((self.start_point.point.y - self.grid.info.origin.position.y) / self.grid.info.resolution)
        end_x = int((self.end_point.point.x - self.grid.info.origin.position.x) / self.grid.info.resolution)
        end_y = int((self.end_point.point.y - self.grid.info.origin.position.y) / self.grid.info.resolution)

        priority_queue.put((0, (start_x, start_y)))
        visited.add((start_x, start_y))

        while not priority_queue.empty():
            _, (x, y) = priority_queue.get() # Extrae el nodo con menor costo.

            # Si llegamos al destino, terminamos.
            if x == end_x and y == end_y:
                break
            
            # Exploramos los vecinos del nodo actual en 4 direcciones: arriba, abajo, izquierda, derecha.
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = x + dx, y + dy

                # Verifica si el vecino es válido (dentro del mapa y no obstáculo).
                if is_valid(nx, ny) and (nx, ny) not in visited:
                    priority = math.sqrt((nx - end_x) ** 2 + (ny - end_y) ** 2) # Selecciona el nodo que se encuentre más cercano a la meta (distancia más corta).
                    priority_queue.put((priority, (nx, ny)))
                    visited.add((nx, ny))
                    parent[(nx, ny)] = (x, y) # Agrega la celda visitada a la "queue (cola)" y la anterior a la celda "parent (padre)".

                    # Publica las casillas visitadas en el tópico visited_grid
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
    node = GREEDYSolverNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
