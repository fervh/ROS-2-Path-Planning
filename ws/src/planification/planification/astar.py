"""
Autor: Fernando Vela Hidalgo (https://github.com/fervh)
Asignatura: Introducción a la Planificación de Robots
Universidad: Universidad Carlos III de Madrid (UC3M)
Fecha: Octubre 2023

Descripción:
El nodo llamado 'AStarSolverNode' está diseñado para resolver problemas de búsqueda de rutas utilizando el algoritmo A* (A estrella). 
Este nodo se encarga de planificar una ruta óptima desde un punto de inicio a un punto de destino en un mapa representado como una cuadrícula de celdas. 
El nodo recibe información sobre el punto de inicio, el punto de destino y el mapa del entorno, y luego calcula y publica la ruta óptima a seguir.

Como se puede observar luego es muy similar al BFS pero le agrega heurística/costos al algoritmo.

Explicación de 'PriorityQueue':
El nodo utiliza 'PriorityQueue' para administrar los nodos que se están explorando durante el proceso de búsqueda de rutas. 
Se utiliza para elegir el próximo nodo a explorar en función de su costo estimado y prioridad. 
Esta elección se basa en una estimación heurística de cuánto costaría llegar desde el nodo actual al destino. 

Para la heuristica y calculo de costes se utilizan:
'g_cost': costo acumulado desde el punto de inicio hasta cada celda del mapa.
'f_cost': costo total estimado (g + heurística) desde una celda hasta el destino.

Más información:
https://levelup.gitconnected.com/a-star-a-search-for-solving-a-maze-using-python-with-visualization-b0cae1c3ba92

"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid, GridCells
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from queue import PriorityQueue
import math
import time

# Define una clase llamada AStarSolverNode para el nodo principal.
class AStarSolverNode(Node):

    def __init__(self):

        # Inializa valores iniciales.
        super().__init__('a_star_solver_node')
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


    # Función para resolver el problema A* (A estrella).
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
        g_cost = {} # Diccionario para almacenar el costo g desde el inicio hasta cada celda.
        f_cost = {} # Diccionario para almacenar el costo total f (g + heurística) de cada celda.

        # Cálculo de coordenadas en el mapa para inicio y destino.
        start_x = int((self.start_point.point.x - self.grid.info.origin.position.x) / self.grid.info.resolution)
        start_y = int((self.start_point.point.y - self.grid.info.origin.position.y) / self.grid.info.resolution)
        end_x = int((self.end_point.point.x - self.grid.info.origin.position.x) / self.grid.info.resolution)
        end_y = int((self.end_point.point.y - self.grid.info.origin.position.y) / self.grid.info.resolution)

        # Inicialización del nodo inicial en la cola de prioridad.
        priority_queue.put((0, (start_x, start_y)))
        g_cost[(start_x, start_y)] = 0
        f_cost[(start_x, start_y)] = self.heuristic(start_x, start_y, end_x, end_y)

        while not priority_queue.empty():
            _, (x, y) = priority_queue.get() # Extrae el nodo con menor costo f.

            # Si llegamos al destino, terminamos.
            if x == end_x and y == end_y:
                break
            
            # Exploramos los vecinos del nodo actual en 4 direcciones: arriba, abajo, izquierda, derecha.
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = x + dx, y + dy

                # Verifica si el vecino es válido (dentro del mapa y no obstáculo).
                if is_valid(nx, ny):
                    tentative_g = g_cost[(x, y)] + 1 # Calcula el costo g del vecino.

                    # Si el vecino aún no se ha explorado o tiene un costo menor, actualiza el costo y añade el vecino a la cola de prioridad.
                    if (nx, ny) not in g_cost or tentative_g < g_cost[(nx, ny)]:
                        g_cost[(nx, ny)] = tentative_g
                        f_cost[(nx, ny)] = tentative_g + self.heuristic(nx, ny, end_x, end_y)
                        priority_queue.put((f_cost[(nx, ny)], (nx, ny)))

                        visited.add((nx, ny))

                        # Convierte las celdas visitadas a puntos en el mapa.
                        visited_cells = [Point(x=nx * self.grid.info.resolution + self.grid.info.origin.position.x + 0.5,
                                                y=ny * self.grid.info.resolution + self.grid.info.origin.position.y + 0.5,
                                                z=0.0) for nx, ny in visited]
                        
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
        if (end_x, end_y) in g_cost:
            current_x, current_y = end_x, end_y
            while (current_x, current_y) != (start_x, start_y):
                self.route.append(Point(x=current_x * self.grid.info.resolution + self.grid.info.origin.position.x + 0.5,
                                        y=current_y * self.grid.info.resolution + self.grid.info.origin.position.y + 0.5,
                                        z=0.0))
                current_x, current_y = self.get_parent(g_cost, current_x, current_y)

            self.route.reverse()

            # Publica la ruta óptima en el tópico 'optimal_route'.
            grid_cells_msg = GridCells()
            grid_cells_msg.header = Header()
            grid_cells_msg.header.frame_id = 'map'
            grid_cells_msg.cell_width = self.grid.info.resolution
            grid_cells_msg.cell_height = self.grid.info.resolution
            grid_cells_msg.cells = self.route
            self.route_pub.publish(grid_cells_msg)

    # Función heurística para la estimación de costo.
    def heuristic(self, x, y, end_x, end_y):
        #Estimación del costo (distancia) desde la celda (x, y) hasta el destino (end_x, end_y).
        return math.sqrt((x - end_x) ** 2 + (y - end_y) ** 2)

    # Función para obtener el padre de una celda.
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
