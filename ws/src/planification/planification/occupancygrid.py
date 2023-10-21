"""
Autor: Fernando Vela Hidalgo (https://github.com/fervh)
Asignatura: Introducción a la Planificación de Robots
Universidad: Universidad Carlos III de Madrid (UC3M)
Fecha: Octubre 2023

Descripción:
El nodo OccupancyGridPublisher convierte datos de una matriz en 2D provenientes de tópicos 'matriz_2d' y 'dimensiones' en un mensaje de tipo OccupancyGrid ('occupancy_grid').
Esto es útil para representarlo fácilmente utilizando RVIZ2 en ROS2.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import OccupancyGrid

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_publisher')

        # Crea subscribers a los topic 'matriz_2d' y 'dimensiones'.
        self.subscription_matrix = self.create_subscription(Int32MultiArray, 'matriz_2d', self.matrix_callback, 10)
        self.subscription_dimensions = self.create_subscription(Int32MultiArray, 'dimensiones', self.dimensions_callback, 10)
        
        # Crea un publisher para el topic 'occupancy_grid'.
        self.publisher = self.create_publisher(OccupancyGrid, 'occupancy_grid', 10)
        self.occupancy_grid = None

    # Convierte los valores de la matriz en datos de ocupación y publica la cuadrícula.
    def matrix_callback(self, msg):
        if self.occupancy_grid is not None:
            occupancy_data = [0 if val == 0 else 100 for val in msg.data]
            self.occupancy_grid.data = occupancy_data
            self.publisher.publish(self.occupancy_grid)

    # Inicializa la cuadrícula de ocupación con dimensiones y valores predeterminados.
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
