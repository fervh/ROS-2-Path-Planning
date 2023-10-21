"""
Autor: Fernando Vela Hidalgo (https://github.com/fervh)
Asignatura: Introducción a la Planificación de Robots
Universidad: Universidad Carlos III de Madrid (UC3M)
Fecha: Octubre 2023

Descripción:
ClickHandlerNode actúa como un intermediario que permite a un usuario especificar los puntos de inicio y destino en un mapa utilizando RVIZ2 de ROS2.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

# Define una clase/nodo llamada ClickHandlerNode.
class ClickHandlerNode(Node):

    def __init__(self):

        # Inializa valores iniciales.
        super().__init__('click_handler_node')
        self.subscription = self.create_subscription(PointStamped, 'clicked_point', self.clicked_point_callback, 10)
        self.start_point = None
        self.end_point = None
        self.x = 0

    def clicked_point_callback(self, msg):
        # Ajusta las coordenadas de punto para que estén en el centro de la celda.
        msg.point.x = float(int(msg.point.x))+0.5
        msg.point.y = float(int(msg.point.y))+0.5
        msg.point.z = float(int(msg.point.z))

        # Registra el punto de inicio.
        if self.x == 0:
            self.start_point = msg
            self.get_logger().info(f"Start Point: ({msg.point.x}, {msg.point.y}, {msg.point.z})")
            self.x = 1

        # Registra el punto de final.
        else:
            self.end_point = msg
            self.get_logger().info(f"End Point: ({msg.point.x}, {msg.point.y}, {msg.point.z})")
            
            # Llama a la función para publicar los puntos de inicio y destino.
            self.publish_start_end_points()
            self.x = 0

    # Publica los puntos de inicio y destino.
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
