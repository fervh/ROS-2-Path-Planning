# ROS 2 Path Planning

Este es un workspace de ROS 2 que se utiliza para la planificación de rutas utilizando los algoritmos BFS, GREEDY y ASTAR. El paquete `planification` contiene los nodos y scripts necesarios para generar mapas y ejecutar la planificación de rutas.

## Requisitos

- ROS 2 Foxy o superior (En mi caso he utilizado ROS 2 GALACTIC)
- Python 3


## Instalación

Asegúrate de que tengas ROS 2 instalado y configurado correctamente en tu sistema. Luego, clona este repositorio en tu espacio de trabajo de ROS 2 y compila los paquetes.

```bash
git clone https://github.com/fervh/ROS-2-Path-Planning.git

cd ws

source /opt/ros/galactic/setup.bash

colcon build --symlink-install

source install/local_setup.bash
```

## Uso

### Generar un Mapa

Para generar un mapa, utiliza el siguiente comando:
```bash
cd ws/src/planification/data
python3 generate_map.py --map map12.csv --rows 50 --columns 50
```
Este comando generará un archivo CSV llamado map12.csv con 50 filas y 50 columnas. Puedes ajustar los valores de `--map` , `--rows` y `--columns` según tus necesidades.

### Ejecutar la Planificación de Rutas

Para ejecutar la planificación de rutas con un algoritmo específico, utiliza el siguiente comando:
```bash
ros2 launch planification general.launch.py csv_file:=map12.csv mode:=bfs sleep_time:=0.01
```
Asegúrate de reemplazar `bfs` con el algoritmo que deseas utilizar: `bfs`, `greedy` o `astar`. Puedes ajustar el valor de `sleep_time` según tus preferencias (utiliza un float).

Una vez ejecutado, puedes utilizar `Publish Point` para establecer los puntos de inicio y final de la ruta

![Publish Point](/media/img1.png)

### Ejemplo de la Planificación de Rutas
![Ejemplo](/media/img6.png)
[VIDEO EJEMPLO](/media/video1.mp4)
