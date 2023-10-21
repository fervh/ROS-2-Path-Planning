"""
Autor: Fernando Vela Hidalgo (https://github.com/fervh)
Asignatura: Introducción a la Planificación de Robots
Universidad: Universidad Carlos III de Madrid (UC3M)
Fecha: Octubre 2023

Descripción:
Este programa genera un laberinto aleatorio y lo guarda en un archivo CSV con las dimensiones especificadas.

Está basado en la "Generación del laberinto perfecto".
1 - Se genera un laberinto lleno de 1
2 - Se elige una celda al azar y se establece con un 0 o como visitada
3 - Examina las celdas que estén separadas por dos casillas (neightbors/vecinos) es decir dejando como minimo una celda en medio.
4 - Elige uno de esos vecinos y le establece un 0 al neighbor y a la casilla que hay entre ambas dos creando un camino.
5 - Sigue avanzando sucesivamente hasta completar todo el mapa
https://en.wikipedia.org/wiki/Maze_generation_algorithm
https://www.uaeh.edu.mx/scige/boletin/huejutla/n1/a4.html

"""

import argparse
import random
import csv

# Función para generar un laberinto con las dimensiones dadas.
def generate_maze(rows, columns):
    # Crea una matriz (laberinto) inicializada con '1' (obstáculos) en todas las celdas.
    maze = [['1' for _ in range(columns)] for _ in range(rows)]

    # Elige una celda aleatoria como punto de partida en una casilla par (2) y la marca como '0'.
    start_row = random.randrange(0, rows, 2)
    start_row = random.randrange(0, rows, 2)
    start_col = random.randrange(0, columns, 2)
    maze[start_row][start_col] = '0'

    # Inicializa una lista llamada "frontier (frontera)" con el punto de partida.
    frontier = [(start_row, start_col)]

    # Mientras haya celdas en el "frontier"...
    while frontier:

        # Selecciona una celda del "frontier".
        current = frontier.pop()
        row, col = current

        # Genera una lista de vecinos en celdas adyacentes de la celda actual y selecciona una aleatoriamente.
        neighbors = [(row+2, col), (row-2, col), (row, col+2), (row, col-2)]
        random.shuffle(neighbors)

        # Para cada vecino potencial...
        for nrow, ncol in neighbors:

            # Comprueba si el vecino está dentro de los límites del laberinto y es una celda "obstáculo" ('1').
            if 0 <= nrow < rows and 0 <= ncol < columns and maze[nrow][ncol] == '1':
                maze[nrow][ncol] = '0' # Marca el vecino como visitado (sin obstáculo (0)).
                maze[(nrow+row)//2][(ncol+col)//2] = '0' # Marca la celda entre el vecino y la actual como visitada.
                frontier.append((nrow, ncol)) # Agrega el vecino al "frontier".

    return maze


# Función para guardar el laberinto en un archivo CSV.
def save_maze_to_csv(maze, filename):
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        for row in maze:
            writer.writerow(row)


# Función principal del programa.
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", type=str, required=True, help="Nombre del archivo CSV del laberinto")
    parser.add_argument("--rows", type=int, required=True, help="Número de filas del laberinto")
    parser.add_argument("--columns", type=int, required=True, help="Número de columnas del laberinto")
    args = parser.parse_args()

    maze = generate_maze(args.rows, args.columns)
    save_maze_to_csv(maze, args.map)

if __name__ == "__main__":
    main()

