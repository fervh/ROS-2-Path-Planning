import argparse
import random
import csv

def generate_maze(rows, columns):
    maze = [['1' for _ in range(columns)] for _ in range(rows)]

    start_row = random.randrange(0, rows, 2)
    start_col = random.randrange(0, columns, 2)
    maze[start_row][start_col] = '0'

    frontier = [(start_row, start_col)]
    while frontier:
        current = frontier.pop()
        row, col = current

        neighbors = [(row+2, col), (row-2, col), (row, col+2), (row, col-2)]
        random.shuffle(neighbors)

        for nrow, ncol in neighbors:
            if 0 <= nrow < rows and 0 <= ncol < columns and maze[nrow][ncol] == '1':
                maze[nrow][ncol] = '0'
                maze[(nrow+row)//2][(ncol+col)//2] = '0'
                frontier.append((nrow, ncol))

    return maze

def save_maze_to_csv(maze, filename):
    with open(filename, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        for row in maze:
            writer.writerow(row)

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

