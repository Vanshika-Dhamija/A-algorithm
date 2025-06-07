import heapq
import pygame
import random
from functools import total_ordering

# Constants
WIDTH, HEIGHT = 600, 600
ROWS, COLS = 20, 20
CELL_SIZE = WIDTH // COLS

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (100, 100, 255)  # Light blue so the maze remains visible
GREEN = (0, 255, 0)
RED = (255, 0, 0)
GRAY = (200, 200, 200)

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("A* Pathfinding with ghost and food")

# Loading images for start and goal
character_img = pygame.image.load("ghost.jpg")
destination_img = pygame.image.load("fruit.jpg")
character_img = pygame.transform.scale(character_img, (CELL_SIZE, CELL_SIZE))
destination_img = pygame.transform.scale(destination_img, (CELL_SIZE, CELL_SIZE))


@total_ordering
class Cell:
    def __init__(self, x, y):
        self.x, self.y = x, y
        self.walls = {"top": True, "right": True, "bottom": True, "left": True}
        self.visited = False
        self.f_score = float("inf")

    def __lt__(self, other):
        return self.f_score < other.f_score

    def __eq__(self, other):
        return (self.x, self.y) == (other.x, other.y)

    def __hash__(self):
        return hash((self.x, self.y))


def heuristic(cell1, cell2):
    """Manhattan distance heuristic for A*."""
    return abs(cell1.x - cell2.x) + abs(cell1.y - cell2.y)


def remove_wall(current, neighbor):
    """Remove walls between two adjacent cells."""
    dx = neighbor.x - current.x
    dy = neighbor.y - current.y

    if dx == 1:  # Neighbor is to the right
        current.walls["right"] = False
        neighbor.walls["left"] = False
    elif dx == -1:  # Neighbor is to the left
        current.walls["left"] = False
        neighbor.walls["right"] = False
    elif dy == 1:  # Neighbor is below
        current.walls["bottom"] = False
        neighbor.walls["top"] = False
    elif dy == -1:  # Neighbor is above
        current.walls["top"] = False
        neighbor.walls["bottom"] = False


def generate_maze(maze):
    """Randomly generates a maze using Depth-First Search (DFS)."""
    stack = []
    start_cell = maze[0][0]
    start_cell.visited = True
    stack.append(start_cell)

    while stack:
        current = stack[-1]
        neighbors = [
            maze[current.x + dx][current.y + dy]
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
            if 0 <= current.x + dx < COLS and 0 <= current.y + dy < ROWS and not maze[current.x + dx][current.y + dy].visited
        ]

        if neighbors:
            neighbor = random.choice(neighbors)
            remove_wall(current, neighbor)
            neighbor.visited = True
            stack.append(neighbor)
        else:
            stack.pop()


def get_neighbors(maze, cell):
    """Returns the valid neighbors of a cell in the maze."""
    neighbors = []
    x, y = cell.x, cell.y

    if y > 0 and not maze[x][y - 1].walls["bottom"]:  # Top neighbor
        neighbors.append(maze[x][y - 1])
    if y < ROWS - 1 and not maze[x][y + 1].walls["top"]:  # Bottom neighbor
        neighbors.append(maze[x][y + 1])
    if x > 0 and not maze[x - 1][y].walls["right"]:  # Left neighbor
        neighbors.append(maze[x - 1][y])
    if x < COLS - 1 and not maze[x + 1][y].walls["left"]:  # Right neighbor
        neighbors.append(maze[x + 1][y])

    return neighbors


def astar(maze, start, goal):
    """A* algorithm to find the shortest path from start to goal."""
    open_set = []
    heapq.heappush(open_set, (0, start))

    g_score = {cell: float("inf") for row in maze for cell in row}
    f_score = {cell: float("inf") for row in maze for cell in row}
    came_from = {}

    g_score[start] = 0
    f_score[start] = heuristic(start, goal)

    while open_set:
        _, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path  # Return the reconstructed path

        for neighbor in get_neighbors(maze, current):
            tentative_g_score = g_score[current] + 1

            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    return None  # No path found


# an empty maze
maze = [[Cell(x, y) for y in range(ROWS)] for x in range(COLS)]

# Generate random walls using DFS
generate_maze(maze)

# start and goal positions
start = maze[0][0]
goal = maze[ROWS - 1][COLS - 1]

# A* algorithm
path = astar(maze, start, goal)

# Pygame loop to visualize the maze and path
running = True
while running:
    screen.fill(WHITE)

    # maze walls
    for row in maze:
        for cell in row:
            x, y = cell.x * CELL_SIZE, cell.y * CELL_SIZE

            if cell.walls["top"]:
                pygame.draw.line(screen, BLACK, (x, y), (x + CELL_SIZE, y), 2)
            if cell.walls["right"]:
                pygame.draw.line(screen, BLACK, (x + CELL_SIZE, y), (x + CELL_SIZE, y + CELL_SIZE), 2)
            if cell.walls["bottom"]:
                pygame.draw.line(screen, BLACK, (x, y + CELL_SIZE), (x + CELL_SIZE, y + CELL_SIZE), 2)
            if cell.walls["left"]:
                pygame.draw.line(screen, BLACK, (x, y), (x, y + CELL_SIZE), 2)

    # path with transparency
    if path:
        for cell in path:
            pygame.draw.rect(
                screen, BLUE, (cell.x * CELL_SIZE + 2, cell.y * CELL_SIZE + 2, CELL_SIZE - 4, CELL_SIZE - 4), 0
            )

    screen.blit(character_img, (start.x * CELL_SIZE, start.y * CELL_SIZE))
    screen.blit(destination_img, (goal.x * CELL_SIZE, goal.y * CELL_SIZE))

    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

pygame.quit()
