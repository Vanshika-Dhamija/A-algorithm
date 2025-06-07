# A-algorithm
A* Pathfinding Maze Game

This is a Python project that visualizes the A* pathfinding algorithm inside a randomly generated maze. The goal is to find the shortest path from a ghost (start point) to a fruit (end point) using A* logic. The maze is generated using depth-first search (DFS), and the entire game is displayed using Pygame.


Features:
- Random maze generation using DFS
- A* pathfinding algorithm to find the shortest path
- Custom icons:
  - ghost.jpg for the character
  - fruit.jpg for the goal
- Real-time visual display using Pygame


How to Run:

1. Install Python and Pygame:
   pip install pygame

2. Place two image files in the same folder:
   - ghost.jpg – this is used as the start character
   - fruit.jpg – this is used as the goal item

3. Make sure the Python file is named something like:
   A star algorithm.py

4. Run the script:
   python "A star algorithm.py"


How It Works:

- A 20x20 grid maze is created randomly by removing walls between cells using DFS.
- A* pathfinding finds the shortest path between the top-left corner and the bottom-right corner.
- The ghost image is displayed at the start, and the fruit image at the goal.
- The shortest path is shown in light blue.
- Maze walls remain visible even when the path is shown.


File Structure:

project-folder/
├── A star algorithm.py
├── ghost.jpg
└── fruit.jpg

