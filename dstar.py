import heapq
import math

class DStar:
    def __init__(self, grid):
        """
        Initialize the D* algorithm with a grid.
        0: free space
        1: obstacle
        """
        self.grid = grid
        self.rows = len(grid)
        self.cols = len(grid[0])
        self.open_list = []
        self.g_values = {}
        self.rhs_values = {}
        self.km = 0
        self.goal = None
        self.start = None

    def heuristic(self, a, b):
        """Eucledean distance heuristic for grids.""" 
        return math.sqrt(((a[0] - b[0]) ** 2 ) + ((a[1] - b[1]) ** 2))
        # return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def initialize(self, start, goal):
        """Initialize the D* algorithm with the start and goal points."""
        self.start = start
        self.goal = goal
        self.g_values[goal] = float('inf')
        self.rhs_values[goal] = 0
        heapq.heappush(self.open_list, (self.heuristic(start, goal), goal))

    def update_vertex(self, u):
        """Update the vertex values (g and rhs)."""
        if u != self.goal:
            neighbors = self.get_neighbors(u)
            self.rhs_values[u] = min([self.g_values.get(neighbor, float('inf')) + 1 for neighbor in neighbors])

        if (self.g_values.get(u, float('inf')), u) in self.open_list:
            self.open_list.remove((self.g_values.get(u, float('inf')), u))
            heapq.heapify(self.open_list)

        if self.g_values.get(u, float('inf')) != self.rhs_values.get(u, float('inf')):
            heapq.heappush(self.open_list, (self.calculate_key(u), u))

    def calculate_key(self, u):
        """Calculate the key used to prioritize the vertices in the open list."""
        g_rhs = min(self.g_values.get(u, float('inf')), self.rhs_values.get(u, float('inf')))
        return g_rhs + self.heuristic(self.start, u) + self.km

    def compute_shortest_path(self):
        """Compute the shortest path by expanding the open list."""
        while self.open_list and (self.open_list[0][0] < self.calculate_key(self.start) or self.rhs_values.get(self.start, float('inf')) != self.g_values.get(self.start, float('inf'))):
            _, u = heapq.heappop(self.open_list)
            if self.g_values.get(u, float('inf')) > self.rhs_values.get(u, float('inf')):
                self.g_values[u] = self.rhs_values[u]
                neighbors = self.get_neighbors(u)
                for neighbor in neighbors:
                    self.update_vertex(neighbor)
            else:
                self.g_values[u] = float('inf')
                neighbors = self.get_neighbors(u)
                for neighbor in neighbors + [u]:
                    self.update_vertex(neighbor)

    def get_neighbors(self, node):
        """Get the neighboring nodes that are walkable."""
        neighbors = []
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]  # Right, Left, Down, Up
        for direction in directions:
            neighbor = (node[0] + direction[0], node[1] + direction[1])
            if 0 <= neighbor[0] < self.rows and 0 <= neighbor[1] < self.cols and self.grid[neighbor[0]][neighbor[1]] != 1:
                neighbors.append(neighbor)
        return neighbors

    def update_grid(self, obstacle):
        """Update the grid when new obstacles are discovered."""
        # Only update if the obstacle wasn't already placed
        if self.grid[obstacle[0]][obstacle[1]] == 1:
            print(f"Obstacle already at {obstacle}")
            return

        # Set the new obstacle
        self.grid[obstacle[0]][obstacle[1]] = 1
        print(f"Obstacle added at {obstacle}")
        self.km += self.heuristic(self.start, obstacle)  # Update km for the heuristic adjustment
        self.update_vertex(obstacle)  # Update vertex for the obstacle
        self.compute_shortest_path()  # Recompute the path


    def get_path(self):
        """Retrieve the path from the start to the goal."""
        path = []
        current = self.start
        while current != self.goal:
            path.append(current)
            neighbors = self.get_neighbors(current)
            current = min(neighbors, key=lambda neighbor: self.g_values.get(neighbor, float('inf')))
            if self.g_values.get(current, float('inf')) == float('inf'):
                return None  # No valid path found
        path.append(self.goal)
        return path

# Example usage
grid = [
    [0, 0, 0, 0, 0],
    [0, 1, 1, 1, 1],
    [0, 1, 0, 0, 0],
    [0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0]
]

start = (0, 0)
goal = (4, 4)
dstar = DStar(grid)
dstar.initialize(start, goal)
dstar.compute_shortest_path()

# Get the initial path
path = dstar.get_path()
if path:
    print("Initial Path:", path)
else:
    print("No path found.")

# Simulate discovering an obstacle
dstar.update_grid((2, 1))

# Get the new path after updating with the obstacle
new_path = dstar.get_path()
if new_path:
    print("New Path:", new_path)
else:
    print("No path found after obstacle.")
