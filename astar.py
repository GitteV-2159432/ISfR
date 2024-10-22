import heapq
import math

class AStar:
    def __init__(self, grid):
        self.grid = grid

        self.rows = len(grid)
        self.cols = len(grid[0])

        self.directions = [
            (0, 1), (0, -1), (1, 0), (-1, 0), # cardinal directions
            (1, 1), (-1, -1), (1, -1), (-1, 1)  # diagonal directions
        ]

    # Euclidean distance
    def _heuristic(self, a, b):
        return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

    def _reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def a_star(self, start, goal):
        open_list = []
        heapq.heappush(open_list, (0, start))
        
        g_score = {start: 0}
        f_score = {start: self._heuristic(start, goal)}
        
        came_from = {}
        
        while open_list:
            _, current = heapq.heappop(open_list)

            if current == goal:
                return self._reconstruct_path(came_from, current)
            
            for direction in self.directions:
                neighbor = (current[0] + direction[0], current[1] + direction[1])
                
                # Check if the neighbor is within the grid bounds
                if 0 <= neighbor[0] < self.rows and 0 <= neighbor[1] < self.cols:
                    # Check if the neighbor is walkable (not an obstacle)
                    if self.grid[neighbor[0]][neighbor[1]] == 0:
                        tentative_g_score = g_score[current] + self._heuristic(current, neighbor)
                        
                        if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                            # Update the path and cost
                            came_from[neighbor] = current
                            g_score[neighbor] = tentative_g_score
                            f_score[neighbor] = tentative_g_score + self._heuristic(neighbor, goal)
                            heapq.heappush(open_list, (f_score[neighbor], neighbor))
        
        return None

# Example usage:
grid = [
    [0, 1, 0, 0, 0],
    [0, 1, 0, 1, 0],
    [0, 0, 0, 1, 0],
    [1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0]
]

start = (0, 0)  # Starting coordinates
goal = (4, 4)   # Destination coordinates

# Create an instance of the AStar class
astar = AStar(grid)

# Find the path
path = astar.a_star(start, goal)

if path:
    print("Path found:", path)
else:
    print("No path found")
