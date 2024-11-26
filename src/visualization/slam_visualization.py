import numpy as np
import pyvista as pv
from slam.graph_slam import GraphSlam, PoseVertex
from skimage.draw import line

class SlamPlot():
    def __init__(self, slam: GraphSlam) -> None:
        self._graph = slam.graph
        self._voxel_size = self._graph.voxel_size
        self._plotter = pv.Plotter()
        self._plotter.show(interactive_update=True)

        slam.register_on_pose_added(self.add_pose)
        slam.register_on_optimized_added(self.recreate_all)

    def recreate_all(self, *args):
        self._plotter.clear()
        for pose in self._graph.adjacency_list:
            self._plotter.add_mesh(pv.Sphere(0.02, pose.get_position()), color="purple")
            self._plotter.add_points(pv.PolyData(pose.get_point_cloud_global()))
            self._draw_voxel_grid(pose)

        for edge in self._graph.get_all_constraints():
            self._plotter.add_lines(np.array([edge.from_pose_vertex.get_position(), edge.to_pose_vertex.get_position()]), color="black", width=1)

        self._plotter.update()

    def add_pose(self, from_pose_vertex: PoseVertex, to_pose_vertex: PoseVertex, transformation_matrix):
        self._plotter.add_mesh(pv.Sphere(0.02, to_pose_vertex.get_position()), color="purple")
        self._plotter.add_points(pv.PolyData(to_pose_vertex.get_point_cloud_global()))
        self._draw_voxel_grid(to_pose_vertex)

        for edge in self._graph.adjacency_list[to_pose_vertex]:
            self._plotter.add_lines(np.array([edge.from_pose_vertex.get_position(), edge.to_pose_vertex.get_position()]), color="black", width=1)

        self._plotter.update()

    def _draw_voxel_grid(self, new_pose: PoseVertex) -> None:
        grid_lines = []

        x, y, z = new_pose.get_position()
        x, y, z = int(np.floor(x / self._voxel_size)), int(np.floor(y / self._voxel_size)), int(np.floor(z / self._voxel_size))
        min_corner = np.array([x, y, z]) * self._voxel_size
        max_corner = min_corner + self._voxel_size

        corners = [
            min_corner,
            [max_corner[0], min_corner[1], min_corner[2]],
            [max_corner[0], max_corner[1], min_corner[2]],
            [min_corner[0], max_corner[1], min_corner[2]],
            [min_corner[0], min_corner[1], max_corner[2]],
            [max_corner[0], min_corner[1], max_corner[2]],
            [max_corner[0], max_corner[1], max_corner[2]],
            [min_corner[0], max_corner[1], max_corner[2]]
        ]
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # Bottom square
            (4, 5), (5, 6), (6, 7), (7, 4),  # Top square
            (0, 4), (1, 5), (2, 6), (3, 7)   # Vertical lines
        ]
        for edge in edges:
            grid_lines.append(corners[edge[0]])
            grid_lines.append(corners[edge[1]])

        if grid_lines:
            self._plotter.add_lines(np.array(grid_lines), color="blue", width=0.1)

class OccupancyGrid:
    def __init__(self, grid_size=100, cell_size=0.1):
        self.grid_size = grid_size
        self.cell_size = cell_size
        self.grid = np.zeros((grid_size, grid_size))  # Start with all cells at 0 (unknown probability)

        self.plotter = pv.Plotter(title="Occupancy Grid")
        self.mesh = None

        # Create structured grid points
        x = np.linspace(0, (grid_size - 1) * cell_size, grid_size)
        y = np.linspace(0, (grid_size - 1) * cell_size, grid_size)
        x, y = np.meshgrid(x, y)
        z = np.zeros_like(x)

        points = np.c_[x.ravel(), y.ravel(), z.ravel()]

        # Create PyVista structured grid
        self.structured_grid = pv.StructuredGrid()
        self.structured_grid.points = points
        self.structured_grid.dimensions = (grid_size, grid_size, 1)

        # Initialize cell scalars
        self.structured_grid.cell_data["values"] = self.grid[:-1, :-1].ravel()

        # Add grid to the plotter
        self.mesh = self.plotter.add_mesh(self.structured_grid, scalars="values", cmap="gray", clim=(-1, 1))
        self.plotter.show(interactive_update=True)

    def world_to_grid(self, x, y):
        """Convert world coordinates to grid indices."""
        grid_x = int((x + self.grid_size * self.cell_size / 2) // self.cell_size)
        grid_y = int((y + self.grid_size * self.cell_size / 2) // self.cell_size)
        return grid_x, grid_y
    
    def transform_to_global(self, pose, lidar_points):
        """Transform LIDAR points to the global frame."""
        x_r, y_r, theta_r = pose[:3]  # Robot's global position and orientation

        # Create rotation matrix
        rotation_matrix = np.array([
            [np.cos(theta_r), -np.sin(theta_r)],
            [np.sin(theta_r), np.cos(theta_r)]
        ])

        global_points = []
        for point in lidar_points:
            local_x, local_y, _ = point  # LIDAR point in local frame
            # Rotate and translate to global coordinates
            global_xy = rotation_matrix @ np.array([local_x, local_y]) + np.array([x_r, y_r])
            global_points.append(global_xy)

        return global_points

    def update_grid(self, pose, lidar_points):
        """Update the occupancy grid with the robot's position and LIDAR points."""
        # Transform LIDAR points to the global frame
        global_points = self.transform_to_global(pose, lidar_points)
        robot_x, robot_y, _ = pose[:3]  # Robot's position in global coordinates
        robot_grid_x, robot_grid_y = self.world_to_grid(robot_x, robot_y)

        # Mark the robot's position as free (do not overwrite explored space)
        if self.grid[robot_grid_x, robot_grid_y] == -1:
            self.grid[robot_grid_x, robot_grid_y] = 0  # Free space

        for global_x, global_y in global_points:
            grid_x, grid_y = self.world_to_grid(global_x, global_y)

            # Check if grid indices are within bounds
            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                # Mark obstacles as occupied
                self.grid[grid_x, grid_y] = min(self.grid[grid_x, grid_y] + 0.2, 1.0)

                # Mark free space along the ray to the obstacle
                self.mark_free_space(robot_grid_x, robot_grid_y, grid_x, grid_y)

    def mark_free_space(self, robot_x, robot_y, target_x, target_y):
        """Mark free space along the robot's path, avoiding overwriting free cells."""
        rr, cc = self.bresenham(robot_x, robot_y, target_x, target_y)

        for r, c in zip(rr, cc):
            # Only mark as free if it was unexplored or not strongly occupied
            if 0 <= r < self.grid_size and 0 <= c < self.grid_size:
                if self.grid[r, c] < 0.8:  # Do not overwrite strong obstacles
                    self.grid[r, c] = max(self.grid[r, c] - 0.1, 0.0)  # Gradually mark as free
                
    def bresenham(self, x0, y0, x1, y1):
        """Line-drawing algorithm for free space."""
        return line(x0, y0, x1, y1)

    def render(self):
        """Render occupancy grid with refined thresholds."""
        binary_grid = np.zeros_like(self.grid)
        binary_grid[self.grid > 0.6] = 1  # Occupied
        binary_grid[self.grid < 0.4] = -1  # Free
        binary_grid[(self.grid >= 0.4) & (self.grid <= 0.6)] = 0  # Uncertain

        # Update scalars for visualization
        self.structured_grid.cell_data["values"] = binary_grid[:-1, :-1].ravel()
        self.plotter.update()