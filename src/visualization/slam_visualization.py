import numpy as np
import pyvista as pv
from slam.graph_slam import GraphSlam, PoseVertex

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