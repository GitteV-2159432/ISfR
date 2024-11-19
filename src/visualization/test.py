import numpy as np
import pyvista as pv
import slam.graph_slam as slam
from typing import List

_created = False
_plotter: pv.Plotter = None
_voxel_size = None
_graph = None

def create(graph: slam.PoseGraph):
    global _created, _plotter, _voxel_size, _graph
    _voxel_size = graph.voxel_size
    _plotter = pv.Plotter()
    _plotter.show(interactive_update=True)
    _graph = graph
    _created = True

def recreate_all(graph: slam.PoseGraph):
    global _graph
    _plotter.clear()
    for pose in graph.adjacency_list:
        _plotter.add_mesh(pv.Sphere(0.02, pose.get_position()), color="purple")
        _plotter.add_points(pv.PolyData(pose.get_point_cloud_global()))
        draw_voxel_grid(pose, _plotter)

    for edge in graph.get_all_constraints():
        _plotter.add_lines(np.array([edge.from_pose_vertex.get_position(), edge.to_pose_vertex.get_position()]), color="black", width=1)

    _graph = graph
    _plotter.update()

def update(new_pose: slam.PoseVertex):
    global _created
    if not _created: return

    _plotter.add_mesh(pv.Sphere(0.02, new_pose.get_position()), color="purple")
    _plotter.add_points(pv.PolyData(new_pose.get_point_cloud_global()))
    draw_voxel_grid(new_pose, _plotter)

    for edge in _graph.adjacency_list[new_pose]:
        _plotter.add_lines(np.array([edge.from_pose_vertex.get_position(), edge.to_pose_vertex.get_position()]), color="black", width=1)

    _plotter.update()

def draw_voxel_grid(new_pose: slam.PoseVertex, plotter):
    global _voxel_size
    grid_lines = []

    x, y, z = new_pose.get_position()
    x, y, z = int(np.floor(x / _voxel_size)), int(np.floor(y / _voxel_size)), int(np.floor(z / _voxel_size))
    min_corner = np.array([x, y, z]) * _voxel_size
    max_corner = min_corner + _voxel_size

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
        plotter.add_lines(np.array(grid_lines), color="blue", width=0.1)