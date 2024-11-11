import numpy as np
import pyvista as pv
import slam.graph_slam as slam
from typing import List

plotter = pv.Plotter()
plotter.show(interactive_update=True)

def update(graph: slam.PoseGraph):
    plotter.clear()
    points = []
    lines = []
    for node in graph.adjacency_list:
        points.append(node.get_position())

    for edge in get_edges(graph):
        lines.append(edge.from_pose_vertex.get_position())
        lines.append(edge.to_pose_vertex.get_position())

    plotter.add_lines(np.array(lines), color="green", connected=False, width=0.2)
    draw_voxel_grid(graph, plotter)

    pd_points = pv.PolyData(np.array(points))
    plotter.add_points(pd_points)
    plotter.update()
    
def get_edges(graph: slam.PoseGraph) -> List[slam.EdgeConstraint]:
    edges = []
    seen_edges = set()
    for edge_list in graph.adjacency_list.values():
        for edge in edge_list:
            edge_pair = (edge.from_pose_vertex, edge.to_pose_vertex)
            if edge_pair not in seen_edges and (edge_pair[1], edge_pair[0]) not in seen_edges:
                edges.append(edge)
                seen_edges.add(edge_pair)
    return edges

def draw_voxel_grid(graph: slam.PoseGraph, plotter):
    voxel_size = graph.voxel_size
    grid_lines = []

    for voxel_key in graph.voxel_grid.keys():
        x, y, z = voxel_key
        min_corner = np.array([x, y, z]) * voxel_size
        max_corner = min_corner + voxel_size

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
