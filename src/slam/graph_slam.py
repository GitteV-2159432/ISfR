import numpy as np
from scipy.spatial.transform import Rotation
from collections import deque
from typing import Dict, List, Set

from slam.icp import icp

class PoseVertex():
    def __init__(self, matrix4x4, point_cloud, voxel_key) -> None:         
        self._matrix = matrix4x4
        self.point_cloud = np.array(point_cloud)
        self.voxel_key = voxel_key
    
    def get_position(self): return _matrix2translation(self._matrix)
    def get_rotation(self): return _matrix2rotation(self._matrix)
    def get_homogeneous_matrix(self): return self._matrix

    def get_point_cloud_global(self):
        homogeneous_points = np.hstack((self.point_cloud, np.ones((self.point_cloud.shape[0], 1))))
        return (self._matrix @ homogeneous_points.T).T[:, :3]

class EdgeConstraint():
    def __init__(self, transformation, information, from_pose_vertex, to_pose_vertex) -> None:
        self.transformation = transformation
        self.information = information
        self.from_pose_vertex: PoseVertex = from_pose_vertex
        self.to_pose_vertex: PoseVertex = to_pose_vertex

class PoseGraph():
    def __init__(self, voxel_size = 0.5) -> None:
        self.adjacency_list: Dict[PoseVertex, List[EdgeConstraint]] = {}
        self.voxel_grid = {}
        self.voxel_size = voxel_size

    def add_pose(self, matrix4x4, point_cloud) -> PoseVertex:
        voxel_key = self._get_voxel_key(_matrix2translation(matrix4x4))
        pose_vertex = PoseVertex(matrix4x4, point_cloud, voxel_key)
        self.adjacency_list[pose_vertex] = []
        if voxel_key not in self.voxel_grid:
            self.voxel_grid[voxel_key] = []
        self.voxel_grid[voxel_key].append(pose_vertex)
        return pose_vertex

    def add_constraint(self, from_pose_vertex: PoseVertex, to_pose_vertex: PoseVertex, transformation, information) -> None:
        edge_constraint = EdgeConstraint(transformation, information, from_pose_vertex, to_pose_vertex)
        self.adjacency_list[from_pose_vertex].append(edge_constraint)
        self.adjacency_list[to_pose_vertex].append(edge_constraint)     

    def get_nearby_poses(self, position, voxel_range: int = 1) -> List[PoseVertex]:
        nearby_poses = []
        voxel_key = self._get_voxel_key(position)
        for dx in range(-voxel_range, voxel_range + 1):
            for dy in range(-voxel_range, voxel_range + 1):
                for dz in range(-voxel_range, voxel_range + 1):
                    neighbor_voxel_key = (voxel_key[0] + dx, voxel_key[1] + dy, voxel_key[2] + dz)
                    if neighbor_voxel_key in self.voxel_grid:
                        nearby_poses.extend(self.voxel_grid[neighbor_voxel_key])

        return nearby_poses
    
    def get_all_edges(self) -> List[EdgeConstraint]:
        edges = []
        seen_edges = set()
        for edge_list in self.adjacency_list.values():
            for edge in edge_list:
                edge_pair = (edge.from_pose_vertex, edge.to_pose_vertex)
                if edge_pair not in seen_edges and (edge_pair[1], edge_pair[0]) not in seen_edges:
                    edges.append(edge)
                    seen_edges.add(edge_pair)
        return edges

    def get_vertices_within_steps(self, start_vertex: PoseVertex, steps: int) -> List[PoseVertex]:
        if start_vertex not in self.adjacency_list:
            return []

        visited: Set[PoseVertex] = set()
        queue = deque([(start_vertex, 0)]) 
        visited.add(start_vertex)
        result = [start_vertex]

        while queue:
            current_vertex, current_step = queue.popleft()
            if current_step <= steps and current_vertex != start_vertex:
                result.append(current_vertex)

            if current_step >= steps:
                continue

            for edge_constraint in self.adjacency_list[current_vertex]:
                next_vertex = (edge_constraint.to_pose_vertex if edge_constraint.from_pose_vertex == current_vertex else edge_constraint.from_pose_vertex)
                if next_vertex not in visited:
                    visited.add(next_vertex)
                    queue.append((next_vertex, current_step + 1))

        return result

    def _get_voxel_key(self, position):
        x, y, z = position
        voxel_x = int(np.floor(x / self.voxel_size))
        voxel_y = int(np.floor(y / self.voxel_size))
        voxel_z = int(np.floor(z / self.voxel_size))
        return (voxel_x, voxel_y, voxel_z)   

class GraphSlam():
    def __init__(self) -> None:
        self.graph = PoseGraph()
        self.last_pose_vertex = None

    def update(self, transformation_matrix, point_cloud):
        new_pose_vertex = self.graph.add_pose((self.last_pose_vertex.get_homogeneous_matrix() if self.last_pose_vertex else np.eye(4)) @ transformation_matrix, point_cloud)
        if self.last_pose_vertex is not None: self.graph.add_constraint(self.last_pose_vertex, new_pose_vertex, transformation_matrix, 0.0)
        self.detect_loop_closures(new_pose_vertex, 1, 10)
        self.last_pose_vertex = new_pose_vertex

    def detect_loop_closures(self, pose_vertex: PoseVertex, voxel_range, min_steps_separation):
        # TODO: Not very efficient
        nearby_poses = self.graph.get_nearby_poses(pose_vertex.get_position(), voxel_range)
        poses_within_steps = self.graph.get_vertices_within_steps(pose_vertex, min_steps_separation)
        nearby_poses = [nearby_pose for nearby_pose in nearby_poses if nearby_pose not in poses_within_steps]

        for pose in nearby_poses:
            point_cloud_transformed, transformation_matrix, mean_error = icp(pose_vertex.point_cloud, pose.point_cloud, 20, 1e-4)
            if mean_error < 0.1: self.graph.add_constraint(pose_vertex, pose, transformation_matrix, 0.0)

def _matrix2translation(transformation_matrix): return transformation_matrix[:3, 3]
def _matrix2rotation(transformation_matrix): return Rotation.from_matrix(transformation_matrix[:3, :3]).as_euler("xyz")
def _pose2matrix(position, rotation): 
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler("xyz", rotation).as_matrix()
    matrix[:3, 3] = np.array(position)
    return matrix
