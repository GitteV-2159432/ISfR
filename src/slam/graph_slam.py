import numpy as np
from scipy.spatial.transform import Rotation

class PoseVertex():
    def __init__(self, matrix4x4=None, position=(0, 0, 0), rotation=(0, 0, 0)) -> None:
        if matrix4x4 is None:
            self.matrix = np.eye(4)
            self.matrix[:3, :3] = Rotation.from_euler("xyz", rotation).as_matrix()
            self.matrix[:3, 3] = np.array(position)
        else:
            self.matrix = matrix4x4
    
    def get_position(self): return _matrix2translation(self.matrix)
    def get_rotation(self): return _matrix2rotation(self.matrix)
    def get_homogeneous_matrix(self): return self.matrix

class EdgeConstraint():
    def __init__(self, transformation, information, from_pose_vertex, to_pose_vertex) -> None:
        self.transformation = transformation
        self.information = information
        self.from_pose_vertex: PoseVertex = from_pose_vertex
        self.to_pose_vertex: PoseVertex = to_pose_vertex

class PoseGraph():
    def __init__(self, voxel_size = 0.5) -> None:
        self.adjacency_list = {}
        self.voxel_grid = {}
        self.voxel_size = voxel_size

    def add_pose(self, pose_vertex: PoseVertex) -> None:
        self.adjacency_list[pose_vertex] = []

        voxel_key = self._get_voxel_key(pose_vertex.get_position())
        if voxel_key not in self.voxel_grid:
            self.voxel_grid[voxel_key] = []
        self.voxel_grid[voxel_key].append(pose_vertex)

    def add_constraint(self, from_pose_vertex: PoseVertex, to_pose_vertex: PoseVertex, transformation, information) -> None:
        edge_constraint = EdgeConstraint(transformation, information, from_pose_vertex, to_pose_vertex)
        self.adjacency_list[from_pose_vertex].append(edge_constraint)
        self.adjacency_list[to_pose_vertex].append(edge_constraint)     

    def get_nearby_poses(self, position, voxel_radius: int = 1):
        nearby_poses = []
        voxel_key = self._get_voxel_key(position)
        for dx in range(-voxel_radius, voxel_radius + 1):
            for dy in range(-voxel_radius, voxel_radius + 1):
                for dz in range(-voxel_radius, voxel_radius + 1):
                    neighbor_voxel_key = (voxel_key[0] + dx, voxel_key[1] + dy, voxel_key[2] + dz)
                    if neighbor_voxel_key in self.voxel_grid:
                        nearby_poses.extend(self.voxel_grid[neighbor_voxel_key])

        return nearby_poses

    def _get_voxel_key(self, position):
        x, y, z = position
        voxel_x = int(np.floor(x / self.voxel_size))
        voxel_y = int(np.floor(y / self.voxel_size))
        # voxel_z = int(np.floor(z / self.voxel_size))
        return (voxel_x, voxel_y, 0)   

class GraphSlam():
    def __init__(self) -> None:
        self.graph = PoseGraph()
        self.last_pose_vertex = PoseVertex()

        self.graph.add_pose(self.last_pose_vertex)

    def update(self, transformation_matrix):
        new_pose_vertex = PoseVertex(self.last_pose_vertex.get_homogeneous_matrix() @ transformation_matrix)
        self.graph.add_pose(new_pose_vertex)
        self.graph.add_constraint(self.last_pose_vertex, new_pose_vertex, transformation_matrix, 0.0)
        self.last_pose_vertex = new_pose_vertex

def _matrix2translation(transformation_matrix): return transformation_matrix[:3, 3]
def _matrix2rotation(transformation_matrix): return Rotation.from_matrix(transformation_matrix[:3, :3]).as_euler("xyz")

