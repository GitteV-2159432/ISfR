import numpy as np
from scipy.spatial.transform import Rotation
from scipy.sparse.linalg import spsolve
from scipy.sparse import csr_matrix
from collections import deque
from typing import Dict, List, Set, Tuple

from slam.icp import icp

"""
References:
    - https://www.researchgate.net/publication/231575337_A_tutorial_on_graph-based_SLAM
    - https://www.youtube.com/watch?v=saVZtgPyyJQ
"""


class PoseVertex():
    """Represents a pose in the :py:class:`PoseGraph` with a transformation matrix and an associated point cloud"""

    def __init__(self, transformation_matrix, point_cloud) -> None:         
        self._matrix = transformation_matrix
        self.point_cloud = np.array(point_cloud)
    
    def get_position(self) -> np.ndarray: 
        """
        Returns the translation component of the transformation matrix.

        Returns:
            out (np.ndarray): A 1x3 numpy array containing the translation component (x, y, z).
        """
        return _matrix2translation(self._matrix)
    
    def get_rotation(self) -> np.ndarray: 
        """
        Returns the rotation component in Euler angles.

        Returns:
            out (np.ndarray): A 1x3 numpy array containing the rotation in Euler angles (roll, pitch, yaw).
        """
        return _matrix2rotation(self._matrix)
    
    def get_homogeneous_matrix(self) -> np.ndarray: 
        """
        Returns the homogeneous transformation matrix.

        Returns:
            out (np.ndarray): A 4x4 numpy array representing the homogeneous transformation matrix.
        """
        return self._matrix

    def get_point_cloud_global(self):
        """
        Transforms the local point cloud of this vertex into the global frame using the vertex’s transformation matrix.
        
        returns:
            out (np.ndarray): An nx3 numpy array of 3D points transformed into the global frame.
        """
        homogeneous_points = np.hstack((self.point_cloud, np.ones((self.point_cloud.shape[0], 1))))
        return (self._matrix @ homogeneous_points.T).T[:, :3]

class EdgeConstraint():
    """Represents a constraint between two :py:class:`PoseVertex` objects in the graph, including a transformation and an information matrix."""

    def __init__(self, transformation_matrix, information_matrix, from_pose_vertex, to_pose_vertex) -> None:
        """
        Initializes an edge constraint between two poses. The edge includes the relative transformation between the two poses and the associated information matrix, which describes the uncertainty.

        Parameters:
            transformation_matrix (np.ndarray): A 4x4 homogeneous matrix representing the transformation between two poses.
            information_matrix (np.ndarray): A 6x6 matrix representing the information (inverse of the covariance) of the transformation.
            from_pose_vertex (PoseVertex): The source pose vertex.
            to_pose_vertex (PoseVertex): The target pose vertex.
        """
        self.transformation = transformation_matrix
        self.information = information_matrix
        self.from_pose_vertex: PoseVertex = from_pose_vertex
        self.to_pose_vertex: PoseVertex = to_pose_vertex

class PoseGraph():
    """
    Manages a graph structure where nodes are :py:class:`PoseVertex` objects, and edges are :py:class:`EdgeConstraint` objects. 
    Provides functionality to add poses, manage constraints, and optimize the graph.
    """

    def __init__(self, voxel_size = 0.5) -> None:
        self.adjacency_list: Dict[PoseVertex, List[EdgeConstraint]] = {}
        self.voxel_grid = {}
        self.voxel_size = voxel_size

    def add_pose(self, transformation_matrix, point_cloud) -> PoseVertex:
        """
        Adds a new pose to the graph and places it in a voxel grid.

        Parameters:
            transformation_matrix (np.ndarray): The estimated pose as a 4x4 homogenous matrix 
            point_cloud (np.ndarray): The point cloud

        Returns:
            out (PoseVertex): The newly created :py:class:`PoseVertex` object
        """
        voxel_key = self._get_voxel_key(_matrix2translation(transformation_matrix))
        pose_vertex = PoseVertex(transformation_matrix, point_cloud)
        self.adjacency_list[pose_vertex] = []
        if voxel_key not in self.voxel_grid:
            self.voxel_grid[voxel_key] = []
        self.voxel_grid[voxel_key].append(pose_vertex)
        return pose_vertex

    def add_constraint(self, from_pose_vertex: PoseVertex, to_pose_vertex: PoseVertex, transformation: np.ndarray, information: np.ndarray) -> None:
        """
        Adds a constraint between two poses.

        Parameters:
            from_pose_vertex (PoseVertex): The source pose vertex.
            to_pose_vertex (PoseVertex): The target pose vertex.
            transformation (np.ndarray): A 4x4 homogeneous matrix representing the relative transformation.
            information (np.ndarray): A 6x6 information matrix.
        """
        edge_constraint = EdgeConstraint(transformation, information, from_pose_vertex, to_pose_vertex)
        self.adjacency_list[from_pose_vertex].append(edge_constraint)
        self.adjacency_list[to_pose_vertex].append(edge_constraint)     

    def get_nearby_poses(self, position, voxel_range: int = 1) -> List[PoseVertex]:
        """
        Retrieves nearby poses based on spatial locality.

        Parameters:
            position (np.ndarray): A 1x3 numpy array representing the position to search around.
            voxel_range (int): The range (in number of voxels) to search for nearby poses.

        Returns:
            out (List[PoseVertex]): A list of nearby :py:class:`PoseVertex` objects
        """
        nearby_poses = []
        voxel_key = self._get_voxel_key(position)
        for dx in range(-voxel_range, voxel_range + 1):
            for dy in range(-voxel_range, voxel_range + 1):
                for dz in range(-voxel_range, voxel_range + 1):
                    neighbor_voxel_key = (voxel_key[0] + dx, voxel_key[1] + dy, voxel_key[2] + dz)
                    if neighbor_voxel_key in self.voxel_grid:
                        nearby_poses.extend(self.voxel_grid[neighbor_voxel_key])

        return nearby_poses
    
    def get_all_constraints(self) -> List[EdgeConstraint]:
        """
        Returns a list of all unique edge constraints in the graph.

        Returns:
            out (List[EdgeConstraint]): A list of all :py:class:`EdgeConstraint` objects in the graph
        """
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
        """
        Retrieves vertices within a certain number of steps from a start vertex.

        Parameters:
            start_vertex (PoseVertex): The starting pose vertex.
            steps (int): The maximum number of steps away from the start vertex to consider.

        Returns:
            out (List[PoseVertex]): a list of all :py:class:`PoseVertex` objects that are `steps` or less steps away from the start vertex
        """
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

    def _get_voxel_key(self, position) -> Tuple[float, float, float]:
        """
        Computes a voxel key based on the position for spatial partitioning.

        Parameters:
            position (np.ndarray): The position of the voxel to get the key of.

        Returns:
            out (Tuple[float, float, float]): The key of the voxel on the given position.
        """
        x, y, z = position
        voxel_x = int(np.floor(x / self.voxel_size))
        voxel_y = int(np.floor(y / self.voxel_size))
        voxel_z = int(np.floor(z / self.voxel_size))
        return (voxel_x, voxel_y, voxel_z)  

    def optimize(self, iterations = 1) -> None:
        """
        Optimizes the graph using non-linear least squares.
        """
        def compute_error(transformation_from, transformation_to, expected_transformation):
            transformation_error = np.linalg.inv(expected_transformation) @ np.linalg.inv(transformation_from) @ transformation_to # TODO: Check if this is correct!!
            return _matrix2state(transformation_error)

        def jacobian_numerical(transformation_from, transformation_to, expected_transformation, epsilon=1e-6):
            error_original = compute_error(transformation_from, transformation_to, expected_transformation)
            jacobian_from = np.zeros((6, 6))
            jacobian_to = np.zeros((6, 6))

            for i in range(6):
                perturbed_from = _matrix2state(transformation_from)
                perturbed_from[i] += epsilon
                error_perturbed = compute_error(_state2matrix(perturbed_from), transformation_to, expected_transformation)
                jacobian_from[:, i] = (error_perturbed - error_original) / epsilon

            for i in range(6):
                perturbed_to = _matrix2state(transformation_to)
                perturbed_to[i] += epsilon
                error_perturbed = compute_error(transformation_from, _state2matrix(perturbed_to), expected_transformation)
                jacobian_to[:, i] = (error_perturbed - error_original) / epsilon

            return jacobian_from, jacobian_to

        for i in range(iterations):
            # Todo: Use csr or csc to represent the hessian
            # Todo:   row_indices = np.array([0, 1, 2, 2])
            # Todo:   col_indices = np.array([0, 2, 0, 1])
            # Todo:   values = np.array([1, 3, 2, 4])
            # Todo:   csr = csr_matrix((values, (row_indices, col_indices)), shape=(3, 3))

            vertex2index = {vertex: index for index, vertex in enumerate(self.adjacency_list)}

            gradient = np.zeros(6 * len(vertex2index))
            hessian = csr_matrix((6 * len(vertex2index), 6 * len(vertex2index)))
            for constraint in self.get_all_constraints():
                # Calculate error
                matrix_from = constraint.from_pose_vertex.get_homogeneous_matrix()
                matrix_to = constraint.to_pose_vertex.get_homogeneous_matrix()
                matrix_expected = constraint.transformation

                error = compute_error(matrix_from, matrix_to, matrix_expected)
                jacobian_i, jacobian_j = jacobian_numerical(matrix_from, matrix_to, matrix_expected)

                # Calculate contributions
                gradient_i = error.T @ constraint.information @ jacobian_i
                gradient_j = error.T @ constraint.information @ jacobian_j

                hessian_ii = jacobian_i.T @ constraint.information @ jacobian_i
                hessian_ij = jacobian_i.T @ constraint.information @ jacobian_j
                hessian_ji = jacobian_j.T @ constraint.information @ jacobian_i
                hessian_jj = jacobian_j.T @ constraint.information @ jacobian_j

                # Fill global gradient and hessian
                i = vertex2index[constraint.from_pose_vertex]
                j = vertex2index[constraint.to_pose_vertex]
                gradient[6*i:6*(i+1)] += gradient_i
                gradient[6*j:6*(j+1)] += gradient_j
                hessian[6*i:6*(i+1), 6*i:6*(i+1)] += hessian_ii
                hessian[6*i:6*(i+1), 6*j:6*(j+1)] += hessian_ij
                hessian[6*j:6*(j+1), 6*i:6*(i+1)] += hessian_ji
                hessian[6*j:6*(j+1), 6*j:6*(j+1)] += hessian_jj

            hessian[0:6, 0:6] += np.eye(6) * 1e10
            pose_updates = spsolve(hessian, -gradient)
            for vertex, index in vertex2index.items():
                update = pose_updates[6*index:6*(index+1)]
                translation_update = update[:3]
                rotation_update = update[3:]

                current_translation = vertex.get_position()
                current_rotation = vertex.get_rotation()

                new_translation = current_translation + translation_update
                new_rotation_matrix = Rotation.from_euler('xyz', rotation_update).as_matrix() @ Rotation.from_euler('xyz', current_rotation).as_matrix()

                new_pose = np.eye(4)
                new_pose[:3, :3] = new_rotation_matrix
                new_pose[:3, 3] = new_translation
                vertex._matrix = new_pose

class GraphSlam():
    """Implements a SLAM system using a pose graph. Manages the integration of new poses and loop closure detection."""

    def __init__(self, travel_distance_threshold: float = 0.1, loop_closure_error_threshold: float = 0.6) -> None:
        self.graph = PoseGraph()
        self.last_pose_vertex = None
        self.travel_distance = 0.0
        self.combined_transformation = np.eye(4)

        self.travel_distance_threshold = travel_distance_threshold
        self.loop_closure_error_threshold = loop_closure_error_threshold

        self.on_pose_added_callbacks = []
        self.on_optimized_callbacks = []
 
    def update(self, transformation_matrix: np.ndarray, point_cloud: np.ndarray) -> None:
        """
        Updates the SLAM system with a new pose and detects loop closures if necessary.

        Parameters: 
            transformation_matrix (np.ndarray): The transformation matrix representing the motion from the previous pose.
            point_cloud (np.ndarray): The point cloud data associated with the new pose.
        """
        self.combined_transformation = self.combined_transformation @ transformation_matrix
        self.travel_distance += np.linalg.norm(_matrix2translation(transformation_matrix))
        if self.travel_distance < self.travel_distance_threshold:
            return

        new_pose_vertex = self.graph.add_pose((self.last_pose_vertex.get_homogeneous_matrix() if self.last_pose_vertex else np.eye(4)) @ self.combined_transformation, point_cloud)
        if self.last_pose_vertex is not None: self.graph.add_constraint(self.last_pose_vertex, new_pose_vertex, self.combined_transformation, np.eye(6))
        closure = self._detect_loop_closures(new_pose_vertex, 1, 10)
        if closure: 
            self.graph.optimize()
            for func in self.on_optimized_callbacks: func(self.last_pose_vertex, new_pose_vertex, self.combined_transformation)

        for func in self.on_pose_added_callbacks: func(self.last_pose_vertex, new_pose_vertex, self.combined_transformation)

        self.last_pose_vertex = new_pose_vertex
        self.travel_distance = 0.0
        self.combined_transformation = np.eye(4)

    def register_on_pose_added(self, func) -> None:
        """Registers a callback for when a new pose is added."""
        if callable(func): self.on_pose_added_callbacks.append(func)

    def register_on_optimized_added(self, func) -> None:
        """Registers a callback for when the graph is optimized."""
        if callable(func): self.on_optimized_callbacks.append(func)

    def _detect_loop_closures(self, pose_vertex: PoseVertex, voxel_range: int, min_steps_separation: int) -> Tuple[float, PoseVertex, PoseVertex, np.ndarray, np.ndarray] | None:
        """
        Detects loop closures using nearby poses and returns the best closure if found.

        Parameters:
            pose_vertex (PoseVertex): The current pose vertex to check for loop closures.
            voxel_range (int): The spatial range (in voxels) to search for nearby poses.
            min_steps_separation (int): The minimum number of steps from the starting vertex to consider two poses as candidates for loop closure.

        Returns:
            Tuple[float, PoseVertex, PoseVertex, np.ndarray, np.ndarray] | None: The best loop closure found (if any), including the error, 
            involved vertices, transformation matrix, and information matrix. Returns None if no loop closure is detected.
        """
        nearby_poses = self.graph.get_nearby_poses(pose_vertex.get_position(), voxel_range)
        poses_within_steps = self.graph.get_vertices_within_steps(pose_vertex, min_steps_separation)
        nearby_poses = [nearby_pose for nearby_pose in nearby_poses if nearby_pose not in poses_within_steps]

        loop_closures = [] # [(error, from, to, transformation_matrix, information_matrix)]
        for pose in nearby_poses:
            point_cloud_transformed, transformation_matrix, mean_error = icp(pose_vertex.point_cloud, pose.point_cloud, 20, 1e-4)
            if mean_error < self.loop_closure_error_threshold: 
                loop_closures.append((mean_error, pose, pose_vertex, transformation_matrix, np.eye(6)))

        if len(loop_closures) > 0:
            min_error_closure = min(loop_closures, key=lambda x: x[0])
            self.graph.add_constraint(min_error_closure[1], min_error_closure[2], min_error_closure[3], min_error_closure[4])
            return min_error_closure
        return None

def _matrix2translation(transformation_matrix) -> np.ndarray: 
    """
    Extracts the translation component from a homogeneous transformation matrix.

    Parameters:
        transformation_matrix (np.ndarray): A 4x4 numpy array representing the homogeneous transformation matrix.
    Returns:
        out (np.ndarray): A 1x3 numpy array containing the translation component (x, y, z).
    """
    return transformation_matrix[:3, 3]

def _matrix2rotation(transformation_matrix) -> np.ndarray: 
    """
    Extracts the rotation component from a homogeneous transformation matrix and converts it into Euler angles.

    Parameters:
        transformation_matrix (np.ndarray): A 4x4 numpy array representing the homogeneous transformation matrix.
    Returns:
        out (np.ndarray): A 1x3 numpy array containing the rotation in Euler angles (roll, pitch, yaw).
    """
    return Rotation.from_matrix(transformation_matrix[:3, :3]).as_euler("xyz")

def _matrix2state(transformation_matrix) -> np.ndarray: 
    """
    Converts a homogeneous transformation matrix into a state vector that includes both translation and rotation.

    Parameters:
        transformation_matrix (np.ndarray) : A 4x4 numpy array representing the homogeneous transformation matrix.
    Returns:
        out (np.ndarray): A 1x6 numpy array containing the translation and rotation in Euler angles (x, y, z, roll, pitch, yaw).
    """
    return np.concatenate((_matrix2translation(transformation_matrix), _matrix2rotation(transformation_matrix))).flatten()

def _state2matrix(state) -> np.ndarray: 
    """
    Converts a state vector, which includes translation and rotation in Euler angles into a homogeneous transformation matrix.

    Parameters:
        state (np.ndarray): A 1x6 numpy array containing translation (x, y, z) and rotation (roll, pitch, yaw).
    Returns:
        out (np.ndarray): A 4x4 numpy array representing the homogeneous transformation matrix.
    """
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler("xyz", state[3:]).as_matrix()
    matrix[:3, 3] = np.array(state[:3])
    return matrix
