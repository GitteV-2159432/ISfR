#! Implementation based on https://github.com/ClayFlannigan/icp/blob/master/icp.py

import numpy as np
from scipy.spatial import cKDTree

def icp(source_points, target_points, max_iterations=50, tolerance=1e-6):
    """
    Perform the Iterative Closest Point algorithm to align source_points to target_points.

    Parameters:
        source_points (np.ndarray): The source point cloud, shape (N, 3).
        target_points (np.ndarray): The target point cloud, shape (M, 3).
        max_iterations (int): Maximum number of iterations to perform.
        tolerance (float): Convergence tolerance.

    Return:
        np.ndarray: Transformed source points, aligned to target points.
        np.ndarray: Final transformation matrix (4x4).
        float: Final transformation matrix (4x4).
    """

    transformation_matrix = np.eye(4)  

    def best_fit_transform(source, target):
        source_centroid = np.mean(source, axis=0)
        target_centroid = np.mean(target, axis=0)

        source_centered = source - source_centroid
        target_centered = target - target_centroid
        covariance_matrix = np.dot(source_centered.T, target_centered)

        # Singular Value Decomposition
        U, S, Vt = np.linalg.svd(covariance_matrix)
        rotation = np.dot(Vt.T, U.T)

        # Ensure a proper rotation -> determinant should be +1, if it is -1 it should be flipped
        if np.linalg.det(rotation) < 0:
            Vt[2, :] *= -1
            rotation = np.dot(Vt.T, U.T)

        # Compute the translation
        translation = target_centroid - np.dot(rotation, source_centroid)

        # Build the transformation matrix
        transformation = np.eye(4)
        transformation[:3, :3] = rotation
        transformation[:3, 3] = translation

        return transformation

    for _ in range(max_iterations):
        # Find the nearest neighbors in the target point cloud
        tree = cKDTree(target_points)
        distances, indices = tree.query(source_points)

        # Transformation between matched points
        closest_points = target_points[indices]
        transformation = best_fit_transform(source_points, closest_points)

        # Apply the transformation to the source point cloud
        source_points_homogeneous = np.hstack((source_points, np.ones((source_points.shape[0], 1))))
        source_points_transformed = np.dot(transformation, source_points_homogeneous.T).T[:, :3]

        # Check convergence
        mean_error = np.mean(distances)
        if mean_error < tolerance:
            break

        # Update source points and transformation matrix
        source_points = source_points_transformed
        transformation_matrix = np.dot(transformation, transformation_matrix)

    return source_points, transformation_matrix, mean_error