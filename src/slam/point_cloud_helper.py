import numpy as np

def rotation_matrix_from_euler_angles(x: float, y: float, z: float) -> np.ndarray:
    """
    Generates a rotation matrix from Euler angles in radians.

    Params:
        x (float): rotation around the x axis in radians
        y (float): rotation around the y axis in radians
        z (float): rotation around the z axis in radians
    Returns:
        np.ndArray: The rotation matrix
    """
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(x), -np.sin(x)],
                    [0, np.sin(x), np.cos(x)]])
    
    R_y = np.array([[np.cos(y), 0, np.sin(y)],
                    [0, 1, 0],
                    [-np.sin(y), 0, np.cos(y)]])
    
    R_z = np.array([[np.cos(z), -np.sin(z), 0],
                    [np.sin(z), np.cos(z), 0],
                    [0, 0, 1]])

    return np.dot(R_z, np.dot(R_y, R_x))

def apply_translation(points: np.ndarray, translation: np.ndarray) -> np.ndarray:
    """
    Applies translation to a set of points.
    """
    return points + translation

def apply_rotation(points: np.ndarray, rotation: np.ndarray) -> np.ndarray:
    """
    Applies rotation to a set of points.
    """
    return np.dot(points, rotation.T) 

def apply_scale(point_cloud: np.ndarray, scale_factor: float) -> np.ndarray:
    """
    Applies scales to a set of points.
    """
    return point_cloud * scale_factor

def add_noise_to_point_cloud(point_cloud: np.ndarray, standard_deviation: float) -> np.ndarray:
    """
    Adds Gaussian noise to a point cloud.
    """
    return point_cloud + np.random.normal(0, standard_deviation, point_cloud.shape)

def downsample_point_cloud(point_cloud: np.ndarray, factor: float) -> np.ndarray:
    """
    Downsamples the point cloud to a given factor.
    """
    original_size = point_cloud.shape[0]
    num_points_to_keep = int(original_size * np.clip(factor, 0, 1))
    return point_cloud[np.random.choice(original_size, num_points_to_keep, replace=False)]
