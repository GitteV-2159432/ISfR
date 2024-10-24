from sapien.core import Entity, Pose, Scene
from sapien.sensor.sensor_base import SensorEntity

from typing import Optional, List, Tuple
from copy import deepcopy as copy
from scipy.spatial.transform import Rotation
import numpy as np

class LidarSensorConfig:
    def __init__(self) -> None:
        """
        An instance of this class is required to initialize LidarSensor.
        """
        
        self.detection_range: float = 10
        """Maximum range from where the lidar can detect an object in meters"""

        self.field_of_view: float = 360
        """Field of view in the horizontal plane in degrees"""

        self.samples: int = 100
        """Number of rays around the horizontal plane"""
                
        self.noise_standard_deviation_distance: float = 0.01
        """Standard deviation for the added normal distributed noise on the detected distance"""
        
        self.noise_standard_deviation_angle_horizontal: float = 0.01
        """Standard deviation for the added normal distributed noise on the horizontal angle of the ray"""
        
        self.noise_standard_deviation_angle_vertical: float = 0
        """Standard deviation for the added normal distributed noise on the vertical angle of the ray"""
        
        self.noise_outlier_chance: float = 0.001
        """Chance for a random outlier to occur"""

        self.randomize_start_angle: bool = True
        """Chance for a random outlier to occur"""


class LidarSensor(SensorEntity):
    """
    This class simulates a lidar sensor. Refer to LidarSensorConfig for configurable parameters.
    """
        
    def __init__(self, sensor_name: str, scene: Scene, config: LidarSensorConfig, mount_entity: Optional[Entity] = None, pose: Optional[Pose] = None) -> None:
        """
        :param sensor_name: name of the sensor.
        :param scene: scene that the sensor is attached to.
        :param config: configuration of the sensor.
        :param mount: optionally mount the sensor to an actor. If mounted, the sensor will move along with the actor.
        :param pose: If not mounted, this will be the global pose for the sensor. Otherwise, it will be the local pose relative to the mounted actor.
        """

        super().__init__()

        # Basic configuration
        self.name = sensor_name
        self._scene = scene
        self._config = config
        self._results: List[Tuple[float, float, float]] = []
        self.temp = 0

        # self._pose is global if not mounted and local if mounted
        self._mount = mount_entity
        if pose is None:
            self._pose = Pose()
        else:
            self._pose = pose

    def set_pose(self, pose: Pose) -> None:
        """
        If mount exists, set local pose of the sensor relative to mounted actor. Otherwise, set global pose of the sensor.
        """
        self._pose = pose

    def get_config(self) -> LidarSensorConfig:
        return copy(self._config)

    def get_pose(self) -> None:
        if self._mount is None:
            return copy(self._pose)
        else:
            return copy(self._mount.get_pose() * self._pose)
        
    def simulate(self) -> None:
        """
        Simulates the LiDAR.
        """

        start_angle = 0
        if self._config.randomize_start_angle:
            start_angle = np.random.uniform(0, self._config.field_of_view)

        results = []
        for i in range(self._config.samples):
            angle_horizontal = (i / max(self._config.samples - 1, 1) * self._config.field_of_view + start_angle) % self._config.field_of_view - (self._config.field_of_view / 2.0) + np.random.normal(0, self._config.noise_standard_deviation_angle_horizontal)
            angle_vertical = np.random.normal(0, self._config.noise_standard_deviation_angle_vertical)
                
            ray_direction = self._compute_ray_direction(np.radians(angle_horizontal), np.radians(angle_vertical))
            distance = self._raycast(ray_direction)

            distance += np.random.normal(0, self._config.noise_standard_deviation_distance)
            if np.random.rand() < self._config.noise_outlier_chance:
                distance = np.random.uniform(0, self._config.detection_range)

            results.append((angle_horizontal, angle_vertical, distance))

        self._results = results
    
    def get_points(self) -> List[Tuple[float, float, float]]:
        """
        :return: A list of tuples (angle_horizontal, angle_vertical, distance)
        """
        if len(self._results) > 0:
            return self._results
        else:
            raise ValueError("run the simulate function before getting the points")

    def get_point_cloud(self) -> List[Tuple[float, float, float]]:
        """
        :return: A list of tuples (position_x, position_y, position_z)
        """
        if len(self._results) == 0:
            raise ValueError("run the simulate function before getting the points")

        results_array = np.array(self._results)
        angles_horizontal = results_array[:, 0]
        angles_vertical = results_array[:, 1]
        distances = results_array[:, 2]

        ray_directions = self._compute_ray_directions(np.radians(angles_horizontal), np.radians(angles_vertical))
        local_hit_positions = ray_directions * distances[:, np.newaxis]
        return local_hit_positions.tolist()

    def _compute_ray_direction(self, angle_horizontal: float, angle_vertical: float) -> np.ndarray:
        """
        Compute the ray direction based on horizontal and vertical angles.

        :param angle_horizontal: Horizontal angle in radians.
        :param angle_vertical: Vertical angle in radians.
        :return: A unit vector representing the ray direction in world coordinates.
        """
        x = np.cos(angle_vertical) * np.cos(angle_horizontal)
        y = np.cos(angle_vertical) * np.sin(angle_horizontal)
        z = np.sin(angle_vertical)
        return np.array([x, y, z])

    def _compute_ray_directions(self, angle_horizontal: np.ndarray, angle_vertical: np.ndarray) -> np.ndarray:
        """
        Compute a list of ray directions based on a list of horizontal and vertical angles.

        :param angle_horizontal: Horizontal angle in radians (array).
        :param angle_vertical: Vertical angle in radians (array).
        :return: A 2D array where each row represents a unit vector for the ray direction in world coordinates.
        """
        x = np.cos(angle_vertical) * np.cos(angle_horizontal)
        y = np.cos(angle_vertical) * np.sin(angle_horizontal)
        z = np.sin(angle_vertical)
        
        return np.vstack((x, y, z)).T

    def _raycast(self, direction_local: np.ndarray) -> float:
        """
        Perform a raycast in the scene along a specified local direction.

        :param ray_direction: The direction of the ray in local coordinates.
        :return: The distance to the first detected object, or the maximum detection range if no object is detected.
        """
        pose_global = self.get_pose()
        direction_global = Rotation.from_quat(pose_global.q, scalar_first=True).apply(direction_local) # sapien uses wxyz for quaternions

        hit_info = self._scene.physx_system.raycast(pose_global.p, direction_global, self._config.detection_range)

        if hit_info is None:
            return self._config.detection_range

        return hit_info.distance