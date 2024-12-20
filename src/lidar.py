from sapien.core import Entity, Pose, Scene
from sapien.sensor.sensor_base import SensorEntity

from typing import Optional, List, Tuple
from copy import deepcopy as copy
from scipy.spatial.transform import Rotation
import numpy as np
import json

class LidarSensorConfig:
    def __init__(self, config_file_name: Optional[str] = None) -> None:
        """
        An instance of this class is required to initialize LidarSensor.
        """
        
        self.detection_range_max: float = 10.0
        """Maximum range from where the lidar can detect an object in meters"""

        self.detection_range_min: float = 0.1
        """Maximum range from where the lidar can detect an object in meters"""

        self.vertical_field_of_view: float = 0.0
        """Field of view in the vertical plane in degrees"""

        self.horizontal_field_of_view: float = 360.0
        """Field of view in the horizontal plane in degrees"""

        self.vertical_samples: int = 1
        """Number of rays around the vertical plane"""

        self.horizontal_samples: int = 150
        """Number of rays around the horizontal plane"""
                
        self.noise_standard_deviation_distance: float = 0.01
        """Standard deviation for the added normal distributed noise on the detected distance"""
        
        self.noise_standard_deviation_angle_horizontal: float = 0.01
        """Standard deviation for the added normal distributed noise on the horizontal angle of the ray"""
        
        self.noise_standard_deviation_angle_vertical: float = 0.0
        """Standard deviation for the added normal distributed noise on the vertical angle of the ray"""
        
        self.noise_outlier_chance: float = 0.0
        """Chance for a random outlier to occur"""

        if config_file_name:
            with open(config_file_name, 'r') as file:
                config = json.load(file)

            self.detection_range_max = config['specifications']['detection_range']['max']
            self.detection_range_min = config['specifications']['detection_range']['min']
            self.vertical_field_of_view = config['specifications']['fov']['vertical']
            self.horizontal_field_of_view = config['specifications']['fov']['horizontal']
            self.vertical_samples = config['specifications']['samples']['vertical']
            self.horizontal_samples = config['specifications']['samples']['horizontal']
            self.noise_standard_deviation_distance = config['specifications']['accuracy']['range_0_10m_(mm)'] * 1e-3
            self.noise_standard_deviation_angle_horizontal = config['specifications']['accuracy']['range_0_10m_(mm)'] * 1e-3
            self.noise_standard_deviation_angle_vertical = config['specifications']['accuracy']['range_0_10m_(mm)'] * 1e-3

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

        # self._pose is global if not mounted and local if mounted
        self._mount = mount_entity
        if pose is None:
            self._pose = Pose()
        else:
            self._pose = pose

        self._angles_horizontal = []
        self._angles_vertical = []
        for horizontal_sample_index in range(self._config.horizontal_samples):
            for vertical_sample_index in range(self._config.vertical_samples):
                self._angles_horizontal.append((horizontal_sample_index / (self._config.horizontal_samples - 1) * self._config.horizontal_field_of_view) % self._config.horizontal_field_of_view - (self._config.horizontal_field_of_view / 2.0))
                self._angles_vertical.append((vertical_sample_index / (self._config.vertical_samples - 1)) * self._config.vertical_field_of_view)
        self._ray_directions: List[Tuple[float, float]] = self._compute_ray_directions(np.radians(self._angles_horizontal), np.radians(self._angles_vertical))

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
        results = []
        for ray_direction, angle_horizontal, angle_vertical in zip(self._ray_directions, self._angles_horizontal, self._angles_vertical):
            distance = self._raycast(ray_direction)

            if distance < self._config.detection_range_max:
                distance += np.random.normal(0, self._config.noise_standard_deviation_distance)
                
            if np.random.rand() < self._config.noise_outlier_chance:
                distance = np.random.uniform(0, self._config.detection_range_max)

            angle_horizontal += np.random.normal(0, self._config.noise_standard_deviation_angle_horizontal)
            angle_vertical += np.random.normal(0, self._config.noise_standard_deviation_angle_vertical)
            results.append((np.radians(angle_horizontal), np.radians(angle_vertical), distance))

        self._results = results
    
    def get_measurements(self, include_max=False, in_degrees=False) -> List[Tuple[float, float, float]]:
        """
        :return: A list of tuples (angle_horizontal, angle_vertical, distance)
        """
        if len(self._results) <= 0:
            raise ValueError("run the simulate function before getting the points")

        results = self._results if include_max else [r for r in self._results if r[2] < self._config.detection_range_max]
        if in_degrees: return [(np.rad2deg(results[0]), np.rad2deg(results[1]), results[2])]
        else: return results

    def get_point_cloud(self, include_max=False) -> List[Tuple[float, float, float]]:
        """
        :return: A list of tuples (position_x, position_y, position_z)
        """
        if len(self._results) <= 0:
            raise ValueError("run the simulate function before getting the points")

        results = self._results if include_max else [r for r in self._results if r[2] < self._config.detection_range_max]
        results = np.array(results)
        angles_horizontal = results[:, 0]
        angles_vertical = results[:, 1]
        distances = results[:, 2]

        ray_directions = self._compute_ray_directions(angles_horizontal, angles_vertical)
        local_hit_positions = ray_directions * distances[:, np.newaxis]
        #publish_lidar_points(local_hit_positions.tolist())
        return local_hit_positions.tolist()

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

        # TODO: Add minimum distance

        pose_global = self.get_pose()
        direction_global = Rotation.from_quat(pose_global.q, scalar_first=True).apply(direction_local) # sapien uses w, x, y, z for quaternions

        hit_info = self._scene.physx_system.raycast(pose_global.p, direction_global, self._config.detection_range_max)

        if hit_info is None:
            return self._config.detection_range_max

        return hit_info.distance
    
class GenerateDepthmap:
    def __init__(self, image_shape, camera_matrix):
        self.image_shape = image_shape
        self.camera_matrix = camera_matrix
        self.depthmap = None
    def create_depthmap(self, points_3d):
        """
        Generate depthmap with 3d points
        points_3d: points coming from observation of environment
        image_shape: shape of image
        camera_matrix = 3x3 matrix of the camera to project from 3d to 2d
        
        Returns 2D array with same shape as image. 
        Contains closest z-val or np.inf
        z of depthmap can be asked by: z=depthmap[x,y]
        """
        #:2 because colored img, height&width van image
        h,w = self.image_shape[:2]
        #vul depthmap met inf
        self.depthmap = np.full((h,w), np.inf)
        
        for point in points_3d:
            x,y,z = point[0], point[1],point[2]
            #ignore negative values for z => behind camera
            if z <=0:
                continue
            #project punt naar 2d vlak met cam matrix
            pixel_coords = self.camera_matrix @ np.array([x,y,z])[:3]
            pixel_coords /= pixel_coords[2]
            u,v= int(pixel_coords[0]), int(pixel_coords[1])
            #check of punt binnen de grenzen ligt
            if 0 <=u < w and 0 <= v<h:
                #select closest
                self.depthmap[v,u] = min(self.depthmap[v,u],z)
        return self.depthmap
    def get_depthmap(self, x,y):
        if self.depthmap is None:
            raise ValueError("Depthmap is none!")
        return self.depthmap[y,x]