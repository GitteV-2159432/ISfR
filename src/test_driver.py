import sapien as sapien
from sapien import Scene, Pose
from sapien.utils import Viewer

import numpy as np
from scipy.spatial.transform import Rotation
from typing import Tuple
from datetime import datetime

from create_primitive import box
    
class driver:
    def __init__(self, scene: Scene, viewer: Viewer) -> None:
        size = .5
        self.last_time = datetime.now()

        self._viewer = viewer
        self.move_speed = 1
        self.turn_speed = 1
        self.eye_left_offset = np.array([size / 2 - .06, size / 2 - .06, size / 2 - .06])
        self.eye_right_offset = np.array([size / 2 - .06, -size / 2 + .06, size / 2 - .06])
        self.camera_offset = np.array([0, 0, size])

        self.velocity = 0.0
        self.angular_velocity = 0.0

        self.body = box(
            scene,
            Pose(p=[0, 0, size / 2]),
            half_size=[size / 2] * 3,
            color=[0.0, 1.0, 0.0],
            name="driver",
            is_kinematic=True,
        )

        self.eye_left = box(
            scene,
            Pose(),
            half_size=[.1] * 3,
            color=[0.0, 0.0, 1.0],
            name="driver eye left",
            is_kinematic=True,
        )

        self.eye_right = box(
            scene,
            Pose(),
            half_size=[.1] * 3,
            color=[0.0, 0.0, 1.0],
            name="driver eye right",
            is_kinematic=True,
        )

        # Voeg een camera toe aan de robot
        self.camera = scene.add_camera(
            name="driver_camera",
            width=640,   # Cameraresolutie breedte
            height=480,  # Cameraresolutie hoogte
            fovy=1.0,    # Verticale kijkhoek in radianen
            near=0.1,    # Dichterbij snijvlak
            far=100.0    # Veraf snijvlak
        )

        self.previous_pose = self.body.get_pose()

    def update(self) -> None:
        time_step = (datetime.now() - self.last_time).microseconds * float(1e-6)
        self.last_time = datetime.now()

        move_direction_local = np.array([
            int(self._viewer.window.key_down('i')) - int(self._viewer.window.key_down('k')), 
            0, 
            0
        ])

        turn_direction = int(int(self._viewer.window.key_down('j')) - int(self._viewer.window.key_down('l')))

        self.velocity = move_direction_local[0] * self.move_speed
        self.angular_velocity = turn_direction * self.turn_speed

        pose_global = self.body.get_pose()
        rotation_current = Rotation.from_quat(pose_global.q, scalar_first=True)
        move_direction_global = rotation_current.apply(move_direction_local)

        rotation_new = rotation_current * Rotation.from_euler("xyz", [0, 0, turn_direction * self.turn_speed * time_step], degrees=False)

        position_current = pose_global.p
        position_new = position_current + move_direction_global * self.move_speed * time_step

        self.body.set_pose(Pose(p=position_new, q=rotation_new.as_quat(scalar_first=True)))

        body_pose = self.body.get_pose()
        self.eye_right.set_pose(body_pose * Pose(p=self.eye_right_offset))
        self.eye_left.set_pose(body_pose * Pose(p=self.eye_left_offset))
        self.camera.set_pose(body_pose * Pose(p=self.camera_offset))
        
    def get_odometry_transformation_matrix(self, noise_position=0.0, noise_rotation=0.0):
        """
        Get the relative position of the robot since the last call to this function

        :param return: A 4x4 transformation matrix
        """
        position_previous, euler_previous = self.previous_pose.get_p(), Rotation.from_quat(self.previous_pose.get_q(), scalar_first=True).as_euler("xyz")
        position_current, euler_current = self.body.get_pose().get_p(), Rotation.from_quat(self.body.get_pose().get_q(), scalar_first=True).as_euler("xyz")
        self.previous_pose = self.body.get_pose()

        if not np.all(position_current == position_previous):
            position_current += np.array([np.random.normal(0, noise_position), np.random.normal(0, noise_position), 0])
        if not np.all(euler_current == euler_previous):
            euler_current += np.array([0, 0, np.random.normal(0, noise_rotation)])

        transformation_matrix_previous = np.eye(4)
        transformation_matrix_previous[:3, :3] = Rotation.from_euler("xyz", euler_previous).as_matrix()
        transformation_matrix_previous[:3, 3] = position_previous

        transformation_matrix_current = np.eye(4)
        transformation_matrix_current[:3, :3] = Rotation.from_euler("xyz", euler_current).as_matrix()
        transformation_matrix_current[:3, 3] = position_current

        return np.linalg.inv(transformation_matrix_previous) @ transformation_matrix_current

    def get_odometry_velocity(self) -> Tuple[float, float]:
        """
        Get the odometry from the robot

        :param return: The odometry in the form (velocity: float, angular_velocity: float)
        """
        return self.velocity, self.angular_velocity
    
    def get_rotation(self, in_degrees=False) -> Tuple[float, float, float]:
        return Rotation.from_quat(self.body.get_pose().get_q(), scalar_first=True).as_euler("xyz", degrees=in_degrees)
    
    def get_position(self) -> Tuple[float, float, float]:
        return self.body.get_pose().get_p()