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
        
    def get_odometry(self) -> Tuple[float, float]:
        """
        Get the odometry from the robot

        :param return: The odometry in the form (velocity: float, angular_velocity: float)
        """
        return self.velocity, self.angular_velocity
