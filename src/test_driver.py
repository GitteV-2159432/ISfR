"""Create an articulation (a tree of actors/links connected by joints)

Each actor in the articulation is also called link.
The robot is an instance of articulation.

Concepts:
    - Create an articulation
    - Control the articulation basically (builtin position and velocity controller)
        sapien.physx.PhysxArticulation.set_qf, sapien.physx.PhysxArticulationJoint.set_drive_velocity_target
    - sapien.physx.PhysxArticulation.get_qpos, sapien.physx.PhysxArticulation.get_qvel
"""

import sapien as sapien
from sapien import Scene, Pose
from sapien.utils import Viewer

import numpy as np
from scipy.spatial.transform import Rotation

from create_primitive import box
    
class driver:
    def __init__(self, scene: Scene, viewer: Viewer) -> None:
        size = .5

        self._viewer = viewer
        self.move_speed = .05
        self.turn_speed = 1
        self.eye_left_offset = np.array([size / 2 - .06, size / 2 - .06, size / 2 - .06])
        self.eye_right_offset = np.array([size / 2 - .06, -size / 2 + .06, size / 2 - .06])
        self.camera_offset = np.array([0, 0, size])

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

    def update(self) -> None:
        move_direction_local = np.array([
            int(self._viewer.window.key_down('i')) - int(self._viewer.window.key_down('k')), 
            int(self._viewer.window.key_down('j')) - int(self._viewer.window.key_down('l')), 
            0
        ])

        turn_direction = int(self._viewer.window.key_down('u')) - int(self._viewer.window.key_down('o'))

        pose_global = self.body.get_pose()
        rotation_current = Rotation.from_quat(pose_global.q, scalar_first=True)
        move_direction_global = rotation_current.apply(move_direction_local)

        rotation_new = rotation_current * Rotation.from_euler("xyz", [0, 0, turn_direction * self.turn_speed], degrees=True)

        position_current = pose_global.p
        position_new = position_current + move_direction_global * self.move_speed

        self.body.set_pose(Pose(p=position_new, q=rotation_new.as_quat(scalar_first=True)))

        body_pose = self.body.get_pose()
        self.eye_right.set_pose(body_pose * Pose(p=self.eye_right_offset))
        self.eye_left.set_pose(body_pose * Pose(p=self.eye_left_offset))
        self.camera.set_pose(body_pose * Pose(p=self.camera_offset))
        
