import sapien.core as sapien
from sapien import Pose
from sapien.utils import Viewer
import numpy as np
from scipy.spatial.transform import Rotation
import cv2

class DifferentialDriveRobot:
    def __init__(self, scene: sapien.Scene, viewer: Viewer, target_position: np.ndarray) -> None:
        self.scene = scene
        self.viewer = viewer
        self.target_position = target_position
        
        self.move_speed = 0.05
        self.turn_speed = 1.0

        # Robot dimensions
        body_size = (1.0, 0.5, 0.25)
        tire_radius = 0.15
        density = 1.0

        # Create articulation builder
        builder = scene.create_articulation_builder()

        # Car body (root of the articulation)
        body = builder.create_link_builder()
        body.set_name('body')
        body_half_size = np.array(body_size) / 2
        body.add_box_collision(half_size=body_half_size, density=density)
        body.add_box_visual(half_size=body_half_size)

        # Left wheel
        left_wheel = builder.create_link_builder(body)
        left_wheel.set_name('left_wheel')
        left_wheel.add_capsule_collision(Pose(), radius=tire_radius, half_length=0.05, density=density)
        left_wheel.add_capsule_visual(Pose(), radius=tire_radius, half_length=0.05)
        left_wheel.set_joint_name('left_joint')
        left_wheel.set_joint_properties(
            'revolute',
            limits=[[-np.pi, np.pi]],
            pose_in_parent=Pose(p=[0, body_half_size[1] + 0.05, -body_half_size[2]]),
            pose_in_child=Pose(p=[0, 0, 0]),
        )

        # Right wheel
        right_wheel = builder.create_link_builder(body)
        right_wheel.set_name('right_wheel')
        right_wheel.add_capsule_collision(Pose(), radius=tire_radius, half_length=0.05, density=density)
        right_wheel.add_capsule_visual(Pose(), radius=tire_radius, half_length=0.05)
        right_wheel.set_joint_name('right_joint')
        right_wheel.set_joint_properties(
            'revolute',
            limits=[[-np.pi, np.pi]],
            pose_in_parent=Pose(p=[0, -body_half_size[1] - 0.05, -body_half_size[2]]),
            pose_in_child=Pose(p=[0, 0, 0]),
        )

        # Build the articulation
        self.robot = builder.build()
        self.robot.set_root_pose(Pose(p=[0, 0, 1.0]))  # Adjust the initial position as needed

        # Store body and wheels for easy access
        self.body = self.robot.get_links()[0]
        self.left_wheel_joint = self.robot.get_joints()[0]
        self.right_wheel_joint = self.robot.get_joints()[1]

        # Set drive properties
        self.left_wheel_joint.set_drive_property(stiffness=0.0, damping=10.0)
        self.right_wheel_joint.set_drive_property(stiffness=0.0, damping=10.0)

    def move_to_target(self) -> None:
        # Get current position of the robot
        current_position = self.body.get_pose().p

        # Calculate the direction to the target
        direction_to_target = self.target_position - current_position
        distance_to_target = np.linalg.norm(direction_to_target)

        if distance_to_target > 0.01:  # Set a small threshold to stop
            # Normalize direction vector
            direction_to_target_normalized = direction_to_target / distance_to_target
            
            # Move the robot towards the target
            new_position = current_position + direction_to_target_normalized * self.move_speed
            self.body.set_pose(Pose(p=new_position, q=self.body.get_pose().q))

    def update(self) -> None:
        # Call move_to_target in update to move the robot continuously
        self.move_to_target()
        
        # Handle keyboard input for additional controls (if necessary)
        move_direction_local = np.array([
            int(self.viewer.window.key_down('i')) - int(self.viewer.window.key_down('k')), 
            int(self.viewer.window.key_down('j')) - int(self.viewer.window.key_down('l')), 
            0
        ])
        
        turn_direction = int(self.viewer.window.key_down('u')) - int(self.viewer.window.key_down('o'))
        pose_global = self.body.get_pose()
        rotation_current = Rotation.from_quat(pose_global.q, scalar_first=True)
        move_direction_global = rotation_current.apply(move_direction_local)

        rotation_new = rotation_current * Rotation.from_euler("xyz", [0, 0, turn_direction * self.turn_speed], degrees=True)

        position_current = pose_global.p
        position_new = position_current + move_direction_global * self.move_speed

        self.body.set_pose(Pose(p=position_new, q=rotation_new.as_quat(scalar_first=True)))

        # Update wheel velocities
        left_speed = self.move_speed * (1 - turn_direction)
        right_speed = self.move_speed * (1 + turn_direction)
        self.left_wheel_joint.set_drive_velocity_target(np.array([left_speed]))
        self.right_wheel_joint.set_drive_velocity_target(np.array([right_speed]))

