import sapien.core as sapien
import create_primitive
from scipy.spatial.transform import Rotation

def create_wall_grid(scene: sapien.Scene, grid_size, spacing, wall_height=2.0, wall_thickness=0.2, wall_length=5.0) -> None:
    offset = (grid_size - 1) * spacing / 2.0
    
    for i in range(grid_size + 1):  # grid_size + 1 to create closed boundaries
        # Vertical walls along Y-axis
        x = i * spacing - offset
        create_primitive.box(
            scene,
            sapien.Pose(p=[x, -offset, wall_height / 2], q=Rotation.from_euler('xyz', [0, 0, 0], degrees=True).as_quat()),
            half_size=[wall_thickness / 2, wall_length / 2, wall_height / 2],
            color=[0.7, 0.7, 0.7],
            name="vertical_wall",
            is_kinematic=True,
        )
        create_primitive.box(
            scene,
            sapien.Pose(p=[x, offset, wall_height / 2], q=Rotation.from_euler('xyz', [0, 0, 0], degrees=True).as_quat()),
            half_size=[wall_thickness / 2, wall_length / 2, wall_height / 2],
            color=[0.7, 0.7, 0.7],
            name="vertical_wall",
            is_kinematic=True,
        )
    
    for j in range(grid_size + 1):
        # Horizontal walls along X-axis
        y = j * spacing - offset
        create_primitive.box(
            scene,
            sapien.Pose(p=[-offset, y, wall_height / 2], q=Rotation.from_euler('xyz', [0, 0, 90], degrees=True).as_quat()),
            half_size=[wall_length / 2, wall_thickness / 2, wall_height / 2],
            color=[0.7, 0.7, 0.7],
            name="horizontal_wall",
            is_kinematic=True,
        )
        create_primitive.box(
            scene,
            sapien.Pose(p=[offset, y, wall_height / 2], q=Rotation.from_euler('xyz', [0, 0, 90], degrees=True).as_quat()),
            half_size=[wall_length / 2, wall_thickness / 2, wall_height / 2],
            color=[0.7, 0.7, 0.7],
            name="horizontal_wall",
            is_kinematic=True,
        )
