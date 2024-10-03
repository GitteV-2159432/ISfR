import sapien
import create_primitive
from scipy.spatial.transform import Rotation

def create_grid(scene: sapien.Scene, grid_size, spacing) -> None:
    offset = (grid_size - 1) * spacing / 2.0
    radius = .25
    length = 2
    for i in range(0, grid_size):
        for j in range(0, grid_size):
            x = i * spacing - offset
            y = j * spacing - offset
            create_primitive.capsule(
                scene,
                sapien.Pose(p=[x, y, 0], q=Rotation.from_euler('xyz', [0, 90, 0], degrees=True).as_quat()),
                radius=radius,
                half_length=length / 2,
                color=[1.0, 0.0, 0.0],
                name="capsule",
                is_kinematic=True,
            )

def load(scene: sapien.Scene) -> None:
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])
    scene.add_ground(altitude=0)
    create_grid(scene, 6, 5)
    