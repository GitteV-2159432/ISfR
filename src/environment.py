import sapien.core as sapien
import create_primitive
from scipy.spatial.transform import Rotation

class Environment:
    def __init__(self, scene: sapien.Scene, grid_size: int = 20, spacing: float = 3.0, 
                 wall_height: float = 2.0, wall_thickness: float = 0.2, wall_length: float = 3.0):
        self.scene = scene
        self.grid_size = grid_size
        self.spacing = spacing
        self.wall_height = wall_height
        self.wall_thickness = wall_thickness
        self.wall_length = wall_length
        self.offset = grid_size // 2
        self.walls = []

        self.grid_matrix = [
            [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1],
            [0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 0, 0, 0, 0],
            [0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
        ]

    def create_wall_grid(self) -> None:
        for i in range(self.grid_size + 1):
            x = i * self.spacing - self.offset
            for j in range(self.grid_size + 1):
                self._create_wall(x, -self.offset + j * self.spacing, rotation_z=0)
                    
        for i in range(self.grid_size + 1):
            y = i * self.spacing - self.offset
            for j in range(self.grid_size + 1):
                self._create_wall(x=-self.offset + j * self.spacing, y=y, rotation_x=90)
                
        self.create_path_from_matrix(padding=1)  # Specify padding here

    def create_path_from_matrix(self, padding: int = 0) -> None:
        for i, row in enumerate(self.grid_matrix):
            for j, cell in enumerate(row):
                if cell == 1:  # Path cell
                    for dx in range(-padding, padding + 1):
                        for dy in range(-padding, padding + 1):
                            x = (i + dx) * self.spacing - self.offset
                            y = (j + dy) * self.spacing - self.offset
                            self._remove_wall(x, y)

    def _create_wall(self, x: float, y: float, rotation_x: float = 0, rotation_y: float = 0, rotation_z: float = 0) -> None:
        wall = create_primitive.box(
            self.scene,
            sapien.Pose(
                p=[x, y, self.wall_height / 2],
                q=Rotation.from_euler('xyz', [rotation_x, rotation_y, rotation_z], degrees=True).as_quat()
            ),
            half_size=[
                self.wall_length / 2 if rotation_z == 90 else self.wall_thickness / 2,
                self.wall_thickness / 2 if rotation_z == 90 else self.wall_length / 2,
                self.wall_height / 2
            ],
            color=[0.7, 0.7, 0.7],
            name="wall",
            is_kinematic=True,
        )
        self.walls.append(wall)

    def _remove_wall(self, x: float, y: float) -> None:
        for wall in self.walls:
            if abs(wall.pose.p[0] - x) < 0.1 and abs(wall.pose.p[1] - y) < 0.1:
                self.scene.remove_actor(wall)
                self.walls.remove(wall)
                break

    def load_scene(self) -> None:
        self.scene.set_ambient_light([0.5, 0.5, 0.5])
        self.scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])
        self.scene.add_ground(altitude=0)
        self.create_wall_grid()
