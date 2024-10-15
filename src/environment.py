import sapien.core as sapien
import create_primitive
from scipy.spatial.transform import Rotation

class Environment:
    def __init__(self, scene: sapien.Scene, grid_size: int = 6, spacing: float = 5.0, 
                 wall_height: float = 2.0, wall_thickness: float = 0.2, wall_length: float = 5.0):
        self.scene = scene
        self.grid_size = grid_size
        self.spacing = spacing
        self.wall_height = wall_height
        self.wall_thickness = wall_thickness
        self.wall_length = wall_length
        self.offset = (grid_size - 1) * spacing / 2.0
        self.number_of_walls = 4
        self.walls = []  # Store references to walls for removal

    def create_wall_grid(self) -> None:
        for i in range(self.grid_size + 1):
            x = i * self.spacing - self.offset
            for j in range(self.number_of_walls):
                self._create_wall(x, -self.offset + j * 5, rotation_z=0)
                self._create_wall(x, self.offset + j * 5, rotation_z=0) 
        
        for i in range(self.grid_size + 1):
            y = i * self.spacing - self.offset
            for j in range(self.number_of_walls):
                self._create_wall(x=-self.offset + j * 5, y=y, rotation_x=90)
                self._create_wall(x=self.offset + j * 5, y=y, rotation_x=90)
        
        self.create_path()  # Call create_path after creating walls

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
        self.walls.append(wall)  # Add wall to list for future reference

    def create_path(self) -> None:
        path_positions = [
            (0, 0), (0, 1), (1, 1), (1, 2), (2, 2), (3, 2),
            (3, 3), (3, 4), (4, 4), (5, 4), (5, 5)
        ]
        
        for (i, j) in path_positions:
            x = i * self.spacing - self.offset
            y = j * self.spacing - self.offset
            self._remove_wall(x, y)

    def _remove_wall(self, x: float, y: float) -> None:
        for wall in self.walls:
            if wall.pose.p[0] == x and wall.pose.p[1] == y:
                self.scene.remove_actor(wall)  # Remove the wall from the scene
                self.walls.remove(wall)  # Remove from list to avoid future checks
                break

    def load_scene(self) -> None:
        self.scene.set_ambient_light([0.5, 0.5, 0.5])
        self.scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])
        self.scene.add_ground(altitude=0)
        self.create_wall_grid()
