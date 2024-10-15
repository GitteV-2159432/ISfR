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
        self.offset = 5
        self.number_of_walls = 10
        self.space_between_walls = 2
        self.walls = []
    
        self.path_coordinates = [
            # Start point to the first vertical transition
            (0, 0), (1, 0), (2, 0), (3, 0), (4, 0), (5, 0), (6, 0), (7, 0), (8, 0), (9, 0), (10, 0), (11, 0),
            
        ]




    def create_wall_grid(self) -> None:
        for i in range(self.grid_size + 1):
            x = i * self.spacing - self.offset
            for j in range(self.number_of_walls):
                self._create_wall(x, -self.offset + j * self.space_between_walls, rotation_z=0)
                    
        for i in range(self.grid_size + 1):
            y = i * self.spacing - self.offset
            for j in range(self.number_of_walls):
                self._create_wall(x=-self.offset + j * self.space_between_walls, y=y, rotation_x=90)
                
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
        self.walls.append(wall)

    def create_path(self) -> None:
        # Use predefined coordinates to remove walls and create a path
        for (i, j) in self.path_coordinates:
            # Make path wider by removing walls around the main path
            for dx in [-1, 0, 1]:  # Offset to create a wider path
                for dy in [-1, 0, 1]:
                    x = (i + dx) * self.spacing - self.offset
                    y = (j + dy) * self.spacing - self.offset
                    self._remove_wall(x, y)

    def _remove_wall(self, x: float, y: float) -> None:
        # Remove walls matching the position (x, y) regardless of orientation
        for wall in self.walls:
            if abs(wall.pose.p[0] - x) < 0.1 and abs(wall.pose.p[1] - y) < 0.1:
                self.scene.remove_actor(wall)  # Remove the wall from the scene
                self.walls.remove(wall)  # Remove from list to avoid future checks
                break

    def load_scene(self) -> None:
        self.scene.set_ambient_light([0.5, 0.5, 0.5])
        self.scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])
        self.scene.add_ground(altitude=0)
        self.create_wall_grid()
