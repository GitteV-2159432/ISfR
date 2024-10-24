import sapien.core as sapien
import create_primitive

class Environment:
    def __init__(self, scene: sapien.Scene, grid_size: int = 20, spacing: float = 3.0, wall_height: float = 2.0, wall_thickness: float = 0.2):
        self.scene = scene
        self.grid_size = grid_size
        self.spacing = spacing
        self.wall_height = wall_height
        self.wall_thickness = wall_thickness
        self.offset = grid_size / 2.0
        self.walls = []

        self.grid_matrix = [
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0],
            [0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0],
            [0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 0],
            [0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0],
            [0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 0],
            [0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0],
            [0, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 0, 1, 0],
            [0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0],
            [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
        ]

    def create_wall_grid(self) -> None:
        for x in range(self.grid_size):
            for y in range(self.grid_size):
                if self.grid_matrix[x][y] == 0:
                    self._create_wall( # Center piece
                        x = x * self.spacing - self.offset, 
                        y = y * self.spacing - self.offset, 
                        height = self.wall_height, 
                        width = self.wall_thickness, 
                        length = self.wall_thickness
                    )

        for x in range(self.grid_size - 1):
            for y in range(self.grid_size):
                if self.grid_matrix[x][y] == 0 and self.grid_matrix[x + 1][y] == 0:
                    self._create_wall( # Top piece
                        x = x * self.spacing - self.offset + self.spacing / 2.0, 
                        y = y * self.spacing - self.offset, 
                        height = self.wall_height, 
                        width = self.wall_thickness, 
                        length = self.spacing / 2.0 - self.wall_thickness
                    )
                
        for x in range(self.grid_size):
            for y in range(self.grid_size - 1):
                if self.grid_matrix[x][y] == 0 and self.grid_matrix[x][y + 1] == 0:
                    self._create_wall( # Right piece
                        x = x * self.spacing - self.offset, 
                        y = y * self.spacing - self.offset + self.spacing / 2.0, 
                        height = self.wall_height, 
                        width = self.spacing / 2.0 - self.wall_thickness, 
                        length = self.wall_thickness
                    )
                

    def _create_wall(self, x: float, y: float, height: float, width: float, length: float) -> None:
        wall = create_primitive.box(
            self.scene,
            sapien.Pose(p=[x, y, self.wall_height / 2]),
            half_size=[length, width, height],
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
        self.scene.add_ground(altitude=0, render_half_size=[(self.grid_size / 2.0)] * 2)
        self.create_wall_grid()
