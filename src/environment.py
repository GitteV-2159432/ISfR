import sapien.core as sapien
import create_primitive
import json
from rdf_manager import RDFManager

class Environment:
    def __init__(self, scene: sapien.Scene, wall_data: list):
        self.scene = scene
        self.walls = []
        self.wall_data = wall_data  # JSON-style list of dictionaries with wall information

    def create_wall_grid(self) -> None:
        for wall_info in self.wall_data:
            self._create_wall(
                x=wall_info['x'],
                y=wall_info['y'],
                height=wall_info['height'],
                width=wall_info['width'],
                length=wall_info['length']
            )

    def _create_wall(self, x: float, y: float, height: float, width: float, length: float) -> None:
        wall = create_primitive.box(
            self.scene,
            sapien.Pose(p=[x, y, height / 2]),
            half_size=[length, width, height],
            color=[0.7, 0.7, 0.7],
            name="wall",
            is_kinematic=True,
        )
        self.walls.append(wall)

    def load_scene(self) -> None:
        self.scene.set_ambient_light([0.5, 0.5, 0.5])
        self.scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])
        self.scene.add_ground(altitude=0, render_half_size=[20, 20])  # Example size
        self.create_wall_grid()
