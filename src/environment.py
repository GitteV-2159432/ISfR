import sapien.core as sapien
import create_primitive
from rdflib.namespace import XSD
from scipy.spatial.transform import Rotation

from environment_rdf_manager import EnvironmentRDFManager

class Environment:
    def __init__(self, scene: sapien.Scene, grid_size: int = 20, spacing: float = 3.0, 
                 wall_height: float = 2.0, wall_thickness: float = 0.2, wall_length: float = 3.0,
                 generate_rdf: bool = True, rdf_file: str = "environment.ttl"):
        self.scene = scene
        self.grid_size = grid_size
        self.spacing = spacing
        self.wall_height = wall_height
        self.wall_thickness = wall_thickness
        self.offset = grid_size / 2.0
        self.walls = []
        self.generate_rdf = generate_rdf
        self.rdf_file = rdf_file

        # Initialize the RDF manager if RDF generation is enabled
        if self.generate_rdf:
            self.rdf_manager = EnvironmentRDFManager()
        else:
            self.rdf_manager = None

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
        # Create the wall in the SAPIEN scene
        wall = create_primitive.box(
            self.scene,
            sapien.Pose(p=[x, y, self.wall_height / 2]),
            half_size=[length, width, height],
            color=[0.7, 0.7, 0.7],
            name="wall",
            is_kinematic=True,
        )
        self.walls.append(wall)

        # Add to RDF if enabled
        if self.generate_rdf:
            wall_id = f"Wall_{len(self.walls)}"
            self.rdf_manager.create_wall(
                wall_id, x, y, self.wall_height / 2, rotation_x, rotation_y, rotation_z,
                self.wall_length, self.wall_height, self.wall_thickness
            )

    def _remove_wall(self, x: float, y: float) -> None:
        for wall in self.walls:
            if abs(wall.pose.p[0] - x) < 0.1 and abs(wall.pose.p[1] - y) < 0.1:
                self.scene.remove_actor(wall)
                self.walls.remove(wall)
                
                # Remove the corresponding wall from RDF if RDF generation is enabled
                if self.generate_rdf:
                    wall_id = f"Wall_{len(self.walls) + 1}"
                    self.rdf_manager.delete_wall(wall_id)
                break

    def load_scene(self) -> None:
        self.scene.set_ambient_light([0.5, 0.5, 0.5])
        self.scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])
        self.scene.add_ground(altitude=0, render_half_size=[(self.grid_size / 2.0)] * 2)
        self.create_wall_grid()

        if self.generate_rdf:
            self.rdf_manager.export_rdf(self.rdf_file)

