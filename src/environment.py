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
        self.wall_length = wall_length
        self.offset = grid_size / 2.0
        self.walls = []
        self.generate_rdf = generate_rdf
        self.rdf_file = rdf_file

        # Initialize the RDF manager if RDF generation is enabled
        if self.generate_rdf:
            self.rdf_manager = EnvironmentRDFManager()
        else:
            self.rdf_manager = None

        # The matrix for the environment grid
        self.grid_matrix = [
            # Your grid matrix goes here
        ]

    def create_wall_grid(self) -> None:
        # Iterate through the grid to create walls based on the grid_matrix
        for x in range(self.grid_size):
            for y in range(self.grid_size):
                if self.grid_matrix[x][y] == 0:
                    # Center piece
                    self._create_wall(
                        x=x * self.spacing - self.offset, 
                        y=y * self.spacing - self.offset, 
                        height=self.wall_height, 
                        width=self.wall_thickness, 
                        length=self.wall_thickness
                    )

        # Horizontal walls (top piece)
        for x in range(self.grid_size - 1):
            for y in range(self.grid_size):
                if self.grid_matrix[x][y] == 0 and self.grid_matrix[x + 1][y] == 0:
                    self._create_wall(
                        x=x * self.spacing - self.offset + self.spacing / 2.0, 
                        y=y * self.spacing - self.offset, 
                        height=self.wall_height, 
                        width=self.wall_thickness, 
                        length=self.spacing / 2.0 - self.wall_thickness
                    )
                
        # Vertical walls (right piece)
        for x in range(self.grid_size):
            for y in range(self.grid_size - 1):
                if self.grid_matrix[x][y] == 0 and self.grid_matrix[x][y + 1] == 0:
                    self._create_wall(
                        x=x * self.spacing - self.offset, 
                        y=y * self.spacing - self.offset + self.spacing / 2.0, 
                        height=self.wall_height, 
                        width=self.spacing / 2.0 - self.wall_thickness, 
                        length=self.wall_thickness
                    )

    def _create_wall(self, x: float, y: float, height: float, width: float, length: float) -> None:
        # Use the create_primitive to generate the wall in the SAPIEN scene
        wall = create_primitive.box(width, height, length)
        wall.set_pose(sapien.Pose(p=[x, y, height / 2]))  # Adjust pose based on the height

        # Store the wall for future references
        self.walls.append(wall)

        # Add to RDF if enabled
        if self.generate_rdf:
            wall_id = f"Wall_{len(self.walls)}"
            self.rdf_manager.create_wall(
                wall_id, x, y, height / 2, 0, 0, 0,
                length, height, width
            )

    def load_scene(self) -> None:
        # Set up lighting and ground plane for the scene
        self.scene.set_ambient_light([0.5, 0.5, 0.5])
        self.scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])
        self.scene.add_ground(altitude=0, render_half_size=[(self.grid_size / 2.0)] * 2)

        # Create the grid of walls
        self.create_wall_grid()

        # Export the RDF file if RDF generation is enabled
        if self.generate_rdf:
            self.rdf_manager.export_rdf(self.rdf_file)
