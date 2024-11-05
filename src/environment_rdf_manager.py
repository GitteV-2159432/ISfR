from rdflib import Graph, Literal, Namespace, RDF
from rdflib.namespace import XSD

class EnvironmentRDFManager:
    def __init__(self):
        self.g = Graph()
        self.EX = Namespace("http://isfr.be/scene#")
        self.GEOM = Namespace("http://isfr.be/geometry#")
        self.g.bind("ex", self.EX)
        self.g.bind("geom", self.GEOM)

    # Create: Add new wall with a pose (without rotation)
    def create_wall(self, wall_id, x, y, z, length, height, thickness):
        wall = self.EX[wall_id]
        pose = self.EX[f"Pose_{wall_id}"]

        # Add wall and pose triples
        self.g.add((wall, RDF.type, self.GEOM.Wall))
        self.g.add((wall, self.GEOM.hasPose, pose))

        # Position
        self.g.add((pose, self.GEOM.positionX, Literal(x, datatype=XSD.float)))
        self.g.add((pose, self.GEOM.positionY, Literal(y, datatype=XSD.float)))
        self.g.add((pose, self.GEOM.positionZ, Literal(z, datatype=XSD.float)))

        # Wall dimensions
        self.g.add((wall, self.GEOM.length, Literal(length, datatype=XSD.float)))
        self.g.add((wall, self.GEOM.height, Literal(height, datatype=XSD.float)))
        self.g.add((wall, self.GEOM.thickness, Literal(thickness, datatype=XSD.float)))

    # Read: Query wall pose by wall ID (position only)
    def read_wall_pose(self, wall_id):
        query = f"""
        PREFIX geom: <http://isfr.be/geometry#>
        PREFIX ex: <http://isfr.be/scene#>

        SELECT ?positionX ?positionY ?positionZ
        WHERE {{
            ex:{wall_id} geom:hasPose ?pose .
            ?pose geom:positionX ?positionX ;
                  geom:positionY ?positionY ;
                  geom:positionZ ?positionZ .
        }}
        """
        result = self.g.query(query)
        for row in result:
            print(f"Wall Pose for {wall_id}: X={row.positionX}, Y={row.positionY}, Z={row.positionZ}")

    # Update: Update the position of a wall's pose (no rotation)
    def update_wall_pose(self, wall_id, new_x, new_y, new_z):
        pose = self.EX[f"Pose_{wall_id}"]

        # Remove old position data
        self.g.remove((pose, self.GEOM.positionX, None))
        self.g.remove((pose, self.GEOM.positionY, None))
        self.g.remove((pose, self.GEOM.positionZ, None))

        # Add new position data
        self.g.add((pose, self.GEOM.positionX, Literal(new_x, datatype=XSD.float)))
        self.g.add((pose, self.GEOM.positionY, Literal(new_y, datatype=XSD.float)))
        self.g.add((pose, self.GEOM.positionZ, Literal(new_z, datatype=XSD.float)))

    # Delete: Remove a wall and its pose
    def delete_wall(self, wall_id):
        wall = self.EX[wall_id]
        pose = self.EX[f"Pose_{wall_id}"]

        # Remove all triples associated with the wall and its pose
        self.g.remove((wall, None, None))
        self.g.remove((pose, None, None))

    # Export the updated RDF data to a file
    def export_rdf(self, file_name: str) -> None:
        with open(file_name, "w") as f:
            f.write(self.g.serialize(format="turtle"))
