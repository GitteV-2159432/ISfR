from rdflib import Graph, Namespace, Literal
from rdflib.namespace import RDF, XSD

class RDFManager:
    def __init__(self, rdf_file):
        self.graph = Graph()
        try:
            self.graph.parse(rdf_file, format="turtle")
        except Exception as e:
            print(f"Error loading RDF file: {e}")
        
        self.namespace = Namespace("http://example.org/walls#")
        
    def get_all_walls(self):
        # SPARQL query to retrieve all walls with their properties
        query = f"""
        PREFIX ex: <{self.namespace}>
        SELECT ?wall ?x ?y ?height ?width ?length
        WHERE {{
            ?wall rdf:type ex:Wall ;
                  ex:x ?x ;
                  ex:y ?y ;
                  ex:height ?height ;
                  ex:width ?width ;
                  ex:length ?length .
        }}
        """
        results = self.graph.query(query)
        
        # Collect wall data into a list of dictionaries
        walls = []
        for row in results:
            wall_data = {
                "wall": str(row.wall),
                "x": float(row.x),
                "y": float(row.y),
                "height": float(row.height),
                "width": float(row.width),
                "length": float(row.length),
            }
            walls.append(wall_data)
        
        return walls
