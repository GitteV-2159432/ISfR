import torch
import cv2
import warnings
from rdflib import Graph, Literal, RDF, URIRef, Namespace
import statistics

def frame_to_rdf(frame, depth_graph):
    # Load YOLOv5 model
    model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Perform detection
    results = model(rgb_frame)

    # Create RDF graph
    g = Graph()

    # Define namespaces
    NS = Namespace("http://example.org/ontology/")
    DETECTION = Namespace("http://example.org/detection/")

    # Bind namespaces to the graph
    g.bind("ns", NS)
    g.bind("detection", DETECTION)

    # Process results
    detections = results.pandas().xyxy[0]

    for i, row in detections.iterrows():
        x1, y1, x2, y2, confidence, class_id, label = (
            int(row["xmin"]),
            int(row["ymin"]),
            int(row["xmax"]),
            int(row["ymax"]),
            row["confidence"],
            int(row["class"]),
            row["name"],
        )

        # Draw bounding box
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2)

        # Add label and confidence
        label_text = f"{label}: {confidence:.2f}"
        cv2.putText(
            frame, label_text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2
        )

        # Estimate the depth value
        z_value = depth_graph[y1:y2, x1:x2].mean()

        # Create RDF entity for the detected object
        entity_uri = DETECTION[f"object_{i}"]
        g.add((entity_uri, RDF.type, NS.Object))
        g.add((entity_uri, NS.label, Literal(label)))
        g.add((entity_uri, NS.confidence, Literal(confidence)))
        g.add((entity_uri, NS.coordinates, Literal(f"x1:{x1},y1:{y1},x2:{x2},y2:{y2}")))
        g.add((entity_uri, NS.depth, Literal(z_value)))

    # Serialize RDF graph to a file
    g.serialize("detections.rdf", format="turtle")
    return frame
