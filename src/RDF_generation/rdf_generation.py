from camera_input import capture_camera_frame
from yolo_processing import frame_to_rdf
import cv2
import torch

# Start capturing frames from the camera
for frame in capture_camera_frame():
    # Process each frame with YOLO
    processed_frame = frame_to_rdf(frame)

    # Display the processed frame
    cv2.imshow("Live Detection", processed_frame)
