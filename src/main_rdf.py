import sapien.core as sapien
from sapien import Pose
import numpy as np
import cv2

from environment import Environment
from rdf_manager import RDFManager
import test_driver
import lidar

def main():
    # Load wall data using RDFManager
    rdf_file = "environment.ttl"  # Specify your RDF file path
    rdf_manager = RDFManager(rdf_file)
    wall_data = rdf_manager.get_all_walls()

    # Initialize SAPIEN scene
    scene = sapien.Scene()
    scene.set_timestep(1 / 100.0)

    # Set up viewer
    viewer = scene.create_viewer()
    viewer.set_camera_xyz(x=-12, y=0, z=15)
    viewer.set_camera_rpy(r=0, p=-np.arctan2(2, 2), y=0)
    viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)

    # Initialize Environment with wall_data from RDFManager
    environment = Environment(scene, wall_data)
    environment.load_scene()

    # Set up driver (assuming `test_driver.driver` is a valid function returning a driver object)
    driver = test_driver.driver(scene, viewer)

    # Configure Lidar
    lidar_config = lidar.LidarSensorConfig()
    lidar_config.detection_range = 10
    lidar_config.field_of_view = 360
    lidar_config.samples = 200
    lidar_config.noise_standard_deviation_distance = 0.05
    lidar_config.noise_standard_deviation_angle_horizontal = 0
    lidar_config.noise_standard_deviation_angle_vertical = 0
    lidar_config.noise_outlier_chance = 0
    lidar_config.randomize_start_angle = False

    # Initialize lidar sensor
    lidar_sensor = lidar.LidarSensor(
        "lidar",
        scene,
        lidar_config,
        mount_entity=driver.body,
        pose=Pose(p=np.array([0, 0, 0.5]))
    )

    # Main simulation loop
    while not viewer.closed:
        lidar_sensor.simulate()
        driver.update()

        scene.step()
        scene.update_render()
        viewer.render()

        lidar_sensor.visualize()

if __name__ == "__main__":
    main()