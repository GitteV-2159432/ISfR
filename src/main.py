import sapien.core as sapien
from sapien import Pose
import numpy as np

from environment import Environment
from rdf_manager import RDFManager
import test_driver
import lidar

from slam.graph_slam import GraphSlam, RoomManager
from visualization.slam_visualization import SlamPlot

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

    lidar_config = lidar.LidarSensorConfig('src/sensor_configs/Default.json')
    lidar_sensor = lidar.LidarSensor("lidar", scene, lidar_config, mount_entity=driver.body, pose=Pose(p=np.array([0, 0, 0.5])))

    slam = GraphSlam()
    slam_plot = SlamPlot(slam)

    room_manager = RoomManager(slam)

    while not viewer.closed:
        lidar_sensor.simulate()
        driver.update()

        odometry = driver.get_odometry_transformation_matrix(0.001, 0.01)
        slam.update(odometry, lidar_sensor.get_point_cloud())

        # Detect rooms
        current_room_id = room_manager.detect_new_room(odometry)
        #print(f"Current Room ID: {current_room_id}")

        scene.step()
        scene.update_render()
        viewer.render()

if __name__ == "__main__":
    main()
