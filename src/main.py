import sapien.core as sapien
from sapien import Pose
import numpy as np
import cv2

import communication.infraCommunication
from environment import Environment
import test_driver
import lidar
from communication import infraCommunication
# from rdf_manager import RDFManager
# from RDF_generation.yolo_processing import frame_to_rdf
from slam.graph_slam import GraphSlam
from visualization.slam_visualization import SlamPlot
from visualization.polar_coordinates_plot import PolarCoordinatesPlot

def main():
    # Initialize SAPIEN scene
    scene = sapien.Scene()
    scene.set_timestep(1 / 100.0)

    # Set up viewer
    viewer = scene.create_viewer()
    viewer.set_camera_xyz(x=-12, y=0, z=15)
    viewer.set_camera_rpy(r=0, p=-np.arctan2(2, 2), y=0)
    viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)

    environment = Environment(scene, grid_size=20, spacing=1, wall_height=2.0, wall_thickness=0.2)
    environment.load_scene()

    # Set up driver (assuming `test_driver.driver` is a valid function returning a driver object)
    driver = test_driver.driver(scene, viewer)

    lidar_config = lidar.LidarSensorConfig('src/sensor_configs/Default.json')
    lidar_sensor = lidar.LidarSensor("lidar", scene, lidar_config, mount_entity=driver.body, pose=Pose(p=np.array([0, 0, 0.5])))

    #set communication
    infraComms = infraCommunication.infraCommunication("broker", 8883, "username", "password", "topic")
    infraComms.connect()
    infraComms.subscribe("robotData")
    infraComms.subscribe("testje")
    camera_sensor = scene.add_mounted_camera(
        name= "Camera",
        mount=driver.body,
        pose=Pose(p=np.array([0, 0, .75])),
        width = 640,
        height = 480,
        fovy = 1,
        near = 0.5,
        far = 100
    )

    slam = GraphSlam()
    SlamPlot(slam)
    
    while not viewer.closed:
        lidar_sensor.simulate()
        driver.update()
        #infraComms.publish_lidar_points(infraComms.topic,lidar_sensor.get_point_cloud())
        lidar_data = infraComms.get_lidar_data()
        lidar_pts = infraComms.decode_data(lidar_data)
        print(lidar_pts)
        infraComms.publish_lidar_points("testje",lidar_pts)
        
        camera_sensor.take_picture()
        image = (camera_sensor.get_picture('Color') * 255).clip(0, 255).astype(np.uint8)
        depthmap_generator = lidar.GenerateDepthmap(image.shape[:2], np.eye(3))
        #depthmap = depthmap_generator.create_depthmap(lidar_sensor.get_point_cloud())
        #print(depthmap)

        # cv2.waitKey(1)
        # processed_frame = frame_to_rdf(image)
        # cv2.imshow("Live Detection", processed_frame)

        odometry = driver.get_odometry_transformation_matrix(0.001, 0.001)
        #dit moet ontvangen kunnen worden, data komt van robot
        slam.update(odometry, lidar_pts)

        scene.step()
        scene.update_render()
        viewer.render()
        
if __name__ == "__main__":
    main()
