import sapien
from sapien import Pose

import numpy as np
import cv2

import test_environment
import test_driver
import lidar
from infraCommunication import infraCommunication

def main():
    scene = sapien.Scene()
    scene.set_timestep(1 / 100.0)

    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5])

    viewer = scene.create_viewer()

    viewer.set_camera_xyz(x=-12, y=0, z=15)
    viewer.set_camera_rpy(r=0, p=-np.arctan2(2, 2), y=0)
    viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)

    test_environment.load(scene)
    driver = test_driver.driver(scene, viewer)

    lidar_config = lidar.LidarSensorConfig()
    lidar_config.detection_range = 10
    lidar_config.field_of_view = 360
    lidar_config.samples = 200
    lidar_config.noise_standard_deviation_distance = 0.05
    lidar_config.noise_standard_deviation_angle_horizontal = 0
    lidar_config.noise_standard_deviation_angle_vertical = 0
    lidar_config.noise_outlier_chance = 0
    lidar_config.randomize_start_angle = False
    
    infra_communication = infraCommunication("5cf46dc34dda4e04ab76ff9310e2804c.s2.eu.hivemq.cloud", 8883, "username", "password", "robotData")
    infra_communication.connect()
    infra_communication.set_lidar_context()
    
    lidar_sensor = lidar.LidarSensor("lidar", scene, lidar_config, infra_communication, mount_entity=driver.body, pose=Pose(p=np.array([0, 0, 0.5])))

    while not viewer.closed:
        lidar_sensor.simulate()
        driver.update()

        scene.step()
        scene.update_render()
        viewer.render()

        lidar_sensor.visualize()
    infra_communication.disconnect()

if __name__ == "__main__":
    main()
