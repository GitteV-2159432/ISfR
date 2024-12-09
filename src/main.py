import sapien
from sapien import Pose

import numpy as np
import cv2 as cv

from environment import Environment
import test_driver
import lidar
from slam.graph_slam import GraphSlam, _matrix2translation
from slam.orb_slam_v2 import OrbSlam

from slam.graph_slam import GraphSlam
from visualization.slam_visualization import SlamPlot

def main():
    scene = sapien.Scene()
    scene.set_timestep(1 / 100.0)

    viewer = scene.create_viewer()
    viewer.set_camera_xyz(x=-12, y=0, z=15)
    viewer.set_camera_rpy(r=0, p=-np.arctan2(2, 2), y=0)
    viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)

    environment = Environment(scene, grid_size=20, spacing=1, wall_height=2.0, wall_thickness=0.2)
    environment.load_scene()

    driver = test_driver.driver(scene, viewer)

    lidar_config = lidar.LidarSensorConfig('src/sensor_configs/Default.json')
    lidar_sensor = lidar.LidarSensor("lidar", scene, lidar_config, mount_entity=driver.body, pose=Pose(p=np.array([0, 0, 0.5])))

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

    orb_slam = OrbSlam(camera_matrix=camera_sensor.get_intrinsic_matrix())
    slam = GraphSlam()
    slam_plot = SlamPlot(slam)

    while not viewer.closed:
        scene.step()
        scene.update_render()
        viewer.render()

        lidar_sensor.simulate()
        driver.update()

        robot_odometry = driver.get_odometry_transformation_matrix()

        camera_sensor.take_picture()
        image = (camera_sensor.get_picture('Color') * 255).clip(0, 255).astype("uint8")

        odometry, points_3d = orb_slam.run(image, robot_odometry)

        print("points_3d =", points_3d)
        if len(points_3d) == 0:
            continue
        
        # odometry = driver.get_odometry_transformation_matrix()
        slam.update(robot_odometry, points_3d)

        # print("points_3d:", points_3d)
        # print("lidar:", lidar_sensor.get_point_cloud())

if __name__ == "__main__":
    main()
