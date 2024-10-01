import sapien
from sapien import Pose

import numpy as np
import cv2

import test_environment
import test_driver
import lidar

def points_to_image(points, img_size=(800, 800), scale=10):
    """Project 3D points to a 2D plane for visualization."""
    img = np.zeros(img_size + (3,), dtype=np.uint8)

    for point in points:
        x, y, z = point

        img_x = int(img_size[0] / 2 + y * scale)
        img_y = int(img_size[1] / 2 - x * scale)

        if 0 <= img_x < img_size[0] and 0 <= img_y < img_size[1]: 
            cv2.circle(img, (img_x, img_y), radius=2, color=(0, 255, 0), thickness=-1)
        else:
            print("scale to large")

    return img

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
    lidar_config.noise_standard_deviation_distance = 0.01
    lidar_config.noise_standard_deviation_angle_horizontal = 0.01
    lidar_config.noise_standard_deviation_angle_vertical = 0.001
    lidar_config.noise_outlier_chance = 0.0001
    lidar_config.randomize_start_angle = True
    lidar_sensor = lidar.LidarSensor("lidar", scene, lidar_config, mount_entity=driver.body, pose=Pose(p=np.array([0, 0, 0.5])))

    while not viewer.closed:
        lidar_sensor.simulate()
        driver.update()

        scene.step()
        scene.update_render()
        viewer.render()

        lidar_points = lidar_sensor.get_point_cloud()
        image = points_to_image(lidar_points, scale=30)

        cv2.imshow('LIDAR Points', image)
        cv2.waitKey(1)

if __name__ == "__main__":
    main()
