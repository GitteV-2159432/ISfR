import sapien
from sapien import Pose

import numpy as np

from environment import Environment
import test_driver
import lidar
import slam.graph_slam as graph_slam

import visualization.test as test

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

    slam = graph_slam.GraphSlam()

    travel_distance = 0.0
    anderDing = np.eye(4)
    test.create(slam.graph)
    while not viewer.closed:
        lidar_sensor.simulate()
        driver.update()

        ding = driver.get_odometry_transformation_matrix(0.001, 0.01)
        anderDing = anderDing @ ding
        travel_distance += np.linalg.norm(graph_slam._matrix2translation(ding))
        if travel_distance > 0.1:
            slam.update(anderDing, lidar_sensor.get_point_cloud())
            test.update(slam.last_pose_vertex)
            travel_distance = 0
            anderDing = np.eye(4)

        scene.step()
        scene.update_render()
        viewer.render()

if __name__ == "__main__":
    main()
