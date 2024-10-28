from time import sleep
import sapien
from sapien import Pose

import numpy as np
from scipy.spatial.transform import Rotation

from environment import Environment
import test_driver
from fastslam import FastSLAM, FastSLAM_config
import lidar

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

    fastslam_config = FastSLAM_config()
    fastslam = FastSLAM(fastslam_config)

    while not viewer.closed:
        scene.step()
        scene.update_render()
        viewer.render()
        
        driver.update()
        lidar_sensor.simulate()

        # Convert tuple format from LiDAR (horizontal_angle, vertical_angle, distance) -> (distance, angle) for FastSLAM
        lidar_measurements = [(lp[2], lp[0]) for lp in lidar_sensor.get_measurements(in_degrees=False) if lp[2] < lidar_config.detection_range_max]
        fastslam.run(lidar_measurements, driver.get_odometry()[0], driver.get_odometry()[1])

        # For testing
        driver_ground_truth = (driver.body.get_pose().p[0], driver.body.get_pose().get_p()[1], Rotation.from_quat(driver.body.get_pose().q, scalar_first=True).as_euler('xyz', degrees=False)[2])
        fastslam.visualize(driver_ground_truth)
        lidar_sensor.visualize()

if __name__ == "__main__":
    main()
