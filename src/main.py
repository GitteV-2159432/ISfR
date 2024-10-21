import sapien
from sapien import Pose
import numpy as np

import test_environment
import test_driver
import lidar
from fastslam import FastSLAM, FastSLAM_config
from time import sleep

def main():
    scene = sapien.Scene()
    scene.set_timestep(1 / 100.0)

    viewer = scene.create_viewer()
    viewer.set_camera_xyz(x=-12, y=0, z=15)
    viewer.set_camera_rpy(r=0, p=-np.arctan2(2, 2), y=0)
    viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)

    test_environment.load(scene)
    driver = test_driver.driver(scene, viewer)

    lidar_config = lidar.LidarSensorConfig()
    lidar_config.detection_range = 2
    lidar_config.field_of_view = 10
    lidar_config.samples = 10
    lidar_config.noise_standard_deviation_distance = 0
    lidar_config.noise_standard_deviation_angle_horizontal = 0
    lidar_config.noise_standard_deviation_angle_vertical = 0
    lidar_config.noise_outlier_chance = 0
    lidar_config.randomize_start_angle = False
    lidar_sensor = lidar.LidarSensor("lidar", scene, lidar_config, mount_entity=driver.body, pose=Pose(p=np.array([0, 0, 0.5])))

    fastslam_config = FastSLAM_config()
    fastslam_config.particle_amount = 5
    fastslam_config.velocity_standard_deviation = 0
    fastslam_config.angular_velocity_standard_deviation = 0
    fastslam_config.distance_threshold = 3
    # fastslam_config.measurement_covariance = 0
    # fastslam_config.effective_particle_amount_modifier = 0

    fastslam = FastSLAM(fastslam_config)

    while not viewer.closed:
        lidar_sensor.simulate()
        driver.update()

        scene.step()
        scene.update_render()
        viewer.render()

        lidar_measurements = lidar_sensor.get_measurements()
 
        # Convert tuple format from LiDAR (horizontal_angle, vertical_angle, distance) -> (distance, angle) FastSLAM
        lidar_measurements = [(lp[2], lp[0]) for lp in lidar_measurements if lp[2] < lidar_config.detection_range]

        odometry = driver.get_odometry()

        fastslam.run(lidar_measurements, odometry[0], odometry[1])
        fastslam.visualize()
        # lidar_sensor.visualize()
        # sleep(0.1)

if __name__ == "__main__":
    main()
