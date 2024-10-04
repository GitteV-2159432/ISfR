import sapien
from sapien import Pose
import numpy as np
from datetime import datetime

import test_environment
import test_driver
import lidar
from fastslam import Fastslam, Fastslam_config

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
    lidar_config.detection_range = 10
    lidar_config.field_of_view = 360
    lidar_config.samples = 200
    lidar_config.noise_standard_deviation_distance = 0
    lidar_config.noise_standard_deviation_angle_horizontal = 0
    lidar_config.noise_standard_deviation_angle_vertical = 0 
    lidar_config.noise_outlier_chance = 0
    lidar_config.randomize_start_angle = False
    lidar_sensor = lidar.LidarSensor("lidar", scene, lidar_config, mount_entity=driver.body, pose=Pose(p=np.array([0, 0, 0.5])))

    fastslam_config = Fastslam_config()
    fastslam_config.particle_amount = 0
    fastslam_config.velocity_standard_deviation = 0
    fastslam_config.angular_velocity_standard_deviation = 0
    fastslam_config.mahalanobis_distance_threshold = 0
    fastslam_config.measurement_covariance = 0
    fastslam_config.effective_particle_amount_modifier = 0

    fastslam = Fastslam(fastslam_config)

    last_time = datetime.now()
    while not viewer.closed:
        delta_time = datetime.now() - last_time
        last_time = datetime.now()

        lidar_sensor.simulate()
        driver.update()

        scene.step()
        scene.update_render()
        viewer.render()

        lidar_points = lidar_sensor.get_points()
        odometry = driver.get_odometry()

        fastslam.run(lidar_points, odometry[0], odometry[1], delta_time.microseconds * float(1e6))

if __name__ == "__main__":
    main()
