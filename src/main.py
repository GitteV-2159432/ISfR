import sapien
from sapien import Pose

import numpy as np

from environment import Environment
import test_driver
import lidar

from slam.graph_slam import GraphSlam
from visualization.slam_visualization import SlamPlot, OccupancyGrid

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

    slam = GraphSlam()
    slam_plot = SlamPlot(slam)

    grid = OccupancyGrid(grid_size=400, cell_size=0.1)
    grid.plotter.show(interactive_update=True)

    while not viewer.closed:
        lidar_sensor.simulate()
        driver.update()

        odometry = driver.get_odometry_transformation_matrix(0.001, 0.01)
        slam.update(odometry, lidar_sensor.get_point_cloud())

        if slam.last_pose_vertex is not None:
            pose = slam.last_pose_vertex.get_position()
            point_cloud = lidar_sensor.get_point_cloud()
            grid.update_grid(pose, point_cloud)
            grid.render()

        scene.step()
        scene.update_render()
        viewer.render()

if __name__ == "__main__":
    main()
