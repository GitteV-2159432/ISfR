import sapien.core as sapien
from sapien import Pose
import numpy as np
import cv2

from environment import Environment
from rdf_manager import RDFManager
import test_driver
import lidar
from slam.graph_slam import GraphSlam, _matrix2rotation
from slam.orb_slam import OrbSlam

from slam.graph_slam import GraphSlam
from visualization.slam_visualization import SlamPlot
from RDF_generation.yolo_processing import frame_to_rdf

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

    graph_slam = GraphSlam()
    slam_plot = SlamPlot(graph_slam)
    orb_slam = OrbSlam()

    while not viewer.closed:

        lidar_sensor.simulate()
        driver.update()

        # TODO split odometry and points in 2 different functions
        camera_sensor.take_picture()
        image = np.array(camera_sensor.get_picture("Color"))[:, :, :3]

        cv2.waitKey(1)
        processed_frame = frame_to_rdf(image)
        cv2.imshow("Live Detection", processed_frame)
        # orb_slam.run(image)
        odometry = driver.get_odometry_transformation_matrix(0.001, 0.01)
        # ! TEMP TEST CODE

        # anderDing = anderDing @ odemetry_transformation_matrix
        # travel_distance += np.linalg.norm(graph_slam._matrix2translation(odometry_transformation_matrix))
        # if travel_distance > 0.1:
        #     graph_slam.update(anderDing, points_3d)
        #     test.update(graph_slam.last_pose_vertex)
        #     travel_distance = 0
        #     anderDing = np.eye(4)

        # if int(viewer.window.key_down('r')) == 0 and lock: lock = False
        # if int(viewer.window.key_down('r')) == 1 and not lock:
        #     lock = True
        #     graph_slam.graph.optimize()
        #     test.recreate_all(graph_slam.graph)
        # ! TEMP TEST CODE END

        scene.step()
        scene.update_render()
        viewer.render()

if __name__ == "__main__":
    main()
