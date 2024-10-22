import sapien
from sapien import Pose
import numpy as np
import cv2
import test_driver  # Assuming this contains the Driver class and related code
import paho.mqtt.client as mqtt
import lidar  # Assuming the LidarSensor and LidarSensorConfig are in this module

BROKER = "localhost"  
PORT = 1883
VELOCITY_TOPIC = "robot/desired_velocity"  

# MQTT client setup
mqtt_client = mqtt.Client()

def on_connect(client, userdata, flags, rc):
    print(f"Connected to MQTT broker with result code: {rc}")

mqtt_client.on_connect = on_connect
mqtt_client.connect(BROKER, PORT, 60)
mqtt_client.loop_start()

def main():
    # Scene setup
    scene = sapien.Scene()
    scene.set_timestep(0.1)
    scene.set_ambient_light([0.5, 0.5, 0.5])
    scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], shadow=True)

    # Viewer setup
    viewer = scene.create_viewer()
    ground_material = scene.create_physical_material(0.2, 0.2, 0)
    ground = scene.add_ground(0, True, ground_material)

    # Camera settings
    viewer.set_camera_xyz(x=-10, y=0, z=10)
    viewer.set_camera_rpy(r=0, p=-np.arctan2(2, 2), y=0)
    viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)

    # Define goal position for the robot
    goal_position = np.array([5.0, 5.0, 0.0])  # Example goal at (x=5, y=5, z=0)

    # Initialize driver (robot) with the goal position
    driver = test_driver.Driver(scene, viewer, goal_position)  

    # Lidar sensor setup
    lidar_config = lidar.LidarSensorConfig()
    lidar_config.detection_range = 10
    lidar_config.field_of_view = 360
    lidar_config.samples = 200
    lidar_config.noise_standard_deviation_distance = 0.01
    lidar_config.noise_standard_deviation_angle_horizontal = 0.01
    lidar_config.noise_standard_deviation_angle_vertical = 0.001
    lidar_config.noise_outlier_chance = 0.0001
    lidar_config.randomize_start_angle = True

    # Attach lidar to the robot
    lidar_sensor = lidar.LidarSensor("lidar", scene, lidar_config, mount_entity=driver.robot, pose=Pose(p=np.array([0, 0, 0.5])))

    # Simulation loop
    while not viewer.closed:
        # Simulate lidar and retrieve the point cloud data
        lidar_sensor.simulate()
        lidar_points = lidar_sensor.get_point_cloud()
        
        # Step the scene and render
        scene.step()
        scene.update_render()
        viewer.render()

        # Update the driver with lidar points for obstacle avoidance or navigation
        driver.update(lidar_points)

if __name__ == "__main__":
    main()

mqtt_client.loop_stop()
