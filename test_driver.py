from sapien.core import Scene, Pose
from sapien.utils import Viewer
from simple_pid import PID
import numpy as np
import paho.mqtt.client as mqtt
from scipy.spatial.transform import Rotation
import DWA
import Config

# Assume DWA and Config classes are imported

class Driver:
    def __init__(self, scene: Scene, viewer: Viewer, goal_position: np.ndarray) -> None:
        self._viewer = viewer
        self.move_speed = 1.4
        self.wheel_radius = 0.1
        self.wheel_base = 0.4
        self.target_velocity = 0
        self.max_velocity = 5
        self.is_reset = False  
        self.is_started = False  

        # Initialize robot
        loader = scene.create_urdf_loader()
        loader.fix_root_link = False
        self.robot = loader.load("robot-description/urdf/rikkert.urdf")
        self.robot.set_root_pose(Pose([-6, 0, 0], [1, 0, 0, 0]))
        loader.set_link_material("rear_caster", 0.00001, 0, 0)

        # Get joints
        self.joints = self.get_joints_dict(self.robot)

        # Initialize PID controllers
        self.pid_left = PID(0, 0, 0, setpoint=self.move_speed)
        self.pid_right = PID(0, 0, 0, setpoint=self.move_speed)

        # Initialize DWA with goal position and config
        self.config = Config.Config()  # Fully qualified reference
        self.x = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # Initial state
        self.dwa = DWA.DWA(config=self.config, odom=self.x, goal_pose=Pose(p=goal_position))

        self.setup_mqtt()

    def setup_mqtt(self):
        self.client = mqtt.Client("robot_driver")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        try:
            self.client.connect("localhost", 1883, 60)
        except Exception as e:
            print("Failed to connect to MQTT broker:", e)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT broker")
            self.client.subscribe("robot/pid")
            self.client.subscribe("robot/reset")
            self.client.subscribe("robot/start")
        else:
            print("Failed to connect to MQTT broker, return code:", rc)

    def on_message(self, client, userdata, msg):
        if msg.topic == "robot/pid":
            message = msg.payload.decode()
            try:
                kp, ki, kd = map(float, message.split(","))
                self.pid_left.Kp = kp
                self.pid_left.Ki = ki
                self.pid_left.Kd = kd
                self.pid_right.Kp = kp
                self.pid_right.Ki = ki
                self.pid_right.Kd = kd
                print(f"Updated PID: Kp={round(kp, 2)}, Ki={round(ki, 2)}, Kd={round(kd, 2)}")
            except ValueError as e:
                print(f"Error parsing PID message: {e}")
        elif msg.topic == "robot/reset":
            self.reset_robot()
        elif msg.topic == "robot/start":
            self.is_reset = False  
            self.is_started = True  
            print("Robot started")

    def reset_robot(self):
        self.robot.set_root_pose(Pose([-6, 0, 0], [1, 0, 0, 0]))
        self.target_velocity = 0
        self.joints['left_wheel_joint'].set_drive_velocity_target(0)
        self.joints['right_wheel_joint'].set_drive_velocity_target(0)
        self.pid_left.auto_mode = False  
        self.pid_right.auto_mode = False  
        self.pid_left.set_auto_mode(True, last_output=0)
        self.pid_right.set_auto_mode(True, last_output=0)
        self.is_reset = True
        self.is_started = False  
        print("Robot and PID controllers reset")

    def update(self, lidar_points) -> None:
        if self.is_reset:
            self.joints['left_wheel_joint'].set_drive_velocity_target(0)
            self.joints['right_wheel_joint'].set_drive_velocity_target(0)
            print("Robot is reset and stationary")
        elif self.is_started:
            # Update obstacles for DWA
            filtered_points = np.array([point for point in lidar_points if abs(point[0]) <= 7 and abs(point[1]) <= 7])
            self.dwa.update_obstacles(filtered_points)
            
            # DWA control to get optimal velocities
            control_commands, trajectory = self.dwa.dwa_control()

            x, y, theta, _, _ = self.dwa.x
            self.dwa.render(trajectory)
            # Convert DWA velocities to wheel velocities
            left_velocity_target, right_velocity_target = self.convert_to_wheel_velocities(control_commands[0], control_commands[1])

            print(f"Updating velocities: left={left_velocity_target}, right={right_velocity_target}")
            self.set_wheel_velocities(left_velocity_target, right_velocity_target)

    def convert_to_wheel_velocities(self, linear_velocity: float, angular_velocity: float):
        left_wheel_velocity = linear_velocity - (angular_velocity * self.wheel_base / 2)
        right_wheel_velocity = linear_velocity + (angular_velocity * self.wheel_base / 2)
        return left_wheel_velocity, right_wheel_velocity

    def set_wheel_velocities(self, left_velocity_target: float, right_velocity_target: float) -> None:
        current_left_velocity = self.calculate_total_velocity("left_wheel")
        current_right_velocity = self.calculate_total_velocity("right_wheel")

        # Control signals from PID
        control_left = self.pid_left(current_left_velocity)
        control_right = self.pid_right(current_right_velocity)

        self.joints['left_wheel_joint'].set_drive_velocity_target(control_left)
        self.joints['right_wheel_joint'].set_drive_velocity_target(control_right)

        self.client.publish("robot/wheel_velocities", f"left:{current_left_velocity}, right:{current_right_velocity}")

    def calculate_total_velocity(self, link_name: str) -> float:
        velocity_vector = self.robot.find_link_by_name(link_name).get_linear_velocity()
        v_x, v_y, v_z = velocity_vector
        total_velocity = np.sqrt(v_x**2 + v_y**2 + v_z**2)
        return round(total_velocity, 2)

    def get_joints_dict(self, articulation):
        joints = articulation.get_joints()
        return {joint.get_name(): joint for joint in joints}
