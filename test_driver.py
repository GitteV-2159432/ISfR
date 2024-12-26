from sapien.core import Scene, Pose
from sapien.utils import Viewer
from simple_pid import PID
import numpy as np
import paho.mqtt.client as mqtt
from scipy.spatial.transform import Rotation
import DWA
import Config
import threading
import time

class Driver:
    def __init__(self, scene: Scene, viewer: Viewer, goal_position: np.ndarray, robot_model: str = "rikkert") -> None:
        """
        Initializes the Driver class for controlling the robot.

        :param scene: SAPIEN simulation scene.
        :param viewer: Viewer for visualization.
        :param goal_position: Goal position for the robot.
        :param robot_model: Robot model to use ("rikkert" or "dif_robot").
        """
        self._viewer = viewer
        self.move_speed = 1.4  # Default target speed for the robot.
        self.wheel_radius = 0.1  # Radius of the robot's wheels.
        self.wheel_base = 0.45  # Distance between the wheels.
        self.target_velocity = 0
        self.max_velocity = 5 if robot_model == "dif_robot" else 0.8  # Max velocity based on robot model.
        self.is_reset = False  # Indicates if the robot is reset.
        self.is_started = True  # Indicates if the robot is in motion.
        self.goal_pose = goal_position  # Target position for the robot.

        # Load the appropriate robot model.
        loader = scene.create_urdf_loader()
        loader.fix_root_link = False
        if robot_model == "rikkert":
            self.robot = loader.load("robot-description/urdf/rikkert.urdf")
        elif robot_model == "dif_robot":
            self.robot = loader.load("dif_robot.urdf")
            self.set_dif_robot_materials(scene)
        else:
            raise ValueError("Invalid robot model specified. Choose either 'rikkert' or 'dif_robot'.")

        # Set the robot's initial position.
        self.robot.set_root_pose(Pose([0, 0, 0], [1, 0, 0, 0]))

        # Get the robot's joints.
        self.joints = self.get_joints_dict(self.robot)

        # Initialize PID controllers for the left and right wheels.
        self.pid_left = PID(0, 0, 0, setpoint=self.move_speed)
        self.pid_right = PID(0, 0, 0, setpoint=self.move_speed)

        # Initialize DWA configuration and state.
        self.config = Config.Config()
        self.x = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # Robot state: [x, y, theta, v, omega].
        self.dwa = DWA.DWA(config=self.config, odom=self.x, goal_pose=Pose(p=goal_position))

        # Initialize threading for DWA control.
        self.lock = threading.Lock()
        self.control_commands = None
        self.trajectory = None
        self.lidar_points = None
        self.thread = threading.Thread(target=self.run_dwa_control)
        self.thread.start()

        # Set physical properties for "rikkert" robot.
        if robot_model == "rikkert":
            self.set_physical_properties_rikkert(scene)

        # Setup MQTT communication.
        self.setup_mqtt()

    def set_physical_properties_rikkert(self, scene):
        """
        Sets physical properties for the "rikkert" robot.
        """
        custom_material = scene.create_physical_material(
            static_friction=0.1, dynamic_friction=0.1, restitution=0
        )
        for link in self.robot.get_links():
            for shape in link.get_collision_shapes():
                shape.set_physical_material(custom_material)
        self.set_drive_properties_rikkert()

    def set_drive_properties_rikkert(self):
        """
        Sets drive properties for the "rikkert" robot joints.
        """
        left_joint = self.joints["left_wheel_joint"]
        right_joint = self.joints["right_wheel_joint"]
        caster = self.joints["caster_rolling_joint"]
        caster_zwivel = self.joints["caster_swivel_joint"]

        left_joint.set_drive_properties(stiffness=10, damping=20, force_limit=40.0)
        right_joint.set_drive_properties(stiffness=10, damping=20, force_limit=40.0)
        caster.set_drive_properties(stiffness=10, damping=10, force_limit=40.0)
        caster_zwivel.set_drive_properties(stiffness=10, damping=30, force_limit=100.0)

        left_joint.set_friction(0.1)
        right_joint.set_friction(0.1)

    def set_dif_robot_materials(self, scene):
        """
        Sets materials for the "dif_robot" robot.
        """
        custom_material = scene.create_physical_material(
            static_friction=0, dynamic_friction=0, restitution=0.1
        )
        for link_name in ["caster_wheel", "left_wheel", "right_wheel"]:
            link = self.robot.find_link_by_name(link_name)
            if link:
                for shape in link.get_collision_shapes():
                    shape.set_physical_material(custom_material)

    def setup_mqtt(self):
        """
        Sets up MQTT communication for the robot.
        """
        self.client = mqtt.Client("robot_driver")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        try:
            self.client.connect("localhost", 1883, 60)
        except Exception as e:
            print("Failed to connect to MQTT broker:", e)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        """
        Callback when connected to MQTT broker.
        """
        if rc == 0:
            self.client.subscribe("robot/pid")
            self.client.subscribe("robot/reset")
            self.client.subscribe("robot/start")
        else:
            print("Failed to connect to MQTT broker, return code:", rc)

    def on_message(self, client, userdata, msg):
        """
        Callback when a message is received via MQTT.
        """
        if msg.topic == "robot/pid":
            self.update_pid(msg.payload.decode())
        elif msg.topic == "robot/reset":
            self.reset_robot()
        elif msg.topic == "robot/start":
            self.is_reset = False
            self.is_started = True

    def update_pid(self, message):
        """
        Updates PID parameters from an MQTT message.
        """
        try:
            kp, ki, kd = map(float, message.split(","))
            self.pid_left.Kp = kp
            self.pid_left.Ki = ki
            self.pid_left.Kd = kd
            self.pid_right.Kp = kp
            self.pid_right.Ki = ki
            self.pid_right.Kd = kd
        except ValueError as e:
            print(f"Error parsing PID message: {e}")

    def reset_robot(self):
        """
        Resets the robot state, position, and PID controllers.
        """
        self.robot.set_root_pose(Pose([0, 0, 0], [1, 0, 0, 0]))
        self.target_velocity = 0
        self.control_commands = None
        self.trajectory = None
        self.lidar_points = None
        for pid in [self.pid_left, self.pid_right]:
            pid.set_auto_mode(False)
            pid.set_auto_mode(True, last_output=0)
        self.x = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.dwa.x = np.copy(self.x)
        self.is_reset = True
        self.is_started = False

    def update(self, lidar_points):
        """
        Updates the robot state and sets wheel velocities based on DWA control.
        """
        if self.is_reset:
            self.joints["left_wheel_joint"].set_drive_velocity_target(0)
            self.joints["right_wheel_joint"].set_drive_velocity_target(0)
        elif self.is_started and not self.has_reached_goal():
            self.lidar_points = lidar_points
            if self.trajectory is not None and self.trajectory.size > 0:
                self.dwa.render(self.trajectory)
            if self.control_commands:
                left_target, right_target = self.convert_to_wheel_velocities(
                    self.control_commands[0], self.control_commands[1]
                )
                self.set_wheel_velocities(left_target, right_target)

    def has_reached_goal(self, threshold=0.5):
        """
        Checks if the robot has reached the goal position.
        """
        dx = self.goal_pose[0] - self.x[0]
        dy = self.goal_pose[1] - self.x[1]
        return np.hypot(dx, dy) <= threshold

    def run_dwa_control(self):
        """
        Continuously runs the DWA control loop.
        """
        while True:
            with self.lock:
                if self.is_started and not self.has_reached_goal():
                    self.update_robot_state()
                    if self.lidar_points is not None:
                        filtered_points = np.array(
                            [point for point in self.lidar_points if abs(point[0]) <= 7 and abs(point[1]) <= 7]
                        )
                        self.dwa.update_obstacles(filtered_points)
                    self.control_commands, self.trajectory = self.dwa.dwa_control()

    def update_robot_state(self):
        """
        Updates the robot's state for DWA.
        """
        robot_pose = self.robot.get_root_pose()
        robot_position = robot_pose.p
        robot_orientation = Rotation.from_quat(robot_pose.q, scalar_first=True).as_euler("xyz")[2]
        self.dwa.x[:3] = robot_position[0], robot_position[1], robot_orientation

    def convert_to_wheel_velocities(self, linear_velocity, angular_velocity):
        """
        Converts linear and angular velocities to wheel velocities.
        """
        left_velocity = linear_velocity - (angular_velocity * self.wheel_base / 2)
        right_velocity = linear_velocity + (angular_velocity * self.wheel_base / 2)
        return left_velocity, right_velocity

    def set_wheel_velocities(self, left_target, right_target):
        """
        Sets the wheel velocities using PID controllers.
        """
        current_left = self.calculate_total_velocity("left_wheel")
        current_right = self.calculate_total_velocity("right_wheel")
        left_error = left_target - current_left
        right_error = right_target - current_right
        control_left = self.pid_left(left_error)
        control_right = self.pid_right(right_error)
        self.joints["left_wheel_joint"].set_drive_velocity_target(left_target + control_left)
        self.joints["right_wheel_joint"].set_drive_velocity_target(right_target + control_right)

    def calculate_total_velocity(self, link_name):
        """
        Calculates the total velocity of a link.
        """
        velocity_vector = self.robot.find_link_by_name(link_name).get_linear_velocity()
        return np.linalg.norm(velocity_vector)
    
    def get_joints_dict(self, articulation):
        """
        Returns a dictionary of joint names to joint objects.
        """
        joints = articulation.get_joints()
        return {joint.get_name(): joint for joint in joints}
