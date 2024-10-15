import sapien.core as sapien
from sapien import Scene, Pose
from sapien.utils import Viewer
from simple_pid import PID
import threading
import numpy as np
import paho.mqtt.client as mqtt

class Driver:
    def __init__(self, scene: Scene, viewer: Viewer) -> None:
        self._viewer = viewer
        self.move_speed = 1
        self.wheel_radius = 0.1
        self.wheel_base = 0.4
        self.target_velocity = 0
        self.max_velocity = 5
        loader = scene.create_urdf_loader()
        loader.fix_root_link = False
        self.robot = loader.load("dif_robot.urdf")
        self.robot.set_root_pose(Pose([0, 0, 5], [1, 0, 0, 0]))
        loader.set_link_material("caster_wheel", 0.1, 0, 0)
        self.joints = self.get_joints_dict(self.robot)

        self.pid_left = PID(0, 0, 0, setpoint=self.move_speed)
        self.pid_right = PID(0, 0, 0, setpoint=self.move_speed)
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
                print(f"Updated PID: Kp={round(kp,2)}, Ki={round(ki,2)}, Kd={round(kd,2)}")
            except ValueError as e:
                print(f"Error parsing PID message: {e}")
        elif msg.topic == "robot/reset":
            self.reset_robot()

    def reset_robot(self):
        self.robot.set_root_pose(Pose([0, 0, 0], [1, 0, 0, 0]))
        self.target_velocity = 0
        self.joints['base_left_wheel_joint'].set_drive_velocity_target(0)
        self.joints['base_right_wheel_joint'].set_drive_velocity_target(0)

        print("Robot reset")
    def update(self) -> None:
        # Ensure that the target velocity is set
        self.target_velocity = self.move_speed  # Setting target velocity every update
        left_velocity_target = self.target_velocity
        right_velocity_target = self.target_velocity

        # Debug output to verify the update method is being called
        print(f"Updating velocities: left={left_velocity_target}, right={right_velocity_target}")
        
        self.set_wheel_velocities(left_velocity_target, right_velocity_target)

    def set_wheel_velocities(self, left_velocity_target: float, right_velocity_target: float) -> None:
        current_left_velocity = self.calculate_total_velocity("left_wheel")
        current_right_velocity = self.calculate_total_velocity("right_wheel")

        # Debug outputs to check current velocities
        print(f"Current velocities: left={current_left_velocity}, right={current_right_velocity}")

        # Calculate control signals from PID
        control_left = self.pid_left(current_left_velocity)
        control_right = self.pid_right(current_right_velocity)

        # Ensure control signals are not zero (for debugging)
        print(f"Control signals: left={control_left}, right={control_right}")

        self.joints['base_left_wheel_joint'].set_drive_velocity_target(control_left)
        self.joints['base_right_wheel_joint'].set_drive_velocity_target(control_right)

        # Publish the velocities to MQTT
        self.client.publish("robot/wheel_velocities", f"left:{current_left_velocity}, right:{current_right_velocity}")
    def calculate_total_velocity(self, link_name: str) -> float:
        velocity_vector = self.robot.find_link_by_name(link_name).get_linear_velocity()
        v_x, v_y, v_z = velocity_vector
        total_velocity = np.sqrt(v_x**2 + v_y**2 + v_z**2)
        return round(total_velocity, 2)

    def get_joints_dict(self, articulation):
        joints = articulation.get_joints()
        return {joint.get_name(): joint for joint in joints}

