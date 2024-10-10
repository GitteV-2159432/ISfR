import sapien.core as sapien
from sapien import Scene, Pose
from sapien.utils import Viewer
from simple_pid import PID
import tkinter as tk
from tkinter import ttk
import threading
import numpy as np
import paho.mqtt.client as mqtt

class Driver:
    def __init__(self, scene: Scene, viewer: Viewer) -> None:
        self._viewer = viewer
        self.move_speed = 1
        self.wheel_radius = 0.1
        self.wheel_base = 0.4
        loader = scene.create_urdf_loader()
        loader.fix_root_link = False
        self.robot = loader.load("dif_robot.urdf")
        self.robot.set_root_pose(Pose([0, 0, 5], [1, 0, 0, 0]))
        loader.set_link_material("caster_wheel", 0.1, 0, 0)
        self.joints = self.get_joints_dict(self.robot)
        self.dof_count = self.robot.get_dof()
        print("Number of DOFs:", self.dof_count)
        self.pid_left = PID(0, 0, 0, setpoint=self.move_speed)
        self.pid_right = PID(0, 0, 0, setpoint=self.move_speed)
        self.setup_mqtt()
        self.target_velocity = 0
        self.gui_thread = threading.Thread(target=self.setup_gui)
        self.gui_thread.start()

    def setup_mqtt(self):
        self.client = mqtt.Client("robot_publisher")
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect

        try:
            self.client.connect("localhost", 1883, 60)
        except Exception as e:
            print("Failed to connect to MQTT broker:", e)
        self.client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT broker")
        else:
            print("Failed to connect to MQTT broker, return code:", rc)

    def on_disconnect(self, client, userdata, rc):
        print("Disconnected from MQTT broker")

    def setup_gui(self):
        self.root = tk.Tk()
        self.root.title("PID Controller Settings")

        
        self.add_gui_elements()

        
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    def add_gui_elements(self):
        ttk.Label(self.root, text="Kp:").grid(column=0, row=0)
        self.kp_entry = ttk.Entry(self.root)
        self.kp_entry.insert(0, str(self.pid_left.Kp))
        self.kp_entry.grid(column=1, row=0)

        ttk.Label(self.root, text="Ki:").grid(column=0, row=1)
        self.ki_entry = ttk.Entry(self.root)
        self.ki_entry.insert(0, str(self.pid_left.Ki))
        self.ki_entry.grid(column=1, row=1)

        ttk.Label(self.root, text="Kd:").grid(column=0, row=2)
        self.kd_entry = ttk.Entry(self.root)
        self.kd_entry.insert(0, str(self.pid_left.Kd))
        self.kd_entry.grid(column=1, row=2)

        self.update_button = ttk.Button(self.root, text="Update PID", command=self.update_pid)
        self.update_button.grid(column=0, row=3, columnspan=2)

        self.reset_button = ttk.Button(self.root, text="Reset Robot", command=self.reset_robot)
        self.reset_button.grid(column=0, row=4, columnspan=2)

    def update_pid(self):
        self.pid_left.Kp = float(self.kp_entry.get())
        self.pid_left.Ki = float(self.ki_entry.get())
        self.pid_left.Kd = float(self.kd_entry.get())
        self.pid_right.Kp = float(self.kp_entry.get())
        self.pid_right.Ki = float(self.ki_entry.get())
        self.pid_right.Kd = float(self.kd_entry.get())

        self.target_velocity = 1
        print("PID updated and target velocity set to:", self.target_velocity)

        
        self.client.publish("robot/reset", "Robot has been reset")

    def reset_robot(self):
        self.target_velocity = 0
        print("Target velocity set to zero")

        self.robot.set_root_pose(Pose([0, 0, 5], [1, 0, 0, 0]))
        print("Robot position reset")

        
        self.client.publish("robot/reset", "Robot has been reset")

    def on_closing(self):
        self.client.loop_stop()
        self.root.destroy()

    def update(self) -> None:
        left_velocity_target = self.target_velocity
        right_velocity_target = self.target_velocity

        self.set_wheel_velocities(left_velocity_target, right_velocity_target)

    def set_wheel_velocities(self, left_velocity_target: float, right_velocity_target: float) -> None:
        current_left_velocity = self.calculate_total_velocity("left_wheel")
        current_right_velocity = self.calculate_total_velocity("right_wheel")

        control_left = self.pid_left(current_left_velocity) if left_velocity_target > 0 else -current_left_velocity * 0.5
        control_right = self.pid_right(current_right_velocity) if right_velocity_target > 0 else -current_right_velocity * 0.5

        self.joints['base_left_wheel_joint'].set_drive_velocity_target(control_left)
        self.joints['base_right_wheel_joint'].set_drive_velocity_target(control_right)

        
        self.client.publish("robot/wheel_velocities", f"left:{current_left_velocity}, right:{current_right_velocity}")

    def calculate_total_velocity(self, link_name: str) -> float:
        """Calculate the total linear velocity of the specified link."""
        velocity_vector = self.robot.find_link_by_name(link_name).get_linear_velocity()
        v_x, v_y, v_z = velocity_vector
        total_velocity = np.sqrt(v_x**2 + v_y**2 + v_z**2)
        return round(total_velocity, 2)

    def get_joints_dict(self, articulation):
        """Get a dictionary of joint names to joint objects."""
        joints = articulation.get_joints()
        return {joint.get_name(): joint for joint in joints}
