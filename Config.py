import math
import numpy as np
from enum import Enum

class RobotType(Enum):
    circle = 0
    rectangle = 1

class Config:
    """
    Simulation parameter class for DWA.
    """
    def __init__(self):
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 0.1  # [rad/s]
        self.max_accel = 0.2 # [m/s^2]
        self.max_delta_yaw_rate = 90.0 * math.pi / 180.0  # [rad/s^2]
        self.v_resolution = 0.1  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s] Time horizon for motion prediction
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 0.5
        self.robot_stuck_flag_cons = 0.001  # Constant to prevent robot from getting stuck
        self.robot_type = RobotType.rectangle
        self.robot_radius = 0.25  # [m] for collision check
        self.robot_width = 1  # [m] for collision check (if using rectangle)
        self.robot_length = 1  # [m] for collision check (if using rectangle)
        self.ob = np.array([])  # Will be set dynamically with obstacles
