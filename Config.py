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
        self.max_speed = 0.5  # [m/s]
        self.min_speed = -0.5  # [m/s]F
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.5  # [m/ss]
        self.max_delta_yaw_rate = 90.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.02  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.3  # [s] Time tick for motion prediction
        self.predict_time = 2  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 0.5
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stuck
        self.robot_type = RobotType.circle
        self.robot_radius = 0.25  # [m] for collision check
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 0.5  # [m] for collision check
        self.ob = np.array([])  # Will be set dynamically