import math
import numpy as np
from enum import Enum

# Define the type of robot shape
class RobotType(Enum):
    """
    Enumeration for robot shape types.
    """
    circle = 0      # Circular robot model
    rectangle = 1   # Rectangular robot model

# Configuration class for DWA parameters
class Config:
    """
    Configuration class for the Dynamic Window Approach (DWA) simulation.
    This class contains all the necessary parameters for simulating robot motion,
    defining constraints, and calculating costs.
    """
    def __init__(self):
        # Motion constraints
        self.max_speed = 0.5  # [m/s] Maximum linear speed of the robot
        self.min_speed = -0.5  # [m/s] Minimum linear speed (reverse)
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s] Maximum yaw (angular) rate
        self.max_accel = 0.5  # [m/s^2] Maximum linear acceleration
        self.max_delta_yaw_rate = 90.0 * math.pi / 180.0  # [rad/s^2] Maximum angular acceleration

        # Resolution for control sampling
        self.v_resolution = 0.02  # [m/s] Resolution of sampled velocities
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s] Resolution of sampled yaw rates

        # Time settings
        self.dt = 0.3  # [s] Time step for motion prediction
        self.predict_time = 2  # [s] Duration to predict the robot's trajectory

        # Cost function weights
        self.to_goal_cost_gain = 0.15  # Weight for the cost of heading toward the goal
        self.speed_cost_gain = 1.0  # Weight for the cost of moving at lower speeds
        self.obstacle_cost_gain = 0.5  # Weight for the cost of avoiding obstacles

        # Threshold to prevent robot from getting stuck
        self.robot_stuck_flag_cons = 0.001  # Small constant to avoid numerical issues

        # Robot shape and size
        self.robot_type = RobotType.circle  # Default robot type (circle)
        self.robot_radius = 0.25  # [m] Radius of the circular robot for collision checks
        self.robot_width = 0.5  # [m] Width of the robot (used for rectangular robots)
        self.robot_length = 0.5  # [m] Length of the robot (used for rectangular robots)

        # Obstacle positions
        self.ob = np.array([])  # [x, y] positions of obstacles (initialized as empty)
