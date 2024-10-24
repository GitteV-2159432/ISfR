import numpy as np
import cv2
import Config
class DWA:
    def __init__(self, config: Config, odom, goal_pose):
        self.config = config
        self.x = odom  # Current state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
        self.goal_pose = goal_pose  # Goal position

    def update_obstacles(self, filtered_lidar_points: np.ndarray) -> None:
        """Update obstacles based on LiDAR points."""
        updated_points = []
        robot_orientation = self.x[2]
        transformation_matrix = np.array([
            [np.cos(robot_orientation), -np.sin(robot_orientation)],
            [np.sin(robot_orientation), np.cos(robot_orientation)]
        ])

        for obs in filtered_lidar_points:
            global_ob = np.dot(transformation_matrix, obs[:2]) + self.x[:2]
            updated_points.append(global_ob)
        self.config.ob = np.array(updated_points)

    def dwa_control(self) -> np.ndarray:
        """Compute control command with DWA."""
        u, _ = self.dwa(self.x, self.config, self.goal_pose.p, self.config.ob)
        self.x = self.motion(self.x, u, self.config.dt)  # Update robot state
        return u, _

    def dwa(self, x, config, goal, ob):
        """Core DWA function: calculates optimal velocities."""
        dw = self.calc_dynamic_window(x, config)
        u, trajectory = self.calc_control_and_trajectory(x, dw, config, goal, ob)
        return u, trajectory

    def motion(self, x, u, dt):
        """Motion model."""
        x[2] += u[1] * dt
        x[0] += u[0] * np.cos(x[2]) * dt
        x[1] += u[0] * np.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]
        return x

    def render(self, trajectory):
        """Render the environment, robot, obstacles, and trajectory using OpenCV."""
        width, height = 800, 800
        image = np.zeros((height, width, 3), dtype=np.uint8)
        scale = 60
        offset = 400

        # Draw obstacles
        for obs in self.config.ob:
            obs_x = int(obs[0] * scale) + offset
            obs_y = int(obs[1] * scale) + offset
            cv2.circle(image, (obs_x, obs_y), 5, (0, 0, 255), -1)

        # Draw trajectory
        for point in trajectory:
            x = int(point[0] * scale) + offset
            y = int(point[1] * scale) + offset
            cv2.circle(image, (x, y), 2, (255, 0, 0), -1)

        # Draw the robot's current position
        robot_x = int(self.x[0] * scale) + offset
        robot_y = int(self.x[1] * scale) + offset
        cv2.circle(image, (robot_x, robot_y), 10, (0, 255, 0), -1)
        cv2.circle(image, (robot_x, robot_y), int(self.config.robot_radius * scale), (255, 255, 0), 1)  # Yellow outline for detection radius

        # Draw the goal
        goal_x = int(self.goal_pose.p[0] * scale) + offset
        goal_y = int(self.goal_pose.p[1] * scale) + offset
        cv2.circle(image, (goal_x, goal_y), 10, (255, 255, 0), -1)

        # Show the image in a window
        cv2.imshow("Simulation", image)
        cv2.waitKey(1)

    def calc_dynamic_window(self, x, config):
        """Calculate dynamic window based on current state and config."""
        Vs = [config.min_speed, config.max_speed,
              -config.max_yaw_rate, config.max_yaw_rate]
        Vd = [x[3] - config.max_accel * config.dt,
              x[3] + config.max_accel * config.dt,
              x[4] - config.max_delta_yaw_rate * config.dt,
              x[4] + config.max_delta_yaw_rate * config.dt]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        return dw

    def calc_control_and_trajectory(self, x, dw, config, goal, ob):
        """Calculate control and trajectory based on dynamic window and obstacles."""
        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        
        for v in np.arange(dw[0], dw[1], config.v_resolution):
            for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):
                trajectory = self.predict_trajectory(x_init, v, y, config)
                to_goal_cost = config.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, goal)
                speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
                ob_cost = config.obstacle_cost_gain * self.calc_obstacle_cost(trajectory, ob, config)

                final_cost = to_goal_cost + speed_cost + ob_cost

                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
        
        return best_u, trajectory

    def predict_trajectory(self, x_init, v, y, config):
        """Predict the trajectory for a given velocity and yaw rate."""
        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= config.predict_time:
            x = self.motion(x, [v, y], config.dt)
            trajectory = np.vstack((trajectory, x))
            time += config.dt
        return trajectory

    def calc_obstacle_cost(self, trajectory, ob, config):
        """Calculate obstacle cost based on predicted trajectory and obstacle locations."""
        
        if self.config.ob.size == 0:
            return 100
        ob = np.array(ob)
        ox = ob[:, 0]
        oy = ob[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)

        if config.robot_type ==Config.RobotType.circle:
            if np.array(r <= config.robot_radius).any():
                print("Infinite cost due to collision")
                return float("Inf")

        min_r = np.min(r)
        return 1.0 / min_r  # Lower the cost the further away from obstacles

    def calc_to_goal_cost(self, trajectory, goal):
        """Calculate the cost to the goal."""
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = np.arctan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(np.arctan2(np.sin(cost_angle), np.cos(cost_angle)))
        return cost