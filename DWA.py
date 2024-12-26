import numpy as np
import cv2
import Config
import math

class DWA:
    def __init__(self, config: Config, odom, goal_pose):
        """
        Initializes the Dynamic Window Approach (DWA) class.

        :param config: Configuration object containing DWA parameters.
        :param odom: Current state of the robot [x, y, yaw, v, omega].
        :param goal_pose: Goal position for the robot (Pose object).
        """
        self.config = config  # DWA configuration parameters.
        self.x = odom  # Current state of the robot.
        self.goal_pose = goal_pose  # Goal position.

    def update_obstacles(self, filtered_lidar_points: np.ndarray) -> None:
        """
        Updates the obstacle positions based on LiDAR points.

        :param filtered_lidar_points: Array of LiDAR points.
        """
        updated_points = []
        robot_orientation = self.x[2]  # Robot's orientation (yaw angle).
        transformation_matrix = np.array([
            [np.cos(robot_orientation), -np.sin(robot_orientation)],
            [np.sin(robot_orientation), np.cos(robot_orientation)]
        ])  # Transformation matrix for local-to-global coordinate conversion.

        # Transform each obstacle point to global coordinates.
        for obs in filtered_lidar_points:
            global_ob = np.dot(transformation_matrix, obs[:2]) + self.x[:2]
            updated_points.append(global_ob)
        self.config.ob = updated_points  # Update obstacles in the configuration.

    def dwa_control(self) -> np.ndarray:
        """
        Performs DWA control to calculate the next control inputs.

        :return: Control inputs [linear velocity, angular velocity] and trajectory.
        """
        u, _ = self.dwa(self.x, self.config, self.goal_pose.p, self.config.ob)
        return u, _

    def dwa(self, x, config, goal, ob):
        """
        Calculates the dynamic window and computes the best control inputs.

        :param x: Current state of the robot.
        :param config: Configuration object.
        :param goal: Goal position.
        :param ob: List of obstacles.
        :return: Best control inputs and trajectory.
        """
        dw = self.calc_dynamic_window(x, config)  # Calculate the dynamic window.
        u, trajectory = self.calc_control_and_trajectory(x, dw, config, goal, ob)
        return u, trajectory

    def motion(self, x, u, dt):
        """
        Predicts the next state of the robot based on motion model.

        :param x: Current state of the robot.
        :param u: Control inputs [linear velocity, angular velocity].
        :param dt: Time step.
        :return: Updated state of the robot.
        """
        x[2] += u[1] * dt  # Update yaw angle.
        x[0] += u[0] * math.cos(x[2]) * dt  # Update x-coordinate.
        x[1] += u[0] * math.sin(x[2]) * dt  # Update y-coordinate.
        x[3] = u[0]  # Update linear velocity.
        x[4] = u[1]  # Update angular velocity.
        return x

    def calc_dynamic_window(self, x, config):
        """
        Calculates the dynamic window based on the robot's current state and limits.

        :param x: Current state of the robot.
        :param config: Configuration object.
        :return: Dynamic window [min_v, max_v, min_omega, max_omega].
        """
        # Robot's possible velocities and yaw rates based on constraints.
        Vs = [config.min_speed, config.max_speed,
              -config.max_yaw_rate, config.max_yaw_rate]
        # Robot's achievable velocities and yaw rates considering acceleration limits.
        Vd = [x[3] - config.max_accel * config.dt,
              x[3] + config.max_accel * config.dt,
              x[4] - config.max_delta_yaw_rate * config.dt,
              x[4] + config.max_delta_yaw_rate * config.dt]
        # Dynamic window is the intersection of constraints and achievable ranges.
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
        return dw

    def calc_control_and_trajectory(self, x, dw, config, goal, ob):
        """
        Calculates the best control inputs and trajectory.

        :param x: Current state of the robot.
        :param dw: Dynamic window.
        :param config: Configuration object.
        :param goal: Goal position.
        :param ob: List of obstacles.
        :return: Best control inputs and corresponding trajectory.
        """
        x_init = x[:]
        min_cost = float("inf")  # Initialize minimum cost.
        best_u = [0.0, 0.0]  # Initialize best control inputs.

        # Iterate through velocities and yaw rates within the dynamic window.
        for v in np.arange(dw[0], dw[1], config.v_resolution):
            for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):
                trajectory = self.predict_trajectory(x_init, v, y, config)
                # Calculate costs for the trajectory.
                to_goal_cost = config.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, goal)
                speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
                ob_cost = config.obstacle_cost_gain * self.calc_obstacle_cost(trajectory, ob, config)

                # Total cost for the trajectory.
                final_cost = to_goal_cost + speed_cost + ob_cost

                # Update best control inputs if the current trajectory has lower cost.
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
        
        return best_u, trajectory

    def predict_trajectory(self, x_init, v, y, config):
        """
        Predicts the trajectory for a given control input.

        :param x_init: Initial state of the robot.
        :param v: Linear velocity.
        :param y: Angular velocity.
        :param config: Configuration object.
        :return: Predicted trajectory.
        """
        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        # Simulate the trajectory over the prediction time.
        while time <= config.predict_time:
            x = self.motion(x, [v, y], config.dt)
            trajectory = np.vstack((trajectory, x))
            time += config.dt
        return trajectory

    def calc_obstacle_cost(self, trajectory, ob, config):
        """
        Calculates the cost of a trajectory with respect to obstacles.

        :param trajectory: Predicted trajectory.
        :param ob: List of obstacles.
        :param config: Configuration object.
        :return: Obstacle cost.
        """
        ob = np.array(ob)
        if ob.size == 0:
            return 0  # No obstacles.

        # Calculate distances to obstacles.
        ox = ob[:, 0]
        oy = ob[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)

        if np.array(r <= config.robot_radius).any():
            print("Infinite cost due to collision.")
            return float("Inf")

        min_r = np.min(r)  # Minimum distance to obstacles.
        return 1.0 / min_r  # Cost inversely proportional to the distance.

    def calc_to_goal_cost(self, trajectory, goal):
        """
        Calculates the cost of a trajectory with respect to the goal.

        :param trajectory: Predicted trajectory.
        :param goal: Goal position.
        :return: Goal cost.
        """
        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx) - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(error_angle), math.cos(error_angle)))
        return cost

    def render(self, trajectory):
        """
        Renders the simulation environment using OpenCV.

        :param trajectory: Predicted trajectory to render.
        """
        width, height = 800, 800
        image = np.zeros((height, width, 3), dtype=np.uint8)
        scale = 60
        offset = 400

        # Draw obstacles.
        for obs in self.config.ob:
            obs_x = int(obs[0] * scale) + offset
            obs_y = int(obs[1] * scale) + offset
            cv2.circle(image, (obs_x, obs_y), 5, (0, 0, 255), -1)

        # Draw trajectory.
        for point in trajectory:
            x = int(point[0] * scale) + offset
            y = int(point[1] * scale) + offset
            cv2.circle(image, (x, y), 2, (255, 0, 0), -1)

        # Draw robot and goal.
        robot_x = int(self.x[0] * scale) + offset
        robot_y = int(self.x[1] * scale) + offset
        cv2.circle(image, (robot_x, robot_y), 10, (0, 255, 0), -1)
        cv2.circle(image, (robot_x, robot_y), int(self.config.robot_radius * scale), (255, 255, 0), 1)

        goal_x = int(self.goal_pose.p[0] * scale) + offset
        goal_y = int(self.goal_pose.p[1] * scale) + offset
        cv2.circle(image, (goal_x, goal_y), 10, (255, 255, 0), -1)

        # Flip and rotate the image for display.
        image = cv2.flip(image, 1)
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        cv2.imshow("Simulation", image)
        cv2.waitKey(1)
