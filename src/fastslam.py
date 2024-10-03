import numpy as np
from typing import List, Tuple

"""
Sources:
http://robots.stanford.edu/papers/montemerlo.fastslam-tr.pdf
https://atsushisakai.github.io/PythonRobotics/modules/slam/ekf_slam/ekf_slam.html
https://atsushisakai.github.io/PythonRobotics/modules/slam/FastSLAM1/FastSLAM1.html
"""

class Pose:
    def __init__(self, x: float, y: float, heading: float) -> None:
        self.x = x
        self.y = y
        self.heading = heading

class Landmark:
    def __init__(self, x, y, sigma_x, sigma_y) -> None:
        self.x
        self.y
        self.sigma_x
        self.sigma_y

class Particle:
    def __init__(self, pose: Pose, landmarks: List[Landmark] = [], weight: float = 1.0) -> None:
        self.pose = pose
        self.landmarks = landmarks
        self.weight = weight

    def get_expected_landmark_measurements(self) -> List[Tuple[float, float]]:
        """
        returns the expected measurements from the position of the particle.

        :param return: an array of expected measurement [(distance: float, angle: float)] 
        """
        expected_measurements = []
        for landmark in self.landmarks:
            # Calculate the distance between particle and landmark
            dx = landmark.x - self.pose.x
            dy = landmark.y - self.pose.y
            distance = np.sqrt(dx**2 + dy**2)
            
            # Calculate the angle between the particle and the landmark relative to the particle's heading
            angle = np.arctan2(dy, dx) - self.pose.theta
            angle = (angle + np.pi) % (2 * np.pi) - np.pi
            expected_measurements.append((distance, angle))
        
        return np.array(expected_measurements)
    
    def match_landmark(self, measurement: Tuple[float, float], distance_threshold: float):
        """
        Matches the current measurement with an existing landmark or identifies it as a new one using Mahalanobis distance.
        Based on implementation of: https://atsushisakai.github.io/PythonRobotics/modules/slam/ekf_slam/ekf_slam.html 

        :param measurement: Tuple of (distance, angle) representing the observed landmark measurement
        :param distance_threshold: The maximum distance to an existing landmark to be considered a match
        :return: Matched Landmark if found, or None if it's a new landmark
        """
        mahalanobis_distances = []
        expected_landmark_measurements = self.get_expected_landmark_measurements()
        for i, landmark in enumerate(self.landmarks):
            expected_landmark_measurement = expected_landmark_measurements[i]
            innovation = np.array([measurement[0] - expected_landmark_measurement[0], measurement[1] - expected_landmark_measurement[1]]) # (distance, angle)
            innovation[1] = (innovation[1] + np.pi) % (2 * np.pi) - np.pi
            innovation_covariance = np.diag([landmark.sigma_x ** 2, landmark.sigma_y ** 2]) #TODO: This is a simplified version of the covariance matrix and could be improved

            mahalanobis_distance = innovation.T @ np.linalg.inv(innovation_covariance) @ innovation
            mahalanobis_distances.append(mahalanobis_distance)
        
        mahalanobis_distances.append(distance_threshold)
        min_distance = min(mahalanobis_distances)
        
        if min_distance < distance_threshold:
            matched_landmark_index = mahalanobis_distances.index(min_distance)
            return self.landmarks[matched_landmark_index]
        return None

class Fastslam_config:
    def __init__(self) -> None:
        self.velocity_standard_deviation: float = 0
        self.angular_velocity_standard_deviation: float = 0
        self.mahalanobis_distance_threshold: float = 0

class Fastslam:
    def __init__(self, config: Fastslam_config) -> None:
        self._config = config

    def _init_particles(self, particle_amount: int, area_size: Tuple[int, int]) -> List[Particle]:
        particles = []
        for i in range(particle_amount):
            x, y = [np.random.uniform(area_size[0]), np.random.uniform(area_size[1])]
            theta = np.random.uniform(0, 2*np.pi)
            particles.append(Particle(Pose(x, y, theta)))
        return particles

    def _motion_model(self, particle_pose: Pose, velocity: float, angular_velocity: float, time_step: float) -> Pose:
        """
        Predicts the next state (pose) of a particle given its current pose.

        :param particle_pose: current position of the particle.
        :param velocity: velocity of the robot.
        :param angular_velocity: angulat velocity of the robot.
        :param time_step: time since the last calculation.
        :param velocity_standard_deviation: standard deviation of the gaussian noise on the velocity.
        :param angular_velocity_standard_deviation: standard deviation of the gaussian noise on the angular velocity.
        """
        F = np.eye()  
        
        # Transforms the control inputs (velocity, angular velocity) into position changes.
        B = np.array([[time_step * np.cos(angular_velocity), 0],  
                    [time_step * np.sin(angular_velocity), 0], 
                    [0, time_step]])  
        
        # Extract the current position and orientation (x, y, theta) from the given particle_pose object
        position_current = np.array([particle_pose.x, particle_pose.y, particle_pose.heading]).T
        
        # Includes velocity and angular velocity, with Gaussian noise added.
        U = np.array([velocity + np.random.normal(0, self._config.velocity_standard_deviation), angular_velocity + np.random.normal(0, self._config.angular_velocity_standard_deviation)]).T  
        
        position_predicted = F @ position_current + B @ U
        
        # Normalize the angle to keep it in the range [-pi, pi]
        position_current[2, 0] = (position_current[2, 0] + np.pi) % (2 * np.pi) - np.pi
        return Pose(position_predicted[0, 0], position_predicted[0, 1], position_predicted[0, 2])

    def _predict(self, particles: List[Particle], velocity, angular_velocity, time_step) -> List[Particle]:
        """
        Predicts the next state for all particles using the motion model.
        
        :param particles: List of particles
        :param velocity: Robot velocity
        :param angular_velocity: Robot angular velocity
        :param time_step: Time step for prediction
        :return: Updated list of particles with predicted poses
        """
        for particle in particles:
            particle.pose = self._motion_model(particle.pose, velocity, angular_velocity, time_step)

        return particles

    def _update(self, particles: List[Particle], landmark_measurements: List[Tuple[float, float]]) -> List[Particle]:
        """
        Updates particle states based on measured landmarks.
        
        :param particles: List of particles
        :param landmark_measurements: List of measured landmarks (distance, angle)
        :return: Updated list of particles
        """
        for particle in particles:
            for landmark_measurement in landmark_measurements:
                matched_landmark = particle.match_landmark(landmark_measurement, self._config.mahalanobis_distance_threshold)

                if matched_landmark is None:
                    # Add new landmark
                    distance, angle = landmark_measurement
                    landmark_x = particle.pose.x + distance * np.cos(particle.pose.heading + angle)
                    landmark_y = particle.pose.y + distance * np.sin(particle.pose.heading + angle)
                    particle.landmarks.append(Landmark(landmark_x, landmark_y, sigma_x=1.0, sigma_y=1.0)) # TODO: What should the sigma_x and sigma_y be?

                else:
                    # TODO: Update landmark with new information (make it more precise?)
                    pass

                # TODO: Calculate weight of the particle

        return particles

    def _resample(particles: List[Particle]) -> List[Particle]:
        # TODO: resample the particles relative to the weights
        pass
