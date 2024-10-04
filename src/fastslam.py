import numpy as np
from typing import List, Tuple

"""
Sources:
http://robots.stanford.edu/papers/montemerlo.fastslam-tr.pdf
https://atsushisakai.github.io/PythonRobotics/modules/slam/ekf_slam/ekf_slam.html
https://atsushisakai.github.io/PythonRobotics/modules/slam/FastSLAM1/FastSLAM1.html

https://www.youtube.com/watch?v=IFeCIbljreY
https://www.youtube.com/watch?v=NrzmH_yerBU
"""

class Pose:
    def __init__(self, x: float, y: float, heading: float) -> None:
        self.x = x
        self.y = y
        self.heading = heading

class Landmark:
    def __init__(self, x, y, sigma_x, sigma_y) -> None:
        self.x = x
        self.y = y
        self.sigma_x = sigma_x
        self.sigma_y = sigma_y

    def update(self, particle, measurement: Tuple[float, float], measurement_covariance: np.ndarray):
        """
        Updates the landmarks position and covariance based on the new measurement.
        Based on the implementation of: https://atsushisakai.github.io/PythonRobotics/modules/slam/FastSLAM1/FastSLAM1.html
        """

        h_robot_pose, h_landmark_pose = compute_jacobians(particle, self)
        expected_measurement = self.get_relative_measurement(particle.pose)
        innovation_covariance = h_landmark_pose @ Landmark.get_covariance_matrix() @ h_landmark_pose.T + measurement_covariance
        innovation_covariance = (innovation_covariance + innovation_covariance.T) * 0.5

        delta = np.array([measurement[0] - expected_measurement[0], pi2pi(measurement[1] - expected_measurement[1])]) # (distance, angle)

        SChol = np.linalg.cholesky(innovation_covariance).T
        SCholInv = np.linalg.inv(SChol)

        kalman_gain = Landmark.get_covariance_matrix() @ h_landmark_pose.T @ SCholInv

        updated_position = np.array(expected_measurement).reshape(2, 1) + kalman_gain @ SCholInv.T @ delta
        updated_covariance = Landmark.get_covariance_matrix() - kalman_gain @ (kalman_gain @ SCholInv.T).T

        self.x = updated_position[0, 0]
        self.y = updated_position[1, 0]
        self.sigma_x = updated_covariance[0, 0]
        self.sigma_y = updated_covariance[1, 1]

    def get_covariance_matrix(self) -> np.ndarray:
        """
        Get the covariance matrix (2x2) for the landmark
        """
        return np.diag([self.sigma_x ** 2, self.sigma_y ** 2])
    
    def get_relative_measurement(self, pose: Pose) -> Tuple[float, float]:
        """
        returns the expected measurements from the given pose.

        :param return: Tuple of the expected measurement (distance: float, angle: float) 
        """
        # Calculate the distance between particle and landmark
        dx = self.x - pose.x
        dy = self.y - pose.y
        distance = np.sqrt(dx**2 + dy**2)
        
        # Calculate the angle between the particle and the landmark relative to the particle's heading
        angle = np.arctan2(dy, dx) - pose.heading
        angle = pi2pi(angle)

        return distance, angle
    
    def get_expected_position(self) -> Tuple[float, float]:
        """
        returns the expected position

        :param return: Tuple of the expected measurement (x: float, y: float) 
        """
        return self.x, self.y

class Particle:
    def __init__(self, pose: Pose, landmarks: List[Landmark] = [], weight: float = 1.0) -> None:
        self.pose = pose
        self.landmarks = landmarks
        self.weight = weight
    
    def match_landmark(self, measurement: Tuple[float, float], distance_threshold: float):
        """
        Matches the current measurement with an existing landmark or identifies it as a new one using Mahalanobis distance.
        Based on implementation of: https://atsushisakai.github.io/PythonRobotics/modules/slam/ekf_slam/ekf_slam.html 

        :param measurement: Tuple of (distance, angle) representing the observed landmark measurement
        :param distance_threshold: The maximum distance to an existing landmark to be considered a match
        :return: Matched Landmark if found, or None if it's a new landmark
        """
        mahalanobis_distances = []
        for landmark in self.landmarks:
            expected_landmark_measurement = landmark.get_relative_measurement(self.pose)
            innovation = np.array([measurement[0] - expected_landmark_measurement[0], measurement[1] - expected_landmark_measurement[1]]) # (distance, angle)
            innovation[1] = pi2pi(innovation[1])

            mahalanobis_distance = innovation.T @ np.linalg.inv(landmark.get_covariance_matrix()) @ innovation
            mahalanobis_distances.append(mahalanobis_distance)
        
        mahalanobis_distances.append(distance_threshold)
        min_distance = min(mahalanobis_distances)
        
        if min_distance < distance_threshold:
            matched_landmark_index = mahalanobis_distances.index(min_distance)
            return self.landmarks[matched_landmark_index]
        return None

class Fastslam_config:
    def __init__(self) -> None:
        self.particle_amount = 100
        self.velocity_standard_deviation: float = 0
        self.angular_velocity_standard_deviation: float = 0
        self.mahalanobis_distance_threshold: float = 0
        self.measurement_covariance = np.array([[0, 0], [0, 0]])
        self.effective_particle_amount_modifier = .666666

class Fastslam:
    def __init__(self, config: Fastslam_config) -> None:
        self._config = config
        self.particles = self._init_particles(self._config.particle_amount, (50, 50))

    def run(self, measurements, velocity, angular_velocity, time_step):
        self.particles = self._predict(self.particles, velocity, angular_velocity, time_step)
        self.particles = self._update(self.particles, measurements)
        self.particles = self._resample(self.particles)

    def get_estimated_position(self) -> Tuple[float, float]:
        # TODO
        pass
    
    def get_estimated_map(self) -> None:
        # TODO
        pass

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
        F = np.eye(3)  
        
        # Transforms the control inputs (velocity, angular velocity) into position changes.
        B = np.array([[time_step * np.cos(angular_velocity), 0],  
                    [time_step * np.sin(angular_velocity), 0], 
                    [0, time_step]])  
        
        # Extract the current position and orientation (x, y, heading) from the given particle_pose object
        position_current = np.array([particle_pose.x, particle_pose.y, particle_pose.heading]).T
        
        # Includes velocity and angular velocity, with Gaussian noise added.
        U = np.array([velocity + np.random.normal(0, self._config.velocity_standard_deviation), angular_velocity + np.random.normal(0, self._config.angular_velocity_standard_deviation)]).T  
        
        position_predicted = F @ position_current + B @ U
        position_predicted[2, 0] = pi2pi(position_predicted[2, 0])

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
                    particle.weight *= compute_weight(particle, matched_landmark, landmark_measurement, self._config.measurement_covariance)
                    matched_landmark.update(particle, landmark_measurement, self._config.measurement_covariance)
                    pass

        return particles

    def _resample(self, particles: List[Particle]) -> List[Particle]:
        normalize_weights(particles)

        weights = []
        for particle in particles:
            weights.append(particle.weight)
        weights = np.array(weights)

        effective_particle_amount = 1.0 / (weights @ weights.T)
        if effective_particle_amount < self._config.effective_particle_amount_modifier * len(particles):
            weight_cumulative = np.cumsum(weights)
            resample_base = np.cumsum(weights * 0.0 + 1 / len(particles)) - 1 / len(particles)
            resample_ids = resample_base + np.random.rand(resample_base.shape[0]) / len(particles)

            indices = []
            index = 0
            for i in range(len(particles)):
                while ((index < weight_cumulative.shape[0] - 1) and (resample_ids[i] > weight_cumulative[index])):
                    index += 1
                indices.append(index)

            particles_temp = particles[:]
            for i in range(len(indices)):
                particles[i].pose.x = particles_temp[indices[i]].pose.x
                particles[i].pose.y = particles_temp[indices[i]].pose.y
                particles[i].pose.heading = particles_temp[indices[i]].pose.heading
                particles[i].w = 1.0 / len(particles)  

        return particles

def pi2pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

def normalize_weights(particles : List[Particle]):
    sum_weights = sum([p.weight for p in particles])
    if sum_weights == 0:
        for particle in particles:
                particle.weight = 1.0 / len(particles)
    else:
        for particle in particles:
            particle.weight /= sum_weights

def compute_jacobians(particle: Particle, landmark: Landmark) -> Tuple[np.ndarray, np.ndarray]:
    """
    computes the jacobian in respect for the robot pose and the landmark pose
    Based on implementation of: https://atsushisakai.github.io/PythonRobotics/modules/slam/FastSLAM1/FastSLAM1.html

    :param return: Tuple of h_robot_pose and h_landmark_pose
    """
    dx = landmark.x - particle.pose.x
    dy = landmark.y - particle.pose.y
    squared_distance = dx**2 + dy**2
    distance = np.sqrt(squared_distance)

    # Jacobian with respect to robot pose
    h_robot_pose = np.array([[-dx / distance, -dy / distance, 0.0],
                   [dy / squared_distance, -dx / squared_distance, -1.0]])

    # Jacobian with respect to landmark pose
    h_landmark_pose = np.array([[dx / distance, dy / distance],
                   [-dy / squared_distance, dx / squared_distance]])

    return h_robot_pose, h_landmark_pose

def compute_weight(particle: Particle, matched_landmark: Landmark, measurement: Tuple[float, float], measurement_covariance: np.ndarray) -> float:
    """
    Computes the weight based on how closely the expected measurement and the actual measurement align 
    Based on implementation of: https://atsushisakai.github.io/PythonRobotics/modules/slam/FastSLAM1/FastSLAM1.html
    
    :param return: The calculated weight 
    """
    h_robot_pose, h_landmark_pose = compute_jacobians(particle, matched_landmark)
    expected_measurement = matched_landmark.get_relative_measurement(particle.pose)
    innovation_covariance = h_landmark_pose @ Landmark.get_covariance_matrix() @ h_landmark_pose.T + measurement_covariance
    inverse_innovation_covariance = np.linalg.inv(innovation_covariance)

    delta = np.array([measurement[0] - expected_measurement[0], pi2pi(measurement[1] - expected_measurement[1])]) # (distance, angle)
    num = np.exp(-.5 * delta.T @ inverse_innovation_covariance @ delta)
    den = 2.0 * np.pi * np.sqrt(np.linalg.det(innovation_covariance))
    return num / den
