import numpy as np
from typing import List, Tuple
import cv2
from datetime import datetime
from sklearn.neighbors import KDTree

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
    def __init__(self, pose: Pose, landmarks: List[Landmark] = None, weight: float = 1.0) -> None:
        self.pose = pose
        if landmarks is None:
            self.landmarks = []
            self.landmark_tree = None
        else:
            self.landmark_tree = KDTree([np.array(landmark.get_expected_position()) for landmark in self.landmarks])
        self.weight = weight
        
    
    def match_landmark(self, measurement: Tuple[float, float], distance_threshold: float):
        """
        Matches the current measurement with an existing landmark or identifies it as a new one.

        :param measurement: Tuple of (distance, angle) representing the observed landmark measurement
        :param distance_threshold: The maximum distance to an existing landmark to be considered a match
        :return: Matched Landmark if found, or None if it's a new landmark
        """
        distance, angle = measurement
        measurement_x = self.pose.x + distance * np.cos(pi2pi(self.pose.heading + angle))
        measurement_y = self.pose.y + distance * np.sin(pi2pi(self.pose.heading + angle))
        
        if (self.landmark_tree is not None):
            distance, index = self.landmark_tree.query([(measurement_x, measurement_y)], k=1)
            nearest_index = index[0][0] # Gets the index from the original list
            if distance < distance_threshold:
                return self.landmarks[nearest_index]
        
        new_landmark = Landmark(measurement_x, measurement_y, sigma_x=0.5, sigma_y=1.0) # TODO: What should the sigma_x and sigma_y be?
        self.landmarks.append(new_landmark)
        self.landmark_tree = KDTree([np.array(landmark.get_expected_position()) for landmark in self.landmarks])
        return new_landmark

class FastSLAM_config:
    def __init__(self) -> None:
        self.particle_amount = 100
        self.velocity_standard_deviation: float = 0
        self.angular_velocity_standard_deviation: float = 0
        self.distance_threshold: float = 0
        self.measurement_covariance = np.array([[0, 0], [0, 0]])
        self.effective_particle_amount_modifier = .666666

class FastSLAM:
    def __init__(self, config: FastSLAM_config) -> None:
        self._config = config
        self.last_time = datetime.now()
        self.particles = self._init_particles(self._config.particle_amount, 10)

    def run(self, measurements, velocity, angular_velocity):
        time_step = (datetime.now() - self.last_time).microseconds * float(1e-6)
        self.last_time = datetime.now()

        self.particles = self._predict(self.particles, velocity, angular_velocity, time_step)
        self.particles = self._update(self.particles, measurements)
        # self.particles = self._resample(self.particles)

    def get_estimated_position(self) -> Tuple[float, float]:
        """
        Returns the estimated position of the robot as the weighted average of the particle positions.

        :return: Tuple representing the estimated (x, y) position of the robot.
        """
        total_weight = sum(particle.weight for particle in self.particles)

        if total_weight == 0:
            # If all weights are zero, return the position of the first particle (or default to some position)
            return self.particles[0].pose.x, self.particles[0].pose.y

        # Compute weighted average for x and y positions
        estimated_x = sum(particle.pose.x * particle.weight for particle in self.particles) / total_weight
        estimated_y = sum(particle.pose.y * particle.weight for particle in self.particles) / total_weight

        return estimated_x, estimated_y
    
    def get_estimated_map(self) -> List[Tuple[float, float]]:
        """
        Returns the estimated positions of landmarks based on the particles.
        This function computes a weighted average of landmark positions across all particles.

        :return: List of estimated landmark positions as (x, y) tuples.
        """
        landmark_positions = {}

        for particle in self.particles:
            for landmark in particle.landmarks:
                landmark_position = (landmark.x, landmark.y)
                if landmark_position in landmark_positions:
                    # If landmark already exists, add the weighted position
                    landmark_positions[landmark_position].append((landmark.x, landmark.y, particle.weight))
                else:
                    # Initialize with the current landmark and its particle weight
                    landmark_positions[landmark_position] = [(landmark.x, landmark.y, particle.weight)]

        # Average positions of landmarks across all particles
        estimated_landmarks = []
        for landmark_position, positions in landmark_positions.items():
            total_weight = sum(weight for _, _, weight in positions)
            if total_weight == 0:
                continue

            avg_x = sum(x * weight for x, _, weight in positions) / total_weight
            avg_y = sum(y * weight for _, y, weight in positions) / total_weight
            estimated_landmarks.append((avg_x, avg_y))

        return estimated_landmarks

    def _init_particles(self, particle_amount: int, area_size: float) -> List[Particle]:
        particles = []
        for i in range(particle_amount):
            x, y = [np.random.uniform(-area_size / 2, area_size / 2), np.random.uniform(-area_size / 2, area_size / 2)]
            heading = np.random.uniform(-np.pi, np.pi)
            particles.append(Particle(Pose(x, y, heading)))
        return particles

    def _motion_model(self, particle_pose: Pose, velocity: float, angular_velocity: float, time_step: float) -> Pose:
        """
        Predicts the next state (pose) of a particle given its current pose.

        :param particle_pose: current position of the particle.
        :param velocity: velocity of the robot.
        :param angular_velocity: angular velocity of the robot.
        :param time_step: time since the last calculation.
        :param velocity_standard_deviation: standard deviation of the gaussian noise on the velocity.
        :param angular_velocity_standard_deviation: standard deviation of the gaussian noise on the angular velocity.
        """       
        # Transforms the control inputs (velocity, angular velocity) into position changes.
        B = np.array([[time_step * np.cos(particle_pose.heading), 0],  
                    [time_step * np.sin(particle_pose.heading), 0], 
                    [0, time_step]])  
        
        # Extract the current position and orientation (x, y, heading) from the given particle_pose object
        position_current = np.array([particle_pose.x, particle_pose.y, particle_pose.heading]).T
        
        # Includes velocity and angular velocity, with Gaussian noise added.
        U = np.array([velocity + np.random.normal(0, self._config.velocity_standard_deviation), angular_velocity + np.random.normal(0, self._config.angular_velocity_standard_deviation)]).T  
        
        position_predicted = np.eye(3) @ position_current + B @ U
        position_predicted[2] = pi2pi(position_predicted[2])

        return Pose(position_predicted[0], position_predicted[1], position_predicted[2])

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
        :param landmark_measurements: List of measured landmarks (distance, angle in radians)
        :return: Updated list of particles
        """
        counter = 0
        for particle in particles:
            for landmark_measurement in landmark_measurements:
                matched_landmark = particle.match_landmark(landmark_measurement, self._config.distance_threshold)
                
                # else:
                #     particle.weight *= compute_weight(particle, matched_landmark, landmark_measurement, self._config.measurement_covariance)
                #     matched_landmark.update(particle, landmark_measurement, self._config.measurement_covariance)
                #     pass
        
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
    
    def visualize(self):
        size = 800
        scale = 50
        sigma_scale = 10
        image = np.zeros((size, size, 3), dtype=np.uint8)

        for particle in self.particles:
            particle_point = (int(particle.pose.y * scale + size / 2), 
                              int(particle.pose.x * scale + size / 2))

            heading_vector = (int(np.sin(particle.pose.heading) * 20), 
                              int(np.cos(particle.pose.heading) * 20))

            particle_heading_point = (particle_point[0] + heading_vector[0],
                                      particle_point[1] + heading_vector[1])

            cv2.line(image, particle_point, particle_heading_point, (0, 255, 0), 2)
            cv2.circle(image, particle_point, 5, (0, 0, 255), -1)

            for landmark in particle.landmarks:
                landmark_point = (int(landmark.y * scale + size / 2), 
                                  int(landmark.x * scale + size / 2))
                cv2.ellipse(image, landmark_point, (5, 5), 0, 0, 360, (255, 0, 0), 2)

        cv2.imshow("FastSLAM", image)
        cv2.waitKey(1)
        return


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
