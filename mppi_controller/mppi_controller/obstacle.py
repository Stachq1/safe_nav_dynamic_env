import numpy as np
from ellipsoid_msgs.msg import Ellipsoid

class Obstacle:
    def __init__(self, msg: Ellipsoid, enlargement_radius=0.6):
        """
        Initialize the Obstacle class by enlarging the ellipsoid.

        :param msg: The ROS 2 message containing the ellipsoid properties.
        :param enlargement_radius: The additional radius to enlarge the ellipsoid.
        """
        # Extract data from the message
        self.center = np.array([msg.center.x, msg.center.y])  # Extract the 2D center
        self.velocity = np.array([msg.velocity.x, msg.velocity.y])  # Extract the 2D velocity
        self.init_a_matrix = msg.a_matrix.reshape(2, 2)  # Convert flattened array to 2x2 matrix

        # Enlarge the ellipsoid
        self.enlarged_a_matrix = self.enlarge_ellipsoid(self.init_a_matrix, enlargement_radius)

    def enlarge_ellipsoid(self, a_matrix, enlargement_radius):
        """
        Enlarge the 2D ellipsoid represented by the a_matrix.

        :param a_matrix: The 2x2 matrix representing the original ellipsoid.
        :param enlargement_radius: The radius to enlarge the ellipsoid.
        :return: The enlarged 2x2 matrix.
        """
        # Decompose the matrix to extract the ellipse parameters
        eigenvalues, eigenvectors = np.linalg.eig(a_matrix)

        # Scale the eigenvalues to enlarge the ellipsoid
        scaled_eigenvalues = eigenvalues + enlargement_radius

        # Reconstruct the enlarged matrix
        enlarged_matrix = eigenvectors @ np.diag(scaled_eigenvalues) @ np.linalg.inv(eigenvectors)
        return enlarged_matrix

    def get_trajectory(self, horizon, dt):
        """
        Compute and return the trajectory of the obstacle over a given horizon.

        :param horizon: The number of steps to compute.
        :param dt: The time step size.
        :return: A list of 2D positions representing the trajectory.
        """
        time_steps = np.arange(horizon + 1) * dt
        trajectory = self.center + np.outer(time_steps, self.velocity)
        return trajectory

    def compute_dynamic_obstacle_cost(self, trajectories, horizon, dt, cutoff_distance=5.0):
        """
        Compute the cost of the robot's trajectories based on the distance to the ellipsoid boundary.
        :param trajectories: A 3D numpy array of robot trajectories (batch, time, 2D position).
        :param horizon: The number of steps in the horizon.
        :param dt: The time step size.
        :param cutoff_distance: The minimum distance for cost computation.
        :return: A 1D numpy array of costs for each trajectory.
        """
        # Get obstacle trajectory: (time, 2)
        obstacle_trajectory = self.get_trajectory(horizon, dt)

        # Relative positions: (batch, time, 2)
        relative_positions = trajectories[:, :, :2] - obstacle_trajectory[np.newaxis, :, :]

        # Transform positions to ellipsoid frame: (batch, time, 2)
        transformed_positions = relative_positions @ self.enlarged_a_matrix.T

        # Compute quadratic form p.T @ A @ p for each relative position: (batch, time)
        norm_squared = np.sum(relative_positions * transformed_positions, axis=2)

        # Compute distances to ellipsoid boundary
        # Outside ellipsoid: distance to surface
        # Inside ellipsoid: distance is zero
        scale_factors = np.sqrt(norm_squared)
        scale_factors[norm_squared <= 1] = 1  # No scaling for points inside the ellipsoid
        surface_positions = relative_positions / scale_factors[:, :, np.newaxis]  # Nearest points on ellipsoid surface
        distances = np.linalg.norm(relative_positions - surface_positions, axis=2)
        distances[norm_squared <= 1] = 0  # Zero distance for points inside the ellipsoid

        # Apply cutoff distance and compute costs
        masked_distances = np.where(distances >= cutoff_distance, np.inf, distances)
        costs = np.sum(np.exp(-masked_distances), axis=1)

        return costs

    def check_collision(self, position):
        """
        Check if a given position is inside the enlarged ellipsoid.

        :param position: The 2D position to check.
        :return: True if the position is inside the ellipsoid, False otherwise.
        """
        # Compute the relative position
        relative_position = position - self.center

        # Transform the position to the ellipsoid frame
        transformed_position = relative_position @ self.enlarged_a_matrix.T

        # Compute the quadratic form p.T @ A @ p
        norm_squared = np.sum(relative_position * transformed_position)

        # Check if the position is inside the ellipsoid
        return norm_squared <= 1
