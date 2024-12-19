import numpy as np
from ellipsoid_msgs.msg import Ellipsoid

class Obstacle:
    def __init__(self, msg: Ellipsoid, enlargement_radius=0.25):
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

    def compute_dynamic_obstacle_cost(self, trajectories, horizon, dt, cutoff_distance=0.4):
        distances = np.linalg.norm(trajectories[:, :, :2] - self.get_trajectory(horizon, dt), axis=2)
        masked_distances = np.where(distances >= cutoff_distance, np.inf, distances)
        return np.sum(np.exp(-masked_distances), axis=1)
