import numpy as np

from geometry_msgs.msg import Pose, Point, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

class Obstacle:
    def __init__(self, start_pos, end_pos, vel, marker_publisher):
        # Initialize the points that the obstacle will move around
        self.start_pos = start_pos
        self.end_pos = end_pos

        # Initialize the current position of the obstacle and its velocity
        self.curr_pos = (start_pos + end_pos) / 2
        self.vel = vel * (end_pos - start_pos) / np.linalg.norm(end_pos - start_pos)

        # Generate on ID for each obstacle
        self.id = np.random.randint(10000)     # TODO: can get repeated!

        # Initialize the marker publisher
        self.marker_publisher_ = marker_publisher

    def move(self, dt):
        # Move the obstacle based on its velocity
        self.curr_pos += self.vel * dt

        # If the obstacle reaches the start/end position, change its direction
        if np.linalg.norm(self.curr_pos - self.end_pos) < 0.1:
            self.vel *= -1
        elif np.linalg.norm(self.curr_pos - self.start_pos) < 0.1:
            self.vel *= -1

    def get_position(self) -> np.ndarray:
        return self.curr_pos

    def get_trajectory(self, horizon, dt) -> np.ndarray:
        trajectory = np.zeros((horizon + 1, 2))
        curr_pos = np.copy(self.curr_pos)
        vel = np.copy(self.vel)
        trajectory[0, :] = curr_pos

        for i in range(horizon):
            # Append the current position to the trajectory list
            trajectory[i + 1, :] = curr_pos

            # Update the position based on velocity and time step
            curr_pos += vel * dt

            # Reverse direction if the obstacle reaches either the start or end position
            if np.linalg.norm(curr_pos - self.end_pos) < 0.1:
                vel *= -1
            elif np.linalg.norm(curr_pos - self.start_pos) < 0.1:
                vel *= -1

        return trajectory

    def compute_static_obstacle_cost(self, trajectories, cutoff_distance=0.4):
        distances = np.linalg.norm(trajectories[:, :, :2] - self.get_position(), axis=2)  # Shape (num_samples, horizon + 1)
        masked_distances = np.where(distances >= cutoff_distance, np.inf, distances)
        return np.sum(np.exp(-masked_distances), axis=1)  # Sum over horizon

    def compute_dynamic_obstacle_cost(self, trajectories, horizon, dt, cutoff_distance=0.4):
        distances = np.linalg.norm(trajectories[:, :, :2] - self.get_trajectory(horizon, dt), axis=2)  # Shape (num_samples, horizon + 1)
        masked_distances = np.where(distances >= cutoff_distance, np.inf, distances)
        return np.sum(np.exp(-masked_distances), axis=1)  # Sum over horizon

    def visualize_obstacle(self, stamp):
        # Create and publish a Marker for each obstacle
        header = Header()
        header.stamp = stamp
        header.frame_id = "map"

        marker = Marker(
            header=header,
            ns="obstacle",
            id=self.id,
            type=Marker.SPHERE,
            action=Marker.ADD,
            pose=Pose(position=Point(x=self.curr_pos[0], y=self.curr_pos[1], z=0.0)),
            scale=Vector3(x=0.3, y=0.3, z=0.3),
            color=ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        )

        # Publish the obstacle marker
        self.marker_publisher_.publish(marker)
