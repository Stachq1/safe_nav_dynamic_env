import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import Pose, Point, Vector3
from mppi_controller.obstacle import Obstacle
from ellipsoid_msgs.msg import EllipsoidArray, Ellipsoid
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

class MPPIControllerSim(Node):
    def __init__(self):
        super().__init__('mppi_controller_sim')
        self.get_logger().info('MPPIControllerSim node has started')
        self.marker_publisher_ = self.create_publisher(Marker, '/robot_state', 10)
        self.obstacle_subscriber_ = self.create_subscription(EllipsoidArray, '/obstacles', self.obstacle_callback, 10)

        # Parameter handling
        self.declare_parameter('dt', 0.1)
        self.declare_parameter('horizon', 20)
        self.declare_parameter('num_samples', 10000)
        self.declare_parameter('cost_weights.control_cost_weight', 1.0)
        self.declare_parameter('cost_weights.goal_cost_weight', 2.0)
        self.declare_parameter('cost_weights.terminal_goal_cost_weight', 3.0)
        self.declare_parameter('cost_weights.obstacle_cost_weight', 9.0)

        self.dt = self.get_parameter('dt').value
        self.horizon = self.get_parameter('horizon').value
        self.num_samples = self.get_parameter('num_samples').value
        self.cost_weights = {
            'control': self.get_parameter('cost_weights.control_cost_weight').value,
            'goal': self.get_parameter('cost_weights.goal_cost_weight').value,
            'terminal_goal': self.get_parameter('cost_weights.terminal_goal_cost_weight').value,
            'obstacle': self.get_parameter('cost_weights.obstacle_cost_weight').value,
        }

        # Initialize the timer for updating the state
        self.timer = self.create_timer(self.dt, self.update_state)

        self.curr_state = np.array([-5.0, 5.0, 0.0])
        self.goal = np.array([5.0, -5.0, 0.0])
        self.obstacles = []
        self.prev_controls = np.random.normal(0, 1.0, size=(self.horizon, 2))
        self.num_collisions = 0

    def obstacle_callback(self, msg: EllipsoidArray):
        self.obstacles = []
        for obs_msg in msg.ellipsoids:
            # Convert each ellipsoid message into an ellipsoid object
            self.obstacles.append(Obstacle(obs_msg))
        return

    def dynamics(self, state, control):
        state[:, 0] = state[:, 0] + control[:, 0] * np.cos(state[:, 2]) * self.dt       # x = x + v * cos(theta) * dt
        state[:, 1] = state[:, 1] + control[:, 0] * np.sin(state[:, 2]) * self.dt       # y = y + v * sin(theta) * dt
        state[:, 2] = state[:, 2] + control[:, 1] * self.dt                             # theta = theta + w * dt
        return state

    def sample_trajectories(self):
        # Initialize the trajectories with the initial state
        trajectories = np.zeros((self.num_samples, self.horizon + 1, 3))
        trajectories[:, 0, :] = self.curr_state

        # Compute the controls by taking previous controls, shifting it in time and adding Gaussian noise
        controls = np.zeros((self.num_samples, self.horizon, 2))
        controls[:, :-1 :] = self.prev_controls[1:, :]
        controls[:, -1, :] = self.prev_controls[-1, :]  # Repeat the last control for the last time step

        delta_controls = np.zeros((self.num_samples, self.horizon, 2))
        delta_controls[:, :, 0] = np.random.normal(0, 0.2, size=(self.num_samples, self.horizon))
        delta_controls[:, :, 1] = np.random.normal(0, 0.1, size=(self.num_samples, self.horizon))

        controls += delta_controls

        # Perform the trajectory sampling over the horizon
        for t in range(self.horizon):
            # Compute the new states for all samples based on the controls and update trajectories
            trajectories[:, t + 1, :] = self.dynamics(trajectories[:, t, :], controls[:, t, :])
        return trajectories, controls

    def cost_function(self, trajectories, controls, obstacles):
        # Goal Cost: Euclidean distance from all trajectory steps (except last one) to the goal
        goal_costs = self.cost_weights['goal'] * np.sum(np.linalg.norm(trajectories[:, :-1, :2] - self.goal[:2], axis=2), axis=1)

        # Terminal Goal Cost: Euclidean distance from the last state in trajectory to the goal
        terminal_goal_costs = self.cost_weights['terminal_goal'] * np.linalg.norm(trajectories[:, -1, :2] - self.goal[:2], axis=1)

        # Obstacle Cost: Repulsive cost for each trajectory based on proximity to each obstacle
        obstacle_costs = np.zeros(trajectories.shape[0])  # Shape (num_samples,)
        for obs in obstacles:
            obstacle_costs += obs.compute_dynamic_obstacle_cost(trajectories, self.horizon, self.dt)
        obstacle_costs *= self.cost_weights['obstacle']

        # Control Cost: L2 norm of the control commands
        control_costs = self.cost_weights['control'] * np.sum(np.square(controls), axis=(1, 2))  # Shape (num_samples,)

        return goal_costs + terminal_goal_costs + obstacle_costs + control_costs

    def select_best_trajectory(self, trajectories, controls, obstacles):
        costs = self.cost_function(trajectories, controls, obstacles)
        best_index = np.argmin(costs)
        return trajectories[best_index, :, :], controls[best_index, :, :]

    def at_goal(self):
        return np.linalg.norm(self.curr_state[:2] - self.goal[:2]) < 0.1

    def visualize_robot_and_goal(self):
        # Create and initialize the Marker for the robot (a small sphere)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        # Visualize the robot as a red sphere
        robot_marker = Marker(
            header=header,
            ns="robot",
            id=0,
            type=Marker.SPHERE,
            action=Marker.ADD,
            pose=Pose(position=Point(x=self.curr_state[0], y=self.curr_state[1], z=0.0)),
            scale=Vector3(x=0.1, y=0.1, z=0.1),
            color=ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        )

        # Visualize the goal as a blue sphere (or any other color/shape)
        goal_marker = Marker(
            header=header,
            ns="goal",
            id=1,
            type=Marker.SPHERE,
            action=Marker.ADD,
            pose=Pose(position=Point(x=self.goal[0], y=self.goal[1], z=0.0)),
            scale=Vector3(x=0.1, y=0.1, z=0.1),
            color=ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        )

        # Publish both the robot and goal markers
        self.marker_publisher_.publish(robot_marker)
        self.marker_publisher_.publish(goal_marker)

    def visualize_trajectory(self, trajectory):
        # Create and initialize the Marker for the trajectory (line strip)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "map"

        marker = Marker(
            header=header,
            ns="trajectory",
            id=2,
            type=Marker.LINE_STRIP,
            action=Marker.ADD,
            scale=Vector3(x=0.05, y=0.05, z=0.05),  # Thickness of the line
            color=ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)  # Blue color for the trajectory
        )

        # Add points from the trajectory (z = 0 for 2D)
        marker.points = [Point(x=state[0], y=state[1], z=0.0) for state in trajectory]

        # Publish the trajectory marker
        self.marker_publisher_.publish(marker)

    def update_state(self):
        # Get current obstacles (create a deep copy)
        obstacles = np.copy(self.obstacles)

        # Check if there are currently any collisions
        for obs in obstacles:
            if obs.check_collision(self.curr_state[:2]):
                self.num_collisions += 1
                self.get_logger().warn(f"Collision detected!")

        # Sample trajectories
        trajectories, controls = self.sample_trajectories()
        best_trajectory, best_controls = self.select_best_trajectory(trajectories, controls, obstacles)

        # Update the previous control for the next iteration
        self.prev_controls = best_controls

        # Clip the controls to be within the limits of SPOT
        best_controls[0, 0] = np.clip(best_controls[0, 0], -1.0, 1.0)
        best_controls[0, 1] = np.clip(best_controls[0, 1], -0.5, 0.5)

        # Compute the next state (tweak dimensions to work with vectorized function)
        self.curr_state = self.dynamics(self.curr_state.reshape(1, -1), best_controls[0, :].reshape(1, -1))
        self.curr_state = self.curr_state.flatten()

        # Apply random disturbance to position (x, y)
        disturbance = np.random.normal(0, 0.001, size=self.curr_state.shape)
        self.curr_state = self.curr_state + disturbance

        # Visualize the state
        self.visualize_robot_and_goal()
        self.visualize_trajectory(best_trajectory)

        if self.at_goal() and rclpy.ok():  # Use rclpy.ok() to check if ROS 2 is still running
            self.get_logger().warn(f"Total collisions: {self.num_collisions}")
            rclpy.shutdown()  # Use rclpy.shutdown() to shutdown the ROS 2 node
