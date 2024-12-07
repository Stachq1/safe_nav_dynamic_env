import rclpy
from rclpy.node import Node
import numpy as np
from mppi_controller.obstacle import Obstacle

from geometry_msgs.msg import Pose, Point, Twist, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

class MPPIController(Node):
    def __init__(self):
        super().__init__('mppi_controller')
        self.get_logger().info('MPPIController node has started')
        self.twist_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_publisher_ = self.create_publisher(Marker, '/mppi_visualization', 10)
        self.timer = self.create_timer(0.3, self.update_state)

        self.num_samples = 10000
        self.horizon = 40
        self.dt = 0.05

        self.curr_state = np.array([0.0, 0.0, 0.0])                                                # Starting position
        self.goal = np.array([5.0, 5.0, 0.0])                                                      # Goal position
        self.obstacles = [Obstacle(np.array([1.0, 2.0]), np.array([4.0, 2.0]), 0.4, self.marker_publisher_),
                          Obstacle(np.array([1.0, 4.0]), np.array([4.0, 3.0]), 0.5, self.marker_publisher_),
                          Obstacle(np.array([5.0, 4.0]), np.array([4.0, 5.0]), 1.0, self.marker_publisher_)]
        self.prev_controls = np.random.normal(0, 1.0, size=(self.horizon, 2))    # Previous control commands

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

    def cost_function(self, trajectories, controls, control_cost_weight=1.0, goal_cost_weight=3.0, terminal_goal_cost_weight=6.0, obstacle_cost_weight=1.5):
        # Goal Cost: Euclidean distance from all trajectory steps (except last one) to the goal
        goal_costs = goal_cost_weight * np.sum(np.linalg.norm(trajectories[:, :-1, :2] - self.goal[:2], axis=2), axis=1)

        # Terminal Goal Cost: Euclidean distance from the last state in trajectory to the goal
        terminal_goal_costs = terminal_goal_cost_weight * np.linalg.norm(trajectories[:, -1, :2] - self.goal[:2], axis=1)

        # Obstacle Cost: Repulsive cost for each trajectory based on proximity to each obstacle
        obstacle_costs = np.zeros(trajectories.shape[0])  # Shape (num_samples,)
        for obs in self.obstacles:
            obstacle_costs += obs.compute_dynamic_obstacle_cost(trajectories, self.horizon, self.dt)
        obstacle_costs *= obstacle_cost_weight

        # Control Cost: L2 norm of the control commands
        control_costs = control_cost_weight * np.sum(np.square(controls), axis=(1, 2))  # Shape (num_samples,)

        return goal_costs + terminal_goal_costs + obstacle_costs + control_costs

    def select_best_trajectory(self, trajectories, controls):
        costs = self.cost_function(trajectories, controls)
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
        # Sample trajectories
        trajectories, controls = self.sample_trajectories()
        best_trajectory, best_controls = self.select_best_trajectory(trajectories, controls)

        # Update the previous control for the next iteration
        self.prev_controls = best_controls

        # Send control command
        cmd = Twist()
        cmd.linear.x = best_controls[0, 0]
        cmd.angular.z = best_controls[0, 1]
        self.twist_publisher_.publish(cmd)

        # Compute the next state (tweak dimensions to work with vectorized function)
        self.curr_state = self.dynamics(self.curr_state.reshape(1, -1), best_controls[0, :].reshape(1, -1))
        self.curr_state = self.curr_state.flatten()

        # Apply random disturbance to position (x, y)
        # disturbance = np.random.normal(0, 0.001, size=self.curr_state.shape)
        # self.curr_state = self.curr_state + disturbance

        # Visualize the state
        self.visualize_robot_and_goal()
        self.visualize_trajectory(best_trajectory)

        # Move the obstacles and visualize them
        for obs in self.obstacles:
            obs.move(self.dt)
            obs.visualize_obstacle(self.get_clock().now().to_msg())

        if self.at_goal() and rclpy.ok():  # Use rclpy.ok() to check if ROS 2 is still running
            rclpy.shutdown()  # Use rclpy.shutdown() to shutdown the ROS 2 node
