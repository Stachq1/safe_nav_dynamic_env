import rclpy
from rclpy.node import Node
import numpy as np
import time

from geometry_msgs.msg import Pose, Point, Vector3, Twist
from mppi_controller.obstacle import Obstacle
from nav_msgs.msg import Odometry
from ellipsoid_msgs.msg import EllipsoidArray, Ellipsoid
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker

# SPOT
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand
from bosdyn.client.time_sync import TimeSyncClient, TimeSyncThread

class MPPIController(Node):
    def __init__(self, robot):
        super().__init__('mppi_controller')
        self.get_logger().info('MPPIController node has started')

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

        # Ensure time synchronization with SPOT and create command client
        self.robot = robot
        self.robot_command_client = self.robot.ensure_client(RobotCommandClient.default_service_name)
        self.time_sync_client = self.robot.ensure_client(TimeSyncClient.default_service_name)
        self.time_sync_thread = TimeSyncThread(self.time_sync_client)
        self.time_sync_thread.start()
        self.time_sync_thread.wait_for_sync()

        # Power on robot and stand up
        self.robot.power_on(timeout_sec=20)
        blocking_stand(self.robot_command_client, timeout_sec=10)

        # Initialize ROS publishers and subscribers
        self.robot_state_vis_publisher_ = self.create_publisher(Marker, '/robot_state', 10)
        self.robot_goal_vis_publisher_ = self.create_publisher(Marker, '/robot_goal', 10)
        self.robot_traj_vis_publisher_ = self.create_publisher(Marker, '/robot_trajectory', 10)
        self.control_publisher = self.create_publisher(Twist, '/best_controls', 10)
        self.odometry_subscriber = self.create_subscription(Odometry, '/Odometry', self.odometry_callback, 10)
        self.obstacle_subscriber_ = self.create_subscription(EllipsoidArray, '/obstacles', self.obstacle_callback, 10)
        self.timer = self.create_timer(self.dt, self.update_state)

        # Initialize the current state, goal, obstacles and previous controls
        self.curr_state = np.array([])
        self.goal = np.array([])
        self.obstacles = []
        self.prev_controls = np.random.normal(0, 0.25, size=(self.horizon, 2))

    def obstacle_callback(self, msg: EllipsoidArray):
        self.obstacles = []
        for obs_msg in msg.ellipsoids:
            # Convert each ellipsoid message into an ellipsoid object
            self.obstacles.append(Obstacle(obs_msg))
        return

    def quaternion_to_euler(self, quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y ** 2 + z ** 2))
        return yaw

    def odometry_callback(self, msg: Odometry):
        # Extract the position and orientation from the Odometry message (according to model frame)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        # Convert the quaternion to yaw angle
        yaw = self.quaternion_to_euler(orientation)
        self.curr_state = np.array([x, y, yaw])
        if self.goal.size == 0:
            self.goal = np.array([x, y, yaw])
        return

    def dynamics(self, state, control):
        state[:, 0] = state[:, 0] + control[:, 0] * np.cos(state[:, 2]) * self.dt       # x = x + v * cos(theta) * dt
        state[:, 1] = state[:, 1] + control[:, 0] * np.sin(state[:, 2]) * self.dt       # y = y + v * sin(theta) * dt
        state[:, 2] = state[:, 2] + control[:, 1] * self.dt                             # theta = theta + w * dt
        return state

    def sample_trajectories(self, state):
        # Initialize the trajectories with the initial state
        trajectories = np.zeros((self.num_samples, self.horizon + 1, 3))
        trajectories[:, 0, :] = state

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

    def visualize_robot_and_goal(self, state):
        # Create and initialize the Marker for the robot (a small sphere)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_init"

        # Visualize the robot as a red sphere
        robot_marker = Marker(
            header=header,
            ns="robot",
            id=0,
            type=Marker.SPHERE,
            action=Marker.ADD,
            pose=Pose(position=Point(x=state[0], y=state[1], z=0.0)),
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
            pose=Pose(position=Point(x=-self.goal[0], y=self.goal[1], z=0.0)),
            scale=Vector3(x=0.1, y=0.1, z=0.1),
            color=ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
        )

        # Publish both the robot and goal markers
        self.robot_state_vis_publisher_.publish(robot_marker)
        self.robot_goal_vis_publisher_.publish(goal_marker)

    def visualize_trajectory(self, trajectory):
        # Create and initialize the Marker for the trajectory (line strip)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera_init"

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
        self.robot_traj_vis_publisher_.publish(marker)

    def update_state(self):
        # Wait for the current state and goal to be initialized
        if self.curr_state.size == 0 or self.goal.size == 0:
            return

        # Get the state of the SPOT (create a deep copy)
        state = np.copy(self.curr_state)

        # Get current obstacles (create a deep copy)
        obstacles = np.copy(self.obstacles)

        # Sample trajectories
        trajectories, controls = self.sample_trajectories(state)
        best_trajectory, best_controls = self.select_best_trajectory(trajectories, controls, obstacles)

        # Update the previous control for the next iteration
        self.prev_controls = best_controls

        # Clamp the computed controls to the SPOT's velocity limits
        best_controls[0, 0] = np.clip(best_controls[0, 0], -1.0, 1.0)
        best_controls[0, 1] = np.clip(best_controls[0, 1], -0.5, 0.5)

        # Record the controls in ROSbag
        msg = Twist()
        msg.linear.x = best_controls[0, 0]
        msg.angular.z = best_controls[0, 1]
        self.control_publisher.publish(msg)

        # Send control command to the SPOT
        command = RobotCommandBuilder.synchro_velocity_command(v_x=best_controls[0, 0], v_y=0.0, v_rot=best_controls[0, 1])
        end_time_secs = time.time() + self.dt + 0.1 # Ensure the command is executed for the duration of dt + 0.1s as buffer
        self.robot_command_client.robot_command(command=command, end_time_secs=end_time_secs)

        # Visualize the state
        self.visualize_robot_and_goal(state)
        self.visualize_trajectory(best_trajectory)

        # if self.at_goal() and rclpy.ok():
            # If at goal, sit down and shutdown the node
            # self.robot_command_client.robot_command(RobotCommandBuilder.synchro_sit_command())
            # rclpy.shutdown()
