import rclpy
from mppi_controller import MPPIController

import bosdyn.client.util
from bosdyn.client import create_standard_sdk
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive

def main(args=None):
    rclpy.init(args=args)

    # Check if we should simulate the robot
    param_node = rclpy.Node('')

    param_node.declare_parameter('parameter_name_1')
    sim = param_node.get_parameter('parameter_name_1').value
    param_node.get_logger().info(f"Sim: {sim}")

    param_node.destroy_node()

    if not sim:
        sdk = create_standard_sdk('MPPIController')
        robot = sdk.create_robot('192.168.50.3')
        bosdyn.client.util.authenticate(robot)
        lease_client = robot.ensure_client(LeaseClient.default_service_name)

    # Create both nodes
    mppi_node = MPPIController()

    # Spin the node with a lease keep alive
    if not sim:
        with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            rclpy.spin(mppi_node)
    else:
        rclpy.spin(mppi_node)

    # When finished, destroy the nodes
    mppi_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
