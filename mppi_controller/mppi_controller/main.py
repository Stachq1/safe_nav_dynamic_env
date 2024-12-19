import rclpy
from mppi_controller import MPPIController
from mppi_controller import MPPIControllerSim

import bosdyn.client.util
from bosdyn.client import create_standard_sdk
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive

def main(args=None):
    rclpy.init(args=args)

    # Check if we should simulate the robot
    param_node = rclpy.Node('')

    param_node.declare_parameter('sim', True)
    sim = param_node.get_parameter('sim').value
    param_node.get_logger().info(f"Sim: {sim}")

    param_node.destroy_node()

    if not sim:
        sdk = create_standard_sdk('MPPIController')
        robot = sdk.create_robot('192.168.50.3')
        bosdyn.client.util.authenticate(robot)
        lease_client = robot.ensure_client(LeaseClient.default_service_name)
        mppi_node = MPPIController(robot)
        with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
            rclpy.spin(mppi_node)
    else:
        mppi_node = MPPIControllerSim(None)
        rclpy.spin(mppi_node)

    # When finished, destroy the nodes
    mppi_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
