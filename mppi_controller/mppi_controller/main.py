import rclpy
from mppi_controller import MPPIController

import bosdyn.client.util
from bosdyn.client import create_standard_sdk
from bosdyn.client.lease import LeaseClient, LeaseKeepAlive

def main(args=None):
    rclpy.init(args=args)

    sdk = create_standard_sdk('MPPIController')
    robot = sdk.create_robot('192.168.50.3')
    bosdyn.client.util.authenticate(robot)
    lease_client = robot.ensure_client(LeaseClient.default_service_name)

    # Create both nodes
    mppi_node = MPPIController()

    # Spin the node with a lease keep alive
    with LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        rclpy.spin(mppi_node)

    # When finished, destroy the nodes
    mppi_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
