import rclpy
from mppi_controller import MPPIController

def main(args=None):
    rclpy.init(args=args)

    # Create both nodes
    mppi_node = MPPIController()

    # Spin the nodes in parallel
    rclpy.spin(mppi_node)

    # When finished, destroy the nodes
    mppi_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
