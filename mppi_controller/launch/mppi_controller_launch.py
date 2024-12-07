import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node for the MPPIController
        Node(
            package='mppi_controller',
            executable='mppi_controller_node',
            name='mppi_controller',
            output='screen',
            parameters=[]
        )
    ])
