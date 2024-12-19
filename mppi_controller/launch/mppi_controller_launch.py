import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('mppi_controller'),
        'config',
        'default.yaml'
    )

    return LaunchDescription([
        Node(
            package='mppi_controller',
            executable='mppi_controller_node',
            name='mppi_controller',
            output='screen',
            parameters=[config_file]
        )
    ])
