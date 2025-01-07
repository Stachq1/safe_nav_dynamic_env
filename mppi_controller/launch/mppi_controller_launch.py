import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('mppi_controller'),
        'config',
        'default.yaml'
    )

    # List of topics to record
    topics_to_record = [
        '/mppi_visualization',
        '/Odometry',
        '/obstacles',
        '/best_controls',
        '/ellipsoids',
        '/detections/cluster/dynamic',
        '/clusters'
    ]

    return LaunchDescription([
        # Launch the mppi_controller node
        Node(
            package='mppi_controller',
            executable='mppi_controller_node',
            name='mppi_controller',
            output='screen',
            parameters=[config_file]
        ),

        # Launch the rosbag recorder for the specified topics
        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', '-o', 'mppi_bag'] + topics_to_record,
            output='screen'
        )
    ])
