from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
  play_rosbag_flag = DeclareLaunchArgument(
    'play_rosbag',
    default_value='false',
    description='Flag to determine if the rosbag should be played'
  )

  config_file = os.path.join(
    get_package_share_directory('dynablox_ros'),
    'config',
    'motion_detector',
    'default.yaml'
  )

  play_dynablox_data_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(
        get_package_share_directory('dynablox_ros'),
        'launch',
        'play_dynablox_data.py'
      )
    ),
    launch_arguments={
      'bag_file': '/root/rosbags/indoor/indoor.db3', 
      'player_rate': '1'
    }.items(),
    condition=IfCondition(LaunchConfiguration('play_rosbag'))
  )

  motion_detector_node = Node(
      package='dynablox_ros',
      executable='motion_detector_node',
      name='motion_detector_node',
      output='screen',
      arguments=['--alsologtostderr'],
      parameters=[config_file]
  )

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='rviz'
  )

  return LaunchDescription([
    play_rosbag_flag,
    play_dynablox_data_launch,
    motion_detector_node,
    rviz_node
  ])
