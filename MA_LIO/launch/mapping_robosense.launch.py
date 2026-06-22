import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('ma_lio')
    default_config = os.path.join(pkg_share, 'config', 'RoboSense.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz_cfg', 'ma_lio.rviz')

    config_file = LaunchConfiguration('config_file')
    use_rviz = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config,
                              description='Path to the ma_lio params YAML'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('rviz_cfg', default_value=default_rviz),

        Node(
            package='ma_lio',
            executable='malio_mapping',
            name='laser_mapping',
            output='screen',
            parameters=[config_file],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_cfg],
            condition=IfCondition(use_rviz),
        ),
    ])
