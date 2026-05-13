#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('vacusim_robot_planning')
    default_map_path = os.path.join(package_share, 'maps', 'default_map.txt')

    map_path_argument = DeclareLaunchArgument(
        'map_path',
        default_value=default_map_path,
        description='Numeric map file used by the A* planner',
    )

    planner_node = Node(
        package='vacusim_robot_planning',
        executable='planner_node',
        name='vacusim_planner',
        output='screen',
        parameters=[{
            'map_path': LaunchConfiguration('map_path'),
        }],
    )

    return LaunchDescription([
        map_path_argument,
        planner_node,
    ])
