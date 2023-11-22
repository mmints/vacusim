'''
Author: Mark O. Mints (mmints@uni-koblenz.de)
'''

import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.utils import controller_url_prefix


def generate_launch_description():
    package_dir = get_package_share_directory('vacusim_robot_driver')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'robot.urdf')).read_text()

    robot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'Custom_iRobot_Create'},
        parameters=[
            {'robot_description': robot_description},
        ]
    )

    return LaunchDescription([
        robot_driver,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])