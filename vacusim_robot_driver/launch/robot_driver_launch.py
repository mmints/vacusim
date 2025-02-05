'''
Author: Mark O. Mints (mmints@uni-koblenz.de)
'''

import os
import launch
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

from webots_ros2_driver.webots_controller import WebotsController

def generate_launch_description():
    package_dir = get_package_share_directory('vacusim_robot_driver')
    robot_description_path = os.path.join(package_dir, 'resource', 'robot.urdf')

    robot_driver = WebotsController(
        robot_name='Custom_iRobot_Create',
        parameters=[
            {'robot_description': robot_description_path},
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
