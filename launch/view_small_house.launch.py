import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription([
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [get_package_share_directory(
                    'aws_robomaker_small_house_world'), '/launch/small_house.launch.py']
            ),
            launch_arguments={
                'gui': 'true'
            }.items()
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
