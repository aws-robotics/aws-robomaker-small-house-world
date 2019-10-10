# /*******************************************************************************
# * Copyright 2019 ROBOTIS CO., LTD.
# *
# * Licensed under the Apache License, Version 2.0 (the "License");
# * you may not use this file except in compliance with the License.
# * You may obtain a copy of the License at
# *
# *     http://www.apache.org/licenses/LICENSE-2.0
# *
# * Unless required by applicable law or agreed to in writing, software
# * distributed under the License is distributed on an "AS IS" BASIS,
# * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# * See the License for the specific language governing permissions and
# * limitations under the License.
# *******************************************************************************/

# /* Author: Darby Lim */

import os
import sys

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
    turtlebot_urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    turtlebot_urdf_file_path = os.path.join(get_package_share_directory('turtlebot3_description_reduced_mesh'), 'urdf', turtlebot_urdf_file_name)
    x_pos = launch.substitutions.LaunchConfiguration('x_pos', default='3.5')
    y_pos = launch.substitutions.LaunchConfiguration('y_pos', default='1.0')
    yaw = launch.substitutions.LaunchConfiguration('yaw', default='0.0')

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            name='follow_route',
            default_value='true'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('aws_robomaker_small_house_world'), 
                    'launch', 
                    'small_house.launch.py'
                )
            )
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(
                    'turtlebot3_description_reduced_mesh'), 'launch', 'spawn_turtlebot.launch.py')
            ),
            launch_arguments={
                'gui': launch.substitutions.LaunchConfiguration('gui'),
                'gazebo_model_path': os.path.split(get_package_share_directory('turtlebot3_description_reduced_mesh'))[0],
                'x_pos': x_pos,
                'y_pos': y_pos,
                'yaw': yaw
            }.items()
        ),
       
        launch_ros.actions.Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')
            }],
            arguments=[
                turtlebot_urdf_file_path
            ]
        ),
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('aws_robomaker_small_house_world'), 
                    'launch', 
                    'turtlebot3_navigation.launch.py'
                )
            )
        )
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()
