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
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'waffle_pi')

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    launch_file_dir = get_package_share_directory('aws_robomaker_small_house_world')
    small_house_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'launch', 'small_house.launch.py'))
    )

    turtlebot3_description_reduced_mesh = get_package_share_directory('turtlebot3_description_reduced_mesh')
    turtlebot3_description_reduced_mesh_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_description_reduced_mesh, 'launch', 'spawn_turtlebot.launch.py'))
    )
    turtlebot3_bringup_launch_file_dir = get_package_share_directory('turtlebot3_bringup')
    turtlebot3_state_publisher_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_bringup_launch_file_dir, 'launch', 'turtlebot3_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    nav_launch_dir = get_package_share_directory('aws_robomaker_small_house_world')
    navigation2_node = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(nav_launch_dir, 'launch', 'turtlebot3_navigation.launch.py'))
    )

    return LaunchDescription([
        small_house_launch,
        turtlebot3_description_reduced_mesh_launch,
        turtlebot3_state_publisher_launch,
        navigation2_node
    ])
