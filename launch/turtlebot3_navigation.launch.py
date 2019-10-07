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
import shutil
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']
MAP_PATH = os.environ['HOME']

# Below changes (line no 33 to 46 and 52) are added as a workaround for long path for map file.
# This change is required for testing on Robomaker.
pkg_dir = get_package_share_directory('aws_robomaker_small_house_world')
src_path = os.path.join(pkg_dir, 'maps', 'turtlebot3_waffle_pi')

print("Source path = ", src_path)

directory = '/tmp/maps'
if not os.path.exists(directory):
    os.makedirs(directory)

src_files = os.listdir(src_path)
for file_name in src_files:
    full_file_name = os.path.join(src_path, file_name)
    if os.path.isfile(full_file_name):
        shutil.copy(full_file_name, directory)

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')

    map_path = os.path.join('/tmp/maps/'
                            'map.yaml')
    map_dir = LaunchConfiguration('map',
                                  default=map_path)
    param_file_name = TURTLEBOT3_MODEL + '.yaml'
    param_dir = LaunchConfiguration(
        'params',
        default=os.path.join(
            get_package_share_directory('aws_robomaker_small_house_world'),
            'param',
            param_file_name))
    nav_launch_dir = get_package_share_directory('turtlebot3_navigation2')
    navigation2_node = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(nav_launch_dir, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={'map': map_dir, 'use_sim_time': use_sim_time, 'params': param_dir}.items()
    )

    return LaunchDescription([
        navigation2_node
    ])
