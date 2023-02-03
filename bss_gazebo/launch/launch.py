#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Darby Lim

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import xacro
import yaml


# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    #Gazebo World Path
    world_file_name = 'nobook.world'
    world = os.path.join(get_package_share_directory('bss_gazebo'),
                         'worlds', world_file_name)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    #Set position and orientation of sdf
    position = [2.00, -0.50, 0.00]
    orientation = [0.00, 0.00, 0.00]

    # ***** ROBOT DESCRIPTION ***** #
    # UR10 Description file package:
    ur10_description_path = os.path.join(
    get_package_share_directory('ur10_ros2_gazebo'))
    # UR10 ROBOT urdf file path:
    xacro_file = os.path.join(ur10_description_path,
                              'urdf',
                              'ur10.urdf.xacro')

    # Generate ROBOT_DESCRIPTION for UR10:
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),

        Node(package='gazebo_ros', 
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=['-topic', 'robot_description','-entity', 'ur10',
            "-x", str(position[0]), "-y", str(position[1]), "-z", str(position[2]),
            "-R",str(orientation[0]), "-P",str(orientation[1]),"-Y",str(orientation[2])
            ]
            )
    ])
