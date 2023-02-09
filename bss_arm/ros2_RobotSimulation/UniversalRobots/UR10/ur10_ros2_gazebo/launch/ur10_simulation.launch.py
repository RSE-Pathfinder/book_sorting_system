#!/usr/bin/python3

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Seemal Asif      - s.asif@cranfield.ac.uk                                   #
#           Phil Webb        - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: July, 2022.                                                                    #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA (2022) ROS2.0 ROBOT SIMULATION. URL: https://github.com/IFRA-Cranfield/ros2_RobotSimulation.

#  Reference: This UR10 Gazebo Simulation package has been created thanks to the information provided in the following GitHub repositories:
#   - Universal Robots ROS2 Driver: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver 
#   - Universal Robots ROS2 Gazebo Simulation: https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation

# ur10_simulation.launch.py:
# Launch file for the Universal Robots UR10 Robot GAZEBO SIMULATION in ROS2 Foxy:

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    
    # ***** GAZEBO ***** #   
    # DECLARE Gazebo WORLD file:
    ur10_ros2_gazebo = os.path.join(
        get_package_share_directory('ur10_ros2_gazebo'),
        'worlds',
        'ur10.world')
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': ur10_ros2_gazebo}.items(),
             )

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

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ur10'],
                        output='screen')

    # ***** CONTROLLERS ***** #
    # Joint STATE Controller:
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'joint_state_controller'], 
        output='screen'
    )
    # Joint TRAJECTORY Controller:
    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'ur10_robot_controller'], # 'joint_trajectory_controller'
        output='screen'
    )
    #Static TF:
    #static_tf = Node(package='tf2_ros',
    #                executable='static_transform_publisher',
    #                name='static_transform_publisher',
    #                output='log',
    #                arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'ur_base'])

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        gazebo, 
        node_robot_state_publisher,
    #   static_tf,
        spawn_entity
    ])