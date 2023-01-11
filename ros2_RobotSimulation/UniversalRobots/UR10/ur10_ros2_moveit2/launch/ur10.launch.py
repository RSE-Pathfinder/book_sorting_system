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

# ur10.launch.py:
# Launch file for the UR10 Robot GAZEBO + MoveIt!2 SIMULATION in ROS2 Foxy:

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
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

    
    # *********************** Gazebo *********************** # 
    
    # DECLARE Gazebo WORLD file:
    ur10_gazebo = os.path.join(
        get_package_share_directory('ur10_ros2_gazebo'),
        'worlds',
        'ur10.world')
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': ur10_gazebo}.items(),
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
    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ur10'],
                        output='screen')

    # ***** STATIC TRANSFORM ***** #
    # NODE -> Static TF:
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )
    # Publish TF:
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # ***** CONTROLLERS ***** #
    # ur10 robot controller:
    load_ur10_robot_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'ur10_robot_controller'],
        output='screen'
    )
    # Joint STATE Controller:
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_start_controller', 'joint_state_controller'],
        output='screen'
    )
    # ros2_control:
    ros2_controllers_path = os.path.join(
        get_package_share_directory("ur10_ros2_gazebo"),
        "config",
        "ur10_controller.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros2_controllers_path],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )
    # Load controllers: 
    load_controllers = []
    for controller in [
        "ur10_robot_controller",
        "joint_state_controller",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]


    # *********************** MoveIt!2 *********************** #   
    
    # Command-line argument: RVIZ file?
    rviz_arg = DeclareLaunchArgument(
        "rviz_file", default_value="False", description="Load RVIZ file."
    )

    # *** PLANNING CONTEXT *** #
    # Robot description, URDF:
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory("ur10_ros2_gazebo"),
            "urdf",
            "ur10.urdf.xacro",
        )
    )
    robot_description = {"robot_description": robot_description_config.toxml()}
    # Robot description, SRDF:
    robot_description_semantic_config = load_file(
        "ur10_ros2_moveit2", "config/ur10.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_config
    }

    # Kinematics.yaml file:
    kinematics_yaml = load_yaml(
        "ur10_ros2_moveit2", "config/kinematics.yaml"
    )
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Move group: OMPL Planning.
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        "ur10_ros2_moveit2", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # MoveIt!2 Controllers:
    moveit_simple_controllers_yaml = load_yaml(
        "ur10_ros2_moveit2", "config/ur10_controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # START NODE -> MOVE GROUP:
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )

    # RVIZ:
    load_RVIZfile = LaunchConfiguration("rviz_file")
    rviz_base = os.path.join(get_package_share_directory("ur10_ros2_moveit2"), "launch")
    rviz_full_config = os.path.join(rviz_base, "ur10_moveit2.rviz")
    rviz_empty_config = os.path.join(rviz_base, "ur10_moveit2_empty.rviz")
    rviz_node_empty = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_empty_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        condition=IfCondition(load_RVIZfile),
    )
    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
        ],
        condition=UnlessCondition(load_RVIZfile),
    )

    return LaunchDescription(
        [
            # Gazebo nodes:
            gazebo, 
            spawn_entity,
            
            # ROS2_CONTROL:
            static_tf,
            robot_state_publisher,
            ros2_control_node,
            
            RegisterEventHandler(
                OnProcessExit(
                    target_action = spawn_entity,
                    on_exit = [

                        # MoveIt!2:
                        rviz_arg,
                        run_move_group_node,

                        # RVIZ with 5s delay:
                        TimerAction(
                            period=5.0,
                            actions=[rviz_node_full],
                        )

                    ]
                )
            )
        ]
        + load_controllers
    )