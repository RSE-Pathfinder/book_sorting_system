import os

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Constants for paths to different files and folders
    package_name = 'wrp_ros2'
    robot_name_in_model = 'scout_v2'
    urdf_file_path = 'urdf/scout_v2/scout_v2.xacro'

    # Pose where we want to spawn the robot
    spawn_x_val = '2.0'
    spawn_y_val = '12.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '-1.50'

    # Set the path to different files and folders.
    config_file = "config/scout_v2_control.yaml"

    # Extract URDF using XACRO 
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(package_name), urdf_file_path]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Extract Config file
    scout_diff_drive_controller = PathJoinSubstitution(
        [FindPackageShare("bss_shelf_bringup"), config_file],
    )

    # Controller node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, scout_diff_drive_controller],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.    
    robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[robot_description],
    )

    # ROS2 Control Joint Broadcaster Controller
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # ROS2 Control diff drive controller
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diff_drive_base_controller"],
        output="screen",
    )

    # Spawn the robot
    robot_entity_spawner = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=['-entity', robot_name_in_model, 
    '-topic', 'robot_description',
    '-x', spawn_x_val,
    '-y', spawn_y_val,
    '-z', spawn_z_val,
    '-Y', spawn_yaw_val],
    output='screen')

    # Add any actions
    nodes = [
        #control_node,
        robot_state_publisher,
        robot_entity_spawner,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ]
    
    return LaunchDescription(nodes)