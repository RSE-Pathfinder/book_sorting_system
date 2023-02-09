from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Constants for paths to different files and folders
    package_name = 'wrp_ros2'
    robot_name_in_model = 'scout_v2'
    urdf_file_path = 'urdf/scout_v2/scout_v2_nav.xacro'

    # Pose where we want to spawn the robot
    spawn_x_val = '0.0'
    spawn_y_val = '0.0'
    spawn_z_val = '0.0'
    spawn_yaw_val = '0.0'

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

    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    #Robot State   
    robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[robot_description],
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
        robot_state_publisher,
        robot_entity_spawner,
        joint_state_publisher_node,
    ]
    
    return LaunchDescription(nodes)